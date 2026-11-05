"""Compare locked wrist branches on ego_retarget with the Rust safety filter."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
import geo_kin
from geo_kin_core.frames import load_frames
from geo_kin_core.spec import sha256_of_file
from dex_ur_gantry_teleop import model_path
from dex_ur_gantry_teleop.input import from_recording


def study(path, hand, branch, z_offset):
    stream=load_frames(path)
    config=model_path().parent/'safety_capsules.json'
    s=geo_kin.RetargetSession(robot='urgantry',hand=hand,functional_offset=False,
        base_alignment_mode='first_frame',arm_ik_limited=False,collision_avoidance=False,
        safety_filter=True,safety_config=config.read_text(),control_rate_hz=60,
        wrist_branch=branch,workspace_offset=(0,0,z_offset))
    if s.model_sha256 != sha256_of_file(model_path(hand)):
        raise RuntimeError('Wheel/model signature mismatch')
    q={side:np.array(s.default_q[side]) for side in ('left','right')}
    s.reset(q['right'],q['left'])
    for side in q:
        assert s.filter_arm_safety(side,q[side],q[side])['accepted']
    counts=Counter(); errors=[]; paired=[]; held=0; valid=0; close=0; movement=0.
    for i in range(len(stream)):
        f=from_recording(stream[i],s)
        out=s.solve(f,q_current_right=q['right'],q_current_left=q['left'])
        positions={}; targets={}; good=[]
        for side in q:
            goal=getattr(out,'q_goal_'+side)
            if goal is not None:
                movement+=float(np.sum(np.abs(goal-q[side])));q[side]=goal
            if i<60:continue # report task tracking after a common one-second startup
            target=getattr(f,side+'_eef',None)
            if target is None:continue
            valid+=1;held+=int(goal is None)
            counts[out.diag.solution_branch.get(side,'unknown')]+=1
            fk=s.eef_fk(side,q[side])
            p=f.R_world_upper_body.T@(target.position-f.p_world_upper_body)+[0,0,z_offset]
            r=f.R_world_upper_body.T@target.rotation
            e=float(np.linalg.norm(fk['position']-p))
            angle=float(np.degrees(np.arccos(np.clip((np.trace(fk['rotation'].T@r)-1)/2,-1,1))))
            errors.append([e,angle]);good.append(e<=.02 and angle<=10)
            positions[side]=fk['position'];targets[side]=p
        if len(good)==2:
            close+=int(all(good));paired.append(float(np.linalg.norm((positions['left']-positions['right'])-(targets['left']-targets['right']))))
    a=np.asarray(errors)
    return dict(clip=path.parent.name,hand=hand,branch=branch,z_offset=z_offset,
        valid_arm_frames=valid,held_arm_frames=held,statuses=dict(counts),
        position_mean_mm=float(a[:,0].mean()*1000),position_p95_mm=float(np.percentile(a[:,0],95)*1000),
        orientation_mean_deg=float(a[:,1].mean()),orientation_p95_deg=float(np.percentile(a[:,1],95)),
        bimanual_tracking_percent=100*close/len(paired) if paired else None,
        relative_hand_error_mean_mm=float(np.mean(paired)*1000),joint_travel_rad=movement)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--frames',type=Path,help='Study one converted stream instead of all default streams')
    p.add_argument('--branches',nargs='+',choices=['wrist_up','wrist_down'],default=['wrist_up','wrist_down'])
    a=p.parse_args();results=[]
    paths=[a.frames] if a.frames else sorted(a.root.glob('*/frames.npz'))
    for path in paths:
        for hand in ('sharpa','wuji'):
            for z in (0.,-.05,.05):
                for branch in a.branches:
                    r=study(path,hand,branch,z);results.append(r)
                    print(json.dumps(r),flush=True)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    wheel_stamp=Path(geo_kin.__file__).resolve().parent.parent/'.geo-kin-wheel-sha256'
    a.output.write_text(json.dumps({'protocol':'60 Hz; fixed first-frame virtual body; no functional offset; unrestricted joints; branch-specific home; metrics after 1 s startup; bimanual tracking <=20 mm and <=10 degrees',
        'upstream_revision':'907dbaae8a14c448b5d8a41e8e03bce28c8d5e6e',
        'wheel_sha256':wheel_stamp.read_text().strip() if wheel_stamp.exists() else None,
        'extension_sha256':hashlib.sha256(next(Path(geo_kin.__file__).parent.glob('*.so')).read_bytes()).hexdigest(),
        'input_sha256':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
        'safety_config_sha256':hashlib.sha256((model_path().parent/'safety_capsules.json').read_bytes()).hexdigest(),'results':results},indent=2)+'\n')

if __name__=='__main__': main()

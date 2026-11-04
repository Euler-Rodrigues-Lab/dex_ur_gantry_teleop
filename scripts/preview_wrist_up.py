"""Near-table wrist branch comparison; 1 = original, 2 = wrist up inward, 3 = wrist up outward (default).

This is a synthetic target based on the mustard sample, not reconstructed video
motion. Branch switching is a static comparison, not a collision-free trajectory.
"""
import argparse
import json
import subprocess
import time
from pathlib import Path
import mujoco
import numpy as np
import geo_kin
from dex_ur_gantry_teleop import load_model, model_path
from dex_ur_gantry_teleop.input import from_recording
from dex_ur_gantry_teleop.viewer import managed_viewer, configure_camera
from xrt_devices.integrations.geo_kin import open_motion_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hand', choices=['sharpa','wuji'], default='sharpa')
    parser.add_argument('--render', type=Path, help='Save comparison images instead of opening viewer')
    args = parser.parse_args()
    s = geo_kin.RetargetSession(robot='urgantry', hand=args.hand,
        functional_offset=False, arm_ik_limited=False, collision_avoidance=False)
    m = load_model(args.hand); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,m.key('home').id)
    source = open_motion_source(frames=str(model_path().parent/'sample_motion/picking_up_mustard.npz'))
    for tick in range(409):
        out = s.solve(from_recording(source.frame_at_time(tick/60),s))
    names=['shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3']
    ids={side:m.jnt_qposadr[[m.joint(f'{side}_{j}_joint').id for j in names]] for side in ['left','right']}
    for side in ids:
        d.qpos[ids[side]]=getattr(out,'q_goal_'+side)
        hj=[m.joint(n).id for n in s.hand_joint_names(side)]
        d.qpos[m.jnt_qposadr[hj]]=getattr(out,'q_goal_'+side+'_hand')
    prev=np.asarray(out.q_goal_right)
    goal=s.eef_fk('right',prev)
    goal['position'][2]=.82-1.37974159
    low=s.eef_ik('right',goal['position'],goal['rotation'],prev)
    if low is None: raise RuntimeError('Near-table target unreachable')
    rng=np.random.default_rng(42); branches=[]
    for seed in [low,*rng.uniform(-np.pi,np.pi,(512,6))]:
        q=s.eef_ik('right',goal['position'],goal['rotation'],seed)
        if q is None: continue
        q=(np.asarray(q)+np.pi)%(2*np.pi)-np.pi
        if any(np.linalg.norm((q-b+np.pi)%(2*np.pi)-np.pi)<1e-5 for b in branches):continue
        fk=s.eef_fk('right',q)
        np.testing.assert_allclose(fk['position'],goal['position'],atol=1e-6)
        np.testing.assert_allclose(fk['rotation'],goal['rotation'],atol=1e-6)
        branches.append(q)
    def heights(q):
        d.qpos[ids['right']]=q;mujoco.mj_forward(m,d)
        return [float(d.xpos[m.body('right_'+n+'_link').id,2]) for n in ['wrist_1','wrist_2','forearm']]
    # Prefer both intermediate wrist centers high; elbow height breaks ties.
    high=max(branches,key=lambda q:(round(min(heights(q)[:2]),6),heights(q)[2]))
    high_height=min(heights(high)[:2])
    wrist_up_branches=[q for q in branches if abs(min(heights(q)[:2])-high_height)<1e-6]
    def outward_distance(q):
        heights(q)
        # Robot +Y left is world -X; right elbow outward is world +X.
        return float(d.xpos[m.body('right_forearm_link').id,0])
    outward=max(wrist_up_branches,key=outward_distance)
    def report(q):
        h=heights(q)
        return dict(wrist_1_z=h[0],wrist_2_z=h[1],elbow_z=h[2],elbow_outward=outward_distance(q),q=np.asarray(q).tolist())
    results=dict(hand=args.hand,synthetic=True,table_top_z=.775,eef_z=.82,
                 branches_found=len(branches),original=report(low),wrist_up=report(high),wrist_up_outward=report(outward))
    print(json.dumps(results,indent=2),flush=True)
    camera=mujoco.MjvCamera();configure_camera(camera)
    camera.lookat[:]=[.05,-.02,.95];camera.distance=1.65;camera.elevation=-12
    if args.render:
        args.render.mkdir(parents=True,exist_ok=True)
        (args.render/'branches.json').write_text(json.dumps(results,indent=2))
        with mujoco.Renderer(m,720,960) as renderer:
            for name,q in [('original',low),('wrist_up',high),('wrist_up_outward',outward)]:
                heights(q);renderer.update_scene(d,camera=camera)
                subprocess.run(['ffmpeg','-v','error','-y','-f','rawvideo','-pixel_format','rgb24',
                    '-video_size','960x720','-i','-','-frames:v','1',str(args.render/f'{name}.png')],
                    input=renderer.render().tobytes(),check=True)
        return
    selection=[outward]
    def key(key):
        if key==ord('1'):selection[0]=low;print('Original branch',flush=True)
        if key==ord('2'):selection[0]=high;print('Wrist up, elbow inward',flush=True)
        if key==ord('3'):selection[0]=outward;print('Wrist up, elbow outward',flush=True)
    heights(outward)
    print('Showing wrist up, elbow outward. Press 1: original / 2: wrist up inward / 3: wrist up outward. Static FK comparison; close to exit.',flush=True)
    with managed_viewer(m,d,key_callback=key,show_left_ui=False,show_right_ui=False) as v:
        with v.lock():
            v.cam.lookat[:]=camera.lookat;v.cam.distance=camera.distance
            v.cam.azimuth=camera.azimuth;v.cam.elevation=camera.elevation
        while v.is_running():
            with v.lock():heights(selection[0])
            v.sync();time.sleep(1/60)


if __name__=='__main__':main()

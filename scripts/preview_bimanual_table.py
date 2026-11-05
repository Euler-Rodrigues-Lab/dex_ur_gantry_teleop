"""Bimanual jar pose. Left/right arrows cycle left arm; up/down cycle right arm."""
import argparse
import json
import subprocess
import time
from pathlib import Path
import geo_kin
import mujoco
import numpy as np
from dex_ur_gantry_teleop import model_path
from dex_ur_gantry_teleop.viewer import managed_viewer, configure_camera


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--render',type=Path)
    p.add_argument('--capsules',action='store_true',help='Show exploratory palm/wrist and column capsules')
    p.add_argument('--safety-config',type=Path,default=model_path().parent/'safety_capsules.json')
    args=p.parse_args()
    s=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',functional_offset=False,
                            arm_ik_limited=False,collision_avoidance=False,
                            safety_filter=True,safety_config=args.safety_config.read_text())
    spec=mujoco.MjSpec.from_file(str(model_path('sharpa')))
    spec.worldbody.add_geom(name='preview_jar',type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0.,.045,.855],size=[.055,.08,0],rgba=[.35,.55,.65,1],contype=0,conaffinity=0)
    spec.worldbody.add_geom(name='preview_lid',type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        pos=[0.,.045,.940],size=[.058,.005,0],rgba=[.9,.7,.12,1],contype=0,conaffinity=0)
    m=spec.compile();d=mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m,d,m.key('home').id)
    world_r=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
    origin=np.array([0.,-.3825,1.37974159])
    rng=np.random.default_rng(64)
    reports={}; all_branches={}; arm_ids={}; selected={}
    for side,sign in [('left',-1),('right',1)]:
        ids=m.jnt_qposadr[[m.joint(f'{side}_{n}_joint').id for n in
                         ['shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3']]]
        names=s.hand_joint_names(side)
        finger_q=np.zeros(len(names))
        for i,n in enumerate(names):
            if 'thumb' not in n:
                if n.endswith('MCP_FE'):finger_q[i]=.25
                elif n.endswith('PIP'):finger_q[i]=.45
                elif n.endswith('DIP'):finger_q[i]=.25
        finger_q *= [np.sign(np.sum(m.joint(n).range)) or 1 for n in names]
        d.qpos[m.jnt_qposadr[[m.joint(n).id for n in names]]]=finger_q
        zero=np.zeros(len(names));flex=zero.copy()
        index=next(i for i,n in enumerate(names) if n.endswith('index_MCP_FE'))
        flex[index]=.05*np.sign(np.sum(m.joint(names[index]).range))
        tips0=s.hand_fingertips_at(side,zero);tips1=s.hand_fingertips_at(side,flex)
        home=s.eef_fk(side,s.default_q[side])
        bend=home['rotation'].T@home['hand_rotation']@(tips1['index']-tips0['index'])
        # Palm frame +X points along fingers; use flexion to identify palm side.
        x=np.array([1.,0,0]);y=np.array([0.,sign*np.sign(bend[1]),0.]);z=np.cross(x,y)
        rotation=np.column_stack([x,y,z])
        assert sign*(rotation@bend)[1]>0
        target_world=np.array([sign*.105,.005,.915])
        target=world_r.T@(target_world-origin)
        branches=[]
        for seed in [s.default_q[side],*rng.uniform(-np.pi,np.pi,(512,6))]:
            q=s.eef_ik(side,target,rotation,seed)
            if q is None:continue
            q=(np.asarray(q)+np.pi)%(2*np.pi)-np.pi
            if any(np.linalg.norm((q-b+np.pi)%(2*np.pi)-np.pi)<1e-5 for b in branches):continue
            fk=s.eef_fk(side,q)
            np.testing.assert_allclose(fk['position'],target,atol=1e-6)
            np.testing.assert_allclose(fk['rotation'],rotation,atol=1e-6)
            branches.append(q)
        if not branches:raise RuntimeError(f'{side} target unreachable')
        def measure(q):
            d.qpos[ids]=q;mujoco.mj_forward(m,d)
            wrist=min(d.xpos[m.body(f'{side}_{n}_link').id,2] for n in ['wrist_1','wrist_2'])
            elbow=d.xpos[m.body(f'{side}_forearm_link').id]
            return round(float(wrist),6),float(sign*elbow[0])
        branches.sort(key=lambda q:tuple(np.round(q,8)))
        q=max(branches,key=measure);measure(q)
        assert len(branches)==8, f'Expected eight branches for this nonsingular pose; found {len(branches)}'
        all_branches[side]=branches;arm_ids[side]=ids
        selected[side]=next(i for i,b in enumerate(branches) if np.array_equal(q,b))
        reports[side]=dict(target_world=target_world.tolist(),branches=len(branches),q=q.tolist(),
                           wrist_min_z=measure(q)[0],elbow_z=float(d.xpos[m.body(f'{side}_forearm_link').id,2]),
                           elbow_outward=float(sign*d.xpos[m.body(f'{side}_forearm_link').id,0]))
    mujoco.mj_forward(m,d)
    # Check actual transformed mesh extents for both hands (including fingertips).
    for side in ['left','right']:
        vertices=[]
        for g in range(m.ngeom):
            if not m.body(m.geom_bodyid[g]).name.startswith(side+'_hand_'):continue
            if m.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH:continue
            mesh=m.geom_dataid[g];start=m.mesh_vertadr[mesh];count=m.mesh_vertnum[mesh]
            vertices.append(m.mesh_vert[start:start+count]@d.geom_xmat[g].reshape(3,3).T+d.geom_xpos[g])
        v=np.concatenate(vertices)
        reports[side]['mesh_bounds']=[v.min(axis=0).tolist(),v.max(axis=0).tolist()]
        reports[side]['table_clearance_m']=float(v[:,2].min()-.775)
        assert v[:,2].min()>.775, 'Hand intersects tabletop'
        assert v[:,0].min()>-.56 and v[:,0].max()<.56
        assert v[:,1].min()>-.4725 and v[:,1].max()<.2925
    assert reports['left']['mesh_bounds'][1][0] < reports['right']['mesh_bounds'][0][0], 'Hands overlap laterally'
    # Validate that the chosen arm poses mirror each other about world X=0.
    for name in ['forearm','wrist_1','wrist_2','wrist_3']:
        a=d.xpos[m.body(f'left_{name}_link').id]*np.array([-1.,1.,1.])
        b=d.xpos[m.body(f'right_{name}_link').id]
        np.testing.assert_allclose(a,b,atol=1e-5)
    from dex_ur_gantry_teleop.capsule_preview import draw_safety_state
    def capsule_state():
        state=s.safety_capsules_at("left",d.qpos[arm_ids["left"]])
        right=s.safety_capsules_at("right",d.qpos[arm_ids["right"]])
        state["capsules"]=[c for c in state["capsules"] if c["side"]=="left"]+[c for c in right["capsules"] if c["side"]=="right"]
        return state
    show_capsules=[args.capsules]
    print(json.dumps(reports,indent=2),flush=True)
    camera=mujoco.MjvCamera();configure_camera(camera)
    camera.lookat[:]=[0.,-.02,1.04];camera.distance=1.9;camera.elevation=-20;camera.azimuth=270
    if args.render:
        args.render.mkdir(parents=True,exist_ok=True)
        (args.render/'pose.json').write_text(json.dumps(reports,indent=2))
        with mujoco.Renderer(m,720,960) as r:
            r.update_scene(d,camera=camera)
            if args.capsules:draw_safety_state(r.scene,capsule_state())
            subprocess.run(['ffmpeg','-v','error','-y','-f','rawvideo','-pixel_format','rgb24','-video_size','960x720','-i','-','-frames:v','1',str(args.render/'jar_pose.png')],input=r.render().tobytes(),check=True)
        return
    pending=[]
    def key_callback(key):
        if key in (ord("C"),ord("c")):show_capsules[0]=not show_capsules[0]
        # GLFW key codes; callbacks only enqueue changes for the viewer loop.
        action={263:('left',-1),262:('left',1),265:('right',1),264:('right',-1)}.get(key)
        if action:pending.append(action)
    def announce():
        print(f"Left branch {selected['left']+1}/8 | Right branch {selected['right']+1}/8",flush=True)
    with managed_viewer(m,d,key_callback=key_callback,show_left_ui=False,show_right_ui=False) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:]=camera.lookat;viewer.cam.distance=camera.distance
            viewer.cam.azimuth=camera.azimuth;viewer.cam.elevation=camera.elevation
        print('Left/right arrows: LEFT arm. Up/down arrows: RIGHT arm. Branches snap; no trajectory or collision filtering.',flush=True)
        announce()
        print("C toggles capsule overlays; geometry only, no XPBD correction.",flush=True)
        while viewer.is_running():
            with viewer.lock():
                while pending:
                    side,step=pending.pop(0)
                    selected[side]=(selected[side]+step)%len(all_branches[side])
                    d.qpos[arm_ids[side]]=all_branches[side][selected[side]]
                    mujoco.mj_forward(m,d);announce()
                viewer.user_scn.ngeom=0
                if show_capsules[0]:draw_safety_state(viewer.user_scn,capsule_state())
            viewer.sync();time.sleep(1/60)


if __name__=='__main__':main()

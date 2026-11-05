"""Rust safety geometry parity and command integration; requires gantry wheel."""
import json
import numpy as np
import mujoco
import pytest
import geo_kin
from dex_ur_gantry_teleop import model_path
from dex_ur_gantry_teleop.capsule_preview import draw_safety_state
from geo_kin_core.types import EEFFrame, EEFPose

CONFIG=model_path().parent/'safety_capsules.json'

def session(hand='sharpa',config=None,**kwargs):
    return geo_kin.RetargetSession(robot='urgantry',hand=hand,functional_offset=False,
        arm_ik_limited=False,safety_filter=True,safety_config=json.dumps(config) if config else CONFIG.read_text(),**kwargs)

@pytest.mark.parametrize('hand',['sharpa','wuji'])
def test_rust_capsules_match_mujoco_local_frames_and_overlay(hand):
    s=session(hand);cfg=json.loads(CONFIG.read_text())
    m=mujoco.MjModel.from_xml_path(str(model_path(hand)));d=mujoco.MjData(m)
    rng=np.random.default_rng(83)
    for side in ['left','right']:
        joints=[m.joint(f'{side}_{n}_joint').id for n in ['shoulder_pan','shoulder_lift','elbow','wrist_1','wrist_2','wrist_3']]
        for _ in range(60):
            q=rng.uniform(-4,4,6);d.qpos[m.jnt_qposadr[joints]]=q;mujoco.mj_forward(m,d)
            state=s.safety_capsules_at(side,q)
            actual=[c for c in state['capsules'] if c['side']==side]
            assert len(actual)==len(cfg['capsules'][hand])==3
            for cap,spec in zip(actual,cfg['capsules'][hand]):
                body=m.body(side+'_'+spec['body']).id
                for endpoint in ['a','b']:
                    expected=d.xpos[body]+d.xmat[body].reshape(3,3)@spec[endpoint]
                    np.testing.assert_allclose(cap[endpoint],expected,atol=1e-9)
                assert cap['radius']==spec['radius']
    scene=mujoco.MjvScene(m,maxgeom=30);draw_safety_state(scene,state)
    for i,cap in enumerate(state['capsules']):
        geom=scene.geoms[i]
        np.testing.assert_allclose(geom.pos,(np.array(cap['a'])+cap['b'])/2,atol=1e-6)
        assert geom.size[0]==pytest.approx(cap['radius'])
        assert geom.size[2]==pytest.approx(np.linalg.norm(np.array(cap['b'])-cap['a'])/2,abs=1e-6)


def test_config_is_applied_and_reset_preserves_it():
    cfg=json.loads(CONFIG.read_text());cfg['capsules']['sharpa'][0]['radius']=.052
    s=session(config=cfg);s.reset()
    assert s.safety_state['capsules'][0]['radius']==.052
    assert s.safety_state['reports']==[None,None]
    disabled=geo_kin.RetargetSession(robot='urgantry',hand='sharpa')
    assert disabled.safety_state is None

@pytest.mark.parametrize('change',[('radius',-1),('radius',0),('body','bad_body')])
def test_bad_config_is_rejected(change):
    cfg=json.loads(CONFIG.read_text());key,value=change;cfg['capsules']['sharpa'][0][key]=value
    with pytest.raises(ValueError):session(config=cfg)


def test_solve_executes_filter_and_returns_same_geometry():
    s=session();q={side:s.default_q[side] for side in ['left','right']}
    fields=dict(p_world_upper_body=np.zeros(3),R_world_upper_body=np.eye(3))
    for side in q:
        fk=s.eef_fk(side,q[side]);fields[side+'_eef']=EEFPose(fk['position'],fk['rotation'])
    out=s.solve(EEFFrame(**fields),q_current_left=q['left'],q_current_right=q['right'])
    assert set(out.diag.solution_branch.values())=={'clear'}
    for side in q:np.testing.assert_allclose(getattr(out,'q_goal_'+side),q[side],atol=1e-8)
    assert all(r['status']=='clear' for r in s.safety_state['reports'])
    # Existing penetration is never labeled a corrected/clear command.
    cfg=json.loads(CONFIG.read_text());cfg['table']['height']=4
    blocked=session(config=cfg)
    result=blocked.solve(EEFFrame(**fields),q_current_left=q['left'],q_current_right=q['right'])
    assert result.q_goal_left is None and result.q_goal_right is None
    assert set(result.diag.solution_branch.values())=={'unsafe_current'}


def test_lowered_replay_corrects_and_preserves_clearance_and_velocity():
    from xrt_devices.integrations.geo_kin import open_motion_source
    from dex_ur_gantry_teleop.input import from_recording
    s=session(workspace_offset=(0,0,-.20));cfg=json.loads(CONFIG.read_text())
    current={side:np.asarray(s.default_q[side]) for side in ['left','right']}
    source=open_motion_source(frames=str(model_path().parent/'sample_motion/picking_up_mustard.npz'))
    corrected=0
    for tick in range(int(source.duration*60)+1):
        out=s.solve(from_recording(source.frame_at_time(tick/60),s),q_current_left=current['left'],q_current_right=current['right'])
        for side in current:
            q=getattr(out,'q_goal_'+side)
            if q is not None:
                assert np.max(abs(q-current[side]))<=cfg['solver']['max_joint_velocity']/60+1e-9
                current[side]=q
            corrected+=out.diag.solution_branch[side]=='corrected'
        state=s.safety_state
        for cap in state['capsules']:
            # Independent check of the displayed FK capsule against the plane.
            gap=min(cap['a'][2],cap['b'][2])-cap['radius']-cfg['table']['height']
            assert gap>=cfg['solver']['margin']-cfg['solver']['tolerance']-1e-9
    assert corrected>0


def test_invalid_explicit_safety_joints_raise_python_errors():
    s=session();q=s.default_q['right']
    with pytest.raises(ValueError):s.safety_capsules_at('right',None)
    with pytest.raises(ValueError):s.filter_arm_safety('right',None,q)
    with pytest.raises(ValueError):s.filter_arm_safety('right',q,None)
    with pytest.raises(ValueError):s.filter_arm_safety('right',[float('nan')]*6,q)

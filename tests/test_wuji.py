"""Wuji model, licensed hand solver and EEF integration against MuJoCo."""
from types import SimpleNamespace as NS
import numpy as np
import mujoco
import geo_kin
from geo_kin_core.spec import sha256_of_file
from xrt_devices.integrations.geo_kin import open_motion_source
from dex_ur_gantry_teleop import model_path
from dex_ur_gantry_teleop.input import from_recording
from urgantry_sim.build_urgantry import ARM_JOINTS


def session():
    return geo_kin.RetargetSession(robot='urgantry',hand='wuji',functional_offset=False,arm_ik_limited=False)


def test_wuji_exact_arm_and_hand_fk():
    s=session();m=mujoco.MjModel.from_xml_path(str(model_path('wuji')));d=mujoco.MjData(m)
    assert m.nu==52 and s.model_sha256==sha256_of_file(model_path('wuji'))
    r=np.array([[0.,-1,0],[1,0,0],[0,0,1]]);origin=np.array([0.,-.3825,1.37974159])
    rng=np.random.default_rng(24)
    for side in ('left','right'):
        ids=[m.joint(f'{side}_{name}_joint').id for name in ARM_JOINTS]
        hand_ids=[m.joint(n).id for n in s.hand_joint_names(side)]
        assert len(hand_ids)==20
        for _ in range(100):
            q=rng.uniform(m.jnt_range[ids,0],m.jnt_range[ids,1])
            h=rng.uniform(m.jnt_range[hand_ids,0],m.jnt_range[hand_ids,1])
            d.qpos[:]=0;d.qpos[m.jnt_qposadr[ids]]=q;d.qpos[m.jnt_qposadr[hand_ids]]=h
            mujoco.mj_forward(m,d)
            fk=s.eef_fk(side,q);b=m.body(side+'_hand_palm_link').id
            np.testing.assert_allclose(fk['hand_position'],r.T@(d.xpos[b]-origin),atol=1e-8)
            np.testing.assert_allclose(fk['hand_rotation'],r.T@d.xmat[b].reshape(3,3),atol=1e-8)
            index=m.joint(side+'_hand_finger2_joint1').id
            np.testing.assert_allclose(fk['position'],r.T@(d.xanchor[index]-origin),atol=1e-8)
            solved=s.eef_ik(side,fk['position'],fk['rotation'])
            assert solved is not None
            np.testing.assert_allclose(s.eef_fk(side,solved)['rotation'],fk['rotation'],atol=1e-6)
            tips=s.hand_fingertips_at(side,h)
            for i,finger in enumerate(('thumb','index','middle','ring','pinky')):
                site=m.site(f'{side}_hand_finger{i+1}_tip').id
                expected=d.xmat[b].reshape(3,3).T@(d.site_xpos[site]-d.xpos[b])
                np.testing.assert_allclose(tips[finger],expected,atol=1e-9)


def test_wuji_replays_are_finite_bounded_and_resettable():
    s=session();m=mujoco.MjModel.from_xml_path(str(model_path('wuji')))
    for sample in ('ipman_roll','picking_up_mustard'):
        s.reset()
        source=open_motion_source(frames=str(model_path().parent/'sample_motion'/f'{sample}.npz'))
        first=None
        for tick in range(int(source.duration*60)+1):
            f=from_recording(source.frame_at_time(tick/60),s);out=s.solve(f)
            if first is None:first=out.q_goal_left_hand.copy()
            for side in ('left','right'):
                q=getattr(out,'q_goal_'+side+'_hand');assert q.shape==(20,) and np.isfinite(q).all()
                ids=[m.joint(n).id for n in s.hand_joint_names(side)]
                assert np.all(q>=m.jnt_range[ids,0]-1e-8) and np.all(q<=m.jnt_range[ids,1]+1e-8)
                # Both mounted hand variants retain reachable EEF goals in these clips.
                assert getattr(out,'q_goal_'+side) is not None,(sample,tick,side)
        s.reset();again=s.solve(from_recording(source.frame_at_time(0),s))
        np.testing.assert_allclose(again.q_goal_left_hand,first,atol=1e-10)
        missing=from_recording(source.frame_at_time(0),s);missing.left_fingers_wrist={}
        assert s.solve(missing).q_goal_left_hand is None

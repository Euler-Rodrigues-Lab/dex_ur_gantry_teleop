"""Selected wrist root is immutable across frames, feedback, safety and reset."""
import numpy as np
import pytest
import geo_kin
from dex_ur_gantry_teleop import model_path
from dex_ur_gantry_teleop.input import from_recording
from xrt_devices.integrations.geo_kin import open_motion_source

@pytest.mark.parametrize('hand',['sharpa','wuji'])
@pytest.mark.parametrize('branch',['wrist_up','wrist_down'])
def test_replay_retains_branch_through_safety_and_reset(hand,branch):
    s=geo_kin.RetargetSession(robot='urgantry',hand=hand,functional_offset=False,
        arm_ik_limited=False,wrist_branch=branch,safety_filter=True,
        safety_config=(model_path().parent/'safety_capsules.json').read_text())
    assert s.wrist_branch==branch
    with pytest.raises(AttributeError):s.wrist_branch='wrist_down'
    source=open_motion_source(frames=str(model_path().parent/'sample_motion/picking_up_mustard.npz'))
    count=0
    for _ in range(2):
        s.reset();current={side:np.array(s.default_q[side]) for side in ['left','right']}
        for tick in range(180):
            out=s.solve(from_recording(source.frame_at_time(tick/60),s),q_current_left=current['left'],q_current_right=current['right'])
            for side,elbow in [('left',1),('right',-1)]:
                q=getattr(out,'q_goal_'+side)
                if q is None:continue
                count+=1
                assert np.sin(q[2])*elbow>1e-6
                assert np.sin(q[4])*(elbow if branch=='wrist_down' else -elbow)>1e-6
                current[side]=q
        assert s.wrist_branch==branch
    assert count>0


def test_wrong_branch_reset_and_feedback_hold():
    down=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',wrist_branch='wrist_down',arm_ik_limited=False,functional_offset=False)
    up=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',wrist_branch='wrist_up',arm_ik_limited=False,functional_offset=False)
    with pytest.raises(ValueError):up.reset(down.default_q['right'],down.default_q['left'])
    from geo_kin_core.types import EEFFrame,EEFPose
    fk=up.eef_fk('right',up.default_q['right'])
    frame=EEFFrame(p_world_upper_body=np.zeros(3),R_world_upper_body=np.eye(3),right_eef=EEFPose(fk['position'],fk['rotation']))
    out=up.solve(frame,q_current_right=down.default_q['right'])
    assert out.q_goal_right is None
    assert out.diag.solution_branch['right']=='feedback_branch_mismatch'
    with pytest.raises(ValueError):geo_kin.RetargetSession(robot='urgantry',hand='sharpa',wrist_branch='auto')

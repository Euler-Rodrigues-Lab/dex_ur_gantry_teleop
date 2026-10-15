"""No elbow dependency in raw XR processing; shared legacy paths stay intact."""
from pathlib import Path
import unittest
import numpy as np
from xrt_devices.schemas.body_pose import Bone
from xrt_devices.schemas.openxr_skeletons import FullBodyBoneId as ID
from xrt_devices.integrations.eef import bones_to_eef_action

class EEFInputTests(unittest.TestCase):
    def test_missing_elbows_and_one_hand(self):
        fixture=Path(__file__).resolve().parents[1]/'external/XRT_devices/tests/fixtures/body_processing_reference.npz'
        with np.load(fixture) as d:
            bones=[Bone(int(i),p,q) for i,p,q in zip(d['iobt/input_ids'],d['iobt/input_positions'],d['iobt/input_quaternions'])]
        original=bones_to_eef_action(bones)
        missing=[b for b in bones if b.id not in (ID.FullBody_LeftArmLower,ID.FullBody_RightArmLower)]
        actual=bones_to_eef_action(missing)
        for side in ('left','right'):
            for key in ('position','rotation'):
                np.testing.assert_array_equal(original[side+'_hand_pose'][key],actual[side+'_hand_pose'][key])
        missing=[b for b in missing if b.id!=ID.FullBody_LeftHandWrist]
        actual=bones_to_eef_action(missing)
        self.assertNotIn('left_hand_pose',actual)
        self.assertIn('right_hand_pose',actual)

class MediaPipeEEFTests(unittest.TestCase):
    def test_engagement_ignores_elbow_visibility(self):
        from types import SimpleNamespace as NS
        from xrt_devices._mediapipe_processing import MediaPipeTeleopDevice
        from xrt_devices._mediapipe_tasks import PoseLandmark
        device=MediaPipeTeleopDevice.__new__(MediaPipeTeleopDevice)
        device.mp_pose=NS(PoseLandmark=PoseLandmark);device.eef_only=True
        points=[NS(visibility=1.) for _ in range(33)]
        for key in ('LEFT_ELBOW','RIGHT_ELBOW'):points[getattr(PoseLandmark,key)].visibility=0.
        pose=NS(pose_landmarks=NS(landmark=points))
        device._check_engagement_from_pose(pose);self.assertTrue(device.engaged)
        points[PoseLandmark.LEFT_WRIST].visibility=0.
        device._check_engagement_from_pose(pose);self.assertTrue(device.engaged)
        self.assertFalse(device._eef_visibility['left'])
        device.eef_only=False;device._check_engagement_from_pose(pose);self.assertFalse(device.engaged)

if __name__=='__main__':unittest.main()


def test_replay_restores_measured_distal_joints():
    import geo_kin
    from xrt_devices.integrations.geo_kin import open_motion_source
    from dex_ur_gantry_teleop import model_path
    from dex_ur_gantry_teleop.input import from_recording
    session=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',functional_offset=False)
    for sample in ('ipman_roll','picking_up_mustard'):
        source=open_motion_source(frames=str(model_path().parent/'sample_motion'/f'{sample}.npz'))
        frame=source.frame_at_time(1.)
        converted=from_recording(frame,session)
        positions=dict(zip(frame.skeleton['names'],frame.skeleton['positions']))
        for side in ('left','right'):
            sew=getattr(frame,side+'_sew')
            wrist=frame.p_world_upper_body+frame.R_world_upper_body@sew.W
            rotation=frame.R_world_upper_body@sew.R_world_wrist
            fingers=getattr(converted,side+'_fingers_wrist')
            for finger,bone in [('index','Index'),('middle','Middle'),('ring','Ring'),('pinky','Little')]:
                prefix=finger if finger=='pinky' else finger+'_finger'
                point=fingers[finger][prefix+'_dip']
                np.testing.assert_allclose(wrist+rotation@point,positions[f'FullBody_{side.title()}Hand{bone}Distal'],atol=1e-10)
            assert 'thumb_cmc' in fingers['thumb'] and 'thumb_ip' in fingers['thumb']
            np.testing.assert_array_equal(fingers['index']['index_finger_mcp'],getattr(frame,side+'_fingers')['index']['index_finger_mcp'])
        output=session.solve(converted)
        for side in ('left','right'):
            q=getattr(output,'q_goal_'+side+'_hand')
            names=session.hand_joint_names(side)
            distal=[q[i] for i,name in enumerate(names) if name.endswith(('_PIP','_DIP'))]
            assert np.max(np.abs(distal)) > 0.05


def test_unrestricted_arm_angles_are_not_clipped_by_simulator():
    import mujoco
    from dex_ur_gantry_teleop import model_path
    from dex_ur_gantry_teleop.viz import apply_position_targets
    model=mujoco.MjModel.from_xml_path(str(model_path()));data=mujoco.MjData(model)
    arm=model.actuator('left_shoulder_pan').id
    hand=model.actuator('left_hand_index_PIP').id
    data.ctrl[arm]=10*np.pi;data.ctrl[hand]=100.
    apply_position_targets(model,data,unlimited_actuators=[arm])
    assert data.qpos[model.jnt_qposadr[model.actuator_trnid[arm,0]]] == 10*np.pi
    assert data.qpos[model.jnt_qposadr[model.actuator_trnid[hand,0]]] == model.actuator_ctrlrange[hand,1]


def test_both_replays_track_every_eef_without_offset():
    import geo_kin
    from xrt_devices.integrations.geo_kin import open_motion_source
    from dex_ur_gantry_teleop import model_path
    from dex_ur_gantry_teleop.input import from_recording
    for sample in ('ipman_roll','picking_up_mustard'):
        session=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',functional_offset=False,arm_ik_limited=False)
        source=open_motion_source(frames=str(model_path().parent/'sample_motion'/f'{sample}.npz'))
        initial=source.frame_at_time(0.)
        r=initial.R_world_upper_body;p=initial.p_world_upper_body
        for tick in range(int(source.duration*60)+1):
            frame=from_recording(source.frame_at_time(tick/60),session)
            output=session.solve(frame)
            for side in ('left','right'):
                q=getattr(output,'q_goal_'+side)
                assert q is not None,(sample,tick,side)
                fk=session.eef_fk(side,q);target=getattr(frame,side+'_eef')
                np.testing.assert_allclose(fk['position'],r.T@(target.position-p),atol=1e-6)
                np.testing.assert_allclose(fk['rotation'],r.T@target.rotation,atol=1e-6)


def test_unlimited_binding_preserves_multi_turn_seed_on_reset():
    import geo_kin
    session=geo_kin.RetargetSession(robot='urgantry',hand='sharpa',arm_ik_limited=False)
    q=np.array(session.default_q['left']);q[0]+=8*np.pi
    session.reset(q_init_left=q)
    target=session.eef_fk('left',q)
    solved=session.eef_ik('left',target['position'],target['rotation'])
    np.testing.assert_allclose(solved,q,atol=1e-8)

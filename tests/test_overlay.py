from types import SimpleNamespace
import numpy as np
import mujoco
import geo_kin
from xrt_devices.integrations.geo_kin import open_motion_source
from dex_ur_gantry_teleop import model_path
from dex_ur_gantry_teleop.input import from_recording
from dex_ur_gantry_teleop.visualization import ReplayOverlay


def test_recorded_overlay_alignment_and_reset():
    for sample in ('ipman_roll', 'picking_up_mustard'):
        source = open_motion_source(frames=str(model_path().parent/'sample_motion'/f'{sample}.npz'))
        for mode in ('first_frame', 'shoulder_attached'):
            session = geo_kin.RetargetSession(robot='urgantry', hand='sharpa', base_alignment_mode=mode)
            viewer = SimpleNamespace(user_scn=mujoco.MjvScene(mujoco.MjModel.from_xml_string("<mujoco/>"), maxgeom=2000))
            overlay = ReplayOverlay(viewer, alignment=mode)
            joints = {side:session.default_q[side] for side in ('left', 'right')}
            for t in (0., 0.1):
                recorded = source.frame_at_time(t)
                frame = from_recording(recorded, session)
                assert overlay.draw(recorded, frame, session, joints) > 20
                out = session.solve(frame)
                for side in ('left', 'right'):
                    q = getattr(out, 'q_goal_'+side)
                    if q is not None:
                        expected = overlay.robot_origin + overlay.robot_rotation @ session.eef_fk(side, q)['position']
                        actual = overlay.human._to_view(getattr(frame, side+'_eef').position)
                        np.testing.assert_allclose(actual, expected, atol=1e-6)
                        joints[side] = q
            assert overlay.draw(None, None, session, joints) == 0
            assert viewer.user_scn.ngeom == 0
            overlay.reset()
            assert overlay.anchor is None and overlay.offset is None

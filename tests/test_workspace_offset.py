"""Manual translation must move FK and overlay together and survive resets."""
from types import SimpleNamespace
import numpy as np
import pytest
import geo_kin
from dex_ur_gantry_teleop import model_path
from dex_ur_gantry_teleop.input import from_recording
from dex_ur_gantry_teleop.visualization import ReplayOverlay
from xrt_devices.integrations.geo_kin import open_motion_source


@pytest.mark.parametrize('hand', ['sharpa', 'wuji'])
@pytest.mark.parametrize('alignment', ['first_frame', 'shoulder_attached'])
@pytest.mark.parametrize('automatic', [False, True])
def test_translation_fk_overlay_and_reset(hand, alignment, automatic):
    delta = np.array([.02, -.01, .03])
    common = dict(robot='urgantry', hand=hand, functional_offset=automatic,
                  arm_ik_limited=False, base_alignment_mode=alignment)
    base = geo_kin.RetargetSession(**common)
    shifted = geo_kin.RetargetSession(**common, workspace_offset=delta.tolist())
    source = open_motion_source(frames=str(model_path().parent/'sample_motion/ipman_roll.npz'))
    class Human:
        def set_base_offset(self, p, r): self.p, self.r = p, r
        def draw(self, recorded): return 0
    overlay = ReplayOverlay.__new__(ReplayOverlay)
    overlay.viewer = SimpleNamespace(user_scn=SimpleNamespace(ngeom=0))
    overlay.human = Human()
    overlay.alignment = alignment
    overlay.functional_offset = automatic
    overlay.workspace_offset = delta
    overlay.robot_rotation = np.array([[0.,-1,0],[1,0,0],[0,0,1]])
    overlay.robot_origin = np.array([0.,-.3825,1.37974159])
    for _ in range(2):
        base.reset(); shifted.reset(); overlay.reset()
        for t in [0., .1, .2]:
            recorded = source.frame_at_time(t)
            frame = from_recording(recorded, base)
            a = base.solve(frame); b = shifted.solve(frame)
            for side in ('left', 'right'):
                qa = getattr(a, 'q_goal_'+side); qb = getattr(b, 'q_goal_'+side)
                assert qa is not None and qb is not None
                fa = base.eef_fk(side, qa); fb = shifted.eef_fk(side, qb)
                np.testing.assert_allclose(fb['position']-fa['position'], delta, atol=1e-6)
                np.testing.assert_allclose(fa['rotation'], fb['rotation'], atol=1e-6)
            # Reuse FK-verified calibration to check the visual frame composition.
            reference_p, reference_r = (frame.p_world_upper_body, frame.R_world_upper_body)
            if overlay.anchor is None: overlay.anchor = (reference_p, reference_r)
            if alignment == 'first_frame': reference_p, reference_r = overlay.anchor
            overlay.offset = fa['position'] - reference_r.T @ (frame.right_eef.position-reference_p)
            overlay.draw(recorded, frame, shifted, {})
            human_target = overlay.human.r @ (frame.right_eef.position-overlay.human.p)
            robot_target = overlay.robot_origin + overlay.robot_rotation @ fb['position']
            np.testing.assert_allclose(human_target, robot_target, atol=1e-6)


@pytest.mark.parametrize('offset', [(1,2), (1,2,3,4), (float('nan'),0,0), (0,float('inf'),0)])
def test_invalid_offset(offset):
    with pytest.raises((ValueError, TypeError)):
        geo_kin.RetargetSession(robot='urgantry', hand='wuji', workspace_offset=offset)

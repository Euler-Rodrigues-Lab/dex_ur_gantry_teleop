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


def test_export_preserves_live_capsule_endpoints_and_material():
    from geo_kin_core.viz import capsules
    from dex_ur_gantry_teleop.visualization import append_overlay_scene
    model=mujoco.MjModel.from_xml_string('<mujoco/>')
    live=SimpleNamespace(user_scn=mujoco.MjvScene(model,maxgeom=10))
    exported=mujoco.MjvScene(model,maxgeom=10)
    # Keep an existing robot geom ahead of the appended human overlay.
    exported.ngeom=1
    exported.geoms[0].pos[:]=[9,8,7]
    p1=np.array([0.2,0.1,1.3]);p2=np.array([-0.1,0.4,0.8])
    capsules.add_capsule(live,p1,p2,0.035,(0.15,0.55,1.,0.55))
    capsules.add_sphere(live,[0,0,1.6],0.09)
    live.user_scn.geoms[0].reflectance=0.25
    append_overlay_scene(exported,live.user_scn)
    assert exported.ngeom==3
    np.testing.assert_array_equal(exported.geoms[0].pos,[9,8,7])
    capsule=exported.geoms[1]
    axis=capsule.mat[:,2]
    np.testing.assert_allclose(capsule.pos-axis*capsule.size[2],p1,atol=1e-7)
    np.testing.assert_allclose(capsule.pos+axis*capsule.size[2],p2,atol=1e-7)
    for src,dst in zip(live.user_scn.geoms[:2],exported.geoms[1:3]):
        for name in ('type','size','pos','mat','rgba','reflectance','transparent'):
            np.testing.assert_array_equal(getattr(src,name),getattr(dst,name))

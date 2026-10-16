"""Licensed, kinematic EEF + Sharpa teleoperation and offline replay."""
import argparse
from contextlib import ExitStack, nullcontext
import time
import numpy as np
import mujoco
from geo_kin_core.session import resolve_session
from geo_kin_core.spec import sha256_of_file
from geo_kin_core.types import EEFFrame
from urgantry_sim.build_urgantry import ARM_JOINTS
from . import model_path, load_model
from .input import from_action, from_recording
from .viz import apply_position_targets


def _run():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--hand',choices=['sharpa','wuji'],default='sharpa')
    p.add_argument('--device',choices=['replay','xr','mediapipe'],default='replay')
    p.add_argument('--frames',help='Existing geo_kin_core frame-stream NPZ')
    p.add_argument('--sample',choices=['ipman_roll','picking_up_mustard'],default='ipman_roll')
    p.add_argument('--headless',action='store_true')
    p.add_argument('--no-human-overlay',action='store_true',help='Hide the recorded human skeleton')
    p.add_argument('--steps',type=int,default=0,help='Stop after this many total updates, including loops')
    p.add_argument('--loop',action='store_true',help='Repeat replay; reset robot and calibration each pass')
    p.add_argument('--rate',type=float,default=60)
    p.add_argument('--alignment',choices=['first_frame','shoulder_attached'],default='first_frame')
    offset=p.add_mutually_exclusive_group()
    offset.add_argument('--offset',dest='no_offset',action='store_false',help='Enable fixed initial position offset')
    offset.add_argument('--no-offset',dest='no_offset',action='store_true',help='Use shoulder-frame alignment without translation (default)')
    p.set_defaults(no_offset=True)
    p.add_argument('--arm-joint-limits',action='store_true',help='Enforce model arm limits; default uses continuous unrestricted angles')
    p.add_argument('--host',default='0.0.0.0');p.add_argument('--port',type=int,default=8080)
    p.add_argument('--camera',type=int,default=0)
    args=p.parse_args()
    if not np.isfinite(args.rate) or args.rate<=0 or args.steps<0:p.error('rate must be positive and steps nonnegative')
    if args.loop and args.device!='replay':p.error('--loop is only available for replay')
    if args.device=='replay' and not args.frames:
        args.frames=str(model_path().parent/'sample_motion'/f'{args.sample}.npz')
    session=resolve_session('urgantry',args.hand,backend='licensed',collision_avoidance=False,
                            base_alignment_mode=args.alignment,functional_offset=not args.no_offset,arm_ik_limited=args.arm_joint_limits)
    if sha256_of_file(model_path(args.hand)) != session.model_sha256:
        raise RuntimeError('Simulator XML differs from the geometry embedded in this wheel')
    model=load_model(args.hand);data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    arms={side:[model.actuator(f'{side}_{name}').id for name in ARM_JOINTS] for side in ['left','right']}
    hands={side:[model.actuator(name).id for name in session.hand_joint_names(side)] for side in arms}
    session.reset(data.ctrl[arms['right']],data.ctrl[arms['left']])
    with ExitStack() as stack:
        if args.device=='replay':
            from xrt_devices.integrations.geo_kin import open_motion_source
            source=open_motion_source(frames=args.frames)
        elif args.device=='xr':
            from xrt_devices.integrations.geo_kin import XRDeviceAdapter
            from xrt_devices.integrations.eef import bones_to_eef_action
            source=XRDeviceAdapter(host=args.host,port=args.port,process_bones_to_action_fn=bones_to_eef_action)
            stack.callback(source.cleanup)
        else:
            from xrt_devices import MediaPipeTeleopDevice
            source=MediaPipeTeleopDevice(camera_id=args.camera,eef_only=True)
            stack.callback(source.close)
        viewer=None
        recenter = [False]
        def key_callback(key):
            if key in (ord('R'),ord('r')):recenter[0]=True
        if not args.headless:
            from .viewer import managed_viewer, configure_camera
            viewer=stack.enter_context(managed_viewer(model,data,show_left_ui=False,show_right_ui=False,key_callback=key_callback))
            with viewer.lock():
                configure_camera(viewer.cam)
        overlay=None
        if viewer is not None and args.device=='replay' and not args.no_human_overlay:
            from .visualization import ReplayOverlay
            overlay=ReplayOverlay(viewer,alignment=args.alignment,functional_offset=not args.no_offset)
        replay_tick=0
        count=0;accepted={side:0 for side in arms};held={side:0 for side in arms}
        while (not args.steps or count<args.steps) and (viewer is None or viewer.is_running()):
            tick=time.monotonic()
            if recenter[0]:
                session.reset(data.ctrl[arms['right']],data.ctrl[arms['left']])
                recenter[0]=False
                if overlay:overlay.reset()
            if args.device=='replay':
                if replay_tick/args.rate>source.duration:
                    if not args.loop:break
                    replay_tick=0
                    with viewer.lock() if viewer else nullcontext():
                        mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
                        session.reset(data.ctrl[arms['right']],data.ctrl[arms['left']])
                        if overlay:overlay.reset()
                recorded=source.frame_at_time(replay_tick/args.rate)
                replay_tick+=1
                frame=from_recording(recorded,session)
            elif args.device=='xr':frame=from_action(source.get_raw_action(),session)
            else:
                snapshot=source.get_frame();frame=from_action(None if snapshot is None else snapshot.action,session)
            # Tracking loss holds goals and calibration. Disengage/recenter is
            # explicit through reset, rather than recalibrating on each dropout.
            if overlay:
                with viewer.lock():
                    overlay.draw(recorded,frame,session,{side:data.ctrl[arms[side]].copy() for side in arms})
            out=session.solve(frame or EEFFrame(),q_current_right=data.ctrl[arms['right']],q_current_left=data.ctrl[arms['left']])
            for side in arms:
                q=getattr(out,'q_goal_'+side)
                if q is not None:data.ctrl[arms[side]]=q;accepted[side]+=1
                else:held[side]+=1
                q=getattr(out,'q_goal_'+side+'_hand')
                if q is not None:data.ctrl[hands[side]]=q
            apply_position_targets(model,data,unlimited_actuators=None if args.arm_joint_limits else arms['left']+arms['right'])
            if viewer:viewer.sync()
            count+=1
            if not args.headless:time.sleep(max(0,1/args.rate-(time.monotonic()-tick)))
    print(f'{count} frames; accepted arms={accepted}; held arms={held}; kinematics only')


def main():
    try:
        _run()
    except KeyboardInterrupt:
        # ExitStack closes the viewer/devices before returning to the shell.
        pass


if __name__=='__main__':main()

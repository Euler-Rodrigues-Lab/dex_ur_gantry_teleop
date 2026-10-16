"""Render both full replays as MP4 and looping GIF previews (requires ffmpeg)."""
import os
os.environ.setdefault('MUJOCO_GL', 'egl')
import argparse
from pathlib import Path
import subprocess
from types import SimpleNamespace
import mujoco
import geo_kin
from geo_kin_core.spec import sha256_of_file
from xrt_devices.integrations.geo_kin import open_motion_source
from urgantry_sim.build_urgantry import ARM_JOINTS
from dex_ur_gantry_teleop import model_path, load_model
from dex_ur_gantry_teleop.input import from_recording
from dex_ur_gantry_teleop.viewer import configure_camera
from dex_ur_gantry_teleop.visualization import ReplayOverlay, append_overlay_scene
from dex_ur_gantry_teleop.viz import apply_position_targets


def render(sample, destination, hand="sharpa"):
    session=geo_kin.RetargetSession(robot='urgantry',hand=hand,
        collision_avoidance=False,functional_offset=False,arm_ik_limited=False)
    if sha256_of_file(model_path(hand)) != session.model_sha256:
        raise RuntimeError('Wheel/model signature mismatch')
    model=load_model(hand)
    data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('home').id)
    arms={side:[model.actuator(side+'_'+name).id for name in ARM_JOINTS] for side in ('left','right')}
    hands={side:[model.actuator(name).id for name in session.hand_joint_names(side)] for side in arms}
    session.reset(data.ctrl[arms['right']],data.ctrl[arms['left']])
    source=open_motion_source(frames=str(model_path().parent/'sample_motion'/f'{sample}.npz'))
    viewer=SimpleNamespace(user_scn=mujoco.MjvScene(model,maxgeom=2000))
    overlay=ReplayOverlay(viewer,functional_offset=False)
    camera=mujoco.MjvCamera();configure_camera(camera)
    basename=sample if hand=='sharpa' else f'{sample}_{hand}'
    mp4=destination/f'{basename}.mp4'
    command=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo',
             '-pixel_format','rgb24','-video_size','640x480','-framerate','30',
             '-i','pipe:0','-an','-c:v','libx264','-preset','slow','-crf','20',
             '-pix_fmt','yuv420p','-movflags','+faststart',str(mp4)]
    held={side:0 for side in arms}
    with subprocess.Popen(command,stdin=subprocess.PIPE) as encoder:
        try:
            with mujoco.Renderer(model,height=480,width=640) as renderer:
                for tick in range(int(source.duration*60)+1):
                    recorded=source.frame_at_time(tick/60)
                    frame=from_recording(recorded,session)
                    if tick%2==0:
                        overlay.draw(recorded,frame,session,{side:data.ctrl[arms[side]].copy() for side in arms})
                    output=session.solve(frame,q_current_left=data.ctrl[arms['left']],q_current_right=data.ctrl[arms['right']])
                    for side in arms:
                        q=getattr(output,'q_goal_'+side)
                        if q is None:held[side]+=1
                        else:data.ctrl[arms[side]]=q
                        q=getattr(output,'q_goal_'+side+'_hand')
                        if q is not None:data.ctrl[hands[side]]=q
                    apply_position_targets(model,data,unlimited_actuators=arms['left']+arms['right'])
                    if tick%2:continue
                    renderer.update_scene(data,camera=camera)
                    append_overlay_scene(renderer.scene,viewer.user_scn)
                    encoder.stdin.write(renderer.render().tobytes())
                    if tick%120==0:print(f'{sample}: {tick/60:.0f}/{source.duration:.1f} seconds',flush=True)
        finally:
            encoder.stdin.close()
        if encoder.wait()!=0:raise RuntimeError('Video encoding failed')
    subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-i',str(mp4),
        '-filter_complex','fps=15,scale=480:-1:flags=lanczos,split[a][b];[a]palettegen=stats_mode=diff[p];[b][p]paletteuse=dither=bayer:bayer_scale=3',
        '-loop','0',str(destination/f'{basename}.gif')],check=True)
    print(f'{sample}: complete; held arm updates={held}',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--hand',choices=['sharpa','wuji'],default='sharpa')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'docs/media')
    parser.add_argument('--sample',choices=['ipman_roll','picking_up_mustard'])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    for sample in [args.sample] if args.sample else ['picking_up_mustard','ipman_roll']:
        render(sample,args.output,args.hand)


if __name__=='__main__':main()

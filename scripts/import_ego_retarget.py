"""Convert ego_retarget 21-joint hands to the existing XRT frame-stream format."""
import argparse
import hashlib
from pathlib import Path
import numpy as np
from geo_kin_core.frames import save_frames
from geo_kin_core.types import RetargetFrame, SEWPose
from xrt_devices.integrations.eef import complete_hand_landmarks


def convert(source, destination, *, table_z, fps=60., body_origin=None, heading="first"):
    if not np.isfinite(fps) or fps <= 0 or not np.isfinite(table_z):
        raise ValueError('fps must be positive and table_z finite')
    with np.load(source, allow_pickle=False) as d:
        joints = np.asarray(d['skeleton_world'], float)
        times = np.asarray(d['timestamps'], float)
        valid = np.asarray(d['valid'], bool)
        cameras = np.asarray(d['head_pose_world'], float)
        camera = cameras[0]
    if (joints.shape != (len(times), 2, 21, 3) or valid.shape != (len(times), 2)
            or len(times) < 2 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)):
        raise ValueError('Expected increasing timestamps and (T,2,21,3) skeleton_world')
    if heading not in ('first', 'mean'):
        raise ValueError('heading must be first or mean')
    if cameras.shape != (len(times), 4, 4) or not np.isfinite(camera).all():
        raise ValueError('Expected camera-to-world poses and a finite initial camera')
    directions = cameras[:, :2, 2]
    lengths = np.linalg.norm(directions, axis=1)
    usable = np.isfinite(directions).all(axis=1) & (lengths > 1e-6)
    if heading == 'first':
        if not usable[0]:
            raise ValueError('Initial camera must have a valid horizontal heading')
        horizontal = directions[0] / lengths[0]
    else:
        # Duration-weighted circular mean, not an arithmetic mean of yaw angles.
        # Normalizing each projection prevents head pitch from weighting heading.
        weights = np.empty(len(times))
        weights[0] = (times[1]-times[0])/2
        weights[-1] = (times[-1]-times[-2])/2
        weights[1:-1] = (times[2:]-times[:-2])/2
        if not usable.any():
            raise ValueError('No valid horizontal camera headings')
        horizontal = np.average(directions[usable]/lengths[usable,None], axis=0, weights=weights[usable])
        if np.linalg.norm(horizontal) < 1e-3:
            raise ValueError('Mean heading is ambiguous: opposing headings cancel')
    forward = np.r_[horizontal / np.linalg.norm(horizontal), 0.]
    rotation = np.column_stack((forward, np.cross([0., 0., 1.], forward), [0., 0., 1.]))
    # Virtual shoulder frame: fixed initial camera XY and the gantry's exact
    # shoulder-to-table height. No shoulder tracking is claimed or invented.
    origin = np.array([camera[0, 3], camera[1, 3], table_z + 1.37974159 - .775])
    if body_origin is not None: origin = np.asarray(body_origin, float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError('body_origin must be three finite world coordinates')
    names=[]; parents=[]
    for side in ('Left', 'Right'):
        start=len(names); names.append(f'FullBody_{side}HandWrist'); parents.append(-1)
        for finger in ('Thumb','Index','Middle','Ring','Little'):
            suffixes=('Metacarpal','Proximal','Distal','Tip') if finger=='Thumb' else ('Proximal','Intermediate','Distal','Tip')
            for j,suffix in enumerate(suffixes):
                parents.append(start if j==0 else len(names)-1)
                names.append(f'FullBody_{side}Hand{finger}{suffix}')
    times=times-times[0]
    ticks=np.arange(int(np.floor(times[-1]*fps))+1)/fps
    # Causal sample-and-hold retains irregular source timing and never blends
    # valid tracking through a dropout. Temporal quantization is <= 1/fps.
    indices=np.searchsorted(times,ticks,side='right')-1
    frames=[]
    for i in indices:
        positions=joints[i].copy()
        frame=RetargetFrame(p_world_upper_body=origin.copy(),R_world_upper_body=rotation.copy())
        for h,side in enumerate(('left','right')):
            if not valid[i,h] or not np.isfinite(positions[h]).all():
                positions[h]=np.nan
                continue
            wrist=positions[h,0]
            # Identity world wrist frame: palm rotation is reconstructed from
            # MCP landmarks by geo_kin, independent of MANO root conventions.
            setattr(frame,side+'_sew',SEWPose(S=np.zeros(3),E=np.zeros(3),W=rotation.T@(wrist-origin),R_world_wrist=rotation.T))
            mapping=dict(zip(names[h*21:(h+1)*21],positions[h]))
            setattr(frame,side+'_fingers',complete_hand_landmarks(mapping,side,wrist,np.eye(3)))
        frame.skeleton={'names':names,'parents':parents,'positions':positions.reshape(42,3)}
        frames.append(frame)
    notes=(f'ego_retarget conversion; source sha256={hashlib.sha256(Path(source).read_bytes()).hexdigest()}; '
           f'fixed virtual shoulder origin={origin.tolist()}, heading_mode={heading}, heading={forward.tolist()}, source table_z={table_z}; '
           'no measured shoulders/elbows; all 21 hand joints retained in skeleton; causal sample-and-hold; CC BY-SA 4.0 for OakInk2/EgoVerse data')
    save_frames(destination,frames,fps=fps,source=str(source),notes=notes)
    print(f'{destination}: {len(frames)} frames, {ticks[-1]:.3f}s; body origin {origin.tolist()}')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('destination',type=Path)
    p.add_argument('--table-z',type=float,required=True,help='Source-world tabletop height in metres; OakInk2 unscrew is 0')
    p.add_argument('--heading',choices=['first','mean'],default='first',help='Fixed heading from initial camera or duration-weighted mean camera facing direction')
    p.add_argument('--fps',type=float,default=60.)
    p.add_argument('--body-origin',nargs=3,type=float,help='Override fixed virtual shoulder origin in source-world metres')
    a=p.parse_args();convert(a.source,a.destination,table_z=a.table_z,fps=a.fps,body_origin=a.body_origin,heading=a.heading)

if __name__=='__main__': main()

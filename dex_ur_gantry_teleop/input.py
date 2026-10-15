"""Explicit EEF frames from shared XR/MediaPipe actions or legacy recordings."""
import numpy as np
from geo_kin_core.types import EEFFrame, EEFPose


def from_action(action, session):
    if not action or action.get('engaged') is False:
        return None
    frame = EEFFrame(p_world_upper_body=action.get('p_world_upper_body'),
                     R_world_upper_body=action.get('R_world_upper_body'))
    if frame.p_world_upper_body is None or frame.R_world_upper_body is None:
        return None
    for side in ('left','right'):
        hand = action.get(side+'_hand_pose')
        fingers = action.get(side+'_fingers')
        if hand is None or not fingers:
            continue
        index = fingers.get('index',{}).get('index_finger_mcp')
        if index is None:
            continue
        r_palm = session.human_palm_rotation(fingers)
        if r_palm is None:
            continue
        wrist_p, wrist_r = np.asarray(hand['position']), np.asarray(hand['rotation'])
        if not np.isfinite(wrist_p).all() or not np.isfinite(wrist_r).all():
            continue
        setattr(frame,side+'_eef',EEFPose(wrist_p+wrist_r@np.asarray(index),wrist_r@r_palm))
        setattr(frame,side+'_fingers_wrist',fingers)
    return frame


def from_recording(frame, session):
    if frame is None:
        return None
    action={'p_world_upper_body':frame.p_world_upper_body,
            'R_world_upper_body':frame.R_world_upper_body}
    if frame.p_world_upper_body is None or frame.R_world_upper_body is None:
        return None
    for side in ('left','right'):
        sew=getattr(frame,side+'_sew',None)
        if sew is None or sew.R_world_wrist is None:
            continue
        # Historical recordings store these values in body coordinates,
        # despite the legacy R_world_wrist field name. E is never consulted.
        action[side+'_hand_pose']={'position':frame.p_world_upper_body+frame.R_world_upper_body@sew.W,
            'rotation':frame.R_world_upper_body@sew.R_world_wrist}
        fingers=getattr(frame,side+'_fingers',None)
        if frame.skeleton:
            from xrt_devices.integrations.eef import complete_hand_landmarks
            positions=dict(zip(frame.skeleton['names'],frame.skeleton['positions']))
            hand=action[side+'_hand_pose']
            fingers=complete_hand_landmarks(positions,side,hand['position'],hand['rotation'],fingers)
        action[side+'_fingers']=fingers
    return from_action(action,session)

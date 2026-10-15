"""Recorded human skeleton in the same calibrated frame as the EEF goals."""
import numpy as np
from geo_kin_core.viz import HumanCapsuleViz, capsules


class ReplayOverlay:
    def __init__(self, viewer, *, alignment='first_frame', functional_offset=True):
        self.viewer = viewer
        self.alignment = alignment
        self.functional_offset = functional_offset
        self.human = HumanCapsuleViz(viewer, color=(0.15, 0.55, 1.0, 0.55), size=0.035)
        self.robot_rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
        self.robot_origin = np.array([0., -0.3825, 1.37974159])
        self.reset()

    def reset(self):
        self.anchor = None
        self.offset = None

    def draw(self, recorded, frame, session, joints):
        capsules.clear(self.viewer)
        if frame is None or frame.p_world_upper_body is None or frame.R_world_upper_body is None:
            return 0
        p = np.asarray(frame.p_world_upper_body)
        r = np.asarray(frame.R_world_upper_body)
        if not np.isfinite(p).all() or not np.isfinite(r).all():
            return 0
        if self.anchor is None:
            self.anchor = (p.copy(), r.copy())
        reference_p, reference_r = (p, r) if self.alignment == 'shoulder_attached' else self.anchor
        if self.offset is None:
            targets = [frame.left_eef, frame.right_eef]
            if self.functional_offset:
                if any(t is None for t in targets):
                    self.anchor = None
                    return 0
                local = [reference_r.T @ (t.position-reference_p) for t in targets]
                current = [session.eef_fk(side, joints[side])['position'] for side in ('left', 'right')]
                self.offset = (sum(current)-sum(local))*0.5
            else:
                self.offset = np.zeros(3)
        # HumanCapsuleViz applies rotation @ (world_point - position).
        rotation = self.robot_rotation @ reference_r.T
        translation = self.robot_origin + self.robot_rotation @ self.offset
        self.human.set_base_offset(reference_p - rotation.T @ translation, rotation)
        return self.human.draw(recorded)

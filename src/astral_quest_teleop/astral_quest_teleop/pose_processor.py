"""VR wrist pose preprocessing (Nero-compatible).

VR delta → optional pitch flip → ``vr_to_arm_rot`` → EMA/SLERP.

For Astral the mapped frame is that arm's ``*_base_link``.
``robot_world_to_base_rot`` is kept as an alias of ``vr_to_arm_rot``.
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class PoseProcessor:
    """Incremental VR wrist → delta pose in the robot/IK frame."""

    def __init__(
        self,
        vr_to_arm_rot: np.ndarray | None = None,
        robot_world_to_base_rot: np.ndarray | None = None,
        pos_smoothing: float = 0.8,
        rot_smoothing: float = 0.8,
        motion_scale: float = 0.65,
        flip_pitch: bool = False,
    ):
        if vr_to_arm_rot is not None:
            R = np.asarray(vr_to_arm_rot, dtype=float).reshape(3, 3)
        elif robot_world_to_base_rot is not None:
            R = np.asarray(robot_world_to_base_rot, dtype=float).reshape(3, 3)
        else:
            R = np.eye(3, dtype=float)
        self.R_vr_to_arm = R
        self.R_world_to_base = R  # alias
        self.pos_smoothing = float(pos_smoothing)
        self.rot_smoothing = float(rot_smoothing)
        self.motion_scale = float(motion_scale)
        self.flip_pitch = bool(flip_pitch)

        self.vr_init_pos = None
        self.vr_init_rot = None
        self.vr_current_pos = None
        self.vr_current_rot = None
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()

    def set_vr_zero_point(self, pos: np.ndarray, rot: Rotation) -> None:
        self.vr_init_pos = pos.copy()
        self.vr_init_rot = rot
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()

    @property
    def is_calibrated(self) -> bool:
        return self.vr_init_pos is not None

    def reset(self) -> None:
        self.vr_init_pos = None
        self.vr_init_rot = None
        self.vr_current_pos = None
        self.vr_current_rot = None
        self.smoothed_delta_pos = np.zeros(3)
        self.smoothed_delta_rot = Rotation.identity()

    def update_vr_pose(self, pos: np.ndarray, rot: Rotation) -> None:
        if self.vr_init_pos is None:
            self.set_vr_zero_point(pos, rot)
        self.vr_current_pos = pos
        self.vr_current_rot = rot

    def process(self):
        if self.vr_current_pos is None or self.vr_init_pos is None:
            return np.zeros(3), Rotation.identity()

        raw_delta_pos_vr = (self.vr_current_pos - self.vr_init_pos) * self.motion_scale
        raw_delta_rot_vr = self.vr_current_rot * self.vr_init_rot.inv()

        if self.flip_pitch:
            euler = raw_delta_rot_vr.as_euler("xyz", degrees=False)
            euler[0] = -euler[0]
            raw_delta_rot_vr = Rotation.from_euler("xyz", euler)

        R = self.R_vr_to_arm
        delta_pos_arm = R @ raw_delta_pos_vr
        delta_rot_arm = Rotation.from_matrix(
            R @ raw_delta_rot_vr.as_matrix() @ R.T
        )

        a_p = self.pos_smoothing
        self.smoothed_delta_pos = (
            a_p * self.smoothed_delta_pos + (1.0 - a_p) * delta_pos_arm
        )
        a_r = self.rot_smoothing
        try:
            slerp = Slerp(
                [0, 1],
                Rotation.concatenate([self.smoothed_delta_rot, delta_rot_arm]),
            )
            self.smoothed_delta_rot = slerp(1.0 - a_r)
        except Exception:  # noqa: BLE001
            self.smoothed_delta_rot = delta_rot_arm

        return self.smoothed_delta_pos.copy(), self.smoothed_delta_rot

    def compute_target_pose(
        self,
        delta_pos: np.ndarray,
        delta_rot: Rotation,
        robot_init_pos: np.ndarray,
        robot_init_rot: np.ndarray,
    ) -> np.ndarray:
        """4x4 EE target (same composition as Nero)."""
        T = np.eye(4)
        T[:3, 3] = robot_init_pos + delta_pos
        T[:3, :3] = delta_rot.as_matrix() @ robot_init_rot
        return T

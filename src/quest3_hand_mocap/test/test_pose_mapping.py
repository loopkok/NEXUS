import unittest
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest3_hand_mocap.pose_mapping import (
    map_wrist_pose, quaternion_to_matrix, rotate_pose,
)


LEFT = np.array([[0, 1, 0], [0, 0, 1], [-1, 0, 0]], dtype=float)
RIGHT = np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)
UNITY_TO_ROBOT = np.array([[-1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)


class Quest3SidePoseMappingTests(unittest.TestCase):
    def test_side_maps_rotate_translation_axes_once(self):
        x = np.array([1.0, 0.0, 0.0])
        left, _ = rotate_pose(x, np.array([0, 0, 0, 1]), LEFT)
        right, _ = rotate_pose(x, np.array([0, 0, 0, 1]), RIGHT)
        np.testing.assert_allclose(left, [0, 0, -1])
        np.testing.assert_allclose(right, [0, 0, 1])

    def test_both_profile_maps_send_synthetic_forward_axis_to_arm_x(self):
        raw_forward = np.array([0.0, 1.0, 0.0])
        for side_map in (LEFT, RIGHT):
            mapped, _ = rotate_pose(
                raw_forward, np.array([0.0, 0.0, 0.0, 1.0]), side_map
            )
            np.testing.assert_allclose(mapped, [1.0, 0.0, 0.0])

    def test_pose_orientation_is_conjugated_in_same_basis(self):
        pose_q = np.array([0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8)])
        _, mapped = rotate_pose(np.zeros(3), pose_q, LEFT)
        expected = LEFT @ quaternion_to_matrix(pose_q) @ LEFT.T
        actual = quaternion_to_matrix(mapped)
        np.testing.assert_allclose(actual, expected, atol=1e-8)

    def test_per_side_mapping_overrides_generic_conversion(self):
        raw = np.array([0.2, -0.3, 0.4])
        quat = np.array([0.0, 0.0, 0.0, 1.0])
        mapped, _ = map_wrist_pose(
            raw, quat, mode="per_side", side_rotation=LEFT,
            global_rotation=UNITY_TO_ROBOT,
        )
        np.testing.assert_allclose(mapped, LEFT @ raw)
        self.assertFalse(np.allclose(mapped, UNITY_TO_ROBOT @ raw))

    def test_global_mapping_keeps_astral_conversion_path(self):
        raw = np.array([0.2, -0.3, 0.4])
        mapped, _ = map_wrist_pose(
            raw, np.array([0.0, 0.0, 0.0, 1.0]), mode="global",
            side_rotation=LEFT, global_rotation=UNITY_TO_ROBOT,
        )
        np.testing.assert_allclose(mapped, UNITY_TO_ROBOT @ raw)

    def test_rejects_nonorthogonal_and_nonfinite_basis(self):
        with self.assertRaises(ValueError):
            rotate_pose(np.zeros(3), np.array([0, 0, 0, 1]), np.diag([2, 1, 1]))
        bad = np.eye(3)
        bad[0, 0] = np.nan
        with self.assertRaises(ValueError):
            rotate_pose(np.zeros(3), np.array([0, 0, 0, 1]), bad)


if __name__ == "__main__":
    unittest.main()

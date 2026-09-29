"""Pure pose-frame helpers shared by Quest3 input adapters and tests."""

from __future__ import annotations

import numpy as np


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    quat = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
    norm = float(np.linalg.norm(quat))
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError("quaternion must have a finite nonzero norm")
    x, y, z, w = quat / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=float).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        scale = np.sqrt(trace + 1.0) * 2.0
        quat = [(m[2, 1] - m[1, 2]) / scale,
                (m[0, 2] - m[2, 0]) / scale,
                (m[1, 0] - m[0, 1]) / scale, 0.25 * scale]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        quat = [0.25 * scale, (m[0, 1] + m[1, 0]) / scale,
                (m[0, 2] + m[2, 0]) / scale, (m[2, 1] - m[1, 2]) / scale]
    elif m[1, 1] > m[2, 2]:
        scale = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        quat = [(m[0, 1] + m[1, 0]) / scale, 0.25 * scale,
                (m[1, 2] + m[2, 1]) / scale, (m[0, 2] - m[2, 0]) / scale]
    else:
        scale = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        quat = [(m[0, 2] + m[2, 0]) / scale,
                (m[1, 2] + m[2, 1]) / scale, 0.25 * scale,
                (m[1, 0] - m[0, 1]) / scale]
    quat = np.asarray(quat, dtype=float)
    return quat / np.linalg.norm(quat)


def rotate_pose(
    position: np.ndarray, quaternion_xyzw: np.ndarray, rotation: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Express a pose in a rotated basis, preserving the pose's origin.

    ``rotation`` maps input-frame vector components to output-frame components;
    it may change handedness when converting between coordinate conventions.
    Positions transform as ``R @ p`` and orientations by conjugation:
    ``R_out = R @ R_pose @ R.T``.
    """
    basis = np.asarray(rotation, dtype=float).reshape(3, 3)
    pos = np.asarray(position, dtype=float).reshape(3)
    quat = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
    if not np.isfinite(basis).all() or not np.isfinite(pos).all() or not np.isfinite(quat).all():
        raise ValueError("pose and basis must contain only finite values")
    if not np.isclose(abs(np.linalg.det(basis)), 1.0, atol=1e-3) or not np.allclose(
        basis @ basis.T, np.eye(3), atol=1e-3
    ):
        raise ValueError("basis must be orthonormal")
    pose_rotation = quaternion_to_matrix(quat)
    mapped_rotation = basis @ pose_rotation @ basis.T
    return basis @ pos, matrix_to_quaternion(mapped_rotation)


def map_wrist_pose(
    position: np.ndarray,
    quaternion_xyzw: np.ndarray,
    *,
    mode: str,
    side_rotation: np.ndarray,
    global_rotation: np.ndarray | None,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply either the per-side arm mapping or the generic Quest mapping.

    The side-specific route is an override: it never composes with
    ``global_rotation``. This avoids applying Astral's generic conversion and
    XNero's left/right conversion to the same wrist pose.
    """
    if mode == "per_side":
        return rotate_pose(position, quaternion_xyzw, side_rotation)
    if mode != "global":
        raise ValueError("wrist pose mapping mode must be 'global' or 'per_side'")
    if global_rotation is None:
        return np.asarray(position, dtype=float).reshape(3), np.asarray(
            quaternion_xyzw, dtype=float
        ).reshape(4)
    return rotate_pose(position, quaternion_xyzw, global_rotation)

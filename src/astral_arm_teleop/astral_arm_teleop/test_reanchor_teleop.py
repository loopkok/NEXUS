#!/usr/bin/env python3
"""Regression tests for the HITL ``~/reanchor`` service on AstralTeleopArmNode.

The service must re-derive robot_init_pos/rot from the *current* measured joint
state (fallback: last commanded q) and re-capture vr_init from the current VR
pose, then arm — all without a jump. Tests instantiate the node class via
``__new__`` (no rclpy node spin) and stub ``pose``/``ik``/``safety``/logger.

Runs only where rclpy is importable (ROS-sourced system python); otherwise the
whole suite skips with exit 0. Style matches the other astral_arm_teleop
script-style tests.
"""

import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

from astral_arm_teleop.astral_arm_teleop_node import AstralTeleopArmNode  # noqa: E402


class _FakePose:
    def __init__(self):
        self.vr_current_pos = np.zeros(3)
        self.vr_current_rot = Rotation.identity()
        self.calibrated = 0

    def calibrate_from_current(self):
        self.calibrated += 1
        return self.vr_current_pos is not None and self.vr_current_rot is not None


class _FakeIk:
    def __init__(self, t0):
        self.lower_limits = np.array([-3.14] * 7)
        self.upper_limits = np.array([3.14] * 7)
        self._t0 = t0
        self.synced = None

    def sync_state(self, q):
        self.synced = np.asarray(q)

    def fk(self, q):
        return self._t0


class _FakeSafety:
    def __init__(self):
        self.initial = None

    def set_initial_state(self, q, pos):
        self.initial = (np.asarray(q), np.asarray(pos).copy())


class _FakeLog:
    def __init__(self):
        self.messages = []

    def warn(self, msg, *a, **k):
        self.messages.append(msg)

    def info(self, msg, *a, **k):
        self.messages.append(msg)


def _make_node():
    node = AstralTeleopArmNode.__new__(AstralTeleopArmNode)
    node.side = "left"
    node._homing = False
    node._got_state = True
    node._state_t = time.monotonic()
    node.data_timeout = 1.5
    node.state_q = np.array([0.1, 0.2, 0.3, -0.4, 0.5, 0.0, 0.0])
    node.q_cmd = np.array([-0.1, -0.2, -0.3, -0.9, -0.5, 0.0, 0.0])
    t0 = np.eye(4)
    t0[:3, 3] = [0.30, 0.10, -0.40]
    node.ik = _FakeIk(t0)
    node.pose = _FakePose()
    node.safety = _FakeSafety()
    node._armed = False
    node._disarm_reason = None
    node._flip_needed = False
    node._log = _FakeLog()
    node.get_logger = lambda: node._log
    return node


def _fk_result():
    t0 = np.eye(4)
    t0[:3, 3] = [0.30, 0.10, -0.40]
    return t0


def test_reanchor_blocks_while_homing():
    node = _make_node()
    node._homing = True
    ok, _ = node._reanchor_teleop()
    assert ok is False
    assert node._armed is False


def test_reanchor_requires_vr():
    node = _make_node()
    node.pose.vr_current_pos = None
    node.pose.vr_current_rot = None
    ok, msg = node._reanchor_teleop()
    assert ok is False
    assert "VR" in msg


def test_reanchor_uses_fresh_measured_state():
    node = _make_node()
    ok, _ = node._reanchor_teleop()
    assert ok is True
    # Anchor came from FK(state_q), not the fallback q_cmd.
    assert np.allclose(node.robot_init_pos, _fk_result()[:3, 3])
    assert np.allclose(node.ik.synced, node.state_q)
    assert node.pose.calibrated == 1
    assert node._armed is True
    assert node._disarm_reason is None
    assert np.allclose(node.q_cmd, node.state_q)
    assert node.safety.initial[0] is not None


def test_reanchor_falls_back_to_q_cmd_when_state_missing():
    node = _make_node()
    node._got_state = False
    ok, _ = node._reanchor_teleop()
    assert ok is True
    assert np.allclose(node.ik.synced, node.q_cmd)


def test_reanchor_falls_back_when_state_stale():
    node = _make_node()
    node._state_t = time.monotonic() - 10.0  # older than data_timeout
    ok, _ = node._reanchor_teleop()
    assert ok is True
    assert np.allclose(node.ik.synced, node.q_cmd)


def _run_all():
    tests = [
        test_reanchor_blocks_while_homing,
        test_reanchor_requires_vr,
        test_reanchor_uses_fresh_measured_state,
        test_reanchor_falls_back_to_q_cmd_when_state_missing,
        test_reanchor_falls_back_when_state_stale,
    ]
    failed = 0
    for t in tests:
        try:
            t()
            print(f"PASS {t.__name__}")
        except AssertionError as exc:
            failed += 1
            print(f"FAIL {t.__name__}: {exc}")
    print(f"\n{len(tests) - failed}/{len(tests)} passed")
    return failed


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)

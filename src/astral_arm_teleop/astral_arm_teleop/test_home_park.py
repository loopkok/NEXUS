#!/usr/bin/env python3
"""Regression tests for the HOME / park-to-zero path on AstralTeleopArmNode.

``_go_home`` must disarm and build a joint-space park path
``init_pose → init_waypoints → 零位`` (the reverse of the startup init
homing), sharing the ``_homing_*`` machinery. ``_finish_homing`` in "park"
mode must end at zero on arrival but *never* hard-command a zero target on
timeout. Tests instantiate the node class via ``__new__`` (no rclpy spin)
and stub ``pose``/``ik``/``safety``/logger/parameters, matching
``test_reanchor_teleop.py``.

Runs only where rclpy is importable (ROS-sourced system python); otherwise
the whole suite skips with exit 0.
"""

import sys
import time

import numpy as np

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

from astral_arm_teleop.astral_arm_teleop_node import AstralTeleopArmNode  # noqa: E402


class _Param:
    def __init__(self, value):
        self.value = value


class _FakePose:
    def __init__(self):
        self.resets = 0

    def reset(self):
        self.resets += 1


class _FakeIk:
    def __init__(self):
        self.lower_limits = np.array([-3.0] * 7)
        self.upper_limits = np.array([3.0] * 7)
        self._t0 = np.eye(4)
        self._t0[:3, 3] = [0.30, 0.10, -0.40]

    def fk(self, q):
        return self._t0.copy()


class _FakeSafety:
    def __init__(self):
        self.initial = None

    def set_initial_state(self, q, pos):
        self.initial = (np.asarray(q), np.asarray(pos).copy())


class _FakeLog:
    def __init__(self):
        self.warns = []

    def warn(self, msg, *a, **k):
        self.warns.append(msg)


INIT_Q = np.array([0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00])
WAY1 = np.array([0.20, 0.10, -0.30, -0.50, 0.10, 0.00, 0.00])
WAY2 = np.array([0.05, 0.05, -0.10, -0.20, 0.00, 0.00, 0.00])
WAYPOINTS = np.concatenate([WAY1, WAY2]).tolist()


def _make_node():
    node = AstralTeleopArmNode.__new__(AstralTeleopArmNode)
    node.side = "left"
    node._flip_needed = False
    node._homing = False
    node._armed = False
    node._disarm_reason = None
    node._init_q_hw = INIT_Q.copy()
    node._init_joint_vel = 0.1
    node._homing_t0 = 0.0
    node._homing_last_log = 0.0
    node._homing_last_base = None
    node._homing_started = False
    node._homing_seeded = False
    node._homing_i = 0
    node._homing_path = []
    node._homing_mode = "init"
    node.q_cmd = np.array([0.40, 0.30, -0.90, -1.20, 0.60, 0.00, 0.00])
    node.robot_init_pos = np.array([0.30, 0.10, -0.40])
    node.robot_init_rot = np.eye(3)
    node.state_q = node.q_cmd.copy()
    node._state_t = time.monotonic()
    node.data_timeout = 1.5
    node.ik = _FakeIk()
    node.pose = _FakePose()
    node.safety = _FakeSafety()
    node._log = _FakeLog()
    node.get_logger = lambda: node._log
    node.get_parameter = lambda name: _Param(WAYPOINTS)
    node._homing_target = lambda: node._homing_path[node._homing_i]
    return node


def test_home_disarms_and_builds_reverse_park_path():
    node = _make_node()
    node._armed = True
    ok, _ = node._go_home()
    assert ok is True
    assert node._armed is False
    assert node._disarm_reason == "operator"
    assert node._homing is True
    assert node._homing_mode == "park"
    # init_pose → init_waypoints → zero (terminal target must be exact zero)
    path = node._homing_path
    assert len(path) == 1 + 2 + 1
    assert np.allclose(path[0], INIT_Q)
    assert np.allclose(path[1], WAY1)
    assert np.allclose(path[2], WAY2)
    assert np.allclose(path[-1], np.zeros(7))
    # First step target is init_pose (current → init_pose → … → zero)
    assert np.allclose(node._homing_target(), INIT_Q)


def test_home_rejected_while_homing():
    node = _make_node()
    node._homing = True
    ok, msg = node._go_home()
    assert ok is False
    assert "in progress" in msg


def test_home_park_arrived_ends_at_zero():
    node = _make_node()
    node._homing = True
    node._homing_mode = "park"
    node.q_cmd = np.full(7, 0.05)  # near-zero remainder before last step
    node._finish_homing(now=5.0, reason="arrived")
    assert node._homing is False
    assert node._homing_mode == "init"
    assert np.allclose(node.q_cmd, np.zeros(7))
    # Robot origin re-anchored to FK(zero), not the stale init anchor
    assert np.allclose(node.robot_init_pos, [0.30, 0.10, -0.40])
    assert node.safety.initial is not None
    assert node.pose.resets >= 1
    assert any("Parked at zero" in w for w in node._log.warns)


def test_home_park_timeout_holds_current_never_zero():
    node = _make_node()
    node._homing = True
    node._homing_mode = "park"
    node.q_cmd = np.array([0.40, 0.30, -0.90, -1.20, 0.60, 0.00, 0.00])
    node._finish_homing(now=60.0, reason="timeout")
    assert node._homing is False
    # Timeout at an arbitrary pose must NOT snap the arm to zero.
    assert not np.allclose(node.q_cmd, np.zeros(7))
    assert np.allclose(node.q_cmd, [0.40, 0.30, -0.90, -1.20, 0.60, 0.00, 0.00])
    assert any("Park interrupted" in w for w in node._log.warns)


def test_on_home_ignores_low_level():
    node = _make_node()

    class _Msg:
        def __init__(self, data):
            self.data = data

    node._on_home(_Msg(False))
    assert node._homing is False  # stale/level-low must not trigger park
    node._on_home(_Msg(True))
    assert node._homing is True
    assert node._homing_mode == "park"


def _run_all():
    tests = [
        test_home_disarms_and_builds_reverse_park_path,
        test_home_rejected_while_homing,
        test_home_park_arrived_ends_at_zero,
        test_home_park_timeout_holds_current_never_zero,
        test_on_home_ignores_low_level,
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

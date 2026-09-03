#!/usr/bin/env python3
"""Regression tests for driver Trigger services on the enable path.

``~/enable`` must be idempotent w.r.t. the board power state:
  * robot already powered (auto_ready / one_click_ready ran) → success WITHOUT
    re-sending the WORK→POSITION→enable sequence (web「HOME」实机出现已使能后
    再 enable 误报"电机未使能"，根因=重复使能 + 板端 robot_powered 位不确认);
  * power bit not observed but board online → still success ("enable issued"),
    mirroring the SDK demos' "enable 未确认 robot_powered 仍继续" handling;
  * only a genuinely offline board yields a hard failure.

Instantiate the node via ``__new__`` (no rclpy spin) and stub ``_robot``,
matching the script-style tests in astral_arm_teleop.

Runs only where rclpy is importable (ROS-sourced system python); otherwise
the whole suite skips with exit 0.
"""

import sys

try:
    import rclpy  # noqa: F401
except ImportError:
    print("SKIP: rclpy not importable (source ROS first)")
    sys.exit(0)

from astral_robot_control.driver_node import AstralRobotDriverNode  # noqa: E402


class _FakeRes:
    def __init__(self):
        self.success = None
        self.message = ""


class _FakeRobot:
    def __init__(self, powered=False, motion_mode=1, online=True, enable_ok=False):
        self._robot_powered = powered
        self._motion_mode = motion_mode
        self.is_online = online
        self._enable_ok = enable_ok
        self.calls = []

    def set_system_mode(self, mode):
        self.calls.append(("set_system_mode", mode))

    def set_motion_mode(self, mode):
        self.calls.append(("set_motion_mode", mode))

    def enable(self, enable_timeout_s=2.0):
        self.calls.append(("enable", enable_timeout_s))
        return self._enable_ok


def _make_node(robot=None, dry_run=False):
    node = AstralRobotDriverNode.__new__(AstralRobotDriverNode)
    node.dry_run = dry_run
    node._robot = robot
    node._apply_lpf = lambda: None
    return node


def test_dry_run_skips():
    node = _make_node(robot=None, dry_run=True)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "dry_run" in res.message


def test_no_robot_fails():
    node = _make_node(robot=None)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is False
    assert res.message == "robot not connected"


def test_already_powered_skips_sequence():
    robot = _FakeRobot(powered=True, motion_mode=1)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "already enabled" in res.message
    assert robot.calls == []  # 已上电绝不重复下发 WORK/POSITION/enable


def test_already_powered_but_damping_returns_to_position():
    robot = _FakeRobot(powered=True, motion_mode=0)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert robot.calls == [("set_motion_mode", 1)]


def test_enable_confirmed_ok():
    robot = _FakeRobot(powered=False, online=True, enable_ok=True)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "enable OK" in res.message
    assert ("set_system_mode", "work") in robot.calls
    assert ("enable", 3.0) in robot.calls


def test_power_bit_unconfirmed_but_online_is_success():
    # 板端在线、电源位未在窗口确认：命令已送达，算下发成功（对齐 SDK demo
    # 「enable 未确认 robot_powered 仍继续」）。否则 web HOME 会误报电机未使能。
    robot = _FakeRobot(powered=False, online=True, enable_ok=False)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is True
    assert "enable issued" in res.message


def test_offline_board_is_hard_failure():
    robot = _FakeRobot(powered=False, online=False, enable_ok=False)
    node = _make_node(robot=robot)
    res = _FakeRes()
    node._srv_enable(None, res)
    assert res.success is False
    assert "not confirmed" in res.message


def _run_all():
    tests = [
        test_dry_run_skips,
        test_no_robot_fails,
        test_already_powered_skips_sequence,
        test_already_powered_but_damping_returns_to_position,
        test_enable_confirmed_ok,
        test_power_bit_unconfirmed_but_online_is_success,
        test_offline_board_is_hard_failure,
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

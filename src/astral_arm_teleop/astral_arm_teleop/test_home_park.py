#!/usr/bin/env python3
"""Regression tests for the HOME / park-to-zero path on AstralTeleopArmNode.

``_go_home`` must disarm and build a joint-space park path
``init_pose → init_waypoints → 零位`` (the reverse of the startup init
homing), sharing the ``_homing_*`` machinery. ``_finish_homing`` in "park"
mode must end at zero on arrival but *never* hard-command a zero target on
timeout. Park homing runs the same smooth open-loop stepper as startup init;
measured state is used only as a *follow guard* — when the robot demonstrably
is not following (measured lags q_cmd beyond ``homing_follow_tol``), the
trajectory freezes instead of running ahead and lunging on late enable.
Tests instantiate the node class via ``__new__`` (no rclpy spin) and stub
``pose``/``ik``/``safety``/logger/parameters, matching
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


class _FakePub:
    def __init__(self):
        self.n = 0

    def publish(self, msg):
        self.n += 1


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
        self.infos = []

    def warn(self, msg, *a, **k):
        self.warns.append(msg)

    def info(self, msg, *a, **k):
        self.infos.append(msg)


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
    node._track_homing_state = True
    node._homing_follow_tol = 0.25
    node._park_frozen = False
    node._init_arrive_tol = 0.05
    node._init_timeout = 15.0
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


def _tick_toward(
    node, target: np.ndarray, ticks: int, mode: str = "init", follow: str = "frozen"
) -> float:
    """Drive ``_homing_tick`` toward *target*.

    State seeding / follow modes simulate the closed loop:
      * follow="frozen"  — measured seeded at the initial q_cmd and never
        advances (motor off / stuck at the command pose)
      * follow="perfect" — measured snaps to q_cmd every tick (healthy tracking)
    Returns the final max per-joint error to *target*.
    """
    node._homing_mode = mode
    node._homing_started = False
    node._homing_seeded = False
    node._park_frozen = False
    node._homing_i = 0
    node._homing_path = [np.asarray(target, dtype=float)]
    node._init_joint_vel = 0.2
    node._init_timeout = 600.0
    node._track_homing_state = True
    node._got_state = True
    node._state_t = time.monotonic()
    node.data_timeout = 0.0  # measured state always "fresh"
    node._init_arrive_tol = 1e-6  # don't finish early; only measure stepping
    node.state_q = node.q_cmd.copy()
    node.cmd_pub = _FakePub()
    node.names = [f"j{i}" for i in range(7)]
    node._last_vr_stamp = object()
    node._publish_q = lambda: None  # publish path not under test
    for i in range(ticks):
        if follow == "perfect":
            node.state_q = node.q_cmd.copy()  # 健康跟随：实测追平指令
        node._homing_tick(now=0.001 + i * 0.01, dt=0.01)
    return float(np.max(np.abs(target - node.q_cmd)))


def test_init_homing_advances_even_when_state_frozen():
    # 启动 init 归位 = 改前开环：web「停止→启动」只是把栈拉起来，电机是否
    # 已使能交给 driver auto_ready。即便实测关节一时没动（电机未跟上/急停
    # 后使能晚到），q_cmd 也必须逐拍朝 init 目标推进，否则臂会"原地不动"。
    node = _make_node()
    node.q_cmd = INIT_Q + 0.30
    err = _tick_toward(node, INIT_Q, ticks=50, mode="init", follow="frozen")
    assert err < 0.30 - 8e-3  # 50 ticks × 0.2 rad/s × 0.01s = 0.10 rad 推进


def test_park_homing_advances_smoothly_when_following():
    # HOME park 正常路径 = 与 init 一致的纯开环匀速推进（实机反馈只做跟随
    # 守卫，正常跟随稳态滞后远小于 follow_tol 时不介入）——不得把命令钉在
    # "实测+一步"上变成阶梯采样（那会导致一卡一卡 + 慢）。健康跟随 50 ticks
    # 下 q_cmd 应推进 ~0.10 rad（与 init 相同）。
    node = _make_node()
    node.q_cmd = INIT_Q + 0.30
    err = _tick_toward(node, INIT_Q, ticks=50, mode="park", follow="perfect")
    assert err < 0.30 - 8e-3


def _park_run(node, target: np.ndarray, ticks: int, follow: str) -> float:
    """Run a *seeded* park homing (no re-seeding) toward *target*.

    ``_homing_started/_seeded`` stay True so the initial seed doesn't overwrite
    ``q_cmd`` — the robot is assumed mid-run. ``follow``:
      * "frozen"  — measured pinned at its first value (robot stopped moving)
      * "perfect" — measured snaps to q_cmd every tick (healthy tracking)
    Returns the final max per-joint error to *target*.
    """
    node._homing_mode = "park"
    node._homing_started = True
    node._homing_seeded = True
    node._homing = True
    node._park_frozen = False
    node._homing_i = 0
    node._homing_path = [np.asarray(target, dtype=float)]
    node._init_joint_vel = 0.2
    node._init_timeout = 600.0
    node._track_homing_state = True
    node._got_state = True
    node._state_t = time.monotonic()
    node.data_timeout = 0.0
    node._init_arrive_tol = 1e-6
    node.cmd_pub = _FakePub()
    node.names = [f"j{i}" for i in range(7)]
    node._last_vr_stamp = object()
    node._publish_q = lambda: None
    for i in range(ticks):
        if follow == "perfect":
            node.state_q = node.q_cmd.copy()
        node._homing_tick(now=0.001 + i * 0.01, dt=0.01)
        if not node._homing:
            break  # 到达/超时已结束 park（真实 _loop 此后不再调 _homing_tick）
    return float(np.max(np.abs(target - node.q_cmd)))


def test_park_freezes_when_robot_stops_following():
    # HOME park 冻结守卫：运行中机器人（实测）停住不动，q_cmd 开环领先一旦
    # 超过 homing_follow_tol（电机失能/堵转没在跟随）→ 轨迹冻结，q_cmd 不许
    # "内部空跑"一路领先到零位；否则等电机恢复时从远处猛扑过来。
    node = _make_node()
    node.q_cmd = np.full(7, 0.5)  # 距零位 0.5 rad
    node.state_q = node.q_cmd.copy()  # 起初健康跟随
    err = _park_run(node, np.zeros(7), ticks=300, follow="frozen")
    assert node._park_frozen is True  # 实测停住 → 冻结
    # 冻结后 q_cmd 不会一路跑到零位：领先被钳在 tol 附近
    gap = float(np.max(np.abs(node.state_q - node.q_cmd)))
    assert gap <= node._homing_follow_tol + 0.02
    assert err > 0.20  # 没跑到零位（冻结在 ~0.25 处）
    assert any("冻结" in w for w in node._log.warns)


def test_park_resumes_when_measured_catches_up():
    # 冻结解除：实测追近指令（电机恢复跟随）→ 冻结解除，继续开环推进到零位。
    node = _make_node()
    node.q_cmd = np.full(7, 0.5)
    node.state_q = node.q_cmd.copy()
    _park_run(node, np.zeros(7), ticks=300, follow="frozen")
    assert node._park_frozen is True
    err = _park_run(node, np.zeros(7), ticks=400, follow="perfect")
    assert node._park_frozen is False  # 恢复跟随 → 解除冻结
    assert err < 0.05  # 从冻结处继续走完，接近零位


class _Bool:
    def __init__(self, data):
        self.data = data


def test_disarm_cancels_in_progress_homing():
    # 硬件模式按钮（web 阻尼/就绪/归零等）会先发 /teleop/disarm。若启动
    # init 归位或 HOME park 仍在发流，必须取消——否则这条陈旧轨迹会持续
    # 下发，把 driver 后来显式下发的归零目标覆盖掉（臂回到阻尼前位姿）。
    node = _make_node()
    node._homing = True
    node._homing_started = True
    node._armed = True
    node._on_disarm(_Bool(False))  # level-low must be ignored
    assert node._homing is True  # stale False must not cancel anything
    node._on_disarm(_Bool(True))
    assert node._homing is False
    assert node._homing_started is False
    assert node._armed is False
    assert node._disarm_reason == "operator"
    assert any("homing cancelled" in w for w in node._log.warns)


def test_disarm_noop_when_not_homing():
    node = _make_node()
    node._armed = True
    node._on_disarm(_Bool(True))
    assert node._homing is False
    assert node._armed is False
    assert node._disarm_reason == "operator"


def _run_all():
    tests = [
        test_home_disarms_and_builds_reverse_park_path,
        test_home_rejected_while_homing,
        test_home_park_arrived_ends_at_zero,
        test_home_park_timeout_holds_current_never_zero,
        test_on_home_ignores_low_level,
        test_init_homing_advances_even_when_state_frozen,
        test_park_homing_advances_smoothly_when_following,
        test_park_freezes_when_robot_stops_following,
        test_park_resumes_when_measured_catches_up,
        test_disarm_cancels_in_progress_homing,
        test_disarm_noop_when_not_homing,
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

"""Astral Robot ROS2 driver node.

Wraps ``astral_robot_sdk`` (UDP → control board) behind a Wuji/XHand-style
joint topic contract so teleop / IK nodes never import the SDK.

Contract (default)::

  Sub  /left_arm/joint_commands   JointState.position[7]
  Sub  /right_arm/joint_commands  JointState.position[7]
  Sub  /astral/joint_commands     JointState.position[18]   (optional full-body)
  Sub  /head/joint_commands       JointState [head_yaw, head_pitch] → 0x31/0x32
  Sub  /{left,right}_gripper/joint_commands  JointState  (radians)
  Sub  /{left,right}_gripper/command         Float64 0=open 1=closed → 0x97/0x98
  Pub  /left_arm/joint_states
  Pub  /right_arm/joint_states
  Pub  /astral/joint_states       (18-DoF: arms+waist+head)
  Pub  /head/joint_states
  Pub  /{left,right}_gripper/joint_states   (last commanded; not OBS 0x31/0x32)

  Srv  ~/ready    Trigger  — one_click_ready (WORK→POSITION→enable→zero)
                             （先清空缓存目标：归零后 100Hz 重发只复读零位，
                             不把阻尼前的陈旧位姿顶回来）
  Srv  ~/enable   Trigger  — WORK→POSITION→enable，**不回零**
                             （遥操恢复归位路径用：只使能，不抢 teleop 正在
                             发布的 joint_commands 目标；已上电直接成功跳过，
                             电源位未确认但板端在线也算下发成功）
  Srv  ~/home     Trigger  — 归零：已在阻尼先切回 POSITION，再 set_all_joints_zero
                             并清缓存（未上电直接报"先 ~/ready"，不静默）
  Srv  ~/estop    Trigger  — disable / e-stop (真断电)，同时清缓存
  Srv  ~/damping  Trigger  — motion_mode=0 阻尼释放（可手动拖拽），同时清缓存
  Srv  ~/position Trigger  — motion_mode=1 位置保持
"""

from __future__ import annotations

from functools import partial
import threading
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from astral_robot_control.joint_layout import (
    ASTRAL_NS,
    CMD_RATIO_SUFFIX,
    CMD_SUFFIX,
    HEAD_JOINT_NAMES,
    HEAD_NS,
    LEFT_ARM_JOINT_NAMES,
    LEFT_ARM_NS,
    LEFT_GRIPPER_NS,
    NUM_ARM_JOINTS,
    NUM_JOINTS,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_ARM_NS,
    RIGHT_GRIPPER_NS,
    ROBOT_JOINT_NAMES,
    STATE_SUFFIX,
    pack_named_positions,
    split_full_q,
)


def _sensor_data_qos() -> QoSProfile:
    """BEST_EFFORT depth=1 — 指令/状态只留最新帧，避免控制环吃到排队旧样本。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class AstralRobotDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_robot_driver")

        # --- hardware / SDK ---
        self.declare_parameter("control_board_ip", "192.168.10.2")
        self.declare_parameter("board_cmd_port", 5001)
        self.declare_parameter("local_ip", "0.0.0.0")
        self.declare_parameter("local_port", 8081)
        self.declare_parameter("obs_hz", 50)
        self.declare_parameter("ctrl_hz", 50)
        self.declare_parameter("lpf_enable", True)
        # 代码默认 0.85≈近直通（配 teleop 平滑 0.25）；config/astral_robot.yaml 会覆盖。
        self.declare_parameter("lpf_alpha", 0.85)
        self.declare_parameter("auto_ready", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("enable_full_body_cmd", True)
        self.declare_parameter("command_timeout_s", 1.5)
        self.declare_parameter("state_publish_rate", 50.0)
        self.declare_parameter("control_rate", 50.0)

        # --- topic namespaces (override if needed) ---
        self.declare_parameter("left_arm_ns", LEFT_ARM_NS)
        self.declare_parameter("right_arm_ns", RIGHT_ARM_NS)
        self.declare_parameter("astral_ns", ASTRAL_NS)
        self.declare_parameter("head_ns", HEAD_NS)
        self.declare_parameter("enable_head_cmd", True)
        self.declare_parameter("left_gripper_ns", LEFT_GRIPPER_NS)
        self.declare_parameter("right_gripper_ns", RIGHT_GRIPPER_NS)
        self.declare_parameter("enable_gripper_cmd", True)
        # Float64 0–1 → set_gripper_angle (CMD 0x97/0x98；不是 0x31/0x32)
        # 真机方向：0.8=张开, 0.0=合拢。
        self.declare_parameter("left_gripper_open_rad", 0.8)
        self.declare_parameter("left_gripper_closed_rad", 0.0)
        self.declare_parameter("right_gripper_open_rad", 0.8)
        self.declare_parameter("right_gripper_closed_rad", 0.0)

        self.board_ip = str(self.get_parameter("control_board_ip").value)
        self.board_port = int(self.get_parameter("board_cmd_port").value)
        self.local_ip = str(self.get_parameter("local_ip").value)
        self.local_port = int(self.get_parameter("local_port").value)
        self.obs_hz = int(self.get_parameter("obs_hz").value)
        self.ctrl_hz = int(self.get_parameter("ctrl_hz").value)
        self.lpf_enable = bool(self.get_parameter("lpf_enable").value)
        self.lpf_alpha = float(self.get_parameter("lpf_alpha").value)
        self.auto_ready = bool(self.get_parameter("auto_ready").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.enable_full_body_cmd = bool(
            self.get_parameter("enable_full_body_cmd").value
        )
        self.command_timeout_s = float(
            self.get_parameter("command_timeout_s").value
        )
        state_rate = float(self.get_parameter("state_publish_rate").value)
        control_rate = float(self.get_parameter("control_rate").value)

        left_ns = str(self.get_parameter("left_arm_ns").value).strip("/")
        right_ns = str(self.get_parameter("right_arm_ns").value).strip("/")
        astral_ns = str(self.get_parameter("astral_ns").value).strip("/")
        head_ns = str(self.get_parameter("head_ns").value).strip("/")
        left_g_ns = str(self.get_parameter("left_gripper_ns").value).strip("/")
        right_g_ns = str(self.get_parameter("right_gripper_ns").value).strip("/")
        self.enable_head_cmd = bool(self.get_parameter("enable_head_cmd").value)
        self.enable_gripper_cmd = bool(self.get_parameter("enable_gripper_cmd").value)
        self._grip_open_rad = {
            "left": float(self.get_parameter("left_gripper_open_rad").value),
            "right": float(self.get_parameter("right_gripper_open_rad").value),
        }
        self._grip_closed_rad = {
            "left": float(self.get_parameter("left_gripper_closed_rad").value),
            "right": float(self.get_parameter("right_gripper_closed_rad").value),
        }

        qos = _sensor_data_qos()
        self._left_cmd_topic = f"/{left_ns}/{CMD_SUFFIX}"
        self._right_cmd_topic = f"/{right_ns}/{CMD_SUFFIX}"
        self._full_cmd_topic = f"/{astral_ns}/{CMD_SUFFIX}"
        self._head_cmd_topic = f"/{head_ns}/{CMD_SUFFIX}"
        self._left_state_topic = f"/{left_ns}/{STATE_SUFFIX}"
        self._right_state_topic = f"/{right_ns}/{STATE_SUFFIX}"
        self._full_state_topic = f"/{astral_ns}/{STATE_SUFFIX}"
        self._head_state_topic = f"/{head_ns}/{STATE_SUFFIX}"
        self._left_grip_cmd_topic = f"/{left_g_ns}/{CMD_SUFFIX}"
        self._right_grip_cmd_topic = f"/{right_g_ns}/{CMD_SUFFIX}"
        self._left_grip_ratio_topic = f"/{left_g_ns}/{CMD_RATIO_SUFFIX}"
        self._right_grip_ratio_topic = f"/{right_g_ns}/{CMD_RATIO_SUFFIX}"
        self._left_grip_state_topic = f"/{left_g_ns}/{STATE_SUFFIX}"
        self._right_grip_state_topic = f"/{right_g_ns}/{STATE_SUFFIX}"

        self._pub_left = self.create_publisher(
            JointState, self._left_state_topic, qos
        )
        self._pub_right = self.create_publisher(
            JointState, self._right_state_topic, qos
        )
        self._pub_full = self.create_publisher(
            JointState, self._full_state_topic, qos
        )
        self._pub_head = self.create_publisher(
            JointState, self._head_state_topic, qos
        )
        self._pub_left_grip = self.create_publisher(
            JointState, self._left_grip_state_topic, qos
        )
        self._pub_right_grip = self.create_publisher(
            JointState, self._right_grip_state_topic, qos
        )

        self.create_subscription(
            JointState, self._left_cmd_topic, self._on_left_cmd, qos
        )
        self.create_subscription(
            JointState, self._right_cmd_topic, self._on_right_cmd, qos
        )
        if self.enable_full_body_cmd:
            self.create_subscription(
                JointState, self._full_cmd_topic, self._on_full_cmd, qos
            )
        if self.enable_head_cmd:
            self.create_subscription(
                JointState, self._head_cmd_topic, self._on_head_cmd, qos
            )
        if self.enable_gripper_cmd:
            self.create_subscription(
                JointState,
                self._left_grip_cmd_topic,
                partial(self._on_grip_js, "left"),
                qos,
            )
            self.create_subscription(
                JointState,
                self._right_grip_cmd_topic,
                partial(self._on_grip_js, "right"),
                qos,
            )
            self.create_subscription(
                Float64,
                self._left_grip_ratio_topic,
                partial(self._on_grip_ratio, "left"),
                qos,
            )
            self.create_subscription(
                Float64,
                self._right_grip_ratio_topic,
                partial(self._on_grip_ratio, "right"),
                qos,
            )

        self.create_service(Trigger, "~/ready", self._srv_ready)
        self.create_service(Trigger, "~/enable", self._srv_enable)
        self.create_service(Trigger, "~/home", self._srv_home)
        self.create_service(Trigger, "~/estop", self._srv_estop)
        self.create_service(Trigger, "~/damping", self._srv_damping)
        self.create_service(Trigger, "~/position", self._srv_position)

        self._lock = threading.Lock()
        self._left_cmd: Optional[List[float]] = None
        self._right_cmd: Optional[List[float]] = None
        self._full_cmd: Optional[List[float]] = None
        self._head_cmd: Optional[List[float]] = None
        self._left_cmd_t = 0.0
        self._right_cmd_t = 0.0
        self._full_cmd_t = 0.0
        self._head_cmd_t = 0.0
        self._use_full_priority = False
        self._grip_rad: dict = {"left": None, "right": None}

        self._robot = None
        self._connect_sdk()

        self.create_timer(1.0 / max(1.0, state_rate), self._on_state_timer)
        self.create_timer(1.0 / max(1.0, control_rate), self._on_control_timer)

        extra_cmd = ""
        if self.enable_head_cmd:
            extra_cmd += f", {self._head_cmd_topic}"
        if self.enable_gripper_cmd:
            extra_cmd += f", {self._left_grip_cmd_topic}|{self._left_grip_ratio_topic}"
        self.get_logger().info(
            "Astral driver ready: "
            f"board={self.board_ip}:{self.board_port} "
            f"dry_run={self.dry_run} "
            f"cmd=[{self._left_cmd_topic}, {self._right_cmd_topic}"
            + (f", {self._full_cmd_topic}" if self.enable_full_body_cmd else "")
            + extra_cmd
            + "] "
            f"state=[{self._left_state_topic}, {self._right_state_topic}, "
            f"{self._full_state_topic}, {self._head_state_topic}] "
            "(0x31/0x32=head via 0x90; grippers via 0x97/0x98)"
        )

    # ------------------------------------------------------------------ SDK
    def _connect_sdk(self) -> None:
        if self.dry_run:
            self.get_logger().warn(
                "dry_run=true — SDK not connected; commands logged only"
            )
            return
        try:
            from astral_robot_sdk import (
                RobotFactory,
                RobotModel,
                create_robot_config,
            )
        except ImportError as exc:
            raise RuntimeError(
                "未找到 astral_robot_sdk。请先：\n"
                "  pip install -e /path/to/astral_robot_sdk"
            ) from exc

        cfg = create_robot_config(
            robot=RobotModel.ROBOTMAIN,
            control_board_ip=self.board_ip,
            board_cmd_port=self.board_port,
            local_ip=self.local_ip,
            local_port=self.local_port,
            obs_hz=self.obs_hz,
            ctrl_hz=self.ctrl_hz,
        )
        self._robot = RobotFactory.create_robot(cfg)
        self.get_logger().info(f"Connecting to {self.board_ip} ...")
        self._robot.connect()
        if self.auto_ready:
            ok = self._robot.one_click_ready(enable_timeout_s=3.0)
            if not ok:
                self.get_logger().warn(
                    "one_click_ready 未确认上电，继续运行（检查板端）"
                )
            else:
                self.get_logger().info("one_click_ready OK")
        self._apply_lpf()

    def _apply_lpf(self) -> None:
        """Push SESSION target LPF after WORK (one_click_ready may reset it)."""
        if self._robot is None:
            return
        alpha = min(1.0, max(1e-3, float(self.lpf_alpha)))
        try:
            self._robot.set_lpf(self.lpf_enable, alpha)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"set_lpf failed: {exc}")
            return
        self.get_logger().info(
            f"lpf enable={self.lpf_enable} alpha={alpha:.3f} "
            f"(obs={self.obs_hz} ctrl={self.ctrl_hz} Hz)"
        )

    def destroy_node(self) -> bool:
        try:
            if self._robot is not None:
                try:
                    self._robot.disconnect()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"disconnect: {exc}")
                self._robot = None
        finally:
            return super().destroy_node()

    # ------------------------------------------------------------------ cmds
    def _on_left_cmd(self, msg: JointState) -> None:
        q = pack_named_positions(msg.name, msg.position, LEFT_ARM_JOINT_NAMES)
        if len(q) != NUM_ARM_JOINTS:
            self.get_logger().warning(
                f"left_arm cmd len={len(q)} != {NUM_ARM_JOINTS}",
                throttle_duration_sec=2.0,
            )
            return
        with self._lock:
            self._left_cmd = q
            self._left_cmd_t = time.monotonic()
            self._use_full_priority = False

    def _on_right_cmd(self, msg: JointState) -> None:
        q = pack_named_positions(msg.name, msg.position, RIGHT_ARM_JOINT_NAMES)
        if len(q) != NUM_ARM_JOINTS:
            self.get_logger().warning(
                f"right_arm cmd len={len(q)} != {NUM_ARM_JOINTS}",
                throttle_duration_sec=2.0,
            )
            return
        with self._lock:
            self._right_cmd = q
            self._right_cmd_t = time.monotonic()
            self._use_full_priority = False

    def _on_full_cmd(self, msg: JointState) -> None:
        q = pack_named_positions(msg.name, msg.position, ROBOT_JOINT_NAMES)
        if len(q) != NUM_JOINTS:
            self.get_logger().warning(
                f"astral cmd len={len(q)} != {NUM_JOINTS}",
                throttle_duration_sec=2.0,
            )
            return
        with self._lock:
            self._full_cmd = q
            self._full_cmd_t = time.monotonic()
            self._use_full_priority = True

    def _on_head_cmd(self, msg: JointState) -> None:
        q = pack_named_positions(msg.name, msg.position, HEAD_JOINT_NAMES)
        if len(q) != 2:
            self.get_logger().warning(
                f"head cmd len={len(q)} != 2",
                throttle_duration_sec=2.0,
            )
            return
        with self._lock:
            self._head_cmd = q
            self._head_cmd_t = time.monotonic()

    def _set_grip_rad(self, side: str, rad: float) -> None:
        with self._lock:
            self._grip_rad[side] = float(rad)

    def _on_grip_js(self, side: str, msg: JointState) -> None:
        # 机械夹爪：话题名 left_gripper/right_gripper → CMD 0x97/0x98
        expected = [f"{side}_gripper"]
        q = pack_named_positions(msg.name, msg.position, expected)
        if not q:
            return
        self._set_grip_rad(side, q[0])

    def _on_grip_ratio(self, side: str, msg: Float64) -> None:
        r = max(0.0, min(1.0, float(msg.data)))
        lo = self._grip_open_rad[side]
        hi = self._grip_closed_rad[side]
        self._set_grip_rad(side, lo + r * (hi - lo))

    def _send_grippers(self) -> None:
        if not self.enable_gripper_cmd:
            return
        with self._lock:
            targets = dict(self._grip_rad)
        for side, rad in targets.items():
            if rad is None:
                continue
            self._send_one_gripper(side, rad)

    def _send_one_gripper(self, side: str, rad: float) -> None:
        right_hand = side == "right"
        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] set_gripper_angle {side}={rad:.3f} rad (0x97/0x98)",
                throttle_duration_sec=1.0,
            )
            return
        if self._robot is None:
            return
        try:
            self._robot.set_gripper_angle(float(rad), right_hand=right_hand)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"set_gripper_angle({side}) failed: {exc}",
                throttle_duration_sec=1.0,
            )

    def _send_head(self) -> None:
        if not self.enable_head_cmd:
            return
        now = time.monotonic()
        with self._lock:
            head = None if self._head_cmd is None else list(self._head_cmd)
            head_t = self._head_cmd_t
        if head is None:
            return
        if self.command_timeout_s > 0 and (now - head_t) > self.command_timeout_s:
            return
        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] move_head_js yaw={head[0]:.3f} pitch={head[1]:.3f}",
                throttle_duration_sec=1.0,
            )
            return
        if self._robot is None:
            return
        try:
            self._robot.move_head_js(head[0], head[1])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"move_head_js failed: {exc}", throttle_duration_sec=1.0
            )

    def _clear_cmd_cache(self) -> None:
        """Drop cached upstream targets so the control timer won't replay a stale pose.

        The control timer re-sends the last upstream joint target at control rate
        while it is "fresh" (command_timeout_s). Manual-mode services
        (ready/home/damping/estop/enable) change what the arm should do next — a
        stale cached pose (e.g. the pose right before 阻尼释放) must not be re-asserted
        once motion mode returns to POSITION, or it overrides the explicit target
        (zero / hold) and the arm snaps back to the pre-damping pose.
        """
        with self._lock:
            self._left_cmd = None
            self._right_cmd = None
            self._full_cmd = None
            self._head_cmd = None
            self._left_cmd_t = 0.0
            self._right_cmd_t = 0.0
            self._full_cmd_t = 0.0
            self._head_cmd_t = 0.0
            self._use_full_priority = False

    def _on_control_timer(self) -> None:
        now = time.monotonic()
        with self._lock:
            use_full = self._use_full_priority
            full = None if self._full_cmd is None else list(self._full_cmd)
            full_t = self._full_cmd_t
            left = None if self._left_cmd is None else list(self._left_cmd)
            right = None if self._right_cmd is None else list(self._right_cmd)
            left_t = self._left_cmd_t
            right_t = self._right_cmd_t

        if use_full and full is not None:
            if self.command_timeout_s <= 0 or (now - full_t) <= self.command_timeout_s:
                self._send_full(full)
            self._send_grippers()
            return

        # Arm-only path: need at least one fresh side; hold the other.
        left_fresh = (
            left is not None
            and (
                self.command_timeout_s <= 0
                or (now - left_t) <= self.command_timeout_s
            )
        )
        right_fresh = (
            right is not None
            and (
                self.command_timeout_s <= 0
                or (now - right_t) <= self.command_timeout_s
            )
        )
        if left_fresh or right_fresh:
            if left is None:
                left = [0.0] * NUM_ARM_JOINTS
            if right is None:
                right = [0.0] * NUM_ARM_JOINTS
            self._send_arms(left, right)

        self._send_head()
        self._send_grippers()

    def _send_arms(self, left: List[float], right: List[float]) -> None:
        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] move_arm_js L={np.round(left, 3).tolist()} "
                f"R={np.round(right, 3).tolist()}",
                throttle_duration_sec=1.0,
            )
            return
        if self._robot is None:
            return
        try:
            self._robot.move_arm_js(left, right)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"move_arm_js failed: {exc}", throttle_duration_sec=1.0
            )

    def _send_full(self, q18: List[float]) -> None:
        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] move_js q18={np.round(q18, 3).tolist()}",
                throttle_duration_sec=1.0,
            )
            return
        if self._robot is None:
            return
        try:
            self._robot.move_js(q18)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"move_js failed: {exc}", throttle_duration_sec=1.0
            )

    # ------------------------------------------------------------------ state
    def _read_q18(self) -> Optional[List[float]]:
        if self.dry_run or self._robot is None:
            return [0.0] * NUM_JOINTS
        try:
            angles = self._robot.get_joint_angles()
            pos = list(angles.msg.positions)
            if len(pos) < NUM_JOINTS:
                pos = pos + [0.0] * (NUM_JOINTS - len(pos))
            return pos[:NUM_JOINTS]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"get_joint_angles: {exc}", throttle_duration_sec=2.0
            )
            return None

    def _on_state_timer(self) -> None:
        q = self._read_q18()
        if q is None:
            return
        left, right, _waist, head = split_full_q(q)
        stamp = self.get_clock().now().to_msg()

        msg_l = JointState()
        msg_l.header.stamp = stamp
        msg_l.name = list(LEFT_ARM_JOINT_NAMES)
        msg_l.position = [float(x) for x in left]
        self._pub_left.publish(msg_l)

        msg_r = JointState()
        msg_r.header.stamp = stamp
        msg_r.name = list(RIGHT_ARM_JOINT_NAMES)
        msg_r.position = [float(x) for x in right]
        self._pub_right.publish(msg_r)

        msg_f = JointState()
        msg_f.header.stamp = stamp
        msg_f.name = list(ROBOT_JOINT_NAMES)
        msg_f.position = [float(x) for x in q]
        self._pub_full.publish(msg_f)

        msg_h = JointState()
        msg_h.header.stamp = stamp
        msg_h.name = list(HEAD_JOINT_NAMES)
        msg_h.position = [float(x) for x in head]
        self._pub_head.publish(msg_h)

        # 夹爪无 OBS 槽位（0x31/0x32 是头）；发布最近一次指令角
        with self._lock:
            gl = self._grip_rad.get("left")
            gr = self._grip_rad.get("right")
        msg_gl = JointState()
        msg_gl.header.stamp = stamp
        msg_gl.name = ["left_gripper"]
        msg_gl.position = [0.0 if gl is None else float(gl)]
        self._pub_left_grip.publish(msg_gl)

        msg_gr = JointState()
        msg_gr.header.stamp = stamp
        msg_gr.name = ["right_gripper"]
        msg_gr.position = [0.0 if gr is None else float(gr)]
        self._pub_right_grip.publish(msg_gr)

    # ------------------------------------------------------------------ srvs
    def _srv_ready(self, _req, res):
        if self.dry_run:
            res.success = True
            res.message = "dry_run: skipped one_click_ready"
            return res
        if self._robot is None:
            res.success = False
            res.message = "robot not connected"
            return res
        try:
            # 清掉缓存里的陈旧上游目标：一键就绪后 driver 的 100Hz 重发只应
            # 复读"归零"，不能把阻尼释放前残留的旧位姿重新顶上来。
            self._clear_cmd_cache()
            ok = self._robot.one_click_ready(enable_timeout_s=3.0)
            self._apply_lpf()
            res.success = bool(ok)
            res.message = "one_click_ready OK" if ok else "enable not confirmed"
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = str(exc)
        return res

    def _srv_enable(self, _req, res):
        """上电使能、**不回零**：WORK→POSITION→enable。

        与 ``~/ready``（one_click_ready 末尾 set_all_joints_zero）的区别：
        遥操启动/急停后恢复时 teleop 正沿 init_waypoints→init_pose 走关节
        轨迹，此刻回零会与轨迹目标抢——本服务只把电机使能起来，目标完全交给
        teleop 的 joint_commands。

        两个经验性修正（实机确认位失灵/重复使能风险）：
        ① 已在使能态（``_robot_powered`` True）直接成功返回、**不重复下发**——
        用户报告：auto_ready/一键就绪已上电后 web「启动/HOME」再 enable 会
        出现"电机未使能"误报（重复使能 + 确认位不可靠）；
        ② enable() 靠轮询板端 obs 帧的 ``robot_powered`` 确认，该位在此板子
        上常不置位（SDK demo 同款"未确认仍继续"），只要板端在线（obs 帧在流）
        就算下发成功——命令已送达，是否观测到电源位不影响实际已上电。
        """
        if self.dry_run:
            res.success = True
            res.message = "dry_run: skipped enable"
            return res
        robot = self._robot
        if robot is None:
            res.success = False
            res.message = "robot not connected"
            return res
        try:
            powered = bool(getattr(robot, "_robot_powered", False))
            if powered:
                if getattr(robot, "_motion_mode", None) != 1:
                    robot.set_motion_mode(1)
                res.success = True
                res.message = "already enabled（已上电，跳过重复使能）"
                return res
            robot.set_system_mode("work")
            time.sleep(0.05)
            robot.set_motion_mode(1)
            time.sleep(0.05)
            ok = robot.enable(enable_timeout_s=3.0)
            self._apply_lpf()
            # 下电（急停/失能）前缓存的上游目标此刻已陈旧：清掉，避免上电瞬间
            # 控制定时器把旧位姿重发出去，与本次 enable 后的新工作流抢目标。
            self._clear_cmd_cache()
            online = bool(robot.is_online)
            if ok:
                res.success = True
                res.message = "enable OK (不回零)"
            elif online:
                # 电源位未在窗口内确认，但板端在线（obs 帧持续）——命令已送达，
                # 对齐 SDK demo「enable 未确认 robot_powered 仍继续」的处理。
                res.success = True
                res.message = "enable issued（已下发；板端电源位未确认）"
            else:
                res.success = False
                res.message = "enable not confirmed (板端离线)"
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = str(exc)
        return res

    def _srv_home(self, _req, res):
        if self.dry_run:
            res.success = True
            res.message = "dry_run: skipped home"
            return res
        if self._robot is None:
            res.success = False
            res.message = "robot not connected"
            return res
        try:
            robot = self._robot
            powered = bool(getattr(robot, "_robot_powered", False))
            # 急停下电后单靠 home 无法归零（没上电位置环不生效），给明确提示
            # 而不是静默无动作。
            if not powered:
                res.success = False
                res.message = "robot not powered — 先 ~/ready（一键就绪）再归零"
                return res
            if getattr(robot, "_motion_mode", None) != 1:
                # 阻尼（motion=0）下板端忽略位置目标：先切回 POSITION 归零才生效。
                robot.set_motion_mode(1)
                time.sleep(0.05)
            robot.set_all_joints_zero()
            # 归零是显式目标：清掉缓存的陈旧上游目标，否则控制定时器在
            # command_timeout_s 窗口内会 100Hz 重发"阻尼释放前的位姿"把它顶掉。
            self._clear_cmd_cache()
            res.success = True
            res.message = "set_all_joints_zero"
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = str(exc)
        return res

    def _srv_estop(self, _req, res):
        if self.dry_run:
            res.success = True
            res.message = "dry_run: skipped estop"
            return res
        if self._robot is None:
            res.success = False
            res.message = "robot not connected"
            return res
        try:
            self._robot.e_stop()
            # 下电后缓存的上游目标已作废：清掉，避免重新上电（~/enable/~/ready）
            # 的瞬间被 100Hz 重发旧位姿抢目标。
            self._clear_cmd_cache()
            res.success = True
            res.message = "e_stop / disable"
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = str(exc)
        return res

    def _srv_damping(self, _req, res):
        """阻尼释放：运动模式切 0（阻尼），电机仍上电、关节可手动拖拽。

        典型用法：遥操停止后臂保持在遥操末位姿，点此按钮后可手动把臂拖回 home。
        需机器人已上电（work+enable）；若已下电需先 ~/ready。
        """
        if self.dry_run:
            res.success = True
            res.message = "dry_run: skipped damping"
            return res
        if self._robot is None:
            res.success = False
            res.message = "robot not connected"
            return res
        try:
            self._robot.set_motion_mode(0)
            # 进入阻尼即离开位置控制：清掉缓存目标，防止后续 ~/home ~/ready
            # 切回 POSITION 时控制定时器把"阻尼前的旧位姿"重新发出去。
            self._clear_cmd_cache()
            res.success = True
            res.message = "motion_mode=0 (damping, 可手动拖拽)"
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = str(exc)
        return res

    def _srv_position(self, _req, res):
        """位置保持：运动模式切 1（位置），关节恢复位置保持。

        典型用法：阻尼释放拖回 home 后点此按钮，臂在当前位置保持（不再可拖拽）。
        """
        if self.dry_run:
            res.success = True
            res.message = "dry_run: skipped position"
            return res
        if self._robot is None:
            res.success = False
            res.message = "robot not connected"
            return res
        try:
            self._robot.set_motion_mode(1)
            res.success = True
            res.message = "motion_mode=1 (position, 位置保持)"
        except Exception as exc:  # noqa: BLE001
            res.success = False
            res.message = str(exc)
        return res


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstralRobotDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()

"""Astral Robot ROS2 driver node.

Wraps ``astral_robot_sdk`` (UDP → control board) behind a Wuji/XHand-style
joint topic contract so teleop / IK nodes never import the SDK.

Contract (default)::

  Sub  /left_arm/joint_commands   JointState.position[7]
  Sub  /right_arm/joint_commands  JointState.position[7]
  Sub  /astral/joint_commands     JointState.position[18]   (optional full-body)
  Pub  /left_arm/joint_states
  Pub  /right_arm/joint_states
  Pub  /astral/joint_states       (18-DoF, always)

  Srv  ~/ready   Trigger  — one_click_ready (WORK→POSITION→enable→zero)
  Srv  ~/home    Trigger  — set_all_joints_zero
  Srv  ~/estop   Trigger  — disable / e-stop
"""

from __future__ import annotations

import threading
import time
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from astral_robot_control.joint_layout import (
    ASTRAL_NS,
    CMD_SUFFIX,
    GRIPPER_JOINT_NAMES,
    LEFT_ARM_JOINT_NAMES,
    LEFT_ARM_NS,
    NUM_ARM_JOINTS,
    NUM_JOINTS,
    RIGHT_ARM_JOINT_NAMES,
    RIGHT_ARM_NS,
    ROBOT_JOINT_NAMES,
    STATE_SUFFIX,
    WAIST_JOINT_NAMES,
    pack_named_positions,
    split_full_q,
)


def _sensor_data_qos() -> QoSProfile:
    """BEST_EFFORT — match Wuji / teleop joint stream."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
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
        self.declare_parameter("auto_ready", True)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("enable_full_body_cmd", True)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("state_publish_rate", 50.0)
        self.declare_parameter("control_rate", 50.0)

        # --- topic namespaces (override if needed) ---
        self.declare_parameter("left_arm_ns", LEFT_ARM_NS)
        self.declare_parameter("right_arm_ns", RIGHT_ARM_NS)
        self.declare_parameter("astral_ns", ASTRAL_NS)

        self.board_ip = str(self.get_parameter("control_board_ip").value)
        self.board_port = int(self.get_parameter("board_cmd_port").value)
        self.local_ip = str(self.get_parameter("local_ip").value)
        self.local_port = int(self.get_parameter("local_port").value)
        self.obs_hz = int(self.get_parameter("obs_hz").value)
        self.ctrl_hz = int(self.get_parameter("ctrl_hz").value)
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

        qos = _sensor_data_qos()
        self._left_cmd_topic = f"/{left_ns}/{CMD_SUFFIX}"
        self._right_cmd_topic = f"/{right_ns}/{CMD_SUFFIX}"
        self._full_cmd_topic = f"/{astral_ns}/{CMD_SUFFIX}"
        self._left_state_topic = f"/{left_ns}/{STATE_SUFFIX}"
        self._right_state_topic = f"/{right_ns}/{STATE_SUFFIX}"
        self._full_state_topic = f"/{astral_ns}/{STATE_SUFFIX}"

        self._pub_left = self.create_publisher(
            JointState, self._left_state_topic, qos
        )
        self._pub_right = self.create_publisher(
            JointState, self._right_state_topic, qos
        )
        self._pub_full = self.create_publisher(
            JointState, self._full_state_topic, qos
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

        self.create_service(Trigger, "~/ready", self._srv_ready)
        self.create_service(Trigger, "~/home", self._srv_home)
        self.create_service(Trigger, "~/estop", self._srv_estop)

        self._lock = threading.Lock()
        self._left_cmd: Optional[List[float]] = None
        self._right_cmd: Optional[List[float]] = None
        self._full_cmd: Optional[List[float]] = None
        self._left_cmd_t = 0.0
        self._right_cmd_t = 0.0
        self._full_cmd_t = 0.0
        self._use_full_priority = False

        self._robot = None
        self._connect_sdk()

        self.create_timer(1.0 / max(1.0, state_rate), self._on_state_timer)
        self.create_timer(1.0 / max(1.0, control_rate), self._on_control_timer)

        self.get_logger().info(
            "Astral driver ready: "
            f"board={self.board_ip}:{self.board_port} "
            f"dry_run={self.dry_run} "
            f"cmd=[{self._left_cmd_topic}, {self._right_cmd_topic}"
            + (f", {self._full_cmd_topic}" if self.enable_full_body_cmd else "")
            + "] "
            f"state=[{self._left_state_topic}, {self._right_state_topic}, "
            f"{self._full_state_topic}]"
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
            if self.command_timeout_s > 0 and (now - full_t) > self.command_timeout_s:
                return
            self._send_full(full)
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
        if not left_fresh and not right_fresh:
            return
        if left is None:
            left = [0.0] * NUM_ARM_JOINTS
        if right is None:
            right = [0.0] * NUM_ARM_JOINTS
        self._send_arms(left, right)

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
        left, right, _waist, _grip = split_full_q(q)
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
            ok = self._robot.one_click_ready(enable_timeout_s=3.0)
            res.success = bool(ok)
            res.message = "one_click_ready OK" if ok else "enable not confirmed"
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
            self._robot.set_all_joints_zero()
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
            res.success = True
            res.message = "e_stop / disable"
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

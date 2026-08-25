#!/usr/bin/env python3
"""Quest3 hand_landmarks → /{side}_gripper/command (0=open, 1=closed).

Also publishes JointState radians on /{side}_gripper/joint_commands so
astral_robot_control can drive set_gripper_angle.

Swap this node later for a hardware adapter that publishes the same topics.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from astral_gripper_teleop.pinch import (
    close_ratio_from_pinch,
    pinch_distance_m,
    ratio_to_rad,
)


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class PinchGripperNode(Node):
    def __init__(self) -> None:
        super().__init__("pinch_gripper_node")

        self.declare_parameter("hand_side", "left")
        self.declare_parameter("landmark_topic", "")
        self.declare_parameter("command_topic", "")
        self.declare_parameter("joint_command_topic", "")
        self.declare_parameter("joint_name", "")
        # Pinch distances in the same units as landmarks (Quest wrist-local meters).
        self.declare_parameter("open_dist_m", 0.08)
        self.declare_parameter("close_dist_m", 0.015)
        # Actuator radians for motor 0x31 / 0x32 — tune on hardware.
        # 真机方向：0.8=张开, 0.0=合拢（与 driver/sim 一致）。
        self.declare_parameter("open_rad", 0.8)
        self.declare_parameter("closed_rad", 0.0)
        self.declare_parameter("ema_alpha", 0.4)
        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("input_timeout_s", 0.4)
        # hold = keep last ratio; open = force 0 on timeout
        self.declare_parameter("on_timeout", "hold")
        self.declare_parameter("log_interval_s", 2.0)

        side = str(self.get_parameter("hand_side").value).strip().lower()
        if side not in ("left", "right"):
            raise ValueError(f"hand_side 须为 left|right，收到: {side}")
        self.side = side

        landmark_topic = str(self.get_parameter("landmark_topic").value).strip()
        command_topic = str(self.get_parameter("command_topic").value).strip()
        joint_topic = str(self.get_parameter("joint_command_topic").value).strip()
        joint_name = str(self.get_parameter("joint_name").value).strip()
        if not landmark_topic:
            landmark_topic = f"hand_landmarks/{side}"
        if not command_topic:
            command_topic = f"/{side}_gripper/command"
        if not joint_topic:
            joint_topic = f"/{side}_gripper/joint_commands"
        if not joint_name:
            joint_name = f"{side}_gripper"

        self.landmark_topic = landmark_topic
        self.command_topic = command_topic
        self.joint_topic = joint_topic
        self.joint_name = joint_name
        self.open_dist = float(self.get_parameter("open_dist_m").value)
        self.close_dist = float(self.get_parameter("close_dist_m").value)
        self.open_rad = float(self.get_parameter("open_rad").value)
        self.closed_rad = float(self.get_parameter("closed_rad").value)
        self.ema_alpha = float(self.get_parameter("ema_alpha").value)
        self.input_timeout_s = float(self.get_parameter("input_timeout_s").value)
        self.on_timeout = str(self.get_parameter("on_timeout").value).strip().lower()
        self.log_interval_s = float(self.get_parameter("log_interval_s").value)
        rate = float(self.get_parameter("publish_rate").value)

        if self.open_dist <= self.close_dist:
            raise ValueError("open_dist_m 必须大于 close_dist_m")

        qos = _sensor_qos()
        self._pub_cmd = self.create_publisher(Float64, self.command_topic, qos)
        self._pub_js = self.create_publisher(JointState, self.joint_topic, qos)
        self.create_subscription(
            PoseArray, self.landmark_topic, self._on_landmarks, qos
        )

        self._ratio: float = 0.0
        self._ema_ready = False
        self._last_lm_t = 0.0
        self._last_dist = 0.0
        self._last_log = time.monotonic()

        self.create_timer(1.0 / max(1.0, rate), self._on_timer)
        self.get_logger().info(
            f"pinch→gripper[{side}] landmarks={self.landmark_topic} "
            f"cmd={self.command_topic} js={self.joint_topic} "
            f"pinch=[{self.close_dist:.3f},{self.open_dist:.3f}]m "
            f"rad=[{self.open_rad:.3f},{self.closed_rad:.3f}]"
        )

    def _on_landmarks(self, msg: PoseArray) -> None:
        if len(msg.poses) < 9:
            return
        pts = np.array(
            [[p.position.x, p.position.y, p.position.z] for p in msg.poses],
            dtype=np.float64,
        )
        if not np.isfinite(pts[:9]).all():
            return
        if float(np.linalg.norm(pts[4])) < 1e-6 and float(np.linalg.norm(pts[8])) < 1e-6:
            return
        dist = pinch_distance_m(pts)
        raw = close_ratio_from_pinch(dist, self.open_dist, self.close_dist)
        if not self._ema_ready or self.ema_alpha >= 1.0:
            self._ratio = raw
            self._ema_ready = True
        else:
            a = max(0.0, min(1.0, self.ema_alpha))
            self._ratio = a * raw + (1.0 - a) * self._ratio
        self._last_dist = dist
        self._last_lm_t = time.monotonic()

    def _on_timer(self) -> None:
        now = time.monotonic()
        stale = (
            self._last_lm_t <= 0.0
            or (now - self._last_lm_t) > self.input_timeout_s
        )
        if stale:
            if self._last_lm_t <= 0.0:
                return
            if self.on_timeout == "open":
                self._ratio = 0.0
            elif self.on_timeout != "hold":
                return

        ratio = max(0.0, min(1.0, self._ratio))
        rad = ratio_to_rad(ratio, self.open_rad, self.closed_rad)

        cmd = Float64()
        cmd.data = float(ratio)
        self._pub_cmd.publish(cmd)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.joint_name]
        js.position = [float(rad)]
        self._pub_js.publish(js)

        if self.log_interval_s > 0 and (now - self._last_log) >= self.log_interval_s:
            self._last_log = now
            self.get_logger().info(
                f"[gripper {self.side}] dist={self._last_dist*1000:.0f}mm "
                f"close={ratio:.2f} rad={rad:.3f} stale={stale}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PinchGripperNode()
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

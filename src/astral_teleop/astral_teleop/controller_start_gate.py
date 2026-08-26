"""Touch controller button → teleop external start gate.

Subscribes to a ``sensor_msgs/Joy`` topic (default the left Touch controller)
and, on the rising edge of a chosen button, publishes ``/teleop/start`` (Bool
true) — the same one-shot signal astral_arm_teleop consumes (with
``require_start_signal:=true``) to capture ``vr_init`` and arm.

Default button index 5 = grip click (middle finger). Re-pressing re-captures
the zero (re-center), matching the web "开始遥操" button and the CLI
``ros2 topic pub --once /teleop/start``.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class ControllerStartGate(Node):
    def __init__(self) -> None:
        super().__init__("controller_start_gate")

        self.declare_parameter("joy_topic", "quest3/left_controller_joy")
        self.declare_parameter("button_index", 5)
        self.declare_parameter("start_topic", "/teleop/start")

        joy_topic = str(self.get_parameter("joy_topic").value).strip()
        self.button_index = int(self.get_parameter("button_index").value)
        start_topic = str(self.get_parameter("start_topic").value).strip()

        # RELIABLE + VOLATILE: one-shot, not latched (match monitor_node).
        self._pub = self.create_publisher(Bool, start_topic, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, _sensor_qos())
        self._prev_pressed = False

        self.get_logger().info(
            f"controller_start_gate joy={joy_topic} button={self.button_index} "
            f"(5=gripClick) → {start_topic}"
        )

    def _on_joy(self, msg: Joy) -> None:
        btns = list(msg.buttons) if msg.buttons else []
        if self.button_index < 0 or self.button_index >= len(btns):
            return
        pressed = bool(btns[self.button_index])
        # Rising edge only — hold does not spam start.
        if pressed and not self._prev_pressed:
            self._pub.publish(Bool(data=True))
            self.get_logger().info(
                f"controller button {self.button_index} pressed → {self._pub.topic}"
            )
        self._prev_pressed = pressed


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControllerStartGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

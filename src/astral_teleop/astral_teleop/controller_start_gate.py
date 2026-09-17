"""Touch controller button → teleop external start gate (inference-aware).

Subscribes to a ``sensor_msgs/Joy`` topic (default the left Touch controller)
and, on the rising edge of a chosen button, either:

* ``/teleop/start`` + ``/teleop/armed`` (Bool true) — the normal one-shot that
  astral_arm_teleop consumes (``require_start_signal:=true``) to capture
  ``vr_init`` and arm; or
* ``/policy_inference/cmd`` = "takeover" — when the policy node is active
  (``/policy_inference/state`` activity ∈ {policy, playback}), so grip becomes
  the HITL takeover trigger instead of arming teleop (which would race the
  policy writing ``joint_commands``).

Default button index 5 = grip click (middle finger). Re-pressing re-captures
the zero (re-center), matching the web "开始遥操" button and the CLI
``ros2 topic pub --once /teleop/start``.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, String

from astral_teleop.start_gate_logic import decide_start_action


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class ControllerStartGate(Node):
    def __init__(self) -> None:
        super().__init__("controller_start_gate")

        self.declare_parameter("joy_topic", "quest3/left_controller_joy")
        self.declare_parameter("button_index", 5)
        self.declare_parameter("start_topic", "/teleop/start")
        # 工作位/HOME/暂停发过 latched /teleop/disarm 会把夹爪 pinch 仲裁门
        # 关死，门只在 /teleop/armed=true 时重开——用本闸门（gripClick）启动
        # 遥操时必须一并发 armed=true，否则手柄对夹爪永远无响应。臂节点未
        # 校准会忽略 armed（无害），真正 arm 由 /teleop/start 完成。
        self.declare_parameter("armed_topic", "/teleop/armed")
        # 推理感知：策略活跃（policy/playback）时 grip 改为 HITL 接管。
        self.declare_parameter("policy_state_topic", "/policy_inference/state")
        self.declare_parameter("policy_cmd_topic", "/policy_inference/cmd")

        joy_topic = str(self.get_parameter("joy_topic").value).strip()
        self.button_index = int(self.get_parameter("button_index").value)
        start_topic = str(self.get_parameter("start_topic").value).strip()
        armed_topic = str(self.get_parameter("armed_topic").value).strip()
        state_topic = str(self.get_parameter("policy_state_topic").value).strip()
        self._policy_cmd_topic = str(self.get_parameter("policy_cmd_topic").value).strip()

        # RELIABLE + VOLATILE: one-shot, not latched (match monitor_node).
        self._pub = self.create_publisher(Bool, start_topic, 10)
        self._pub_armed = self.create_publisher(Bool, armed_topic, 10)
        self._policy_cmd_pub = self.create_publisher(String, self._policy_cmd_topic, 10)
        # BEST_EFFORT sub is compatible with the mocap Joy publisher (RELIABLE
        # default) and CLI `ros2 topic pub` — no dual-sub needed.
        self.create_subscription(Joy, joy_topic, self._on_joy, _sensor_qos())
        # 策略节点状态（latched，新订阅者立即拿到当前值）；无人发布=None。
        self._activity: str | None = None
        self.create_subscription(String, state_topic, self._on_policy_state, _latched_qos())
        self._prev_pressed = False

        self.get_logger().info(
            f"controller_start_gate joy={joy_topic} button={self.button_index} "
            f"(5=gripClick) → {start_topic} + {armed_topic} "
            f"(policy active → {self._policy_cmd_topic} 'takeover')"
        )

    def _on_policy_state(self, msg: String) -> None:
        try:
            self._activity = json.loads(msg.data).get("activity")
        except Exception:  # noqa: BLE001
            self._activity = None

    def _on_joy(self, msg: Joy) -> None:
        btns = list(msg.buttons) if msg.buttons else []
        if self.button_index < 0 or self.button_index >= len(btns):
            return
        pressed = bool(btns[self.button_index])
        # Rising edge only — hold does not spam start.
        if pressed and not self._prev_pressed:
            if decide_start_action(self._activity) == "takeover":
                self._policy_cmd_pub.publish(String(data="takeover"))
                self.get_logger().info(
                    f"controller button {self.button_index} pressed → "
                    f"{self._policy_cmd_topic} 'takeover' (policy active)"
                )
            else:
                self._pub.publish(Bool(data=True))
                self._pub_armed.publish(Bool(data=True))
                self.get_logger().info(
                    f"controller button {self.button_index} pressed → "
                    f"{self._pub.topic} + {self._pub_armed.topic}"
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

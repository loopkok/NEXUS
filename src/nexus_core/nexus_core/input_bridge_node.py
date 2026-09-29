"""Normalize the selected input and camera sources into NEXUS topics."""

import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CompressedImage, Joy
from std_msgs.msg import String

from .profile import Profile

_LATCHED_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class InputBridgeNode(Node):
    def __init__(self):
        super().__init__("nexus_input_bridge")
        self.declare_parameter("profile_file", "")
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        ns = self.profile.namespace
        self._frames: dict[str, str] = {}
        message_types = {
            "wrist": (PoseStamped, "wrist_pose"),
            "hand": (PoseArray, "hand_landmarks"),
            "controller_joy": (Joy, "controller_joy"),
            "body_joints": (PoseArray, "body_joints"),
            "body_joint_names": (String, "body_joint_names"),
        }
        for channel, selected in self.profile.raw["inputs"].items():
            for semantic, (message_type, output_name) in message_types.items():
                spec = self.profile.input_spec(channel, semantic)
                if spec is None or spec["source"] == "none":
                    continue
                self._relay(message_type, spec["topic"],
                            f"{ns}/input/{channel}/{output_name}",
                            stable_frame=spec.get("frame_policy") == "stable",
                            publisher_qos=_LATCHED_QOS if semantic == "body_joint_names" else None,
                            subscriber_qos=_LATCHED_QOS if semantic == "body_joint_names" else None)
        for camera in self.profile.raw["cameras"]:
            capture_topic = camera.get("capture_topic")
            if capture_topic:
                role = camera["role"]
                self._relay(CompressedImage, capture_topic,
                            f"{ns}/camera/{role}/image/compressed", stable_frame=True)
        self.get_logger().info(f"input bridge profile={self.profile.profile_id} sha256={self.profile.digest}")

    def _relay(self, msg_type, input_topic: str, output_topic: str,
               stable_frame: bool = False, publisher_qos=None, subscriber_qos=None) -> None:
        publisher = self.create_publisher(
            msg_type, output_topic, publisher_qos or qos_profile_sensor_data)

        def forward(msg):
            if hasattr(msg, "header"):
                stamp = msg.header.stamp
                if stamp.sec == 0 and stamp.nanosec == 0:
                    msg.header.stamp = self.get_clock().now().to_msg()
                if stable_frame:
                    frame = msg.header.frame_id
                    if not frame:
                        self.get_logger().error(f"missing frame_id on {input_topic}", throttle_duration_sec=2.0)
                        return
                    previous = self._frames.get(input_topic)
                    if previous and previous != frame:
                        self.get_logger().error(f"frame switch {input_topic}: {previous} -> {frame}; dropped",
                                                throttle_duration_sec=2.0)
                        return
                    self._frames[input_topic] = frame
            publisher.publish(msg)

        self.create_subscription(msg_type, input_topic, forward,
                                 subscriber_qos or qos_profile_sensor_data)


def main() -> None:
    rclpy.init()
    node = InputBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

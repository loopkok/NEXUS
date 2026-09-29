"""ROS single-writer for all configured robot actuators."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .arbiter import CommandArbiter, policy_idle_should_release
from .profile import Profile, ProfileError

# Candidate commands are latest-value control data: avoid reliable DDS
# backpressure and let the mux stale-command watchdog reject a stalled stream.
_CANDIDATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class CommandMuxNode(Node):
    def __init__(self):
        super().__init__("nexus_command_mux")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("control_rate", 100.0)
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("startup_timeout", 5.0)
        path = str(self.get_parameter("profile_file").value)
        self.profile = Profile.load(path)
        self.mux = CommandArbiter(
            self.profile,
            command_timeout=float(self.get_parameter("command_timeout").value),
            state_timeout=float(self.get_parameter("state_timeout").value),
            startup_timeout=float(self.get_parameter("startup_timeout").value),
        )
        ns = self.profile.namespace
        self._outputs = {}
        for component in self.profile.components:
            name = component.name
            self.create_subscription(
                JointState, self.profile.topic(name, "joint_states"),
                lambda msg, n=name: self._state(n, msg), qos_profile_sensor_data)
            for source in ("teleop", "policy", "playback"):
                self.create_subscription(
                    JointState, self.profile.candidate_topic(source, name),
                    lambda msg, s=source, n=name: self._candidate(s, n, msg),
                    _CANDIDATE_QOS)
            self._outputs[name] = self.create_publisher(
                JointState, self.profile.topic(name, "joint_commands"),
                qos_profile_sensor_data)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._status_pub = self.create_publisher(String, f"{ns}/control/state", status_qos)
        self.create_subscription(String, f"{ns}/control/cmd", self._control, 10)
        self.create_subscription(String, f"{ns}/policy/state", self._policy_state, status_qos)
        self.create_subscription(Bool, f"{ns}/control/teleop_start", self._teleop_start, 10)
        self.create_subscription(Bool, f"{ns}/control/teleop_disarm", self._teleop_disarm, 10)
        self.create_service(Trigger, f"{ns}/control/estop", self._estop)
        rate = float(self.get_parameter("control_rate").value)
        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.create_timer(1.0, self._publish_status)
        self.get_logger().info(f"NEXUS mux: {self.profile.profile_id} sha256={self.profile.digest} "
                               f"components={[c.name for c in self.profile.components]}")
        self._publish_status()

    def _state(self, name: str, msg: JointState) -> None:
        try:
            self.mux.update_state(name, list(msg.name), list(msg.position), time.monotonic())
        except ProfileError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def _candidate(self, source: str, name: str, msg: JointState) -> None:
        try:
            self.mux.update_candidate(source, name, list(msg.name), list(msg.position), time.monotonic())
        except ProfileError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=2.0)

    def _select(self, mode: str) -> None:
        try:
            self.mux.select(mode, time.monotonic())
            self._publish_status()
        except ProfileError as exc:
            self.get_logger().error(str(exc))

    def _control(self, msg: String) -> None:
        verb = msg.data.strip().upper()
        if verb == "TELEOP" and self.mux.mode in ("POLICY", "PLAYBACK"):
            self.get_logger().error("request takeover through policy_inference/cmd; reanchor is required")
            return
        if verb == "HOMING" and self.mux.mode not in ("IDLE", "PAUSED", "HOMING"):
            self.get_logger().error(f"homing requires IDLE or PAUSED; current mode is {self.mux.mode}")
            return
        self._select(verb)

    def _policy_state(self, msg: String) -> None:
        try:
            state = str(json.loads(msg.data).get("state", "")).upper()
        except (ValueError, TypeError):
            return
        mapped = {"POLICY": "POLICY", "PLAYBACK": "PLAYBACK", "HUMAN": "TELEOP",
                  "POLICY_PAUSED": "PAUSED", "PLAYBACK_PAUSED": "PAUSED"}.get(state)
        if mapped and mapped != self.mux.mode:
            self._select(mapped)
        elif state == "IDLE" and policy_idle_should_release(self.mux.mode, self.mux.resume_mode):
            self._select("IDLE")

    def _teleop_start(self, msg: Bool) -> None:
        if msg.data and self.mux.mode in ("IDLE", "TELEOP"):
            self._select("TELEOP")

    def _teleop_disarm(self, msg: Bool) -> None:
        if msg.data and self.mux.mode == "TELEOP":
            self._select("IDLE")

    def _estop(self, _req: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._select("ESTOP")
        response.success = True
        response.message = "NEXUS command output stopped; reset hardware separately"
        return response

    def _tick(self) -> None:
        previous_status = (self.mux.mode, self.mux.fault)
        for spec in self.profile.components:
            topic = self.profile.topic(spec.name, "joint_commands")
            if self.count_publishers(topic) > 1:
                self._select("ESTOP")
                self.get_logger().fatal(f"multiple publishers on sole-driver topic {topic}")
                return
        output = self.mux.tick(time.monotonic())
        for name, values in output.items():
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = list(self.profile.component(name).joints)
            msg.position = list(values)
            self._outputs[name].publish(msg)
        if (self.mux.mode, self.mux.fault) != previous_status:
            self._publish_status()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self.mux.status(), ensure_ascii=False)
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = CommandMuxNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

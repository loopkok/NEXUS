"""Joint-level test driver for launch and HITL tests without serial hardware."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .profile import Profile


class FakeDriverNode(Node):
    def __init__(self):
        super().__init__("nexus_fake_driver")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("component", "")
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        self.spec = self.profile.component(str(self.get_parameter("component").value))
        self._q = [min(hi, max(lo, 0.0)) for lo, hi in zip(self.spec.lower, self.spec.upper)]
        self._enabled = False
        self._stopped = False
        self._pub = self.create_publisher(JointState,
            self.profile.topic(self.spec.name, "joint_states"), qos_profile_sensor_data)
        self.create_subscription(JointState, self.profile.topic(self.spec.name, "joint_commands"),
                                 self._command, qos_profile_sensor_data)
        prefix = f"{self.profile.namespace}/drivers/{self.spec.name}"
        for name, callback in (("ready", self._ready), ("enable", self._enable),
                               ("home", self._home), ("estop", self._estop)):
            self.create_service(Trigger, f"{prefix}/{name}", callback)
        self.create_timer(0.02, self._tick)

    def _command(self, msg):
        if not self._enabled or self._stopped:
            return
        values = list(msg.position)
        if list(msg.name) != list(self.spec.joints) or len(values) != self.spec.dim:
            return
        if any(not math.isfinite(v) or v < lo or v > hi
               for v, lo, hi in zip(values, self.spec.lower, self.spec.upper)):
            return
        self._q = values

    def _tick(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.spec.joints)
        msg.position = list(self._q)
        self._pub.publish(msg)

    def _result(self, response, success, message):
        response.success = success
        response.message = message
        return response

    def _ready(self, _request, response):
        return self._result(response, True, "fake feedback ready")

    def _enable(self, _request, response):
        if self._stopped:
            return self._result(response, False, "estop latched")
        self._enabled = True
        return self._result(response, True, "fake driver enabled")

    def _home(self, _request, response):
        if not self._enabled:
            return self._result(response, False, "not enabled")
        self._q = [min(hi, max(lo, 0.0)) for lo, hi in zip(self.spec.lower, self.spec.upper)]
        return self._result(response, True, "fake driver homed")

    def _estop(self, _request, response):
        self._stopped = True
        self._enabled = False
        return self._result(response, True, "fake driver stopped")


def main():
    rclpy.init()
    node = FakeDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

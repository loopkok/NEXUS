"""Standalone Nero CAN driver. It never subscribes to VR or runs IK."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .profile import Profile


class NeroDriverNode(Node):
    def __init__(self):
        super().__init__("nexus_nero_driver")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("side", "left")
        self.declare_parameter("component", "")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("state_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        component = str(self.get_parameter("component").value)
        side_param = str(self.get_parameter("side").value)
        self.spec = self.profile.component(component) if component else self.profile.component(f"{side_param}_arm")
        self.side = self.spec.side or side_param
        if self.spec.driver != "nero_can":
            raise ValueError(f"{self.spec.name} is not a Nero CAN component")
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self._lock = threading.RLock()
        self._enabled = False
        self._stopped = False
        self._homing = False
        self._last_cmd = 0.0
        self._last_state = 0.0
        self._last_q: tuple[float, ...] | None = None
        self._timeout = float(self.get_parameter("command_timeout").value)
        self._robot = None
        if not self.dry_run:
            from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW
            channel = self.profile.adapter_config("nero_can")["channels"][self.side]
            cfg = create_agx_arm_config(robot=ArmModel.NERO, firmeware_version=NeroFW.DEFAULT,
                                        channel=channel, interface="socketcan")
            self._robot = AgxArmFactory.create_arm(cfg)
            self._robot.connect()
        else:
            self._last_q = tuple(0.0 for _ in self.spec.joints)
            self._last_state = time.monotonic()
        self._state_pub = self.create_publisher(
            JointState, self.profile.topic(self.spec.name, "joint_states"), qos_profile_sensor_data)
        self.create_subscription(JointState, self.profile.topic(self.spec.name, "joint_commands"),
                                 self._on_command, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        prefix = f"{self.profile.namespace}/drivers/{self.spec.name}"
        self.create_service(Trigger, f"{prefix}/ready", self._ready)
        self.create_service(Trigger, f"{prefix}/enable", self._enable)
        self.create_service(Trigger, f"{prefix}/home", self._home)
        self.create_service(Trigger, f"{prefix}/estop", self._estop)
        self.create_timer(1.0 / max(1.0, float(self.get_parameter("state_rate").value)), self._poll_state)
        self.create_timer(0.05, self._watchdog)
        self.get_logger().info(f"Nero driver {self.side} profile_sha256={self.profile.digest} dry_run={self.dry_run}")

    def _response(self, response, ok: bool, message: str):
        response.success = ok
        response.message = message
        return response

    def _ready(self, _request, response):
        ok = self._last_q is not None and time.monotonic() - self._last_state < 0.5
        return self._response(response, ok, "fresh measured joints" if ok else "joint feedback unavailable")

    def _enable(self, _request, response):
        if self._stopped:
            return self._response(response, False, "estop latched; restart driver after hardware reset")
        if self._last_q is None or time.monotonic() - self._last_state >= 0.5:
            return self._response(response, False, "fresh feedback required before enable")
        with self._lock:
            if not self.dry_run:
                deadline = time.monotonic() + 2.0
                while time.monotonic() < deadline:
                    if self._robot.enable():
                        break
                    time.sleep(0.02)
                else:
                    return self._response(response, False, "CAN enable timed out")
                self._robot.set_motion_mode("js")
                self._robot.set_auto_set_motion_mode_enabled(False)
            self._enabled = True
            self._last_cmd = time.monotonic()
        return self._response(response, True, "enabled; holding measured pose")

    def _home(self, _request, response):
        if not self._enabled or self._last_q is None:
            return self._response(response, False, "enable and read feedback first")
        target = tuple(float(v) for v in self.profile.adapter_config("nero_can")["home_pose"][self.side])
        if len(target) != self.spec.dim or any(v < lo or v > hi
                                               for v, lo, hi in zip(target, self.spec.lower, self.spec.upper)):
            return self._response(response, False, "profile home pose is outside joint limits")
        if self.dry_run:
            self._last_q = target
            self._last_cmd = time.monotonic()
            return self._response(response, True, "dry-run home pose set")
        try:
            self._homing = True
            with self._lock:
                self._robot.set_motion_mode("j")
                self._robot.set_auto_set_motion_mode_enabled(False)
                self._robot.move_j(list(target))
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                with self._lock:
                    state = self._robot.get_joint_angles()
                if state is not None and len(state.msg) >= self.spec.dim:
                    measured = tuple(float(v) for v in state.msg[:self.spec.dim])
                    self._last_q = measured
                    self._last_state = time.monotonic()
                    if max(abs(a - b) for a, b in zip(measured, target)) < 0.05:
                        with self._lock:
                            self._robot.set_motion_mode("js")
                            self._robot.set_auto_set_motion_mode_enabled(False)
                        self._last_cmd = time.monotonic()
                        self._homing = False
                        return self._response(response, True, "Nero home reached")
                time.sleep(0.05)
        except Exception as exc:
            self.get_logger().error(f"Nero home failed: {exc}")
        with self._lock:
            self._enabled = False
            self._robot.disable()
        self._homing = False
        return self._response(response, False, "Nero home failed or timed out; motors disabled")

    def _estop(self, _request, response):
        with self._lock:
            self._stopped = True
            self._enabled = False
            if self._robot is not None:
                self._robot.disable()
        return self._response(response, True, "CAN motors disabled")

    def _poll_state(self):
        with self._lock:
            try:
                if self.dry_run:
                    values = self._last_q
                else:
                    record = self._robot.get_joint_angles()
                    values = tuple(float(v) for v in record.msg[:self.spec.dim]) if record else None
            except Exception as exc:
                self.get_logger().error(f"CAN feedback failed: {exc}", throttle_duration_sec=2.0)
                return
        if values is None or len(values) != self.spec.dim or any(
                not math.isfinite(v) or v < lo - 0.05 or v > hi + 0.05
                for v, lo, hi in zip(values, self.spec.lower, self.spec.upper)):
            return
        self._last_q = tuple(values)
        self._last_state = time.monotonic()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.spec.joints)
        msg.position = list(values)
        self._state_pub.publish(msg)

    def _on_command(self, msg: JointState) -> None:
        if not self._enabled or self._stopped or self._homing:
            return
        stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
        if stamp > 0 and (int(self.get_clock().now().nanoseconds)-stamp)*1e-9 > self._timeout:
            self.get_logger().warning("Expired Nero command rejected", throttle_duration_sec=2.0)
            return
        if list(msg.name) != list(self.spec.joints) or len(msg.position) != self.spec.dim:
            self.get_logger().error("Nero command joint order/dimension mismatch", throttle_duration_sec=2.0)
            return
        values = tuple(float(v) for v in msg.position)
        if any(not math.isfinite(v) or v < lo or v > hi
               for v, lo, hi in zip(values, self.spec.lower, self.spec.upper)):
            self.get_logger().error("Nero command outside joint limits", throttle_duration_sec=2.0)
            return
        with self._lock:
            try:
                if self._robot is not None:
                    self._robot.move_js(list(values))
                self._last_cmd = time.monotonic()
            except Exception as exc:
                self.get_logger().error(f"CAN command failed: {exc}")

    def _watchdog(self):
        if self._enabled and not self._homing and time.monotonic() - self._last_cmd > self._timeout:
            with self._lock:
                self._enabled = False
                if self._robot is not None and self._last_q is not None:
                    try:
                        self._robot.move_js(list(self._last_q))
                    except Exception as exc:
                        self.get_logger().error(f"CAN hold failed: {exc}")
            self.get_logger().error("Nero command watchdog expired; re-enable required")

    def destroy_node(self):
        with self._lock:
            if self._robot is not None:
                self._robot.disconnect()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = NeroDriverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

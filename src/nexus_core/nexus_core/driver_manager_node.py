"""One lifecycle API across the drivers selected by an assembly profile."""

from __future__ import annotations

import json
import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .adapter_registry import DRIVERS
from .profile import Profile


class DriverManagerNode(Node):
    """Expose ready/enable/home/estop once per profile, regardless of hardware.

    ``ready`` is a read-only health check. Adapter enable, home and estop calls
    are fanned out by the registry's lifecycle scope and the operation returns
    only after every selected adapter has replied.
    """

    def __init__(self):
        super().__init__("nexus_driver_manager")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("state_timeout", 0.5)
        self.declare_parameter("service_timeout", 5.0)
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        self._state_timeout = float(self.get_parameter("state_timeout").value)
        self._service_timeout = float(self.get_parameter("service_timeout").value)
        self._group = ReentrantCallbackGroup()
        self._states: dict[str, tuple[list[float], float, str]] = {}
        self._mode = "UNKNOWN"
        self._clients: dict[str, list[tuple[str, object]]] = {
            operation: [] for operation in ("ready", "enable", "home", "estop")
        }
        self._home_targets = 0
        seen_assembly: set[tuple[str, str]] = set()
        ns = self.profile.namespace
        for component in self.profile.components:
            self.create_subscription(
                JointState, self.profile.topic(component.name, "joint_states"),
                lambda msg, name=component.name: self._on_state(name, msg),
                qos_profile_sensor_data)
            adapter = DRIVERS[component.driver]
            for operation in self._clients:
                if operation == "home" and not adapter.supports_home:
                    continue
                if adapter.lifecycle_namespace:
                    key = component.driver, operation
                    if key in seen_assembly:
                        continue
                    seen_assembly.add(key)
                    # Astral's legacy ``ready`` moves the arm to zero. NEXUS
                    # deliberately implements ready as a measured-state check.
                    if operation == "ready":
                        continue
                    service = f"{adapter.lifecycle_namespace}/{operation}"
                    label = component.driver
                else:
                    service = f"{ns}/drivers/{component.name}/{operation}"
                    label = component.name
                self._clients[operation].append((
                    label, self.create_client(Trigger, service, callback_group=self._group)))
                if operation == "home":
                    self._home_targets += 1

        self._control_pub = self.create_publisher(String, f"{ns}/control/cmd", 10)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{ns}/control/state", self._on_control_state, state_qos)
        self._services = []
        for operation in ("ready", "enable", "home", "estop"):
            self._services.append(self.create_service(
                Trigger, f"{ns}/drivers/{operation}",
                lambda request, response, op=operation: self._handle(op, request, response),
                callback_group=self._group))
        self.get_logger().info(f"driver manager profile={self.profile.profile_id} "
                               f"sha256={self.profile.digest} components="
                               f"{[c.name for c in self.profile.components]}")

    def _on_state(self, component: str, msg: JointState) -> None:
        spec = self.profile.component(component)
        names = list(msg.name)
        values = [float(value) for value in msg.position]
        valid = (names == list(spec.joints) and len(values) == spec.dim
                 and all(math.isfinite(value) for value in values)
                 and all(lo - 0.05 <= value <= hi + 0.05
                         for value, lo, hi in zip(values, spec.lower, spec.upper)))
        if valid:
            self._states[component] = values, time.monotonic(), ""
        else:
            self._states[component] = [], time.monotonic(), "joint state name/order/dimension/value invalid"

    def _on_control_state(self, msg: String) -> None:
        try:
            self._mode = str(json.loads(msg.data).get("mode", "UNKNOWN")).upper()
        except (ValueError, TypeError):
            self._mode = "UNKNOWN"

    def _state_faults(self) -> list[str]:
        now = time.monotonic()
        failures = []
        for component in self.profile.components:
            row = self._states.get(component.name)
            if row is None:
                failures.append(f"{component.name}: no feedback")
            elif row[2]:
                failures.append(f"{component.name}: {row[2]}")
            elif now - row[1] > self._state_timeout:
                failures.append(f"{component.name}: stale feedback")
        return failures

    def _call_all(self, operation: str) -> list[str]:
        clients = self._clients[operation]
        for label, client in clients:
            if not client.wait_for_service(timeout_sec=min(0.5, self._service_timeout)):
                return [f"{label}: {operation} service unavailable"]
        futures = [(label, client.call_async(Trigger.Request())) for label, client in clients]
        deadline = time.monotonic() + self._service_timeout
        while futures and time.monotonic() < deadline and not all(f.done() for _, f in futures):
            time.sleep(0.01)
        failures = []
        for label, future in futures:
            if not future.done():
                failures.append(f"{label}: {operation} timed out")
                continue
            try:
                result = future.result()
                if not result.success:
                    failures.append(f"{label}: {result.message or operation + ' failed'}")
            except Exception as exc:
                failures.append(f"{label}: {operation} service error: {exc}")
        return failures

    def _handle(self, operation: str, _request, response):
        if operation != "estop":
            faults = self._state_faults()
            if faults:
                response.success = False
                response.message = "; ".join(faults)
                return response
        if operation in ("enable", "home") and self._mode not in ("IDLE", "PAUSED"):
            response.success = False
            response.message = f"{operation} requires IDLE or PAUSED; current mode is {self._mode}"
            return response
        if operation == "home" and not self._home_targets:
            response.success = False
            response.message = "no configured driver adapter supports homing"
            return response
        if operation == "estop":
            self._control_pub.publish(String(data="ESTOP"))
        failures = self._call_all(operation)
        if operation == "enable" and failures:
            # A partial enable must not leave only some components powered.
            self._control_pub.publish(String(data="ESTOP"))
            self._call_all("estop")
        response.success = not failures
        response.message = (f"{operation} complete" if not failures
                            else f"{operation} failed: {'; '.join(failures)}")
        return response


def main() -> None:
    rclpy.init()
    node = DriverManagerNode()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

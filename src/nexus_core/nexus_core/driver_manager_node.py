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
        self.declare_parameter("home_timeout", 30.0)
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        self._state_timeout = float(self.get_parameter("state_timeout").value)
        self._service_timeout = float(self.get_parameter("service_timeout").value)
        self._home_timeout = float(self.get_parameter("home_timeout").value)
        self._group = ReentrantCallbackGroup()
        self._states: dict[str, tuple[list[float], float, str]] = {}
        self._mode = "UNKNOWN"
        self._operation_clients: dict[str, list[tuple[str, object]]] = {
            operation: [] for operation in ("ready", "enable", "home", "estop")
        }
        self._home_targets = 0
        self._home_feedback_targets: dict[str, tuple[tuple[float, ...], float]] = {}
        seen_assembly: set[tuple[str, str]] = set()
        ns = self.profile.namespace
        for component in self.profile.components:
            self.create_subscription(
                JointState, self.profile.topic(component.name, "joint_states"),
                lambda msg, name=component.name: self._on_state(name, msg),
                qos_profile_sensor_data)
            adapter = DRIVERS[component.driver]
            if adapter.supports_home and adapter.home_target:
                target = adapter.home_target(
                    self.profile.raw, component.kind, component.side, component.dim)
                if target is not None:
                    values = tuple(float(value) for value in target)
                    if len(values) != component.dim or any(
                            not math.isfinite(value) or value < low or value > high
                            for value, low, high in zip(values, component.lower, component.upper)):
                        raise ValueError(f"{component.name}: adapter home target does not match profile")
                    tolerance = float(adapter.home_tolerance)
                    if not math.isfinite(tolerance) or tolerance <= 0:
                        raise ValueError(f"{component.name}: adapter home_tolerance must be positive")
                    self._home_feedback_targets[component.name] = values, tolerance
            for operation in self._operation_clients:
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
                self._operation_clients[operation].append((
                    label, self.create_client(Trigger, service, callback_group=self._group)))
                if operation == "home":
                    self._home_targets += 1

        self._control_pub = self.create_publisher(String, f"{ns}/control/cmd", 10)
        state_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, f"{ns}/control/state", self._on_control_state, state_qos)
        for operation in ("ready", "enable", "home", "estop"):
            self.create_service(
                Trigger, f"{ns}/drivers/{operation}",
                lambda request, response, op=operation: self._handle(op, request, response),
                callback_group=self._group)
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
        clients = self._operation_clients[operation]
        for label, client in clients:
            if not client.wait_for_service(timeout_sec=min(0.5, self._service_timeout)):
                return [f"{label}: {operation} service unavailable"]
        futures = [(label, client.call_async(Trigger.Request())) for label, client in clients]
        # Home is a measured motion, not a short RPC. Its driver may need
        # substantially longer than ready/enable (Nero's default is 20 s).
        budget = self._home_timeout if operation == "home" else self._service_timeout
        deadline = time.monotonic() + budget
        while futures and time.monotonic() < deadline and not all(f.done() for _, f in futures):
            if operation == "home" and self._mode != "HOMING":
                return [f"homing interrupted by {self._mode}"]
            time.sleep(0.01)
        failures = []
        for label, future in futures:
            if not future.done():
                failures.append(f"{label}: {operation} timed out after {budget:.1f}s")
                continue
            try:
                result = future.result()
                if not result.success:
                    failures.append(f"{label}: {result.message or operation + ' failed'}")
            except Exception as exc:
                failures.append(f"{label}: {operation} service error: {exc}")
        return failures

    def _wait_home_feedback(self) -> list[str]:
        if not self._home_feedback_targets:
            return []
        deadline = time.monotonic() + self._home_timeout
        settled_since = None
        while time.monotonic() < deadline:
            if self._mode != "HOMING":
                return [f"home feedback verification interrupted by {self._mode}"]
            now = time.monotonic()
            reached = True
            for component, (target, tolerance) in self._home_feedback_targets.items():
                state = self._states.get(component)
                if (state is None or state[2] or now - state[1] > self._state_timeout
                        or len(state[0]) != len(target)
                        or max(abs(value - goal) for value, goal in zip(state[0], target)) > tolerance):
                    reached = False
                    break
            if reached:
                if settled_since is None:
                    settled_since = now
                elif now - settled_since >= 0.3:
                    return []
            else:
                settled_since = None
            time.sleep(0.02)
        details = []
        for component, (target, _tolerance) in self._home_feedback_targets.items():
            state = self._states.get(component)
            if state is None or not state[0]:
                details.append(f"{component}: no measured feedback")
            else:
                error = max(abs(value - goal) for value, goal in zip(state[0], target))
                details.append(f"{component}: home error {error:.3f} rad")
        return [f"home target not reached within {self._home_timeout:.1f}s ({'; '.join(details)})"]

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
        homing_mode_entered = False
        if operation == "home":
            self._control_pub.publish(String(data="HOMING"))
            deadline = time.monotonic() + 1.0
            while self._mode != "HOMING" and time.monotonic() < deadline:
                time.sleep(0.01)
            if self._mode != "HOMING":
                response.success = False
                response.message = "command mux did not enter HOMING; driver output remains unchanged"
                return response
            homing_mode_entered = True

        failures = self._call_all(operation)
        if operation == "home" and not failures:
            failures.extend(self._wait_home_feedback())
        if homing_mode_entered and failures:
            # A timed-out service can still be moving. Never resume stale
            # measured hold targets while an adapter has not finished.
            self._control_pub.publish(String(data="ESTOP"))
            failures.extend(self._call_all("estop"))
        elif homing_mode_entered:
            self._control_pub.publish(String(data="IDLE"))
            deadline = time.monotonic() + 1.0
            while self._mode != "IDLE" and time.monotonic() < deadline:
                time.sleep(0.01)
            if self._mode != "IDLE":
                failures.append("command mux did not return to IDLE after homing")
                self._control_pub.publish(String(data="ESTOP"))
                failures.extend(self._call_all("estop"))
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

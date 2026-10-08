"""Standalone Nero CAN driver. It never subscribes to VR or runs IK."""

from __future__ import annotations

import math
import json
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.task import Future
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .profile import Profile


class NeroDriverNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__("nexus_nero_driver", **node_kwargs)
        self.declare_parameter("profile_file", "")
        self.declare_parameter("side", "left")
        self.declare_parameter("component", "")
        self.declare_parameter("dry_run", False)
        self.declare_parameter("state_rate", 20.0)
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("home_timeout", 20.0)
        self.declare_parameter("home_speed_percent", 10)
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
        self._home_future = None
        self._home_response = None
        self._home_target = None
        self._home_settled_since = None
        self._home_settle_reference = None
        self._home_deadline = 0.0
        self._home_next_log = 0.0
        self._last_home_diagnostic = None
        self._control_mode = "UNKNOWN"
        self._global_home_deadline = 0.0
        self._command_not_before_ns = 0
        self._home_timeout = float(self.get_parameter("home_timeout").value)
        self._home_speed = int(self.get_parameter("home_speed_percent").value)
        if not math.isfinite(self._home_timeout) or self._home_timeout <= 0 or not 1 <= self._home_speed <= 100:
            raise ValueError("invalid Nero home timeout/speed")
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
        self.create_service(Trigger, f"{prefix}/diagnostics", self._diagnostics)
        self.create_service(Trigger, f"{prefix}/enable", self._enable)
        # An awaiting home service must not hold the callback group used by
        # feedback, the command watchdog or emergency stop. Keep one executor.
        self._home_group = ReentrantCallbackGroup()
        self.create_service(Trigger, f"{prefix}/home", self._home, callback_group=self._home_group)
        self.create_service(Trigger, f"{prefix}/estop", self._estop)
        self.create_subscription(String, f"{self.profile.namespace}/control/state", self._on_control,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
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

    def _diagnostic_snapshot(self):
        """Read SDK caches only; never request data or send a CAN command."""
        target = self._home_target or tuple(
            self.profile.adapter_config("nero_can")["home_pose"][self.side])
        rows = []
        if self._last_q is not None:
            for name, actual, goal in zip(self.spec.joints, self._last_q, target):
                error = abs(actual - goal)
                rows.append({"name": name, "actual_rad": actual, "target_rad": goal,
                             "error_rad": error, "error_deg": math.degrees(error)})
        snapshot = {"component": self.spec.name, "enabled": self._enabled,
                    "homing": self._homing, "feedback_age_s": time.monotonic() - self._last_state,
                    "tolerance_rad": 0.05, "joints": rows,
                    "pending_joints": [row["name"] for row in rows if row["error_rad"] >= 0.05]}
        if self._robot is None:
            return snapshot
        try:
            record = self._robot.get_arm_status()
            if record is not None:
                snapshot["controller"] = {
                    "feedback_age_s": time.time() - float(record.timestamp),
                    **{field: int(getattr(record.msg, field)) for field in
                       ("ctrl_mode", "arm_status", "mode_feedback", "motion_status", "err_code")}}
        except Exception as exc:
            snapshot["controller_unavailable"] = str(exc)
        snapshot["drivers"] = []
        for index, name in enumerate(self.spec.joints, 1):
            try:
                record = self._robot.get_driver_states(index)
                if record is None:
                    snapshot["drivers"].append({"name": name, "available": False})
                    continue
                flags = record.msg.foc_status
                snapshot["drivers"].append({
                    "name": name, "feedback_age_s": time.time() - float(record.timestamp),
                    **{field: bool(getattr(flags, field)) for field in
                       ("driver_enable_status", "driver_error_status", "collision_status", "stall_status")}})
            except Exception as exc:
                snapshot["drivers"].append({"name": name, "unavailable": str(exc)})
        return snapshot

    def _diagnostics(self, _request, response):
        snapshot = self._diagnostic_snapshot()
        # Preserve the state before failure disables motors. Reading only the
        # post-stop state can incorrectly suggest the brake caused the failure.
        snapshot["last_home_failure_before_stop"] = self._last_home_diagnostic
        return self._response(response, True, json.dumps(snapshot, ensure_ascii=False))

    def _enable(self, _request, response):
        if self._homing:
            return self._response(response, False, "home already in progress")
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

    async def _home(self, _request, response):
        if self._homing:
            return self._response(response, False, "home already in progress")
        if not self._enabled or self._stopped or self._last_q is None or time.monotonic() - self._last_state >= 0.5:
            return self._response(response, False, "enable and read feedback first")
        target = tuple(float(v) for v in self.profile.adapter_config("nero_can")["home_pose"][self.side])
        if len(target) != self.spec.dim or any(not math.isfinite(v) or v < lo or v > hi
                                               for v, lo, hi in zip(target, self.spec.lower, self.spec.upper)):
            return self._response(response, False, "profile home pose is outside joint limits")
        if self.dry_run:
            self._last_q = target
            self._last_cmd = time.monotonic()
            self._command_not_before_ns = self.get_clock().now().nanoseconds
            return self._response(response, True, "dry-run home pose set")
        self._homing = True
        self._home_target = target
        self._home_settled_since = None
        self._home_settle_reference = None
        self._home_deadline = time.monotonic() + self._home_timeout
        self._home_next_log = 0.0
        self._last_home_diagnostic = None
        self._home_future = Future()
        self._home_response = response
        future = self._home_future
        self._command_not_before_ns = self.get_clock().now().nanoseconds
        try:
            with self._lock:
                self._robot.set_motion_mode("j")
                self._robot.set_auto_set_motion_mode_enabled(False)
                self._robot.set_speed_percent(self._home_speed)
                self._robot.move_j(list(target))
            self.get_logger().info(f"Nero home started target_rad={list(target)} speed={self._home_speed}% timeout={self._home_timeout:.1f}s")
        except Exception as exc:
            self._finish_home(False, f"SDK error: {exc}")
        # Feedback timers continue publishing throughout the motion. They
        # resolve this future after measured arrival, timeout or emergency stop.
        return await future

    def _finish_home(self, success: bool, reason: str) -> None:
        if not self._homing:
            return
        if not success:
            self._last_home_diagnostic = self._diagnostic_snapshot()
            pending = ", ".join(
                f"{row['name']}: {row['error_rad']:.4f}rad/{row['error_deg']:.2f}deg"
                for row in self._last_home_diagnostic["joints"] if row["error_rad"] >= 0.05)
            if pending:
                reason += f"; pending joints [{pending}]"
            self.get_logger().error("Nero home diagnostic before stop: " + json.dumps(
                self._last_home_diagnostic, ensure_ascii=False))
        with self._lock:
            try:
                if success:
                    self._robot.set_motion_mode("js")
                    self._robot.set_auto_set_motion_mode_enabled(False)
                    self._robot.set_speed_percent(100)
                else:
                    self._enabled = False
                    self._robot.disable()
            except Exception as exc:
                success, reason = False, f"{reason}; SDK transition error: {exc}"
                self._enabled = False
                try:
                    self._robot.disable()
                except Exception:
                    pass
            self._homing = False
            self._last_cmd = time.monotonic()
            self._command_not_before_ns = self.get_clock().now().nanoseconds
            future, response = self._home_future, self._home_response
            self._home_future = self._home_response = None
        message = f"Nero home reached ({reason})" if success else f"Nero home failed: {reason}; motors disabled"
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        if future is not None and not future.done():
            future.set_result(self._response(response, success, message))

    def _advance_home(self) -> None:
        if not self._homing:
            return
        now = time.monotonic()
        error = max(abs(a - b) for a, b in zip(self._last_q, self._home_target))
        if now >= self._home_deadline:
            self._finish_home(False, f"timeout after {self._home_timeout:.1f}s; max_joint_error={error:.4f} rad actual={list(self._last_q)} target={list(self._home_target)}")
        elif now - self._last_state >= 0.5:
            self._finish_home(False, "measured feedback stale during home")
        elif error >= 0.05:
            self._home_settled_since = None
            self._home_settle_reference = None
        elif (self._home_settle_reference is None
              or max(abs(a - b) for a, b in zip(self._last_q, self._home_settle_reference)) > 0.002):
            self._home_settled_since = now
            self._home_settle_reference = self._last_q
        elif now - self._home_settled_since >= 0.3:
            self._finish_home(True, f"max_joint_error={error:.4f} rad; settled=0.3s drift<=0.002rad")
        if self._homing and now >= self._home_next_log:
            self._home_next_log = now + 2.0
            self.get_logger().info("Nero home progress: " + json.dumps(
                self._diagnostic_snapshot(), ensure_ascii=False))

    def _on_control(self, msg: String) -> None:
        try:
            mode = str(json.loads(msg.data)["mode"]).upper()
        except (ValueError, KeyError, TypeError):
            return
        previous, self._control_mode = self._control_mode, mode
        if mode == "HOMING" and previous != mode:
            # An arm that arrives first must keep holding while the other arm
            # is still homing. Bound this lifecycle wait independently.
            self._global_home_deadline = time.monotonic() + self._home_timeout + 15.0
        elif previous == "HOMING" and mode != previous:
            self._last_cmd = time.monotonic()
            self._command_not_before_ns = self.get_clock().now().nanoseconds
            if self._homing:
                self._finish_home(False, f"global homing interrupted by {mode}")

    def _estop(self, _request, response):
        with self._lock:
            self._stopped = True
            self._enabled = False
            if self._homing:
                self._finish_home(False, "emergency stop")
            elif self._robot is not None:
                self._robot.disable()
        return self._response(response, True, "CAN motors disabled")

    def _poll_state(self):
        with self._lock:
            try:
                if self.dry_run:
                    values = self._last_q
                else:
                    record = self._robot.get_joint_angles()
                    if record is not None and (
                            not math.isfinite(float(record.timestamp))
                            or time.time() - float(record.timestamp) >= 0.5
                            or float(record.timestamp) - time.time() > 0.1):
                        self.get_logger().warning("Stale Nero SDK feedback rejected", throttle_duration_sec=2.0)
                        return
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
        self._advance_home()

    def _on_command(self, msg: JointState) -> None:
        if not self._enabled or self._stopped or self._homing or self._control_mode == "HOMING":
            return
        stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._command_not_before_ns and stamp <= self._command_not_before_ns:
            return
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
        if self._homing:
            self._advance_home()
            return
        if (self._control_mode == "HOMING" and time.monotonic() < self._global_home_deadline
                and time.monotonic() - self._last_state < 0.5):
            return
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
            if self._homing:
                self._finish_home(False, "driver shutting down")
            if self._robot is not None:
                self._robot.disconnect()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = NeroDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

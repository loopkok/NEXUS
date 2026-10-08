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
        self.declare_parameter("home_mode_timeout", 1.0)
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
        self._home_start_q = None
        self._home_q_min = None
        self._home_q_max = None
        self._home_settled_since = None
        self._home_settle_reference = None
        self._home_deadline = 0.0
        self._home_next_log = 0.0
        self._home_phase = "IDLE"
        self._home_mode_requested_wall = 0.0
        self._home_mode_requested_at = 0.0
        self._home_mode_written_at = 0.0
        self._home_mode_deadline = 0.0
        self._home_mode_feedback = None
        self._home_command_attempts = 0
        self._home_command_sent = False
        self._last_home_diagnostic = None
        self._control_mode = "UNKNOWN"
        self._global_home_deadline = 0.0
        self._command_not_before_ns = 0
        self._home_timeout = float(self.get_parameter("home_timeout").value)
        self._home_mode_timeout = float(self.get_parameter("home_mode_timeout").value)
        self._home_speed = int(self.get_parameter("home_speed_percent").value)
        if not math.isfinite(self._home_timeout) or self._home_timeout <= 0 or not 1 <= self._home_speed <= 100:
            raise ValueError("invalid Nero home timeout/speed")
        if not math.isfinite(self._home_mode_timeout) or self._home_mode_timeout <= 0:
            raise ValueError("invalid Nero home mode timeout")
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
            for index, (name, actual, goal) in enumerate(zip(self.spec.joints, self._last_q, target)):
                error = abs(actual - goal)
                row = {"name": name, "actual_rad": actual, "target_rad": goal,
                       "error_rad": error, "error_deg": math.degrees(error)}
                if self._home_start_q is not None:
                    start = self._home_start_q[index]
                    row.update(start_rad=start, displacement_rad=actual-start,
                               observed_range_rad=self._home_q_max[index]-self._home_q_min[index],
                               error_reduction_rad=abs(start-goal)-error)
                rows.append(row)
        snapshot = {"component": self.spec.name, "enabled": self._enabled,
                    "homing": self._homing, "feedback_age_s": time.monotonic() - self._last_state,
                    "home_phase": self._home_phase,
                    "home_command_attempts": self._home_command_attempts,
                    "home_command_sent": self._home_command_sent,
                    "home_mode_feedback": self._home_mode_feedback,
                    "tolerance_rad": 0.05, "joints": rows,
                    "pending_joints": [row["name"] for row in rows if row["error_rad"] >= 0.05],
                    # This describes observed motion, not a diagnosis of a
                    # motor, brake, payload or controller fault.
                    "minimal_motion_pending_joints": [row["name"] for row in rows
                        if row["error_rad"] >= 0.05 and "observed_range_rad" in row
                        and row["observed_range_rad"] <= 0.002]}
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
        snapshot["motors"] = []
        for index, name in enumerate(self.spec.joints, 1):
            try:
                record = self._robot.get_motor_states(index)
                if record is None:
                    snapshot["motors"].append({"name": name, "available": False})
                    continue
                age = time.time() - float(record.timestamp)
                current, velocity = float(record.msg.current), float(record.msg.velocity)
                if not all(math.isfinite(value) for value in (age, current, velocity)):
                    raise ValueError("non-finite motor cache value")
                snapshot["motors"].append({
                    "name": name, "feedback_age_s": age, "fresh": -0.1 <= age < 0.5,
                    "current_a_sdk": current, "velocity_rad_s_sdk": velocity})
            except Exception as exc:
                snapshot["motors"].append({"name": name, "unavailable": str(exc)})
        # Firmware <=1.10 has version-dependent current signs and invalid
        # velocity feedback. Never infer joint motion or torque from these
        # values; use the measured joint position trace above for motion.
        snapshot["motor_feedback_note"] = "SDK cache values; firmware-dependent current sign/velocity; not calibrated torque"
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
        self._home_start_q = self._last_q
        self._home_q_min = list(self._last_q)
        self._home_q_max = list(self._last_q)
        self._home_settled_since = None
        self._home_settle_reference = None
        self._home_deadline = time.monotonic() + self._home_timeout
        self._home_phase = "WAIT_J_MODE"
        self._home_mode_feedback = None
        self._home_command_attempts = 0
        self._home_command_sent = False
        self._home_next_log = 0.0
        self._last_home_diagnostic = None
        self._home_future = Future()
        self._home_response = response
        future = self._home_future
        self._command_not_before_ns = self.get_clock().now().nanoseconds
        try:
            with self._lock:
                self._home_mode_requested_wall = time.time()
                self._home_mode_requested_at = time.monotonic()
                self._home_mode_deadline = min(
                    self._home_deadline, self._home_mode_requested_at + self._home_mode_timeout)
                self._robot.set_motion_mode("j")
                self._robot.set_auto_set_motion_mode_enabled(False)
                self._robot.set_speed_percent(self._home_speed)
                self._home_mode_written_at = time.monotonic()
            self.get_logger().info(
                f"Nero home waiting for fresh CAN/J feedback before target; "
                f"mode_timeout={self._home_mode_timeout:.2f}s target_rad={list(target)} "
                f"speed={self._home_speed}% timeout={self._home_timeout:.1f}s")
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
            minimal_motion = self._last_home_diagnostic["minimal_motion_pending_joints"]
            if minimal_motion:
                reason += f"; minimal measured motion during home: {minimal_motion} (cause undetermined)"
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
            self._home_phase = "IDLE"
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

    def _advance_home_mode(self, now: float) -> None:
        """Wait asynchronously; a pre-switch cached J status cannot open this gate."""
        if now >= self._home_mode_deadline:
            self._finish_home(False,
                f"CAN/J mode confirmation timed out after {self._home_mode_timeout:.2f}s; "
                f"no home target sent; mode_feedback={self._home_mode_feedback}")
            return
        try:
            record = self._robot.get_arm_status()
            if record is None:
                self._home_mode_feedback = {"unavailable": "no controller feedback"}
                return
            stamp = float(record.timestamp)
            age = time.time() - stamp
            if not math.isfinite(stamp) or not math.isfinite(age):
                raise ValueError("non-finite controller timestamp")
            status = {field: int(getattr(record.msg, field)) for field in
                      ("ctrl_mode", "arm_status", "mode_feedback", "motion_status", "err_code")}
            fresh = stamp >= self._home_mode_requested_wall and -0.1 <= age < 0.25
            self._home_mode_feedback = {**status, "feedback_age_s": age,
                                       "after_mode_request": stamp >= self._home_mode_requested_wall,
                                       "wait_elapsed_s": now - self._home_mode_requested_at}
        except Exception as exc:
            self._home_mode_feedback = {"unavailable": str(exc)}
            return
        if not fresh:
            return
        if status["arm_status"] != 0 or status["err_code"] != 0:
            self._finish_home(False, f"controller not healthy before home target: {status}")
            return
        # Match the successful standalone probe's >=20 ms mode-to-target gap.
        # mode_feedback=1 reports J, but does not acknowledge every 0x151 field
        # (in particular the JS/MIT flag). Do not treat this as a full CAN ACK.
        if (status["ctrl_mode"] != 1 or status["mode_feedback"] != 1
                or status["motion_status"] != 0 or now - self._home_mode_written_at < 0.02):
            return
        try:
            with self._lock:
                self._home_phase = "SUBMITTING_TARGET"
                self._home_command_attempts += 1
                self._robot.move_j(list(self._home_target))
                self._home_command_sent = True
                self._home_phase = "MOVING"
            self.get_logger().info(
                "Nero home target submitted once after fresh CAN/J feedback: "
                + json.dumps(self._home_mode_feedback, ensure_ascii=False))
        except Exception as exc:
            # A partially transmitted multi-frame goal must never be retried.
            self._finish_home(False, f"SDK home target error (no retry): {exc}")

    def _advance_home(self) -> None:
        if not self._homing:
            return
        now = time.monotonic()
        error = max(abs(a - b) for a, b in zip(self._last_q, self._home_target))
        if now >= self._home_deadline:
            self._finish_home(False, f"timeout after {self._home_timeout:.1f}s; max_joint_error={error:.4f} rad actual={list(self._last_q)} target={list(self._home_target)}")
        elif now - self._last_state >= 0.5:
            self._finish_home(False, "measured feedback stale during home")
        if not self._homing:
            return
        if self._home_phase == "WAIT_J_MODE":
            self._advance_home_mode(now)
        if not self._homing or self._home_phase != "MOVING":
            return
        if error >= 0.05:
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
        if self._homing:
            self._home_q_min = [min(a, b) for a, b in zip(self._home_q_min, values)]
            self._home_q_max = [max(a, b) for a, b in zip(self._home_q_max, values)]
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

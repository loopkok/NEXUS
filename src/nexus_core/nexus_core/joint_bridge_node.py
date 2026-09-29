"""Hardware-specific conversions at the boundary of canonical JointState topics."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from .profile import Profile

# Candidate commands are latest-value control data: avoid reliable DDS
# backpressure and let the mux stale-command watchdog reject a stalled stream.
_CANDIDATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


class JointBridgeNode(Node):
    def __init__(self):
        super().__init__("nexus_joint_bridge")
        self.declare_parameter("profile_file", "")
        self.declare_parameter("component", "")
        self.declare_parameter("adapter_mode", "")
        self.declare_parameter("side", "")
        self.declare_parameter("candidate_only", False)
        self.profile = Profile.load(str(self.get_parameter("profile_file").value))
        self.component = str(self.get_parameter("component").value)
        self.side = str(self.get_parameter("side").value)
        self.candidate_only = bool(self.get_parameter("candidate_only").value)
        self.spec = self.profile.component(self.component)
        self._enabled = False
        self._estopped = False
        self._last_feedback = 0.0
        self._hand_id = None
        self._wuji_client = None
        self._service_group = ReentrantCallbackGroup()
        mode = str(self.get_parameter("adapter_mode").value)
        if mode == "xhand":
            self._setup_xhand()
        elif mode == "wuji":
            self._setup_wuji()
        elif mode == "astral_gripper":
            self._setup_gripper()
        else:
            raise ValueError(f"unsupported adapter_mode {mode}")
        if mode in ("xhand", "wuji") and not self.candidate_only:
            prefix = f"{self.profile.namespace}/drivers/{self.component}"
            self.create_service(Trigger, f"{prefix}/ready", self._ready,
                                callback_group=self._service_group)
            self.create_service(Trigger, f"{prefix}/enable", self._enable,
                                callback_group=self._service_group)
            self.create_service(Trigger, f"{prefix}/home", self._home,
                                callback_group=self._service_group)
            self.create_service(Trigger, f"{prefix}/estop", self._estop,
                                callback_group=self._service_group)
        self.get_logger().info(f"joint bridge {mode} component={self.component}")

    def _ready(self, _request, response):
        response.success = time.monotonic() - self._last_feedback < 0.5
        response.message = "fresh measured feedback" if response.success else "measured feedback unavailable"
        return response

    def _enable(self, _request, response):
        response.success = not self._estopped and time.monotonic() - self._last_feedback < 0.5
        if response.success and self._wuji_client is not None:
            response.success, response.message = self._wuji_set_enabled(True)
            if not response.success:
                self._enabled = False
                return response
        self._enabled = response.success
        response.message = "driver enabled" if response.success else "estop latched or feedback stale"
        return response

    def _home(self, _request, response):
        response.success = False
        response.message = "hand home is unsupported; use a validated hardware procedure"
        return response

    def _estop(self, _request, response):
        self._enabled = False
        self._estopped = True
        if self._wuji_client is not None:
            response.success, native_message = self._wuji_set_enabled(False)
            response.message = f"software gate latched; Wuji disable: {native_message}"
        else:
            response.success = True
            response.message = "software command gate latched; reset physical driver separately"
        return response

    def _wuji_set_enabled(self, enabled: bool) -> tuple[bool, str]:
        from wujihand_msgs.srv import SetEnabled

        if not self._wuji_client.wait_for_service(timeout_sec=0.5):
            return False, "native set_enabled service unavailable"
        request = SetEnabled.Request()
        request.finger_id = 255
        request.joint_id = 255
        request.enabled = enabled
        done = threading.Event()
        future = self._wuji_client.call_async(request)
        future.add_done_callback(lambda _future: done.set())
        if not done.wait(2.0):
            return False, "native set_enabled timed out"
        try:
            result = future.result()
            return bool(result.success), result.message
        except Exception as exc:
            return False, str(exc)

    def _joint_msg(self, values: list[float]) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.spec.joints)
        msg.position = values
        return msg

    def _setup_xhand(self) -> None:
        from xhand_control_interfaces.msg import XHandCommand, XHandStateArray

        native_names = [name.removeprefix(f"{self.side}_hand_") for name in self.spec.joints]
        native_command_topic = f"/{self.side}_hand/xhand_command"
        native_candidate_topic = f"{self.profile.namespace}/legacy/{self.side}_xhand_candidate"
        native_state_topic = f"/{self.side}_hand/xhand_state"
        candidate_pub = self.create_publisher(
            JointState, self.profile.candidate_topic("teleop", self.component), _CANDIDATE_QOS)
        state_pub = self.create_publisher(
            JointState, self.profile.topic(self.component, "joint_states"), qos_profile_sensor_data)
        driver_pub = self.create_publisher(XHandCommand, native_command_topic, qos_profile_sensor_data)
        rate_window = time.monotonic()
        received_count = 0
        forwarded_count = 0
        invalid_count = 0
        max_callback_gap = 0.0
        max_publish_duration = 0.0
        last_callback_time = 0.0

        def native_candidate(msg: XHandCommand) -> None:
            nonlocal rate_window, received_count, forwarded_count, invalid_count
            nonlocal max_callback_gap, max_publish_duration, last_callback_time
            callback_time = time.monotonic()
            if last_callback_time:
                max_callback_gap = max(max_callback_gap, callback_time - last_callback_time)
            last_callback_time = callback_time
            received_count += 1
            if len(msg.position) != self.spec.dim or set(msg.name) != set(native_names):
                invalid_count += 1
                self.get_logger().error("XHand retarget command names/dimension mismatch", throttle_duration_sec=2.0)
                return
            lut = dict(zip(msg.name, msg.position))
            publish_start = time.monotonic()
            candidate_pub.publish(self._joint_msg([float(lut[n]) for n in native_names]))
            max_publish_duration = max(max_publish_duration, time.monotonic() - publish_start)
            forwarded_count += 1
            now = time.monotonic()
            elapsed = now - rate_window
            if elapsed >= 5.0:
                self.get_logger().info(
                    f"XHand candidate bridge component={self.component} "
                    f"forwarded_hz={forwarded_count / elapsed:.1f} "
                    f"received={received_count} invalid={invalid_count} "
                    f"max_callback_gap_ms={max_callback_gap * 1000.0:.1f} "
                    f"max_publish_ms={max_publish_duration * 1000.0:.1f} "
                    f"in {elapsed:.1f}s")
                rate_window = now
                received_count = 0
                forwarded_count = 0
                invalid_count = 0
                max_callback_gap = 0.0
                max_publish_duration = 0.0

        def native_state(msg: XHandStateArray) -> None:
            if len(msg.hand_states) != 1 or len(msg.hand_id) != 1:
                self.get_logger().error("XHand serial adapter requires exactly one device", throttle_duration_sec=2.0)
                return
            state = msg.hand_states[0]
            if len(state.position) != self.spec.dim or set(state.name) != set(native_names):
                self.get_logger().error("XHand measured state names/dimension mismatch", throttle_duration_sec=2.0)
                return
            lut = dict(zip(state.name, state.position))
            output = self._joint_msg([float(lut[n]) for n in native_names])
            if any(not math.isfinite(v) or v < lo - 0.05 or v > hi + 0.05
                   for v, lo, hi in zip(output.position, self.spec.lower, self.spec.upper)):
                self.get_logger().error("XHand measured state outside limits", throttle_duration_sec=2.0)
                return
            output.header = msg.header
            self._hand_id = int(msg.hand_id[0])
            self._last_feedback = time.monotonic()
            state_pub.publish(output)

        def final_command(msg: JointState) -> None:
            if not self._enabled or self._estopped or self._hand_id is None or time.monotonic() - self._last_feedback >= 0.5:
                return
            if len(msg.position) != self.spec.dim or list(msg.name) != list(self.spec.joints):
                self.get_logger().error("XHand final command mismatch", throttle_duration_sec=2.0)
                return
            if any(not math.isfinite(v) or v < lo or v > hi
                   for v, lo, hi in zip(msg.position, self.spec.lower, self.spec.upper)):
                return
            native = XHandCommand()
            native.hand_id = self._hand_id
            native.name = native_names
            native.position = [float(v) for v in msg.position]
            native.kp = [80.0] * self.spec.dim
            native.ki = [0.0] * self.spec.dim
            native.kd = [0.0] * self.spec.dim
            native.effort_limit = [400.0] * self.spec.dim
            native.mode = 3
            driver_pub.publish(native)

        # Treat the vendor command stream like any other live sensor stream.
        # A small best-effort history avoids DDS acknowledgement stalls while
        # preserving the mux's independent stale-command safety gate.
        self.create_subscription(XHandCommand, native_candidate_topic,
                                 native_candidate, qos_profile_sensor_data)
        if not self.candidate_only:
            self.create_subscription(XHandStateArray, native_state_topic,
                                     native_state, qos_profile_sensor_data)
            self.create_subscription(JointState, self.profile.topic(self.component, "joint_commands"),
                                     final_command, qos_profile_sensor_data)

    def _setup_gripper(self) -> None:
        if self.spec.kind != "gripper" or self.spec.dim != 1:
            raise ValueError("astral_gripper bridge requires one gripper joint")
        opened, closed = self.spec.upper[0], self.spec.lower[0]
        for source in ("teleop", "policy", "playback"):
            candidate_pub = self.create_publisher(
                JointState, self.profile.candidate_topic(source, self.component),
                _CANDIDATE_QOS)

            def on_ratio(msg: Float64, pub=candidate_pub) -> None:
                ratio = float(msg.data)
                if not math.isfinite(ratio):
                    return
                angle = opened * (1.0 - min(1.0, max(0.0, ratio))) + closed * min(1.0, max(0.0, ratio))
                pub.publish(self._joint_msg([angle]))

            self.create_subscription(
                Float64, f"{self.profile.namespace}/legacy/{source}/{self.side}_gripper_ratio",
                on_ratio, _CANDIDATE_QOS)

    def _setup_wuji(self) -> None:
        if self.spec.driver != "wuji_serial" or self.spec.dim != 20:
            raise ValueError("wuji bridge requires a 20 joint Wuji component")
        names = list(self.spec.joints)
        native = [n.removeprefix(f"{self.side}_") for n in names]
        candidate_pub = self.create_publisher(
            JointState, self.profile.candidate_topic("teleop", self.component), _CANDIDATE_QOS)
        state_pub = self.create_publisher(
            JointState, self.profile.topic(self.component, "joint_states"), qos_profile_sensor_data)
        driver_pub = self.create_publisher(
            JointState, f"/{self.side}_hand/joint_commands", qos_profile_sensor_data)
        if not self.candidate_only:
            from wujihand_msgs.srv import SetEnabled
            self._wuji_client = self.create_client(
                SetEnabled, f"/{self.side}_hand/set_enabled",
                callback_group=self._service_group)

        def retarget(msg: JointState) -> None:
            if len(msg.position) != self.spec.dim or set(msg.name) != set(native):
                self.get_logger().error("Wuji retarget command mismatch", throttle_duration_sec=2.0)
                return
            lookup = dict(zip(msg.name, msg.position))
            candidate_pub.publish(self._joint_msg([float(lookup[n]) for n in native]))

        def feedback(msg: JointState) -> None:
            if len(msg.position) != self.spec.dim or set(msg.name) != set(names):
                self.get_logger().error("Wuji measured state mismatch", throttle_duration_sec=2.0)
                return
            lookup = dict(zip(msg.name, msg.position))
            out = self._joint_msg([float(lookup[n]) for n in names])
            if any(not math.isfinite(v) or v < lo - 0.05 or v > hi + 0.05
                   for v, lo, hi in zip(out.position, self.spec.lower, self.spec.upper)):
                return
            out.header = msg.header
            self._last_feedback = time.monotonic()
            state_pub.publish(out)

        def final_command(msg: JointState) -> None:
            if not self._enabled or self._estopped or time.monotonic() - self._last_feedback >= 0.5:
                return
            if list(msg.name) != names or len(msg.position) != self.spec.dim:
                return
            if any(not math.isfinite(v) or v < lo or v > hi
                   for v, lo, hi in zip(msg.position, self.spec.lower, self.spec.upper)):
                return
            driver_pub.publish(msg)

        self.create_subscription(JointState, f"{self.profile.namespace}/legacy/{self.side}_wuji_candidate",
                                 retarget, _CANDIDATE_QOS)
        if not self.candidate_only:
            self.create_subscription(JointState, f"/{self.side}_hand/joint_states",
                                     feedback, qos_profile_sensor_data)
            self.create_subscription(JointState, self.profile.topic(self.component, "joint_commands"),
                                     final_command, qos_profile_sensor_data)


def main() -> None:
    rclpy.init()
    node = JointBridgeNode()
    # The ratio-only Astral gripper bridge has no lifecycle services. On ROS 2
    # Humble, its small callbacks can be starved by MultiThreadedExecutor;
    # use the same single-threaded path as candidate-only bridges.
    if node.candidate_only or node.spec.kind == "gripper":
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()

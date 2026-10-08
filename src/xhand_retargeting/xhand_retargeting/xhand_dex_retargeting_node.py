#!/usr/bin/env python3

import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from dex_retargeting.retargeting_config import RetargetingConfig
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray


class FPSCounter:
    """Simple sliding-window FPS counter."""
    def __init__(self, window=100, print_interval=5.0):
        self._window = window
        self._print_interval = print_interval
        self._times = []
        self._last_print = time.time()
    def tick(self):
        now = time.time()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times.pop(0)
        if len(self._times) < 2:
            return 0.0
        return (len(self._times) - 1) / max(1e-9, self._times[-1] - self._times[0])
    def should_print(self):
        now = time.time()
        if now - self._last_print >= self._print_interval:
            self._last_print = now
            return True
        return False
from std_srvs.srv import Trigger
from xhand_control_interfaces.msg import XHandCommand


XHAND_JOINT_NAMES = [
    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
    "index_bend_joint", "index_joint1", "index_joint2",
    "mid_joint1", "mid_joint2",
    "ring_joint1", "ring_joint2",
    "pinky_joint1", "pinky_joint2",
]

URDF_JOINT_PREFIX = {
    "right": "right_hand_",
    "left": "left_hand_",
}

# XHand control parameters (from XHandConfig / xhand_examples)
XHAND_KP = 80.0
XHAND_KI = 0.0
XHAND_KD = 0.0
XHAND_EFFORT_LIMIT = 400.0  # mA
XHAND_MODE = 3

# Home position in degrees (12 joints), converted to radians at runtime
XHAND_HOME_POSITION_DEG = (
    0.0, 80.66, 33.2,   # thumb: bend, rota1, rota2
    0.0, 5.11, 5.0,     # index: bend, joint1, joint2
    6.53, 5.0,          # mid: joint1, joint2
    6.76, 5.0,          # ring: joint1, joint2
    10.13, 5.0,         # pinky: joint1, joint2
)


class XHandDexRetargetingNode(Node):
    def __init__(self):
        super().__init__("xhand_dex_retargeting_node")

        self.declare_parameter("input_topic", "hand_landmarks")
        self.declare_parameter("viz", True)
        self.declare_parameter("smoothing_alpha", 0.7)
        self.declare_parameter("enable_thumb_fix", True)

        input_topic = self.get_parameter("input_topic").value
        self.viz = self.get_parameter("viz").value
        self.smoothing_alpha = self.get_parameter("smoothing_alpha").value
        self.enable_thumb_fix = self.get_parameter("enable_thumb_fix").value

        # 订阅左右手 topic（both 模式各发各的，单手模式共用 hand_landmarks）
        self.create_subscription(PoseArray, f"{input_topic}/right", self.pose_callback_right,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseArray, f"{input_topic}/left", self.pose_callback_left,
                                 qos_profile_sensor_data)

        self.right_retargeter = self.make_config("right").build()
        self.left_retargeter = self.make_config("left").build()

        self.right_joint_control_pub = self.create_publisher(
            XHandCommand, "/right_hand/xhand_command", 10
        )
        self.left_joint_control_pub = self.create_publisher(
            XHandCommand, "/left_hand/xhand_command", 10
        )

        # Post-retargeting joint-level smoothing state
        self.last_joint_right = None
        self.last_joint_left = None

        # Home position in radians
        self.home_position_rad = [
            math.radians(deg) for deg in XHAND_HOME_POSITION_DEG
        ]

        # ---- Latency metrics publishers + rolling stats ----
        self.right_metrics_pub = self.create_publisher(
            Float64MultiArray, "~/metrics/right_hand", 10
        )
        self.left_metrics_pub = self.create_publisher(
            Float64MultiArray, "~/metrics/left_hand", 10
        )
        self._hand_metrics_window: dict = {"right": [], "left": []}
        self.declare_parameter("print_metrics", False)
        self._print_metrics = self.get_parameter("print_metrics").value
        self._hand_print_last = 0.0
        self._hand_print_interval = 2.0

        # Track last command publish timestamps for control delay
        self._last_cmd_stamp: dict = {"right": 0.0, "left": 0.0}
        self._last_landmark_stamp: dict = {"right": 0.0, "left": 0.0}

        # ---- Clench-based arming ----
        self.declare_parameter("require_clench_to_start", True)  # False = direct start
        self.declare_parameter("dry_run", False)  # True = detect clench but don't move
        self.declare_parameter("clench_angle_threshold", 40.0)   # degrees
        self.declare_parameter("clench_debounce_frames", 30)     # ~0.5s at 60Hz
        self._require_clench = self.get_parameter("require_clench_to_start").value
        self._dry_run = self.get_parameter("dry_run").value
        self._clench_thresh = self.get_parameter("clench_angle_threshold").value
        self._clench_debounce = self.get_parameter("clench_debounce_frames").value
        self._clench_count = 0
        self._armed = not self._require_clench  # direct-start: armed immediately
        self._armed_linger_count = 0
        self._armed_pub = self.create_publisher(
            Bool, "/teleop/armed", 10
        )
        if self._armed:
            # Keep publishing for a few seconds to ensure late subscribers get it
            self._armed_pub.publish(Bool(data=True))
            self._armed_linger_count = 10  # republish 10 times @ 0.2s = 2s
            self.create_timer(0.2, self._armed_linger_callback)
        mode = "CLENCH-TO-START" if self._require_clench else "DIRECT START"
        self.get_logger().info(f"Teleop mode: {mode}")

        # Hand state subscribers (for physical control delay measurement)
        try:
            from xhand_control_interfaces.msg import XHandStateArray
            self.create_subscription(
                XHandStateArray, "/left_hand/xhand_state",
                lambda msg: self._hand_state_cb(msg, "left"), qos_profile_sensor_data,
            )
            self.create_subscription(
                XHandStateArray, "/right_hand/xhand_state",
                lambda msg: self._hand_state_cb(msg, "right"), qos_profile_sensor_data,
            )
        except ImportError:
            pass

        # Home services
        self.right_home_srv = self.create_service(
            Trigger, "~/home_right", self.home_right_callback
        )
        self.left_home_srv = self.create_service(
            Trigger, "~/home_left", self.home_left_callback
        )

        if self.viz:
            self.right_joint_states_pub = self.create_publisher(
                JointState, "/right_hand/joint_states", 10
            )
            self.left_joint_states_pub = self.create_publisher(
                JointState, "/left_hand/joint_states", 10
            )

        # ---- Disarm subscriber (reset clench after episode save) ----
        self.create_subscription(Bool, "/teleop/disarm", self._disarm_callback, 10)

        self.get_logger().info(f"XHand Dex Retargeting Node started, topic: {input_topic}")

    def _armed_linger_callback(self):
        """Re-publish armed signal for late subscribers during startup."""
        if self._armed and getattr(self, '_armed_linger_count', 0) > 0:
            self._armed_pub.publish(Bool(data=True))
            self._armed_linger_count -= 1

    def _disarm_callback(self, msg: Bool):
        """Reset clench state after episode save. Allow re-clench to re-arm."""
        if self._require_clench and self._armed:
            self._armed = False
            self._clench_count = 0
            self._armed_pub.publish(Bool(data=False))
            self.get_logger().info("--- TELEOP DISARMED — hands HOME, wait for re-clench ---")

    def make_config(self, hand_side: str):
        config_dir = Path(get_package_share_directory("xhand_retargeting")) / "config"
        config_path = config_dir / f"xhand_{hand_side}_dexpilot.yml"

        with open(str(config_path), "r") as f:
            yaml_config = yaml.load(f, Loader=yaml.FullLoader)
        cfg = yaml_config["retargeting"]
        cfg["urdf_path"] = str(self._resolve_urdf_path(hand_side))

        return RetargetingConfig.from_dict(cfg)

    def _resolve_urdf_path(self, hand_side: str) -> Path:
        urdf_dir = Path(get_package_share_directory("xhand_retargeting")) / "urdf"
        if hand_side == "right":
            return urdf_dir / "xhand_right.urdf"
        else:
            return urdf_dir / "xhand_left.urdf"

    def retarget_hand(self, pose_data: np.ndarray, retargeter) -> np.ndarray:
        retargeting_type = retargeter.optimizer.retargeting_type
        indices = retargeter.optimizer.target_link_human_indices

        if retargeting_type == "POSITION":
            reference_values = pose_data[indices, :]
        else:
            origin_indices = indices[0, :]
            task_indices = indices[1, :]
            reference_values = pose_data[task_indices, :] - pose_data[origin_indices, :]

        joint_values = retargeter.retarget(reference_values)
        return joint_values

    def _urdf_to_xhand_joint_values(self, joint_values, retargeter, hand_side: str):
        prefix = URDF_JOINT_PREFIX[hand_side]
        urdf_names = [prefix + name for name in XHAND_JOINT_NAMES]

        result = [
            joint_values[retargeter.joint_names.index(name)]
            if name in retargeter.joint_names
            else 0.0
            for name in urdf_names
        ]

        # XHand-specific: right hand needs index_bend inverted; left hand does not
        if hand_side == "right":
            result[3] = -result[3]

        return result

    def publish_joint_controls(self, joint_values: list, hand_side: str):
        msg = self._build_command_msg(joint_values)
        if hand_side == "right":
            self.right_joint_control_pub.publish(msg)
        elif hand_side == "left":
            self.left_joint_control_pub.publish(msg)

    def publish_joint_states(self, joint_values: list, hand_side: str):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = XHAND_JOINT_NAMES
        msg.position = [float(v) for v in joint_values]

        if hand_side == "right":
            self.right_joint_states_pub.publish(msg)
        elif hand_side == "left":
            self.left_joint_states_pub.publish(msg)

    def _build_command_msg(self, joint_values: list) -> XHandCommand:
        msg = XHandCommand()
        msg.hand_id = 0
        msg.name = XHAND_JOINT_NAMES
        msg.position = [float(v) for v in joint_values]
        msg.kp = [XHAND_KP] * len(joint_values)
        msg.ki = [XHAND_KI] * len(joint_values)
        msg.kd = [XHAND_KD] * len(joint_values)
        msg.effort_limit = [XHAND_EFFORT_LIMIT] * len(joint_values)
        msg.mode = XHAND_MODE
        return msg

    def home_right_callback(self, request, response):
        self.get_logger().info("Homing right hand to default position")
        msg = self._build_command_msg(self.home_position_rad)
        self.right_joint_control_pub.publish(msg)
        response.success = True
        response.message = "Right hand homed"
        return response

    def home_left_callback(self, request, response):
        self.get_logger().info("Homing left hand to default position")
        msg = self._build_command_msg(self.home_position_rad)
        self.left_joint_control_pub.publish(msg)
        response.success = True
        response.message = "Left hand homed"
        return response

    def pose_callback_right(self, msg: PoseArray):
        self._process_pose(msg, "right")

    def pose_callback_left(self, msg: PoseArray):
        self._process_pose(msg, "left")

    def _process_pose(self, msg: PoseArray, hand_side: str):
        t_cb = time.time()

        # FPS tracking
        if not hasattr(self, '_r_fps'):
            self._r_fps = FPSCounter(window=100, print_interval=5.0)
        r_fps = self._r_fps.tick()
        if self._r_fps.should_print():
            self.get_logger().info(
                f"[Retarget FPS] {r_fps:.0f} Hz (armed={self._armed})"
            )

        # Extract VR landmark timestamp from header
        vr_stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        data = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in msg.poses])

        if hand_side == "right":
            retargeter = self.right_retargeter
            last_joint = self.last_joint_right
            metrics_pub = self.right_metrics_pub
        else:
            retargeter = self.left_retargeter
            last_joint = self.last_joint_left
            metrics_pub = self.left_metrics_pub

        # Retargeting compute
        t_retarget_start = time.time()
        joint_values = self.retarget_hand(data, retargeter)
        joint_values = self._urdf_to_xhand_joint_values(joint_values, retargeter, hand_side)
        t_retarget_done = time.time()

        # Post-processing
        t_post_start = time.time()
        joint_values = self._postprocess_thumb(data, joint_values)
        if last_joint is not None:
            joint_values = [
                self.smoothing_alpha * curr + (1 - self.smoothing_alpha) * prev
                for curr, prev in zip(joint_values, last_joint)
            ]
        t_post_done = time.time()

        if hand_side == "right":
            self.last_joint_right = joint_values
        else:
            self.last_joint_left = joint_values

        # ---- Clench detection & arming (only when enabled) ----
        if self._require_clench and not self._armed:
            is_clenched = self._is_hand_clenched(joint_values)
            if is_clenched:
                self._clench_count += 1
                if self._clench_count == 1 or self._clench_count % 10 == 0:
                    self.get_logger().info(
                        f"[{hand_side.upper()}] Clench detected "
                        f"({self._clench_count}/{self._clench_debounce})"
                    )
            else:
                self._clench_count = max(0, self._clench_count - 1)

            if self._clench_count >= self._clench_debounce:
                self._armed = True
                self._armed_pub.publish(Bool(data=True))
                self._armed_linger_count = 5  # republish 5 more times
                self.get_logger().info("*** BOTH HANDS CLENCHED — TELEOP ARMED ***")

            # Before armed: publish HOME (open) pose
            if not self._dry_run:
                home_jv = list(self.home_position_rad)
                self.publish_joint_controls(home_jv, hand_side)
                if self.viz:
                    self.publish_joint_states(home_jv, hand_side)
            t_published = time.time()
            self._last_cmd_stamp[hand_side] = t_published
            self._last_landmark_stamp[hand_side] = vr_stamp_sec
            return

        # ---- Normal operation ----
        if not self._dry_run:
            self.publish_joint_controls(joint_values, hand_side)
            if self.viz:
                self.publish_joint_states(joint_values, hand_side)
        t_published = time.time()

        # Record for control delay measurement
        self._last_cmd_stamp[hand_side] = t_published
        self._last_landmark_stamp[hand_side] = vr_stamp_sec

        if self.viz:
            self.publish_joint_states(joint_values, hand_side)

        # ---- Publish software-pipeline metrics (partial: no control_ms yet) ----
        sw_landmark_to_cmd = (t_published - vr_stamp_sec) * 1000.0
        sw_retarget = (t_retarget_done - t_retarget_start) * 1000.0
        sw_post = (t_post_done - t_post_start) * 1000.0

        # Store partial data for this frame (waiting for state feedback)
        self._pending_hand_metrics = getattr(self, '_pending_hand_metrics', {})
        self._pending_hand_metrics[hand_side] = (
            t_published, sw_landmark_to_cmd, sw_retarget, sw_post,
            vr_stamp_sec, t_published,
        )

    def _is_hand_clenched(self, joint_values: list) -> bool:
        """Check if a hand is in clenched (fist) pose.

        Looks at the 4 finger joints (indices 3, 6, 8, 10 in the 12-joint array).
        All must exceed the threshold angle to count as clenched.
        """
        # Joint indices for finger bends: mid_j1, ring_j1, pinky_j1
        # (index_bend excluded — DexPilot outputs small values for it)
        finger_indices = [6, 8, 10]
        angles = [abs(math.degrees(joint_values[i])) for i in finger_indices]
        # Print every ~2 seconds for debugging
        now = time.time()
        if getattr(self, '_clench_debug_last', 0) == 0 or now - self._clench_debug_last > 2.0:
            self._clench_debug_last = now
            self.get_logger().info(
                f"Clench check: mid_j1={angles[0]:.0f}° ring_j1={angles[1]:.0f}° "
                f"pinky_j1={angles[2]:.0f}° (threshold={self._clench_thresh}°)"
            )
        return all(a > self._clench_thresh for a in angles)

    def _hand_state_cb(self, msg, hand_side: str):
        """Receive hand state feedback and compute physical control delay."""
        t_state = time.time()
        pending = getattr(self, '_pending_hand_metrics', {})
        if hand_side not in pending:
            return
        (t_cmd, sw_lm2cmd, sw_retarget, sw_post,
         vr_stamp, t_published) = pending.pop(hand_side)

        # Physical control delay: cmd published → state received
        control_ms = (t_state - t_cmd) * 1000.0
        # E2E: landmark → state
        e2e_ms = (t_state - vr_stamp) * 1000.0

        # Publish full metrics
        metrics = Float64MultiArray()
        metrics.data = [
            float(t_state), float(sw_lm2cmd), float(sw_retarget),
            float(sw_post), float(control_ms), float(e2e_ms),
        ]
        pub = self.right_metrics_pub if hand_side == "right" else self.left_metrics_pub
        pub.publish(metrics)

        # Rolling stats for periodic print
        window = self._hand_metrics_window[hand_side]
        window.append((sw_lm2cmd, sw_retarget, sw_post, control_ms, e2e_ms))
        if len(window) > 100:
            window.pop(0)

        # Periodic print
        now = time.time()
        if self._print_metrics and now - self._hand_print_last >= self._hand_print_interval:
            self._hand_print_last = now
            for side in ("right", "left"):
                w = self._hand_metrics_window.get(side, [])
                if not w:
                    continue
                arr = np.array(w)
                self.get_logger().info(
                    f"\n[{side.upper()} HAND] ── Latency (window={len(w)}) ──\n"
                    f"  Software pipeline:\n"
                    f"    LM→Cmd:    avg={np.mean(arr[:,0]):5.1f}  "
                    f"max={np.max(arr[:,0]):5.1f} ms\n"
                    f"    Retarget:  avg={np.mean(arr[:,1]):5.1f}  "
                    f"max={np.max(arr[:,1]):5.1f} ms\n"
                    f"    PostProc:  avg={np.mean(arr[:,2]):5.1f}  "
                    f"max={np.max(arr[:,2]):5.1f} ms\n"
                    f"  Physical control (ROS+USB+Motor):\n"
                    f"    ═══ Control: avg={np.mean(arr[:,3]):5.1f}  "
                    f"max={np.max(arr[:,3]):5.1f} ms ═══\n"
                    f"  ═══ E2E total:  avg={np.mean(arr[:,4]):5.1f}  "
                    f"max={np.max(arr[:,4]):5.1f} ms ═══"
                )

    def _postprocess_thumb(self, landmarks: np.ndarray, joint_values: list) -> list:
        if not self.enable_thumb_fix:
            return joint_values

        THUMB_ROTA2 = 2
        mcp = landmarks[2]
        ip = landmarks[3]
        tip = landmarks[4]

        total_dist = np.linalg.norm(tip - mcp)
        seg_sum = np.linalg.norm(ip - mcp) + np.linalg.norm(tip - ip)
        straightness = total_dist / max(seg_sum, 0.001)

        if straightness > 0.85:
            factor = (straightness - 0.85) / 0.15 * 0.3
            joint_values[THUMB_ROTA2] *= (1.0 - factor)

        return joint_values


def main(args=None):
    rclpy.init(args=args)
    node = XHandDexRetargetingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

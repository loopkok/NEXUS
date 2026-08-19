#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Time as RosTime

import socket
import threading
import time
import json
import numpy as np
import re
from typing import Tuple

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from std_msgs.msg import String

from quest3_hand_mocap.latency_meter import LatencyMeter


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


OPERATOR2MANO_RIGHT = np.array([
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
])

OPERATOR2MANO_LEFT = np.array([
    [0, 0, -1],
    [1, 0, 0],
    [0, -1, 0],
])


def _quat_normalize(q: np.ndarray) -> np.ndarray:
    """Normalize quaternion ``[x, y, z, w]``; return identity if near-zero."""
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return q / n


def _quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return conjugate of ``[x, y, z, w]`` (inverse for unit quaternions)."""
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def _quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Hamilton product ``q1 ⊗ q2`` for ``[x, y, z, w]``."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array(
        [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ],
        dtype=float,
    )


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector ``v`` by unit quaternion ``q`` (``[x, y, z, w]``)."""
    qv = np.array([v[0], v[1], v[2], 0.0], dtype=float)
    return _quat_multiply(_quat_multiply(q, qv), _quat_conjugate(q))[:3]


def pose_in_parent_frame(
    parent_pos: np.ndarray,
    parent_quat: np.ndarray,
    child_pos: np.ndarray,
    child_quat: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Express child world pose in parent frame: ``T_p_c = inv(T_w_p) @ T_w_c``.

    Computed in Unity axes; convert to robot with :func:`unity_pose_to_robot` after.
    """
    parent_quat = _quat_normalize(parent_quat)
    child_quat = _quat_normalize(child_quat)
    parent_inv = _quat_conjugate(parent_quat)
    rel_pos = _quat_rotate(parent_inv, child_pos - parent_pos)
    rel_quat = _quat_normalize(_quat_multiply(parent_inv, child_quat))
    return rel_pos, rel_quat


# Unity LH (X right, Y up, Z forward) → robot (X left, Y back, Z up):
#   (x, y, z)_r = (-x_u, -z_u, y_u)
UNITY_TO_ROBOT = np.array(
    [
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Unit quaternion ``[x, y, z, w]`` → 3×3 rotation matrix."""
    q = _quat_normalize(q)
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _matrix_to_quat(m: np.ndarray) -> np.ndarray:
    """3×3 rotation matrix → quaternion ``[x, y, z, w]``."""
    t = float(np.trace(m))
    if t > 0.0:
        s = np.sqrt(t + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return _quat_normalize(np.array([x, y, z, w], dtype=float))


def unity_vec_to_robot(v: np.ndarray) -> np.ndarray:
    """Map Unity LH vector/point to robot (X left, Y back, Z up)."""
    return UNITY_TO_ROBOT @ np.asarray(v, dtype=float)


def unity_quat_to_robot(q: np.ndarray) -> np.ndarray:
    """Map Unity LH orientation to robot axes: ``R_r = M R_u M^T``."""
    r_u = _quat_to_matrix(q)
    r_r = UNITY_TO_ROBOT @ r_u @ UNITY_TO_ROBOT.T
    return _matrix_to_quat(r_r)


def unity_pose_to_robot(
    pos: np.ndarray, quat: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Convert Unity LH pose to robot (X left, Y back, Z up)."""
    return unity_vec_to_robot(pos), unity_quat_to_robot(quat)


def unity_landmarks_to_robot(points: np.ndarray) -> np.ndarray:
    """Convert (N,3) Unity LH points to robot axes."""
    return (UNITY_TO_ROBOT @ np.asarray(points, dtype=float).T).T


def adaptive_retargeting_xhand(landmarks):
    """XHand-only pinky length hack (NOT for Wuji Hand).

    Sequentially stretches MCP→PIP→DIP→TIP. Because each step uses the
    already-moved proximal joint, large scales fold the finger into a
    zigzag / Z shape. Keep disabled for Wuji / general MediaPipe consumers.
    """
    landmarks = landmarks.copy()

    pinky_mcp = 17
    pinky_pip = 18
    pinky_dip = 19
    pinky_tip = 20

    pinky_extension = np.linalg.norm(landmarks[pinky_tip] - landmarks[pinky_mcp])

    max_extension = 0.10
    min_extension = 0.03
    extension_ratio = np.clip((pinky_extension - min_extension) / (max_extension - min_extension), 0.0, 1.0)

    base_scale = 1.2
    max_scale = 2.2
    adaptive_scale = base_scale + (max_scale - base_scale) * extension_ratio

    mcp_to_pip_vector = landmarks[pinky_pip] - landmarks[pinky_mcp]
    landmarks[pinky_pip] = landmarks[pinky_mcp] + mcp_to_pip_vector * adaptive_scale

    pip_to_dip_vector = landmarks[pinky_dip] - landmarks[pinky_pip]
    landmarks[pinky_dip] = landmarks[pinky_pip] + pip_to_dip_vector * adaptive_scale

    dip_to_tip_vector = landmarks[pinky_tip] - landmarks[pinky_dip]
    landmarks[pinky_tip] = landmarks[pinky_dip] + dip_to_tip_vector * adaptive_scale

    return landmarks


class Quest3UDPMocap(Node):
    def __init__(self):
        super().__init__("quest3_udp_mocap")
        self.get_logger().info("Quest 3 Hand Mocap Node Initialized")

        # 参数配置
        self.declare_parameter("protocol", "tcp_wired")  # "udp" | "tcp_wired" | "tcp_wireless"
        self.declare_parameter("udp_port", 9000)
        self.declare_parameter("tcp_port", 8000)
        self.declare_parameter("ema_alpha", 0.7)
        self.declare_parameter("viz", True)
        self.declare_parameter("arm_side", "right")
        # XHand pinky stretch — causes Z-shaped pinky if left on for Wuji.
        self.declare_parameter("enable_xhand_pinky_adapt", False)
        # landmark_preprocess:
        #   mano — wrist frame + OPERATOR2MANO (legacy XHand / nero path)
        #   raw  — only Quest→ROS X flip; leave MANO to wuji_retargeting
        #          (required for glove-compatible Wuji retarget / tuning)
        self.declare_parameter("landmark_preprocess", "mano")
        # Unity LH → robot (X left, Y back, Z up). If false: keep Unity axes
        # (landmarks only apply legacy X flip).
        self.declare_parameter("convert_to_robot", True)

        self.protocol = self.get_parameter("protocol").value.lower()
        self.udp_port = self.get_parameter("udp_port").value
        self.tcp_port = self.get_parameter("tcp_port").value
        self.ema_alpha = self.get_parameter("ema_alpha").value
        self.viz = self.get_parameter("viz").value
        self.arm_side = self.get_parameter("arm_side").value.lower()
        self.both = self.arm_side == "both"
        self.enable_xhand_pinky_adapt = bool(
            self.get_parameter("enable_xhand_pinky_adapt").value
        )
        self.landmark_preprocess = str(
            self.get_parameter("landmark_preprocess").value
        ).strip().lower()
        if self.landmark_preprocess not in ("mano", "raw"):
            raise ValueError(
                "landmark_preprocess must be 'mano' or 'raw', "
                f"got {self.landmark_preprocess}"
            )
        self.convert_to_robot = bool(self.get_parameter("convert_to_robot").value)
        self.declare_parameter("print_latency", True)
        self.declare_parameter("latency_print_interval", 2.0)
        self._print_latency = bool(self.get_parameter("print_latency").value)
        self._lat = LatencyMeter(
            float(self.get_parameter("latency_print_interval").value)
        )
        self._last_wrist_arrival = {"left": 0.0, "right": 0.0}

        self.get_logger().info(
            f"Quest 3 Hand Mocap: side=[{self.arm_side.upper()}], "
            f"protocol={self.protocol}, "
            f"preprocess={self.landmark_preprocess}, "
            f"xhand_pinky_adapt={self.enable_xhand_pinky_adapt}, "
            f"convert_to_robot={self.convert_to_robot}"
        )

        # 缓存用于 EMA 平滑滤波
        self.landmark_cache_right = np.zeros((21, 3))
        self.landmark_cache_left = np.zeros((21, 3))

        # World frame for head & wrist (same parent; not wrist-under-head).
        self._world_frame_id = "robot_world" if self.convert_to_robot else "vr_world"

        # 发布者：始终按左右分 topic
        # Frame tree:
        #   robot_world|vr_world ── head
        #                        ── wrist
        #                             └── landmarks (wrist-local)
        self.mocap_pub_right = self.create_publisher(PoseArray, "hand_landmarks/right", 10)
        self.mocap_pub_left = self.create_publisher(PoseArray, "hand_landmarks/left", 10)
        self.wrist_pub_right = self.create_publisher(PoseStamped, "quest3/right_wrist_pose", 10)
        self.wrist_pub_left = self.create_publisher(PoseStamped, "quest3/left_wrist_pose", 10)
        self.head_pub = self.create_publisher(PoseStamped, "quest3/head_pose", 10)
        latch_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.body_pub = self.create_publisher(PoseArray, "quest3/body_joints", 10)
        self.body_names_pub = self.create_publisher(
            String, "quest3/body_joint_names", latch_qos
        )
        self.hips_pub = self.create_publisher(PoseStamped, "quest3/hips_pose", 10)
        self._logged_iobt_fidelity = ""
        
        # 新增：RViz 可视化发布者
        if self.viz:
            self.right_marker_pub = self.create_publisher(MarkerArray, "quest3/right_hand_markers", 10)
            self.left_marker_pub = self.create_publisher(MarkerArray, "quest3/left_hand_markers", 10)
            self.marker_pub = self.create_publisher(MarkerArray, f"quest3/{self.arm_side}_hand_markers", 10)

        # 腕部数据的正则解析器
        self.wrist_pattern = re.compile(r'wrist:,?\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+),\s*([-\d\.]+)')

        self.is_running = True

        # ---- Network listener (UDP / TCP wired / TCP wireless) ----
        if self.protocol == "udp":
            self._start_udp_listener()
        elif self.protocol in ("tcp_wired", "tcp_wireless"):
            self._start_tcp_listener()
        else:
            self.get_logger().error(f"Unknown protocol: {self.protocol}")
            raise ValueError(f"Unknown protocol: {self.protocol}")

    def _start_udp_listener(self):
        """Start UDP listener thread."""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('0.0.0.0', self.udp_port))
        self.socket.settimeout(0.5)
        self.get_logger().info(f"Listening for UDP data on port {self.udp_port}...")
        self.thread = threading.Thread(target=self.udp_listener, daemon=True)
        self.thread.start()

    def _start_tcp_listener(self):
        """Start TCP server listener thread."""
        host = "localhost" if self.protocol == "tcp_wired" else "0.0.0.0"
        self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.tcp_server.bind((host, self.tcp_port))
        self.tcp_server.listen(3)
        self.tcp_server.settimeout(1.0)
        mode = "wired (adb reverse)" if self.protocol == "tcp_wired" else "wireless (WiFi)"
        self.get_logger().info(
            f"TCP server listening on {host}:{self.tcp_port} ({mode})"
        )
        if self.protocol == "tcp_wired":
            self.get_logger().info(
                "  On Quest3: adb reverse tcp:{0} tcp:{0}".format(self.tcp_port)
            )
        self.thread = threading.Thread(target=self.tcp_listener, daemon=True)
        self.thread.start()

    @staticmethod
    def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
        """
        Compute the 3D coordinate frame (orientation only) from detected 3d key points.
        :param keypoint_3d_array: (21, 3) keypoints in the order: [wrist, index_mcp, middle_mcp, ...]
        :return: the coordinate frame of wrist in MANO convention (3x3 rotation matrix)
        """
        assert keypoint_3d_array.shape == (21, 3)
        points = keypoint_3d_array[[0, 5, 9], :]

        x_vector = points[0] - points[2]
        points = points - np.mean(points, axis=0, keepdims=True)
        _, _, v = np.linalg.svd(points)
        normal = v[2, :]

        x = x_vector - np.sum(x_vector * normal) * normal
        x = x / np.linalg.norm(x)
        z = np.cross(x, normal)

        if np.sum(z * (points[1] - points[2])) < 0:
            normal *= -1
            z *= -1
        frame = np.stack([x, normal, z], axis=1)
        return frame

    def process_landmarks(self, landmarks, hand_label):
        """Convert landmarks already in robot axes for downstream consumers.

        Input points are wrist-local after Unity→robot (X left, Y back, Z up).
        ``raw``: center on wrist. ``mano``: scale + SVD wrist frame + OPERATOR2MANO.
        """
        keypoint_3d_array = landmarks.copy()

        if self.landmark_preprocess == "raw":
            # Still center on wrist for stable EMA / viz; Retargeter re-centers.
            return keypoint_3d_array - keypoint_3d_array[0:1, :]

        operator2mano = OPERATOR2MANO_RIGHT if hand_label == "right" else OPERATOR2MANO_LEFT
        keypoint_3d_array *= 1.05
        keypoint_3d_array = keypoint_3d_array - keypoint_3d_array[0:1, :]
        wrist_rot = self.estimate_frame_from_hand_points(keypoint_3d_array)
        joint_pos = keypoint_3d_array @ wrist_rot @ operator2mano

        if self.enable_xhand_pinky_adapt:
            joint_pos = adaptive_retargeting_xhand(joint_pos)

        return joint_pos

    def udp_listener(self):
        """UDP listener thread."""
        while self.is_running and rclpy.ok():
            try:
                data, _ = self.socket.recvfrom(65536)
                arrival_time = time.time()
                message = data.decode('utf-8')
                for line in message.splitlines():
                    if not line: continue
                    self.process_line(line, arrival_time)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f"UDP Listener Error: {e}")

    def tcp_listener(self):
        """TCP server thread — accept connections and handle each."""
        while self.is_running and rclpy.ok():
            try:
                conn, addr = self.tcp_server.accept()
                self.get_logger().info(f"TCP connection from {addr}")
                t = threading.Thread(
                    target=self._tcp_connection_handler,
                    args=(conn, addr), daemon=True,
                )
                t.start()
            except socket.timeout:
                continue
            except Exception as e:
                if self.is_running:
                    self.get_logger().error(f"TCP accept error: {e}")

    def _tcp_connection_handler(self, conn, addr):
        """Handle a single TCP connection — read lines and process."""
        buf = ""
        with conn:
            while self.is_running and rclpy.ok():
                try:
                    data = conn.recv(65536)
                    if not data:
                        self.get_logger().info(f"TCP {addr} disconnected")
                        break
                    arrival_time = time.time()
                    buf += data.decode('utf-8')
                    while '\n' in buf:
                        line, buf = buf.split('\n', 1)
                        if line.strip():
                            self.process_line(line.strip(), arrival_time)
                except Exception as e:
                    self.get_logger().error(f"TCP read error from {addr}: {e}")
                    break

    def _float_to_ros_time(self, t: float) -> RosTime:
        """Convert wall-clock float seconds to ROS Time message."""
        stamp = RosTime()
        stamp.sec = int(t)
        stamp.nanosec = int((t - int(t)) * 1e9)
        return stamp

    def _maybe_log_latency(self) -> None:
        if not self._print_latency or not self._lat.has_samples():
            return
        if not self._lat.should_print():
            return
        self.get_logger().info(f"[Latency][VR] {self._lat.format_and_reset()}")

    def _process_body_line(self, line: str, arrival_time: float) -> None:
        """Parse Movement SDK IOBT: ``body iobt | fid=High: hips,x,y,z,qx,qy,qz,qw ...``."""
        if "|" in line and "fid=" in line.lower():
            try:
                fid = line.split("fid=", 1)[1].split(":", 1)[0].strip()
                if fid and fid != self._logged_iobt_fidelity:
                    self._logged_iobt_fidelity = fid
                    self.get_logger().info(f"[IOBT] Movement SDK fidelity={fid}")
                    if fid.lower() != "high":
                        self.get_logger().warn(
                            "IOBT is not High. Disable Quest passthrough and rebuild/restart HTS."
                        )
            except Exception:
                pass
        payload = line.split(":", 1)[-1].strip()
        chunks = [c.strip() for c in payload.split() if c.strip()]
        names = []
        poses = []
        hips_pos = None
        hips_quat = None
        for chunk in chunks:
            parts = [p.strip() for p in chunk.split(",") if p.strip() != ""]
            if len(parts) < 8:
                continue
            name = parts[0]
            try:
                pos_u = np.array([float(parts[1]), float(parts[2]), float(parts[3])], dtype=float)
                quat_u = _quat_normalize(
                    np.array(
                        [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])],
                        dtype=float,
                    )
                )
            except (TypeError, ValueError):
                continue
            if self.convert_to_robot:
                pos, quat = unity_pose_to_robot(pos_u, quat_u)
            else:
                pos, quat = pos_u, quat_u
            pose = Pose()
            pose.position.x = float(pos[0])
            pose.position.y = float(pos[1])
            pose.position.z = float(pos[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            names.append(name)
            poses.append(pose)
            if name == "hips":
                hips_pos, hips_quat = pos, quat
        if not poses:
            return
        stamp = self._float_to_ros_time(arrival_time)
        msg = PoseArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._world_frame_id
        msg.poses = poses
        self.body_pub.publish(msg)
        names_msg = String()
        names_msg.data = json.dumps(names)
        self.body_names_pub.publish(names_msg)
        if hips_pos is not None and hips_quat is not None:
            hips = PoseStamped()
            hips.header.stamp = stamp
            hips.header.frame_id = self._world_frame_id
            hips.pose.position.x = float(hips_pos[0])
            hips.pose.position.y = float(hips_pos[1])
            hips.pose.position.z = float(hips_pos[2])
            hips.pose.orientation.x = float(hips_quat[0])
            hips.pose.orientation.y = float(hips_quat[1])
            hips.pose.orientation.z = float(hips_quat[2])
            hips.pose.orientation.w = float(hips_quat[3])
            self.hips_pub.publish(hips)

    def process_line(self, line, arrival_time: float = 0.0):
        # FPS tracking
        if not hasattr(self, '_fps'):
            self._fps = FPSCounter(window=100, print_interval=5.0)
        fps = self._fps.tick()
        if self._fps.should_print():
            self.get_logger().info(f"[VR Data Rate] {fps:.0f} Hz")

        line_lower = line.lower()

        # Movement SDK IOBT (must run before "head" match — packet contains a head joint).
        if line_lower.startswith("body"):
            self._process_body_line(line, arrival_time)
            return

        # ===== 头部解析（无左右区分，全局）=====
        # Head: Unity world → optional robot axes (X left, Y back, Z up).
        if "head" in line_lower or "hmd" in line_lower:
            # Format: head, x, y, z, qx, qy, qz, qw  (or colon-separated)
            line_clean = line_lower.replace(":", ",")
            parts = line_clean.split(",")
            if len(parts) >= 7:
                try:
                    vals = [float(p.strip()) for p in parts[-7:]]
                    head_pos_u = np.array(vals[0:3], dtype=float)
                    head_quat_u = _quat_normalize(np.array(vals[3:7], dtype=float))
                    if self.convert_to_robot:
                        head_pos, head_quat = unity_pose_to_robot(head_pos_u, head_quat_u)
                    else:
                        head_pos, head_quat = head_pos_u, head_quat_u
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self._float_to_ros_time(arrival_time)
                    pose_msg.header.frame_id = self._world_frame_id
                    pose_msg.pose.position.x = float(head_pos[0])
                    pose_msg.pose.position.y = float(head_pos[1])
                    pose_msg.pose.position.z = float(head_pos[2])
                    pose_msg.pose.orientation.x = float(head_quat[0])
                    pose_msg.pose.orientation.y = float(head_quat[1])
                    pose_msg.pose.orientation.z = float(head_quat[2])
                    pose_msg.pose.orientation.w = float(head_quat[3])
                    self.head_pub.publish(pose_msg)
                except Exception:
                    pass
            return  # 头部行不继续处理

        side = "right" if "right" in line_lower else "left"

        # 过滤：非 both 模式下只处理指定手
        if not self.both and side != self.arm_side:
            return

        # 1. 腕部：人手 wrist 或 Touch controller（同属世界系）
        if "wrist" in line_lower or "controller" in line_lower:
            parts = line.split(",")
            if len(parts) >= 7:
                try:
                    vals = [float(p.strip()) for p in parts[-7:]]
                    wrist_pos_u = np.array(vals[0:3], dtype=float)
                    wrist_quat_u = _quat_normalize(np.array(vals[3:7], dtype=float))
                    if self.convert_to_robot:
                        wrist_pos, wrist_quat = unity_pose_to_robot(
                            wrist_pos_u, wrist_quat_u
                        )
                    else:
                        wrist_pos, wrist_quat = wrist_pos_u, wrist_quat_u

                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self._float_to_ros_time(arrival_time)
                    pose_msg.header.frame_id = self._world_frame_id
                    pose_msg.pose.position.x = float(wrist_pos[0])
                    pose_msg.pose.position.y = float(wrist_pos[1])
                    pose_msg.pose.position.z = float(wrist_pos[2])
                    pose_msg.pose.orientation.x = float(wrist_quat[0])
                    pose_msg.pose.orientation.y = float(wrist_quat[1])
                    pose_msg.pose.orientation.z = float(wrist_quat[2])
                    pose_msg.pose.orientation.w = float(wrist_quat[3])
                    (
                        self.wrist_pub_right if side == "right" else self.wrist_pub_left
                    ).publish(pose_msg)
                    src = "controller" if "controller" in line_lower else "hand"
                    last = getattr(self, "_wrist_src", None)
                    if last is None:
                        last = {"left": "none", "right": "none"}
                        self._wrist_src = last
                    if last.get(side) != src:
                        last[side] = src
                        self.get_logger().info(
                            f"[HTS] mix left={last.get('left')} right={last.get('right')}"
                        )
                    if self._print_latency and arrival_time > 0.0:
                        self._lat.add(
                            "recv_to_pub", (time.time() - arrival_time) * 1000.0
                        )
                        prev = self._last_wrist_arrival[side]
                        if prev > 0.0:
                            self._lat.add(
                                f"wrist_gap.{side}", (arrival_time - prev) * 1000.0
                            )
                        self._last_wrist_arrival[side] = arrival_time
                        self._maybe_log_latency()
                except Exception as e:
                    self.get_logger().error(f"解析浮点数失败: {e}")
        # 2. 手指：Unity 腕局部 → robot 轴，再 raw/mano + EMA
        if "landmarks" in line.lower():
            parts = line.split(":")
            if len(parts) < 2: return
            values_str = parts[1].split(",")
            values = [float(v.strip()) for v in values_str if v.strip()]

            if len(values) == 63:
                landmarks = np.array(values).reshape(21, 3)
                if self.convert_to_robot:
                    # Full Unity→robot axis map (X left, Y back, Z up).
                    mapped = unity_landmarks_to_robot(landmarks)
                else:
                    # Legacy: X-only flip, keep Unity Y/Z.
                    mapped = np.zeros_like(landmarks)
                    mapped[:, 0] = -landmarks[:, 0]
                    mapped[:, 1] = landmarks[:, 1]
                    mapped[:, 2] = landmarks[:, 2]
                processed_landmarks = self.process_landmarks(mapped, side)
                self.publish_mocap_data(processed_landmarks, side, arrival_time)
    def publish_hand_markers(self, hand_data, hand):
        """新增：将关节点转化为 RViz 的球体列表"""
        m = Marker()
        m.header.frame_id = 'world'  # RViz 的固定坐标系
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = f"hand_{hand}"
        m.id = 0
        m.type = Marker.SPHERE_LIST
        m.action = Marker.ADD
        m.scale.x = 0.015; m.scale.y = 0.015; m.scale.z = 0.015
        m.color.a = 1.0
        
        if hand == "right":
            m.color.r = 0.9; m.color.g = 0.2; m.color.b = 0.2 # 右手红色
            publisher = self.right_marker_pub
        else:
            m.color.r = 0.2; m.color.g = 0.6; m.color.b = 0.9 # 左手蓝色
            publisher = self.left_marker_pub
            
        for i in range(hand_data.shape[0]):
            point = Point(x=float(hand_data[i, 0]), y=float(hand_data[i, 1]), z=float(hand_data[i, 2]))
            m.points.append(point)
            
        ma = MarkerArray()
        ma.markers.append(m)
        publisher.publish(ma)

    def publish_mocap_data(self, landmarks, side, arrival_time: float = 0.0):
        if side == 'right':
            self.landmark_cache_right = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_right
            landmarks = self.landmark_cache_right
        elif side == 'left':
            self.landmark_cache_left = self.ema_alpha * landmarks + (1 - self.ema_alpha) * self.landmark_cache_left
            landmarks = self.landmark_cache_left
        else:
            return

        msg = PoseArray()
        msg.header.stamp = self._float_to_ros_time(arrival_time)
        msg.header.frame_id = f"hand_{side}"
        msg.poses = [
            Pose(
                position=Point(x=float(lm[0]), y=float(lm[1]), z=float(lm[2])),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ) for lm in landmarks
        ]

        if side == "right":
            self.mocap_pub_right.publish(msg)
        else:
            self.mocap_pub_left.publish(msg)

        if self.viz:
            self.publish_hand_markers(landmarks, side)

    def destroy_node(self):
        self.is_running = False
        if hasattr(self, 'socket') and self.socket:
            self.socket.close()
        if hasattr(self, 'tcp_server') and self.tcp_server:
            self.tcp_server.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = Quest3UDPMocap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
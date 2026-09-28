#!/usr/bin/env python3
"""Main ROS2 data collection node for dual-arm dual-hand teleoperation.

Orchestrates:
  - Subscribing to arm feedback from teleop nodes
  - Subscribing to hand state from XHand control nodes
  - Managing camera capture processes
  - Keyboard-based recording control
  - Writing synchronized robot + camera data to HDF5
"""

import os
import sys
import time
import signal
import threading
from typing import Optional, Dict

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

from nero_dual_data_collect.data_writer import RobotDataWriter
from nero_dual_data_collect.camera_manager import CameraDataProcess
from nero_dual_data_collect.keyboard_controller import (
    print_colored, print_status, print_help,
)


class DataCollectNode(Node):
    """ROS2 node for multi-modal teleoperation data collection."""

    def __init__(self):
        super().__init__("data_collect_node")

        # ---- Parameters ----
        self.declare_parameter("data_base_path", os.path.expanduser("~/xnero_data"))
        self.declare_parameter("task_name", "default_task")
        self.declare_parameter("collect_left", True)    # collect left arm + left hand
        self.declare_parameter("collect_right", True)   # collect right arm + right hand
        self.declare_parameter("use_cam_0", True)
        self.declare_parameter("use_cam_1", True)
        self.declare_parameter("use_cam_2", True)

        # Per-camera serials / resolution / fps (override via launch args if needed)
        self.declare_parameter("cam_0_serial", "CP02653000VE")
        self.declare_parameter("cam_0_width", 1280)
        self.declare_parameter("cam_0_height", 720)
        self.declare_parameter("cam_0_fps", 30)
        self.declare_parameter("cam_1_serial", "CV284600002L")
        self.declare_parameter("cam_1_width", 640)
        self.declare_parameter("cam_1_height", 480)
        self.declare_parameter("cam_1_fps", 30)
        self.declare_parameter("cam_2_serial", "CV28460000FV")
        self.declare_parameter("cam_2_width", 640)
        self.declare_parameter("cam_2_height", 480)
        self.declare_parameter("cam_2_fps", 30)

        self.data_base_path = os.path.expanduser(
            self.get_parameter("data_base_path").value
        )
        self.task_name = self.get_parameter("task_name").value
        self.collect_left = self.get_parameter("collect_left").value
        self.collect_right = self.get_parameter("collect_right").value

        # Build camera configs from individual params
        cam_enabled = {
            "cam_0": self.get_parameter("use_cam_0").value,
            "cam_1": self.get_parameter("use_cam_1").value,
            "cam_2": self.get_parameter("use_cam_2").value,
        }
        self.camera_configs = []
        for cam_id in ["cam_0", "cam_1", "cam_2"]:
            if not cam_enabled[cam_id]:
                continue
            self.camera_configs.append({
                "id": cam_id,
                "serial": self.get_parameter(f"{cam_id}_serial").value,
                "width": self.get_parameter(f"{cam_id}_width").value,
                "height": self.get_parameter(f"{cam_id}_height").value,
                "fps": self.get_parameter(f"{cam_id}_fps").value,
            })

        self.get_logger().info(f"Data base path: {self.data_base_path}")
        self.get_logger().info(f"Task name: {self.task_name}")
        self.get_logger().info(
            f"Collect: left={self.collect_left}, right={self.collect_right}, "
            f"cameras={[c['id'] for c in self.camera_configs]}"
        )

        # ---- State machine ----
        self.state = "idle"  # idle | recording | paused
        self._cmd_queue = []  # thread-safe: stdin commands

        # ---- Latest data buffers (thread-safe via lock) ----
        self._data_lock = threading.Lock()
        self._latest: Dict[str, Optional[np.ndarray]] = {
            "left_arm_joints": None,
            "left_arm_tcp": None,
            "right_arm_joints": None,
            "right_arm_tcp": None,
            "left_hand_joints": None,
            "right_hand_joints": None,
        }
        # Track which sources have ever received data
        self._source_active = {k: False for k in self._latest}
        self._source_warned = {k: False for k in self._latest}
        # Pre-mark disabled sides as "warned" so they don't spam warnings
        if not self.collect_left:
            for k in ("left_arm_joints", "left_arm_tcp", "left_hand_joints"):
                self._source_warned[k] = True
                self._source_active[k] = True  # pretend active to skip warnings
        if not self.collect_right:
            for k in ("right_arm_joints", "right_arm_tcp", "right_hand_joints"):
                self._source_warned[k] = True
                self._source_active[k] = True

        # ---- Subscribers (conditional on collect_left / collect_right) ----
        if self.collect_left:
            # Left arm feedback
            self.create_subscription(
                JointState,
                "/nero_teleop_left/joint_states",
                lambda msg: self._arm_joint_callback(msg, "left"),
                10,
            )
            self.create_subscription(
                PoseStamped,
                "/nero_teleop_left/tcp_pose",
                lambda msg: self._arm_tcp_callback(msg, "left"),
                10,
            )

        if self.collect_right:
            # Right arm feedback
            self.create_subscription(
                JointState,
                "/nero_teleop_right/joint_states",
                lambda msg: self._arm_joint_callback(msg, "right"),
                10,
            )
            self.create_subscription(
                PoseStamped,
                "/nero_teleop_right/tcp_pose",
                lambda msg: self._arm_tcp_callback(msg, "right"),
                10,
            )

        # Hand state (try to import XHandStateArray; fallback to JointState)
        try:
            from xhand_control_interfaces.msg import XHandStateArray
            self._has_xhand_msgs = True
        except ImportError:
            self._has_xhand_msgs = False
            self.get_logger().warn(
                "xhand_control_interfaces not available. "
                "Hand state collection disabled."
            )

        if self._has_xhand_msgs:
            from xhand_control_interfaces.msg import XHandStateArray
            if self.collect_left:
                self.create_subscription(
                    XHandStateArray,
                    "/left_hand/xhand_state",
                    lambda msg: self._hand_state_callback(msg, "left"),
                    10,
                )
            if self.collect_right:
                self.create_subscription(
                    XHandStateArray,
                    "/right_hand/xhand_state",
                    lambda msg: self._hand_state_callback(msg, "right"),
                    10,
                )

        # ---- Metrics subscribers (arm + hand latency/accuracy) ----
        self._metrics_lock = threading.Lock()
        self._metrics_buffer: Dict[str, list] = {
            "left_arm": [],
            "right_arm": [],
            "left_hand": [],
            "right_hand": [],
        }

        # Arm metrics from teleop nodes
        self.create_subscription(
            Float64MultiArray,
            "/nero_teleop_left/metrics",
            lambda msg: self._metrics_callback(msg, "left_arm"),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/nero_teleop_right/metrics",
            lambda msg: self._metrics_callback(msg, "right_arm"),
            10,
        )

        # Hand metrics from retargeting node
        self.create_subscription(
            Float64MultiArray,
            "/xhand_dex_retargeting/metrics/left_hand",
            lambda msg: self._metrics_callback(msg, "left_hand"),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/xhand_dex_retargeting/metrics/right_hand",
            lambda msg: self._metrics_callback(msg, "right_hand"),
            10,
        )

        # ---- Episode management ----
        self._episode_idx = 0
        self._episode_path: Optional[str] = None
        self._robot_writer: Optional[RobotDataWriter] = None

        # ---- Camera process ----
        self._start_event = threading.Event()
        self._stop_event = threading.Event()
        self._cam_process: Optional[CameraDataProcess] = None

        # ---- TCP command server (localhost:19999) ----
        self._cmd_queue = []
        def _tcp_server():
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("127.0.0.1", 19999))
            s.listen(1)
            s.settimeout(1.0)
            while True:
                try:
                    conn, _ = s.accept()
                    data = conn.recv(1024).decode().strip().lower()
                    if data:
                        self._cmd_queue.append(data)
                    conn.close()
                except socket.timeout:
                    continue
                except Exception:
                    break
        self._tcp_thread = threading.Thread(target=_tcp_server, daemon=True)
        self._tcp_thread.start()

        # ---- Disarm publisher (for episode save → reset) ----
        self._disarm_pub = self.create_publisher(Bool, "/teleop/disarm", 10)
        self._teleop_armed = False
        self.create_subscription(
            Bool, "/teleop/armed", lambda msg: setattr(self, '_teleop_armed', msg.data), 10,
        )
        # ROS2 topic fallback
        self.create_subscription(
            String, "/data_collect/cmd",
            lambda msg: self._handle_command(msg.data.strip().lower()), 10,
        )
        # ROS2 services (more reliable than topics)
        self.create_service(Trigger, "~/start", lambda req, res: self._svc("start", res))
        self.create_service(Trigger, "~/stop", lambda req, res: self._svc("stop", res))
        self.create_service(Trigger, "~/discard", lambda req, res: self._svc("discard", res))
        self.create_service(Trigger, "~/next", lambda req, res: self._svc("next", res))
        self.create_service(Trigger, "~/pause", lambda req, res: self._svc("pause", res))

        # ---- Recording timer ----
        self._record_timer = self.create_timer(0.02, self._record_tick)  # 50 Hz

        # ---- Shutdown hook ----
        self.get_logger().info("DataCollectNode initialized. Ready.")
        print_help()

    # ==================================================================
    # Callbacks
    # ==================================================================

    def _arm_joint_callback(self, msg: JointState, side: str):
        """Process arm joint state feedback."""
        if not msg.position or len(msg.position) < 7:
            return
        key = f"{side}_arm_joints"
        with self._data_lock:
            self._latest[key] = np.array(msg.position[:7], dtype=np.float32)
            self._source_active[key] = True

    def _arm_tcp_callback(self, msg: PoseStamped, side: str):
        """Process arm TCP pose feedback."""
        key = f"{side}_arm_tcp"
        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        try:
            rpy = Rotation.from_quat([qx, qy, qz, qw]).as_euler("xyz")
        except Exception:
            rpy = np.zeros(3)
        tcp = np.array([px, py, pz, rpy[0], rpy[1], rpy[2]], dtype=np.float32)
        with self._data_lock:
            self._latest[key] = tcp
            self._source_active[key] = True

    def _hand_state_callback(self, msg, side: str):
        """Process XHand state feedback."""
        key = f"{side}_hand_joints"
        if hasattr(msg, "hand_states") and len(msg.hand_states) > 0:
            positions = msg.hand_states[0].position
            if len(positions) >= 12:
                with self._data_lock:
                    self._latest[key] = np.array(
                        positions[:12], dtype=np.float32
                    )
                    self._source_active[key] = True

    def _metrics_callback(self, msg: Float64MultiArray, source: str):
        """Buffer latency/accuracy metrics for later saving."""
        if self.state != "recording":
            return
        with self._metrics_lock:
            self._metrics_buffer[source].append(list(msg.data))

    # ==================================================================
    # Recording tick
    # ==================================================================

    def _record_tick(self):
        """Periodic recording tick — write one frame at 50Hz."""
        if self.state != "recording":
            return

        try:
            self._record_tick_impl()
        except Exception as e:
            self._tick_err_count = getattr(self, '_tick_err_count', 0) + 1
            if self._tick_err_count <= 3:
                self.get_logger().error(f"Record tick error (#{self._tick_err_count}): {e}")

    def _record_tick_impl(self):
        # Debug: log first frame and every 100th
        self._tick_count = getattr(self, '_tick_count', 0) + 1
        if self._tick_count == 1 or self._tick_count % 100 == 0:
            self.get_logger().info(f"Record tick #{self._tick_count}, "
                                   f"left_arm={'OK' if self._latest['left_arm_joints'] is not None else 'NONE'}, "
                                   f"left_hand={'OK' if self._latest['left_hand_joints'] is not None else 'NONE'}")

        with self._data_lock:
            # Snapshot latest data (may be None for inactive sources)
            frame_data = {}
            for k, v in self._latest.items():
                frame_data[k] = v.copy() if v is not None else None

        # ---- Warn once for inactive sources ----
        is_recording_start = (self._robot_writer is not None and
                              self._robot_writer.frame_count == 0)
        if is_recording_start:
            for key in self._latest:
                if not self._source_active[key] and not self._source_warned[key]:
                    self._source_warned[key] = True
                    self.get_logger().warn(
                        f"Source '{key}' has no data yet — "
                        f"will record zeros. Check that the corresponding "
                        f"node is running."
                    )

        # ---- Write to HDF5 ----
        if self._robot_writer is not None:
            # Use zeros for missing/disabled sources — never drop a frame
            def _pad(arr, n):
                if arr is None: return np.zeros(n, dtype=np.float32)
                a = np.asarray(arr, dtype=np.float32).ravel()
                if len(a) >= n: return a[:n]
                return np.pad(a, (0, n - len(a)))
            la_j = _pad(frame_data["left_arm_joints"], 7)
            la_t = _pad(frame_data["left_arm_tcp"], 7)
            ra_j = _pad(frame_data["right_arm_joints"], 7)
            ra_t = _pad(frame_data["right_arm_tcp"], 7)
            lh_j = _pad(frame_data["left_hand_joints"], 12)
            rh_j = _pad(frame_data["right_hand_joints"], 12)

            self._robot_writer.write_frame(
                left_arm_joints=la_j,
                left_arm_tcp=la_t,
                right_arm_joints=ra_j,
                right_arm_tcp=ra_t,
                left_hand_joints=lh_j,
                right_hand_joints=rh_j,
                timestamp=time.time(),
            )

    # ==================================================================
    # Episode management
    # ==================================================================

    def _create_episode_path(self) -> str:
        """Create and return a new episode directory."""
        task_dir = os.path.join(self.data_base_path, "Data", self.task_name)
        os.makedirs(task_dir, exist_ok=True)

        # Find next episode index
        idx = self._episode_idx
        while os.path.exists(os.path.join(task_dir, f"episode{idx}")):
            idx += 1
        self._episode_idx = idx

        ep_path = os.path.join(task_dir, f"episode{idx}")
        os.makedirs(ep_path, exist_ok=True)
        return ep_path

    def _start_episode(self):
        """Prepare for a new episode."""
        # Create episode directory
        self._episode_path = self._create_episode_path()

        # Open robot writer
        self._robot_writer = RobotDataWriter(self._episode_path)
        self._robot_writer.open()

        # Start camera process (might fail if USB perms not set)
        self._start_event.clear()
        self._stop_event.clear()
        try:
            self._cam_process = CameraDataProcess(
                start_event=self._start_event,
                stop_event=self._stop_event,
                episode_path=self._episode_path,
                camera_configs=self.camera_configs,
            )
            self._cam_process.start()
        except Exception as e:
            self.get_logger().warn(f"Camera process failed to start: {e}")
            self._cam_process = None

        # Reset data tracking
        with self._data_lock:
            for k in self._source_active:
                self._source_active[k] = False
                self._source_warned[k] = False
            for k in self._latest:
                self._latest[k] = None

        # Wait for camera thread to finish enumeration (~3s), then print status
        time.sleep(4.0)
        self._print_camera_status()

        self.get_logger().info(f"--- Episode {self._episode_idx} ready ---")
        self.get_logger().info(f"  Path: {self._episode_path}")
        self.get_logger().info("  Use keyboard controller (T3) to start/stop recording.")
        sys.stdout.flush()

    def _start_recording(self):
        """Begin recording data."""
        if self.state == "recording":
            return

        if not self._teleop_armed:
            self.get_logger().info("⚠️ Not armed yet — clench both hands first!")
            sys.stdout.flush()
            return

        # Auto-create episode if writer was closed (after previous stop)
        if self._robot_writer is None:
            self._start_episode()

        # Start camera recording
        self._start_event.set()
        self.state = "recording"
        self.get_logger().info("[RECORDING] Started. Send 'stop' to save, 'discard' to abort.")
        sys.stdout.flush()

    def _pause_recording(self):
        """Pause recording."""
        if self.state != "recording":
            return
        self._stop_event.set()
        self.state = "paused"
        print_status("paused")
        print_colored("  Press [s] to resume, [q] to save, [d] to discard", 37)

    def _resume_recording(self):
        """Resume recording after pause."""
        if self.state != "paused":
            return

        # Create new camera process for resume
        self._start_event = mp.Event()
        self._stop_event = mp.Event()
        self._cam_process = CameraDataProcess(
            start_event=self._start_event,
            stop_event=self._stop_event,
            episode_path=self._episode_path,
            camera_configs=self.camera_configs,
        )
        self._cam_process.start()
        time.sleep(1.0)  # Warm-up
        self._start_event.set()
        self.state = "recording"
        print_status("recording (resumed)")

    def _stop_and_save(self):
        """Stop recording and save episode data."""
        was_recording = self.state == "recording"
        self.state = "saving"

        # Stop camera process
        self._stop_event.set()
        if self._cam_process is not None:
            self._cam_process.join(timeout=3.0)
            self._cam_process = None

        # Close robot writer
        n_frames = self._robot_writer.frame_count if self._robot_writer else 0
        if self._robot_writer is not None:
            self._robot_writer.close()
            self._robot_writer = None

        # Save latency/accuracy metrics
        self._save_metrics()

        # Publish disarm signal (ignore errors during shutdown)
        try:
            self._disarm_pub.publish(Bool(data=True))
        except Exception:
            pass

        self.get_logger().info(
            f"[SAVED] Episode {self._episode_idx}: {n_frames} frames -> {self._episode_path}"
        )
        sys.stdout.flush()
        self._episode_idx += 1
        self.state = "idle"

    def _discard_episode(self):
        """Discard current episode data."""
        self.state = "discard"

        # Stop camera process
        self._stop_event.set()
        if self._cam_process is not None:
            self._cam_process.join(timeout=3.0)
            self._cam_process = None

        # Close and delete robot data
        if self._robot_writer is not None:
            self._robot_writer.close()
            self._robot_writer = None

        # Remove episode files
        if self._episode_path and os.path.exists(self._episode_path):
            import shutil
            shutil.rmtree(self._episode_path)

        # Publish disarm signal (ignore errors during shutdown)
        try:
            self._disarm_pub.publish(Bool(data=True))
        except Exception:
            pass

        self.get_logger().info(f"[DISCARDED] Episode {self._episode_idx} discarded.")
        sys.stdout.flush()
        self.state = "idle"

    def _save_metrics(self):
        """Save buffered latency/accuracy metrics to HDF5."""
        import h5py
        import numpy as np
        try:
            with self._metrics_lock:
                buffers = {k: list(v) for k, v in self._metrics_buffer.items()}
                for k in self._metrics_buffer:
                    self._metrics_buffer[k].clear()

            if not any(buffers.values()):
                return

            metrics_path = os.path.join(self._episode_path, "metrics.h5")
            with h5py.File(metrics_path, "w") as f:
                for source, data in buffers.items():
                    if not data:
                        continue
                    arr = np.array(data, dtype=np.float64)
                    f.create_dataset(source, data=arr)
                    # Summary stats
                    if source.endswith("_arm") and arr.shape[1] >= 8:
                        grp = f.create_group(f"{source}/summary")
                        grp.attrs["e2e_mean_ms"] = float(np.mean(arr[:, 5]))
                        grp.attrs["num_frames"] = len(arr)
                    elif source.endswith("_hand") and arr.shape[1] >= 6:
                        grp = f.create_group(f"{source}/summary")
                        grp.attrs["e2e_mean_ms"] = float(np.mean(arr[:, 5]))
                        grp.attrs["num_frames"] = len(arr)
            print(f"[Metrics] Saved {metrics_path}")
        except Exception as e:
            self.get_logger().warn(f"Metrics save failed (non-fatal): {e}")

    def _next_episode(self):
        """Stop current episode and start a new one."""
        self._stop_and_save()
        self._start_episode()

    def _print_camera_status(self):
        """Print summary of which expected cameras are online vs missing."""
        if self._cam_process is None:
            self.get_logger().warn("=== Camera status: NO camera process ===")
            return
        status = getattr(self._cam_process, 'cameras_status', {})
        online = [cid for cid, s in status.items() if s == "online"]
        offline = [cid for cid, s in status.items() if s != "online"]
        self.get_logger().info("=" * 44)
        self.get_logger().info(f"=== Camera status: {len(online)}/{len(status)} online ===")
        for cid in online:
            self.get_logger().info(f"  ✅ {cid} ONLINE")
        for cid in offline:
            self.get_logger().info(f"  ❌ {cid} OFFLINE — check USB connection")
        self.get_logger().info("=" * 44)
        sys.stdout.flush()

    # ==================================================================
    # Main loop
    # ==================================================================

    def _svc(self, cmd: str, response):
        """Service callback wrapper — runs heavy work in a thread."""
        # Quick-return commands (no blocking)
        if cmd in ("start", "pause"):
            self._handle_command(cmd)
        else:
            # Heavy commands (stop, discard, next) run in thread to avoid blocking service response
            threading.Thread(target=lambda: self._handle_command(cmd), daemon=True).start()
        response.success = True
        response.message = f"Command '{cmd}' accepted (state={self.state})"
        return response

    def _handle_command(self, key: str):
        """Process a data collection command."""
        self.get_logger().info(f"Received command: '{key}' (state={self.state})")
        if not key:
            return
        if key in ("s", "start"):
            if self.state == "idle":
                self._start_recording()
            elif self.state == "paused":
                self._resume_recording()
        elif key in ("p", "pause"):
            if self.state == "recording":
                self._pause_recording()
            elif self.state == "paused":
                self._resume_recording()
        elif key in ("q", "stop", "save"):
            if self.state in ("recording", "paused"):
                self._stop_and_save()
            else:
                self.get_logger().info("Not recording — send 'start' first.")
        elif key in ("d", "discard"):
            if self.state in ("recording", "paused"):
                self._discard_episode()
        elif key in ("n", "next"):
            if self.state in ("recording", "paused"):
                self._next_episode()
        elif key in ("pause", "p"):
            if self.state == "recording":
                self._pause_recording()
        sys.stdout.flush()

    def spin_with_keyboard(self):
        """Main event loop. Commands via FIFO: echo start > /tmp/collect_cmd.fifo"""

        # Start first episode
        self._start_episode()

        self.get_logger().info("TCP server: localhost:19999")
        self.get_logger().info("  echo start | nc 127.0.0.1 19999")
        self.get_logger().info("  echo stop  | nc 127.0.0.1 19999")

        self._spin_running = True

        while rclpy.ok() and self._spin_running:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                break

            # Process FIFO commands (thread-safe queue)
            while self._cmd_queue:
                self._handle_command(self._cmd_queue.pop(0))

    # ==================================================================
    # Cleanup
    # ==================================================================

    def cleanup(self):
        """Clean shutdown."""
        self.get_logger().info("Cleaning up...")

        # Stop recording if active
        if self.state in ("recording", "paused"):
            self._stop_and_save()

        self.get_logger().info("Shutdown complete.")

    def destroy_node(self):
        self.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DataCollectNode()

    # Setup signal handlers
    def sig_handler(signum, frame):
        node.get_logger().info(f"Signal {signum} received, shutting down.")
        node._spin_running = False
        node.cleanup()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        node.spin_with_keyboard()
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception as e:
        node.get_logger().error(f"Error in main loop: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

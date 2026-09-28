#!/usr/bin/env python3

import os
import time
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray

from nero_quest_teleop.latency_tracker import (
    LatencyTracker, arm_metrics_to_array,
)
from nero_quest_teleop.fps_counter import FPSCounter

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW
from ament_index_python.packages import get_package_share_directory

from nero_quest_teleop.ik_solver import IKSolver, NeroParams
from nero_quest_teleop.pose_processor import PoseProcessor
from nero_quest_teleop.safety_filter import SafetyFilter


class NeroTeleopNode(Node):
    def __init__(self):
        super().__init__("nero_teleop_node")

        # ==================== Parameters ====================
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("can_channel", "can_nero_left")
        self.declare_parameter("firmware_version", "default")
        self.declare_parameter("can_interface", "socketcan")

        # IK solver configuration
        self.declare_parameter("solver_type", "analytic_dh")
        self.declare_parameter("ik_service_name", "/ik_solver/solve_ik")

        # URDF / numerical IK params (used when solver_type="urdf_numerical")
        self.declare_parameter("urdf_package", "agx_arm_description")
        self.declare_parameter("urdf_relative_path", "agx_arm_urdf/nero/urdf/nero_description.urdf")
        self.declare_parameter("ik_w_pos", 20.0)
        self.declare_parameter("ik_w_ori", 2.0)
        self.declare_parameter("ik_w_reg", 0.01)
        self.declare_parameter("ik_w_smooth", 2.0)
        self.declare_parameter("ik_max_iter", 8)
        self.declare_parameter("ik_tol", 1e-3)

        # Pose processor parameters
        # Primary: 3x3 rotation matrix (column-major in yaml)
        self.declare_parameter("vr_to_arm_rot", [
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
        ])
        self.declare_parameter("pos_smoothing", 0.5)
        self.declare_parameter("rot_smoothing", 0.7)
        self.declare_parameter("motion_scale", 0.65)
        self.declare_parameter("flip_pitch", False)

        # Safety parameters
        self.declare_parameter("max_joint_vel", 0.05)
        self.declare_parameter("print_metrics", False)  # print full latency breakdown
        self.declare_parameter("print_ik_stats", True)   # print IK success rate + target pose
        self.declare_parameter("dry_run", False)         # True = don't move arm
        self.declare_parameter("workspace_radius", 0.58)
        self.declare_parameter("workspace_z_min", float('-inf'))
        self.declare_parameter("workspace_z_max", float('inf'))
        self.declare_parameter("workspace_x_min", float('-inf'))
        self.declare_parameter("workspace_x_max", float('inf'))
        self.declare_parameter("workspace_y_min", float('-inf'))
        self.declare_parameter("workspace_y_max", float('inf'))
        self.declare_parameter("data_timeout", 1.5)

        # TCP offset — transform from flange to gripper [x, y, z, roll, pitch, yaw]
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Initial pose
        self.declare_parameter("init_pose")
        self.declare_parameter("init_speed_percent", 5)
        self.declare_parameter("move_to_init_pose", True)

        # ==================== Read parameters ====================
        self.arm_side = self.get_parameter("arm_side").value.lower()
        control_rate = self.get_parameter("control_rate").value
        can_channel = self.get_parameter("can_channel").value
        fw_version = self.get_parameter("firmware_version").value
        can_interface = self.get_parameter("can_interface").value

        solver_type = self.get_parameter("solver_type").value
        self.get_logger().info(
            f"[{self.arm_side.upper()}] Initializing Nero teleop node "
            f"(IK: {solver_type})..."
        )

        # ==================== Robot connection ====================
        self.get_logger().info(
            f"Connecting to Nero arm on {can_interface}://{can_channel} (fw={fw_version})..."
        )
        cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=NeroFW.V111 if fw_version == "v111" else NeroFW.DEFAULT,
            channel=can_channel,
            interface=can_interface,
        )
        self.robot = AgxArmFactory.create_arm(cfg)
        self.robot.connect()

        # Enable arm
        self.get_logger().info("Enabling arm...")
        while not self.robot.enable():
            time.sleep(0.01)
        self.get_logger().info("Arm enabled.")

        # Switch to move_js (MIT passthrough) mode for low-latency streaming
        self.robot.set_motion_mode("js")
        self.robot.set_auto_set_motion_mode_enabled(False)

        # TCP offset
        tcp_offset = list(self.get_parameter("tcp_offset").value)
        if any(v != 0.0 for v in tcp_offset):
            self.robot.set_tcp_offset(tcp_offset)

        # Read initial arm state
        init_ja = self.robot.get_joint_angles()
        if init_ja is not None:
            self.init_joint_angles = np.array(init_ja.msg)
        else:
            self.init_joint_angles = np.zeros(7)

        init_flange = self.robot.get_flange_pose()
        if init_flange is not None:
            fp = init_flange.msg
            self.physical_pos = np.array(fp[:3])
            self.physical_rot = Rotation.from_euler("xyz", fp[3:]).as_matrix()
        else:
            self.physical_pos = np.array([-0.45, 0.0, 0.45])
            self.physical_rot = Rotation.from_euler(
                "xyz", [-1.5708, 0.0, -3.14159]
            ).as_matrix()

        self.get_logger().info(
            f"Physical flange: pos={self.physical_pos.tolist()}"
        )

        # ==================== XHand publisher (for shutdown homing) ====================
        self._xhand_home_pub = None
        try:
            from xhand_control_interfaces.msg import XHandCommand
            self._xhand_home_pub = self.create_publisher(
                XHandCommand,
                f"/{self.arm_side}_hand/xhand_command",
                10,
            )
        except ImportError:
            pass
        # Note: initial clench is handled by xhand_retargeting node on startup

        # ----------- Move to initial pose at low speed (optional) -----------
        if self.get_parameter("move_to_init_pose").value:
            self._move_to_init_pose()
            # Re-read joint angles after move
            init_ja = self.robot.get_joint_angles()
            if init_ja is not None:
                self.init_joint_angles = np.array(init_ja.msg)

        # ==================== IK Solver ====================
        solver_type = self.get_parameter("solver_type").value
        if solver_type == "analytic_dh":
            self.ik_solver = IKSolver()
            self.get_logger().info(
                f"IK solver: analytic_dh, nq={self.ik_solver.nq}"
            )
            self.ik_solver.sync_state(self.init_joint_angles[:self.ik_solver.nq])

            # Use DH-model FK as reference (DH and physical kinematics differ)
            T_ref = self.ik_solver.fk(self.init_joint_angles[:self.ik_solver.nq])
            self.robot_init_pos = T_ref[:3, 3].copy()
            self.robot_init_rot = T_ref[:3, :3].copy()
            self.get_logger().info(f"DH-model FK ref: pos={self.robot_init_pos.tolist()}")

            # Warm-up: one solve to prime branch tracking
            self.get_logger().info("Warming up IK solver...")
            t_warm = time.perf_counter()
            _warm_sol = self.ik_solver.solve(T_ref)
            if _warm_sol is not None:
                self.get_logger().info(
                    f"IK warm-up ok ({((time.perf_counter()-t_warm)*1000):.0f}ms)"
                )
            else:
                self.get_logger().warn("IK warm-up failed, first frame may be slow")

        elif solver_type == "urdf_numerical":
            from nero_quest_teleop.urdf_ik_solver import URDFIKSolver
            urdf_pkg = self.get_parameter("urdf_package").value
            urdf_rel = self.get_parameter("urdf_relative_path").value
            try:
                urdf_path = os.path.join(
                    get_package_share_directory(urdf_pkg), urdf_rel
                )
            except Exception:
                self.get_logger().error(f"Cannot find URDF package '{urdf_pkg}'")
                raise
            self.get_logger().info(f"Loading URDF from: {urdf_path}")
            self.ik_solver = URDFIKSolver(
                urdf_path=urdf_path,
                package_dirs=[
                    os.path.dirname(os.path.dirname(os.path.dirname(
                        os.path.dirname(os.path.dirname(urdf_path))
                    )))
                ],
                locked_joints=["joint8"],
                max_iter=int(self.get_parameter("ik_max_iter").value),
                tol=float(self.get_parameter("ik_tol").value),
                w_pos=float(self.get_parameter("ik_w_pos").value),
                w_ori=float(self.get_parameter("ik_w_ori").value),
                w_reg=float(self.get_parameter("ik_w_reg").value),
                w_smooth=float(self.get_parameter("ik_w_smooth").value),
            )
            self.get_logger().info(
                f"IK solver: urdf_numerical (IPOPT), nq={self.ik_solver.nq}"
            )
            self.ik_solver.sync_state(self.init_joint_angles[:self.ik_solver.nq])

            # Use solver's own FK as reference for IK consistency
            T_ref = self.ik_solver.fk(self.init_joint_angles[:self.ik_solver.nq])
            self.robot_init_pos = T_ref[:3, 3].copy()
            self.robot_init_rot = T_ref[:3, :3].copy()
            self.get_logger().info(
                f"URDF FK ref: pos={self.robot_init_pos.tolist()}"
            )

            # Warm-up
            self.get_logger().info("Warming up URDF IK solver...")
            t_warm = time.perf_counter()
            T_ref = self.ik_solver.fk(self.init_joint_angles[:self.ik_solver.nq])
            _warm_sol = self.ik_solver.solve(T_ref)
            if _warm_sol is not None:
                self.get_logger().info(
                    f"IK warm-up ok ({((time.perf_counter()-t_warm)*1000):.0f}ms)"
                )
            else:
                self.get_logger().warn("IK warm-up failed, first frame may be slow")

        else:
            self.get_logger().error(f"Unknown solver_type: {solver_type}")
            raise ValueError(f"Unknown solver_type: {solver_type}")

        # Joint limits for safety filter
        _dp = NeroParams.default()
        self._ik_lower_limits = _dp.joint_limits[:, 0].copy()
        self._ik_upper_limits = _dp.joint_limits[:, 1].copy()
        self._ik_nq = 7

        self.safety_filter = SafetyFilter(
            joint_lower_limits=self.ik_solver.lower_limits,
            joint_upper_limits=self.ik_solver.upper_limits,
            max_joint_vel=float(self.get_parameter("max_joint_vel").value),
            workspace_radius=float(self.get_parameter("workspace_radius").value),
            workspace_z_min=float(self.get_parameter("workspace_z_min").value),
            workspace_z_max=float(self.get_parameter("workspace_z_max").value),
            workspace_x_min=float(self.get_parameter("workspace_x_min").value),
            workspace_x_max=float(self.get_parameter("workspace_x_max").value),
            workspace_y_min=float(self.get_parameter("workspace_y_min").value),
            workspace_y_max=float(self.get_parameter("workspace_y_max").value),
            collision_check_fn=None,
        )
        self.safety_filter.set_initial_state(self.init_joint_angles[:self._ik_nq])

        # ==================== Pose Processor ====================
        # Read vr_to_arm_rot: 9-element flat list → 3x3 matrix
        vr_rot_flat = self.get_parameter("vr_to_arm_rot").value
        self.R_vr_to_arm = np.array(vr_rot_flat, dtype=float).reshape(3, 3)

        self.pose_processor = PoseProcessor(
            vr_to_arm_rot=self.R_vr_to_arm,
            pos_smoothing=float(self.get_parameter("pos_smoothing").value),
            rot_smoothing=float(self.get_parameter("rot_smoothing").value),
            motion_scale=float(self.get_parameter("motion_scale").value),
            flip_pitch=bool(self.get_parameter("flip_pitch").value),
        )

        # ==================== Pre-compute TCP transforms ====================
        tcp_off = self.get_parameter("tcp_offset").value  # [x,y,z,roll,pitch,yaw]
        self._T_flange_to_tcp = np.eye(4, dtype=float)
        self._T_flange_to_tcp[:3, :3] = Rotation.from_euler(
            "xyz", tcp_off[3:]
        ).as_matrix()
        self._T_flange_to_tcp[:3, 3] = np.array(tcp_off[:3], dtype=float)
        # Inverse: TCP → flange
        self._T_tcp_to_flange = np.linalg.inv(self._T_flange_to_tcp)

        # ==================== State ====================
        self.last_vr_time = time.monotonic()
        self.data_timeout = float(self.get_parameter("data_timeout").value)
        self.vr_received = False
        self.emergency_stopped = False
        # Clench trigger: if require_clench_to_start is false, arm immediately
        self.declare_parameter("require_clench_to_start", True)
        self._teleop_armed = not self.get_parameter("require_clench_to_start").value
        self._print_metrics = self.get_parameter("print_metrics").value
        self._print_ik_stats_enabled = self.get_parameter("print_ik_stats").value
        self._dry_run = self.get_parameter("dry_run").value
        self._require_clench = self.get_parameter("require_clench_to_start").value

        # IK stats tracking (lightweight, always on)
        self._ik_success_count = 0
        self._ik_fail_count = 0
        self._ik_total_ms = 0.0
        self._ik_last_print = time.monotonic()
        self._ik_print_interval = 3.0  # print every 3 seconds
        self.prev_loop_time = time.monotonic()
        self.ik_fail_count = 0

        # ==================== Subscribers ====================
        wrist_topic = f"quest3/{self.arm_side}_wrist_pose"
        self.create_subscription(PoseStamped, wrist_topic, self._vr_callback, 10)
        self.get_logger().info(f"Subscribed to: {wrist_topic}")

        # Clench-trigger armed signal from retargeting node
        self.create_subscription(
            Bool, "/teleop/armed", self._armed_callback, 10,
        )
        self.create_subscription(
            Bool, "/teleop/disarm", self._disarm_callback, 10,
        )

        # ==================== Publishers (debug/viz) ====================
        self.target_pose_pub = self.create_publisher(
            PoseStamped, f"nero_teleop/{self.arm_side}_target_ee", 10
        )

        # Feedback publishers for data collection
        self.joint_state_pub = self.create_publisher(
            JointState, f"~/joint_states", 10
        )
        self.tcp_pose_pub = self.create_publisher(
            PoseStamped, f"~/tcp_pose", 10
        )

        # Metrics publisher for latency/accuracy tracking
        self.metrics_pub = self.create_publisher(
            Float64MultiArray, f"~/metrics", 10
        )
        self._latency_tracker = LatencyTracker(
            f"{self.arm_side}_arm", print_interval=2.0
        )

        # XHand home publisher already created above (before move_to_init_pose)

        # ==================== Control Timer ====================
        self.dt = 1.0 / control_rate
        self.timer = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(f"Control loop @ {control_rate:.0f} Hz. Node ready.")

        # ==================== Feedback thread (20Hz CAN read, independent of control loop) ====================
        self._fb_running = True
        self._last_target = None
        self._fb_thread = threading.Thread(target=self._feedback_thread, daemon=True)
        self._fb_thread.start()
        self.get_logger().info("Feedback thread started @ 20Hz")

    # ------------------------------------------------------------------
    def _feedback_thread(self):
        """Independent thread: read CAN + publish joint_states at 20Hz."""
        while self._fb_running and rclpy.ok():
            try:
                time.sleep(0.05)
                ja = self.robot.get_joint_angles()
                fp = self.robot.get_flange_pose()
                if ja is None or fp is None:
                    continue

                # Debug: log actual joint count from CAN
                n_joints = len(ja.msg)
                if not hasattr(self, '_fb_joint_count_logged'):
                    self.get_logger().info(f"CAN returns {n_joints} joints: {[f'{v:.3f}' for v in ja.msg]}")
                    self._fb_joint_count_logged = True

                actual_joints = np.array(ja.msg[:7], dtype=float)
                actual_pos = np.array(fp.msg[:3], dtype=float)
                actual_rpy = fp.msg[3:6]
                q_actual = Rotation.from_euler("xyz", actual_rpy).as_quat()

                # Joint states
                js_msg = JointState()
                js_msg.header.stamp = self.get_clock().now().to_msg()
                js_msg.name = [f"joint{i+1}" for i in range(7)]
                js_msg.position = [float(v) for v in actual_joints]
                self.joint_state_pub.publish(js_msg)

                # TCP pose
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.pose.position.x = float(actual_pos[0])
                pose_msg.pose.position.y = float(actual_pos[1])
                pose_msg.pose.position.z = float(actual_pos[2])
                pose_msg.pose.orientation.x = float(q_actual[0])
                pose_msg.pose.orientation.y = float(q_actual[1])
                pose_msg.pose.orientation.z = float(q_actual[2])
                pose_msg.pose.orientation.w = float(q_actual[3])
                self.tcp_pose_pub.publish(pose_msg)

                # Metrics
                if self._last_target is not None:
                    target_pos = self._last_target[:3, 3]
                    target_quat = Rotation.from_matrix(self._last_target[:3, :3]).as_quat()
                    vr_stamp = getattr(self, '_vr_header_stamp', 0.0)
                    self._latency_tracker.stamp("feedback")
                    metrics = self._latency_tracker.compute_arm_metrics(
                        vr_header_stamp=vr_stamp, target_pos=target_pos,
                        actual_pos=actual_pos, target_quat=target_quat,
                        actual_quat=q_actual,
                    )
                    self.metrics_pub.publish(Float64MultiArray(data=arm_metrics_to_array(metrics)))
            except Exception:
                pass

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    def _print_ik_stats(self, t_ms: float, T_target):
        """Print IK success rate and target pose periodically."""
        if not self._print_ik_stats_enabled:
            return
        now = time.monotonic()
        if now - self._ik_last_print < self._ik_print_interval:
            return
        self._ik_last_print = now
        total = self._ik_success_count + self._ik_fail_count
        if total == 0:
            return
        rate = 100.0 * self._ik_success_count / total
        avg_ms = self._ik_total_ms / max(1, self._ik_success_count)
        self.get_logger().info(
            f"[{self.arm_side.upper()}] IK: {rate:.1f}% ok ({avg_ms:.1f}ms) | "
            f"target=({T_target[0,3]:.3f},{T_target[1,3]:.3f},{T_target[2,3]:.3f}) "
            f"fail={self._ik_fail_count}"
        )

    # ------------------------------------------------------------------
    def _armed_callback(self, msg: Bool):
        """Receive clench-trigger armed signal from retargeting node."""
        if not self._teleop_armed and msg.data:
            self._teleop_armed = True
            self.get_logger().info(
                f"[{self.arm_side.upper()}] Teleop ARMED — starting control."
            )

    def _disarm_callback(self, msg: Bool):
        """Return arm to init pose and reset armed after episode save."""
        if not self._require_clench:
            return
        self._teleop_armed = False
        self.get_logger().info(
            f"[{self.arm_side.upper()}] DISARMED — returning to init pose."
        )
        # Move back to init pose
        try:
            init_pose = list(self.get_parameter("init_pose").value)
            self.robot.set_motion_mode("j")
            self.robot.set_auto_set_motion_mode_enabled(False)
            self.robot.set_speed_percent(25)
            self.robot.move_j(init_pose)
            start = time.monotonic()
            while (time.monotonic() - start) < 10.0:
                ja = self.robot.get_joint_angles()
                if ja is not None:
                    err = np.max(np.abs(np.array(ja.msg[:7]) - np.array(init_pose)))
                    if err < 0.05:
                        self.get_logger().info(
                            f"[{self.arm_side.upper()}] Init pose reached."
                        )
                        break
                time.sleep(0.1)
            self.robot.set_motion_mode("js")
            self.robot.set_auto_set_motion_mode_enabled(False)
        except Exception as e:
            self.get_logger().warn(f"Return to init failed: {e}")

    def _vr_callback(self, msg: PoseStamped):
        # Record VR arrival for latency tracking (non-invasive)
        self._latency_tracker.stamp("vr_cb", time.monotonic())
        self._vr_header_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = [
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w,
        ]

        # Validate quaternion
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            self.get_logger().warn("Invalid quaternion from VR, skipping.")
            return

        rot = Rotation.from_quat(q)

        # Validate position jump
        if self.pose_processor.is_calibrated:
            delta = np.linalg.norm(pos - self.pose_processor.vr_init_pos)
            if delta > 2.0:
                self.get_logger().warn(
                    f"Large VR jump ({delta:.2f}m), recalibrating zero point."
                )
                self.pose_processor.reset()

        self.pose_processor.update_vr_pose(pos, rot)
        self.last_vr_time = time.monotonic()
        self.vr_received = True

    # ------------------------------------------------------------------
    def _control_loop(self):
        now = time.monotonic()
        dt = now - self.prev_loop_time
        self.prev_loop_time = now

        # FPS tracking
        if not hasattr(self, '_ctrl_fps'):
            self._ctrl_fps = FPSCounter(window=200, print_interval=3.0)
        ctrl_fps = self._ctrl_fps.tick()
        if self._ctrl_fps.should_print():
            self.get_logger().info(
                f"[{self.arm_side.upper()}] Control loop: {ctrl_fps:.0f} Hz "
                f"(armed={self._teleop_armed})"
            )

        if self.emergency_stopped:
            return

        # --- Clench trigger not yet armed ---
        if not self._teleop_armed:
            # Keep reading VR for latency tracking, but don't move
            return

        # --- VR data timeout -> hold position ---
        if self.vr_received and (now - self.last_vr_time) > self.data_timeout:
            self.get_logger().warn(
                f"VR data timeout ({now - self.last_vr_time:.1f}s). Holding.",
                throttle_duration_sec=2.0,
            )
            return

        if not self.vr_received:
            return

        # --- Process VR pose -> delta in arm frame ---
        delta_pos, delta_rot = self.pose_processor.process()

        # --- Compute gripper target, then flange target ---
        T_gripper = self.pose_processor.compute_target_pose(
            delta_pos, delta_rot, self.robot_init_pos, self.robot_init_rot
        )

        # --- Convert gripper target to flange target ---
        T_flange_target = T_gripper @ self._T_tcp_to_flange

        # --- Workspace check on flange position ---
        flange_pos = T_flange_target[:3, 3].copy()
        clamped_flange = self.safety_filter.check_workspace(flange_pos)
        if not np.allclose(flange_pos, clamped_flange):
            T_flange_target[:3, 3] = clamped_flange

        # --- IK Solve ---
        self._latency_tracker.stamp("ik_start")
        t_solve_0 = time.perf_counter()
        sol_q = self.ik_solver.solve(T_flange_target)
        t_solve_ms = (time.perf_counter() - t_solve_0) * 1000
        self._latency_tracker.stamp("ik_end")
        if sol_q is None:
            self._ik_fail_count += 1
            target_str = f"({T_flange_target[0,3]:.3f},{T_flange_target[1,3]:.3f},{T_flange_target[2,3]:.3f})"
            self.get_logger().warn(
                f"IK FAIL ({t_solve_ms:.0f}ms) target={target_str}",
                throttle_duration_sec=1.0,
            )
            # Print IK stats periodically
            return
        self._ik_success_count += 1
        self._ik_total_ms += t_solve_ms

        # --- Safety filter ---
        self._latency_tracker.stamp("safety_start")
        safe_q, info = self.safety_filter.filter(sol_q, dt)
        self._latency_tracker.stamp("safety_end")

        if info["collision"]:
            self.get_logger().warn("Self-collision! Holding.", throttle_duration_sec=1.0)
            return

        # --- Build 7-joint command ---
        cmd = np.zeros(7)
        n = min(len(safe_q), 7)
        cmd[:n] = safe_q[:n]

        # Store target for feedback loop
        self._last_target = T_flange_target

        # --- Send to robot ---
        if not self._dry_run:
            try:
                self.robot.move_js(cmd.tolist())
                self._latency_tracker.stamp("cmd_sent")  # physical control starts
            except Exception as e:
                self.get_logger().error(f"move_js failed: {e}")
        else:
            self._latency_tracker.stamp("cmd_sent")  # still track for metrics

    # ------------------------------------------------------------------
    def _move_to_init_pose(self):
        """Move arm to initial teleop pose at low speed, then restore full speed."""
        init_pose = list(self.get_parameter("init_pose").value)
        init_speed = int(self.get_parameter("init_speed_percent").value)

        self.get_logger().warn(
            f"[{self.arm_side.upper()}] Moving to initial pose at {init_speed}% speed: "
            f"{[f'{a:.2f}' for a in init_pose]}"
        )

        self.robot.set_speed_percent(init_speed)
        self.robot.set_motion_mode("j")
        self.robot.set_auto_set_motion_mode_enabled(False)
        self.robot.move_j(init_pose)

        arrival_threshold = 0.05
        timeout = 15.0
        start = time.monotonic()
        last_print = start

        while (time.monotonic() - start) < timeout:
            ja = self.robot.get_joint_angles()
            if ja is not None:
                current = np.array(ja.msg[:7])
                error = np.max(np.abs(current - np.array(init_pose)))
                now_t = time.monotonic()
                if now_t - last_print >= 2.0:
                    self.get_logger().info(
                        f"[{self.arm_side.upper()}] Init pose approach: "
                        f"error={error:.3f} rad, elapsed={now_t - start:.1f}s"
                    )
                    last_print = now_t
                if error < arrival_threshold:
                    break
            time.sleep(0.05)

        self.robot.set_speed_percent(100)
        self.get_logger().warn(
            f"[{self.arm_side.upper()}] Initial pose reached. Speed restored to 100%."
        )

        self.robot.set_motion_mode("js")
        self.robot.set_auto_set_motion_mode_enabled(False)

    # ------------------------------------------------------------------
    def emergency_stop(self):
        if not self.emergency_stopped:
            self.emergency_stopped = True
            self.robot.electronic_emergency_stop()
            self.get_logger().warn("E-STOP triggered!")

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info("Shutting down Nero teleop node...")
        self._fb_running = False
        try:
            # === Hand home first (fast, just publish command) ===
            if self._xhand_home_pub is not None:
                try:
                    from xhand_control_interfaces.msg import XHandCommand
                    import math
                    home_deg = (
                        0.0, 80.66, 33.2,   # thumb
                        0.0, 5.11, 5.0,     # index
                        6.53, 5.0,           # mid
                        6.76, 5.0,           # ring
                        10.13, 5.0,          # pinky
                    )
                    home_rad = [math.radians(d) for d in home_deg]
                    msg = XHandCommand()
                    msg.hand_id = 0
                    msg.name = [
                        "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
                        "index_bend_joint", "index_joint1", "index_joint2",
                        "mid_joint1", "mid_joint2",
                        "ring_joint1", "ring_joint2",
                        "pinky_joint1", "pinky_joint2",
                    ]
                    msg.position = home_rad
                    msg.kp = [80.0] * 12
                    msg.ki = [0.0] * 12
                    msg.kd = [0.0] * 12
                    msg.effort_limit = [400.0] * 12
                    msg.mode = 3
                    self._xhand_home_pub.publish(msg)
                    self.get_logger().info(
                        f"[{self.arm_side.upper()}] Hand home command sent."
                    )
                except Exception:
                    pass  # ROS2 context already shutting down, expected

            # === Arm return-to-zero sequence ===
            # Read current joint angles
            ja = self.robot.get_joint_angles()
            if ja is not None:
                q = np.array(ja.msg[:7], dtype=float)
                self.get_logger().info(
                    f"Current joints: {[f'{v:.3f}' for v in q]}"
                )

                self.robot.set_motion_mode("j")
                self.robot.set_auto_set_motion_mode_enabled(False)
                self.robot.set_speed_percent(25)

                def _move_and_wait(target, label, timeout=4.0):
                    self.get_logger().info(f"  → {label}")
                    self.robot.move_j(list(target))
                    start = time.monotonic()
                    while (time.monotonic() - start) < timeout:
                        ja2 = self.robot.get_joint_angles()
                        if ja2 is not None:
                            q2 = np.array(ja2.msg[:7], dtype=float)
                            # Only check error on joints we're moving
                            move_mask = np.abs(target - q) > 0.02
                            if not np.any(move_mask):
                                break
                            err = np.max(np.abs(q2[move_mask] - target[move_mask]))
                            if err < 0.05:
                                self.get_logger().info(f"    {label} done.")
                                break
                        time.sleep(0.1)
                    q[:] = target  # update reference for next step
                    return target

                # --- Step 1: J2 to 90°, J4 to 0 (indices 1, 3) ---
                t1 = q.copy()
                t1[1] = math.pi / 2  # J2 → 90°
                t1[3] = 0.0          # J4 → 0
                _move_and_wait(t1, "Step 1/3: J2→90°  J4→0")

                # --- Step 2: J1 (index 0) ---
                t2 = q.copy()
                t2[0] = 0.0  # J1
                _move_and_wait(t2, "Step 2/3: J1 → 0")

                # --- Step 3: all remaining to zero (J2 stays at 90°) ---
                t3 = np.zeros(7, dtype=float)
                t3[1] = math.pi / 2  # J2 → 90°
                _move_and_wait(t3, "Step 3/3: all → 0 (J2=90°)")

            self.robot.set_normal_mode()
            time.sleep(0.1)
            self.robot.disconnect()
        except Exception as e:
            self.get_logger().warn(f"Shutdown error: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NeroTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

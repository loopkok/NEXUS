#!/usr/bin/env python3
"""Safe teleop test WITH real Nero arm — conservative velocity limits.

This version connects to the actual arm but uses very restrictive safety
parameters to ensure the arm moves slowly and predictably.

Usage:
  ros2 run nero_quest_teleop test_safe_teleop --ros-args \\
    -p arm_side:=left \\
    -p can_channel:=can_nero_left
"""

import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, NeroFW

from nero_quest_teleop.ik_solver import IKSolver, fk as analytical_fk
from nero_quest_teleop.pose_processor import PoseProcessor
from nero_quest_teleop.safety_filter import SafetyFilter


class SafeTeleopTestNode(Node):
    def __init__(self):
        super().__init__("test_safe_teleop")

        # ----------- Conservative parameters -----------
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("can_channel", "can_nero_left")
        self.declare_parameter("firmware_version", "default")
        self.declare_parameter("can_interface", "socketcan")
        self.declare_parameter("control_rate", 50.0)

        # Pose — very conservative
        self.declare_parameter("vr_to_arm_rot", [
            0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0,
        ])
        self.declare_parameter("pos_smoothing", 0.5)
        self.declare_parameter("rot_smoothing", 0.8)
        self.declare_parameter("motion_scale", 0.65)
        self.declare_parameter("flip_pitch", False)
        self.declare_parameter("solver_type", "analytic_dh")
        self.declare_parameter("ik_max_iter", 4)
        self.declare_parameter("ik_tol", 1e-4)

        # Safety — very conservative
        self.declare_parameter("max_joint_vel", 0.05)
        self.declare_parameter("workspace_radius", 0.58)
        self.declare_parameter("workspace_z_min", float('-inf'))
        self.declare_parameter("workspace_z_max", float('inf'))
        self.declare_parameter("workspace_x_min", float('-inf'))
        self.declare_parameter("workspace_x_max", float('inf'))
        self.declare_parameter("workspace_y_min", float('-inf'))
        self.declare_parameter("workspace_y_max", float('inf'))
        self.declare_parameter("data_timeout", 1.5)

        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("dry_run", True)

        arm_side = self.get_parameter("arm_side").value.lower()
        dry_run = self.get_parameter("dry_run").value

        self.get_logger().warn("=" * 60)
        self.get_logger().warn("SAFE TELEOP TEST — CONSERVATIVE MODE")
        self.get_logger().warn(f"  Arm side: {arm_side}")
        self.get_logger().warn(f"  Dry run (no motion): {dry_run}")
        self.get_logger().warn(
            f"  Max joint vel: {self.get_parameter('max_joint_vel').value} rad/step"
        )
        self.get_logger().warn(
            f"  Motion scale: {self.get_parameter('motion_scale').value}"
        )
        self.get_logger().warn("=" * 60)

        # ----------- Robot connection -----------
        can_channel = self.get_parameter("can_channel").value
        fw_version = self.get_parameter("firmware_version").value
        can_interface = self.get_parameter("can_interface").value

        self.get_logger().info(
            f"Connecting to Nero on {can_interface}://{can_channel}..."
        )
        cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=NeroFW.V111 if fw_version == "v111" else NeroFW.DEFAULT,
            channel=can_channel,
            interface=can_interface,
        )
        self.robot = AgxArmFactory.create_arm(cfg)
        self.robot.connect()

        while not self.robot.enable():
            time.sleep(0.01)
        self.get_logger().info("Arm enabled.")

        self.robot.set_motion_mode("js")
        self.robot.set_auto_set_motion_mode_enabled(False)

        # TCP offset
        tcp_offset = list(self.get_parameter("tcp_offset").value)
        if any(v != 0.0 for v in tcp_offset):
            self.robot.set_tcp_offset(tcp_offset)

        # Read initial state
        init_ja = self.robot.get_joint_angles()
        self.init_joint_angles = (
            np.array(init_ja.msg) if init_ja is not None else np.zeros(7)
        )

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

        # ----------- IK Solver -----------
        solver_type = self.get_parameter("solver_type").value
        if solver_type == "analytic_dh":
            self.ik = IKSolver()
            self.ik.sync_state(self.init_joint_angles[:self.ik.nq])
            T_ref = self.ik.fk(self.init_joint_angles[:self.ik.nq])
            self.robot_init_pos = T_ref[:3, 3].copy()
            self.robot_init_rot = T_ref[:3, :3].copy()
            self.get_logger().info(f"IK: analytic_dh, ref={self.robot_init_pos.round(3).tolist()}")
        elif solver_type == "urdf_numerical":
            from nero_quest_teleop.urdf_ik_solver import URDFIKSolver
            import os as _os
            from ament_index_python.packages import get_package_share_directory
            urdf_pkg = "agx_arm_description"
            urdf_rel = "agx_arm_urdf/nero/urdf/nero_description.urdf"
            urdf_path = _os.path.join(get_package_share_directory(urdf_pkg), urdf_rel)
            self.ik = URDFIKSolver(
                urdf_path=urdf_path, locked_joints=["joint8"],
                max_iter=int(self.get_parameter("ik_max_iter").value),
                tol=float(self.get_parameter("ik_tol").value),
            )
            self.ik.sync_state(self.init_joint_angles[:self.ik.nq])
            T_ref = self.ik.fk(self.init_joint_angles[:self.ik.nq])
            self.robot_init_pos = T_ref[:3, 3].copy()
            self.robot_init_rot = T_ref[:3, :3].copy()
            self.get_logger().info(f"IK: urdf_numerical, ref={self.robot_init_pos.round(3).tolist()}")
        else:
            raise ValueError(f"Unknown solver_type: {solver_type}")

        # ----------- Pose Processor -----------
        vr_flat = self.get_parameter("vr_to_arm_rot").value
        R_vr = np.array(vr_flat, dtype=float).reshape(3, 3)
        self.pose_proc = PoseProcessor(
            vr_to_arm_rot=R_vr,
            pos_smoothing=float(self.get_parameter("pos_smoothing").value),
            rot_smoothing=float(self.get_parameter("rot_smoothing").value),
            motion_scale=float(self.get_parameter("motion_scale").value),
            flip_pitch=bool(self.get_parameter("flip_pitch").value),
        )

        # TCP offset transform
        tcp_off = self.get_parameter("tcp_offset").value
        T_ft = np.eye(4, dtype=float)
        T_ft[:3, :3] = Rotation.from_euler("xyz", tcp_off[3:]).as_matrix()
        T_ft[:3, 3] = np.array(tcp_off[:3], dtype=float)
        self._T_tcp_to_flange = np.linalg.inv(T_ft)

        # ----------- Safety -----------
        self.safety = SafetyFilter(
            joint_lower_limits=self.ik.lower_limits,
            joint_upper_limits=self.ik.upper_limits,
            max_joint_vel=float(self.get_parameter("max_joint_vel").value),
            workspace_radius=float(self.get_parameter("workspace_radius").value),
            workspace_z_min=float(self.get_parameter("workspace_z_min").value),
            workspace_z_max=float(self.get_parameter("workspace_z_max").value),
            workspace_x_min=float(self.get_parameter("workspace_x_min").value),
            workspace_x_max=float(self.get_parameter("workspace_x_max").value),
            workspace_y_min=float(self.get_parameter("workspace_y_min").value),
            workspace_y_max=float(self.get_parameter("workspace_y_max").value),
        )
        self.safety.set_initial_state(self.init_joint_angles[:self.ik.nq])

        # ----------- State -----------
        self.dry_run = dry_run
        self.vr_received = False
        self.last_vr_time = time.monotonic()
        self.data_timeout = float(self.get_parameter("data_timeout").value)
        self.prev_loop_time = time.monotonic()
        self.frame_count = 0
        self.ik_ok = 0
        self.ik_fail = 0
        self.vr_msg_count = 0
        self.start_time = time.monotonic()
        self.last_print = self.start_time

        # ----------- Subscriber -----------
        wrist_topic = f"quest3/{arm_side}_wrist_pose"
        self.create_subscription(PoseStamped, wrist_topic, self._vr_cb, 10)
        self.get_logger().info(f"Subscribed: {wrist_topic}")

        # ----------- Timer -----------
        control_rate = self.get_parameter("control_rate").value
        self.dt = 1.0 / control_rate
        self.timer = self.create_timer(self.dt, self._loop)

        if self.dry_run:
            self.get_logger().warn("DRY RUN MODE — IK computed but NOT sent to robot.")
            self.get_logger().warn("Set dry_run:=false to enable arm motion.")
        else:
            self.get_logger().warn(
                "LIVE MODE — Arm WILL move. Keep emergency stop ready!"
            )
        self.get_logger().info("Waiting for VR data... Move your Quest3 hand to begin.")

    def _vr_cb(self, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            return
        rot = Rotation.from_quat(q)
        self.pose_proc.update_vr_pose(pos, rot)
        self.last_vr_time = time.monotonic()
        self.vr_received = True
        self.vr_msg_count += 1

    def _loop(self):
        now = time.monotonic()
        dt = now - self.prev_loop_time
        self.prev_loop_time = now

        if not self.vr_received:
            return
        if now - self.last_vr_time > self.data_timeout:
            return

        delta_pos, delta_rot = self.pose_proc.process()

        T_gripper = self.pose_proc.compute_target_pose(
            delta_pos, delta_rot, self.robot_init_pos, self.robot_init_rot
        )
        T_flange = T_gripper @ self._T_tcp_to_flange

        # Workspace check on FLANGE position (what IK receives)
        flange_pos = T_flange[:3, 3].copy()
        clamped_flange = self.safety.check_workspace(flange_pos)
        if not np.allclose(flange_pos, clamped_flange):
            T_flange[:3, 3] = clamped_flange

        t_solve_0 = time.perf_counter()
        sol = self.ik.solve(T_flange)
        t_solve_ms = (time.perf_counter() - t_solve_0) * 1000
        self.frame_count += 1

        if sol is None:
            self.ik_fail += 1
        else:
            self.ik_ok += 1

        safe_q, info, cmd = None, {}, np.zeros(7)
        if sol is not None:
            safe_q, info = self.safety.filter(sol, dt)
            n = min(len(safe_q), 7)
            cmd[:n] = safe_q[:n]

        # Print every 2 seconds regardless of success/failure
        if now - self.last_print >= 2.0:
            elapsed = now - self.start_time
            # Instantaneous FPS (frames since last print)
            fps_inst = (
                (self.frame_count - self._last_frame_count) / (now - self.last_print)
                if hasattr(self, "_last_frame_count") else 0
            )
            fps_cum = self.frame_count / elapsed if elapsed > 0 else 0
            ik_rate = (
                self.ik_ok / self.frame_count * 100 if self.frame_count > 0 else 0
            )
            self._last_frame_count = self.frame_count

            # VR callback rate
            vr_rate = self.vr_msg_count / elapsed if elapsed > 0 else 0

            # Read current VR raw position from pose processor for debugging
            vr_raw = (
                self.pose_proc.vr_current_pos
                if self.pose_proc.vr_current_pos is not None
                else np.zeros(3)
            )
            vr_zero = (
                self.pose_proc.vr_init_pos
                if self.pose_proc.vr_init_pos is not None
                else np.zeros(3)
            )
            delta_raw = vr_raw - vr_zero

            self.get_logger().info(
                f"[Loop:{fps_inst:.0f}Hz avg:{fps_cum:.0f}Hz | VR:{vr_rate:.0f}Hz | IK:{ik_rate:.0f}%]"
            )
            self.get_logger().info(
                f"  VR raw=({vr_raw[0]:.3f},{vr_raw[1]:.3f},{vr_raw[2]:.3f}) "
                f"zero=({vr_zero[0]:.3f},{vr_zero[1]:.3f},{vr_zero[2]:.3f}) "
                f"delta=({delta_raw[0]:.3f},{delta_raw[1]:.3f},{delta_raw[2]:.3f})"
            )
            self.get_logger().info(
                f"  Arm delta=({delta_pos[0]:.3f},{delta_pos[1]:.3f},{delta_pos[2]:.3f}) "
                f"Target flange: ({T_flange[0,3]:.3f}, {T_flange[1,3]:.3f}, {T_flange[2,3]:.3f})"
            )

            if sol is not None:
                self.get_logger().info(
                    f"  Joints(deg): {np.degrees(cmd).round(1).tolist()} | t={t_solve_ms:.1f}ms"
                )
                self.get_logger().info(
                    f"  FK err: {np.linalg.norm(T_flange[:3,3]-self.ik.fk(sol)[:3,3])*1000:.2f}mm"
                    f" | safety={info}"
                )

                if not self.dry_run:
                    actual_ja = self.robot.get_joint_angles()
                    if actual_ja is not None:
                        actual = np.array(actual_ja.msg)
                        tracking_err = np.degrees(np.max(np.abs(actual - cmd)))
                        self.get_logger().info(
                            f"  Arm tracking error: {tracking_err:.2f} deg"
                        )
            else:
                self.get_logger().warn(f"  IK FAILED ({t_solve_ms:.0f}ms)")
                self.last_print = now

            self.last_print = now

        # --- Send command ---
        if not self.dry_run and sol is not None:
            try:
                self.robot.move_js(cmd.tolist())
            except Exception as e:
                self.get_logger().error(f"move_js error: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down safe teleop test...")
        elapsed = time.monotonic() - self.start_time
        self.get_logger().info(
            f"Summary: {self.frame_count} frames in {elapsed:.1f}s, "
            f"IK ok={self.ik_ok} fail={self.ik_fail}"
        )
        try:
            self.robot.set_normal_mode()
            time.sleep(0.1)
            self.robot.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafeTeleopTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

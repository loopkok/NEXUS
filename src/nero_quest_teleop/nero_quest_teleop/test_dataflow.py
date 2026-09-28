#!/usr/bin/env python3
"""End-to-end data flow test — NO robot connection.

Subscribes to Quest3 VR wrist topic, runs the full
VR pose -> coordinate mapping -> IK -> safety filter pipeline,
and prints the results. Validates:

1. VR data is received correctly
2. IK produces valid joint angles from live VR input
3. Safety filter catches any violations
4. The full pipeline runs at the target control rate

Usage:
  ros2 run nero_quest_teleop test_dataflow --ros-args -p arm_side:=right
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from nero_quest_teleop.ik_solver import IKSolver
from nero_quest_teleop.pose_processor import PoseProcessor
from nero_quest_teleop.safety_filter import SafetyFilter


class DataFlowTestNode(Node):
    def __init__(self):
        super().__init__("test_dataflow")

        # ---------- Parameters ----------
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("vr_to_arm_rot", [
            0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0,
        ])
        self.declare_parameter("pos_smoothing", 0.5)
        self.declare_parameter("rot_smoothing", 0.7)
        self.declare_parameter("motion_scale", 1.0)
        self.declare_parameter("flip_pitch", True)
        self.declare_parameter("max_joint_vel", 0.15)
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("solver_type", "analytic_dh")

        self.arm_side = self.get_parameter("arm_side").value.lower()
        control_rate = self.get_parameter("control_rate").value

        # ---------- IK Solver ----------
        solver_type = self.get_parameter("solver_type").value
        if solver_type == "analytic_dh":
            self.ik = IKSolver()
        elif solver_type == "urdf_numerical":
            from nero_quest_teleop.urdf_ik_solver import URDFIKSolver
            import os as _os
            from ament_index_python.packages import get_package_share_directory
            urdf_pkg = "agx_arm_description"
            urdf_rel = "agx_arm_urdf/nero/urdf/nero_description.urdf"
            urdf_path = _os.path.join(get_package_share_directory(urdf_pkg), urdf_rel)
            self.ik = URDFIKSolver(urdf_path=urdf_path, locked_joints=["joint8"], max_iter=4, tol=1e-4)
        else:
            raise ValueError(f"Unknown solver_type: {solver_type}")
        self.get_logger().info(f"IK solver: {solver_type}, nq={self.ik.nq}")
        self.ik.sync_state(np.zeros(self.ik.nq))

        # ---------- Pose Processor ----------
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

        # Robot initial pose (approximate)
        self.robot_init_pos = np.array([-0.45, 0.0, 0.45])
        self.robot_init_rot = Rotation.from_euler(
            "xyz", [-1.5708, 0.0, -3.14159]
        ).as_matrix()

        # ---------- Safety Filter ----------
        self.safety = SafetyFilter(
            joint_lower_limits=self.ik.lower_limits,
            joint_upper_limits=self.ik.upper_limits,
            max_joint_vel=float(self.get_parameter("max_joint_vel").value),
        )
        self.safety.set_initial_state(np.zeros(self.ik.nq))

        # ---------- Stats ----------
        self.frame_count = 0
        self.ik_ok_count = 0
        self.ik_fail_count = 0
        self.safety_violations = 0
        self.start_time = time.monotonic()
        self.last_print_time = self.start_time

        # ---------- Subscriber ----------
        wrist_topic = f"quest3/{self.arm_side}_wrist_pose"
        self.create_subscription(PoseStamped, wrist_topic, self._vr_cb, 10)
        self.get_logger().info(f"Listening on: {wrist_topic}")
        self.get_logger().info("Waiting for VR data... (start Quest3 app to begin)")

        # ---------- Timer ----------
        self.dt = 1.0 / control_rate
        self.timer = self.create_timer(self.dt, self._loop)

    def _vr_cb(self, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = [msg.pose.orientation.x, msg.pose.orientation.y,
             msg.pose.orientation.z, msg.pose.orientation.w]
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            return
        rot = Rotation.from_quat(q)
        self.pose_proc.update_vr_pose(pos, rot)

    def _loop(self):
        if not self.pose_proc.is_calibrated:
            return

        delta_pos, delta_rot = self.pose_proc.process()
        T_gripper = self.pose_proc.compute_target_pose(
            delta_pos, delta_rot, self.robot_init_pos, self.robot_init_rot
        )
        T_flange = T_gripper @ self._T_tcp_to_flange

        sol = self.ik.solve(T_flange)
        self.frame_count += 1

        if sol is None:
            self.ik_fail_count += 1
        else:
            self.ik_ok_count += 1
            safe_q, info = self.safety.filter(sol, self.dt)
            if info["clamped"] or info["velocity_limited"] or info["collision"]:
                self.safety_violations += 1

        now = time.monotonic()
        if now - self.last_print_time >= 2.0:
            elapsed = now - self.start_time
            fps = self.frame_count / elapsed if elapsed > 0 else 0
            ik_rate = (
                self.ik_ok_count / self.frame_count * 100
                if self.frame_count > 0 else 0
            )

            self.get_logger().info(
                f"Frames: {self.frame_count} | FPS: {fps:.1f} | "
                f"IK OK: {ik_rate:.0f}% | Fails: {self.ik_fail_count} | "
                f"Safety violations: {self.safety_violations}"
            )

            if sol is not None:
                self.get_logger().info(
                    f"  Target EE: ({T_flange[0,3]:.3f}, {T_flange[1,3]:.3f}, {T_flange[2,3]:.3f}) | "
                    f"Joints (deg): {np.degrees(safe_q).round(1).tolist()}"
                )

            self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node = DataFlowTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        elapsed = time.monotonic() - node.start_time
        node.get_logger().info("\n" + "=" * 60)
        node.get_logger().info("DATA FLOW TEST SUMMARY")
        node.get_logger().info("=" * 60)
        node.get_logger().info(f"  Duration: {elapsed:.1f}s")
        node.get_logger().info(f"  Total frames processed: {node.frame_count}")
        if node.frame_count > 0:
            node.get_logger().info(
                f"  IK success rate: {node.ik_ok_count}/{node.frame_count} "
                f"({node.ik_ok_count/node.frame_count*100:.1f}%)"
            )
        else:
            node.get_logger().info("  No frames processed")
        node.get_logger().info(f"  Safety violations: {node.safety_violations}")
        node.get_logger().info(
            f"  VR data received: {'YES' if node.pose_proc.is_calibrated else 'NO'}"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

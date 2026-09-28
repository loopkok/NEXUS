#!/usr/bin/env python3
"""Dedicated IK solver ROS 2 node (topic-based).

Runs the analytic DH-based IK solver as a standalone node for CPU isolation.
Use this when you want the IK computation in a separate process.

Subscribes:
  ~/target_flange_pose (PoseStamped) — target flange pose for IK
Publishes:
  ~/joint_solution (Float64MultiArray) — 7 joint angles (rad)
  ~/ik_status (String) — status info
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import Float64MultiArray, String

from nero_quest_teleop.ik_solver import IKSolver


class IKSolverNode(Node):
    def __init__(self):
        super().__init__("ik_solver_node")

        self.declare_parameter("solver_type", "analytic_dh")
        solver_type = self.get_parameter("solver_type").value

        if solver_type == "analytic_dh":
            self.ik = IKSolver()
        else:
            self.get_logger().error(f"Unknown solver_type: {solver_type}")
            raise ValueError(f"Unknown solver_type: {solver_type}")

        self.get_logger().info(
            f"IK Solver ready: nq={self.ik.nq}, type={solver_type}"
        )

        # Statistics
        self._solve_count = 0
        self._fail_count = 0
        self._total_time = 0.0
        self._last_print = time.monotonic()

        # Subscriber: target flange pose
        self.sub = self.create_subscription(
            PoseStamped, "~/target_flange_pose", self._pose_cb, 10
        )
        # Publishers
        self.joint_pub = self.create_publisher(
            Float64MultiArray, "~/joint_solution", 10
        )
        self.status_pub = self.create_publisher(String, "~/ik_status", 10)

    def _pose_cb(self, msg: PoseStamped):
        t_start = time.perf_counter()

        # Build 4x4 matrix
        T = np.eye(4, dtype=float)
        T[0, 3] = msg.pose.position.x
        T[1, 3] = msg.pose.position.y
        T[2, 3] = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        T[:3, :3] = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

        sol = self.ik.solve(T)
        elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        self._solve_count += 1
        self._total_time += elapsed_ms

        # Publish joint solution
        out = Float64MultiArray()
        if sol is not None:
            out.data = sol.tolist()
            self.joint_pub.publish(out)
            status = f"OK dt={elapsed_ms:.2f}ms"
        else:
            out.data = [float("nan")] * 7
            self.joint_pub.publish(out)
            self._fail_count += 1
            status = "FAILED"

        self.status_pub.publish(String(data=status))

        # Periodic stats
        now = time.monotonic()
        if now - self._last_print >= 10.0 and self._solve_count > 0:
            avg_ms = self._total_time / self._solve_count
            ok_count = self._solve_count - self._fail_count
            self.get_logger().info(
                f"Stats: {self._solve_count} solves | "
                f"avg {avg_ms:.2f}ms | "
                f"OK {ok_count}/{self._solve_count} "
                f"({ok_count/self._solve_count*100:.1f}%)"
            )
            self._last_print = now


def main(args=None):
    rclpy.init(args=args)
    node = IKSolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

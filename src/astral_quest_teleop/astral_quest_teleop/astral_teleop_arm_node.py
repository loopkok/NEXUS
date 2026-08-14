#!/usr/bin/env python3
"""Single-arm Astral teleop (Nero layout).

Quest wrist → PoseProcessor → IK (``analytic_dh`` or ``urdf_numerical``)
→ SafetyFilter → ``/{side}_arm/joint_commands``.

Use two instances (left + right) for dual-arm (process-level parallel).
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from astral_quest_teleop.ik.factory import make_single_arm_ik
from astral_quest_teleop.pose_processor import PoseProcessor
from astral_quest_teleop.safety_filter import SafetyFilter

_LEFT_NAMES = [
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_elbow_roll",
    "left_elbow_pitch",
    "left_forearm_roll",
    "left_wrist_pitch",
    "left_wrist_roll",
]
_RIGHT_NAMES = [
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_elbow_roll",
    "right_elbow_pitch",
    "right_forearm_roll",
    "right_wrist_pitch",
    "right_wrist_roll",
]

# Joints whose positive direction was flipped to make the arm alpha = +90 deg
# (see ik/analytic.py AstralParams). The DH/IK works in this "flipped" convention;
# the real hardware / MJCF firmware uses the original (SolidWorks) convention, so
# the teleop node flips these joint signs at the IK boundary.
_JOINT_FLIP = {
    "left": [False, True, True, True, False, False, False],
    "right": [False, True, False, True, False, False, False],
}

# Rotation that maps a vector from the original base frame (SolidWorks torso,
# joint1 axis = -X) into the analytic-DH "clean base" frame (joint1 axis = +Z).
# Only the analytic DH solver works in the clean base; the URDF numerical solver
# stays in the original base frame, so this rotation is applied only when
# solver_type == analytic_dh.
_R_BASE_T = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=float,
)


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


class AstralTeleopArmNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_teleop_arm")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("solver_type", "analytic_dh")
        self.declare_parameter("urdf_path", "")  # empty → astral_robot.pin.urdf
        self.declare_parameter("ik_max_iter", 20)
        self.declare_parameter("ik_tol", 1e-8)
        self.declare_parameter("ik_w_pos", 1.0)
        self.declare_parameter("ik_w_ori", 0.3)
        self.declare_parameter("ik_w_reg", 1e-4)
        self.declare_parameter(
            "vr_to_arm_rot",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        self.declare_parameter("pos_smoothing", 0.8)
        self.declare_parameter("rot_smoothing", 0.8)
        self.declare_parameter("motion_scale", 0.65)
        self.declare_parameter("flip_pitch", False)
        self.declare_parameter("tcp_offset", [0.0] * 6)
        self.declare_parameter("max_joint_vel", 0.08)
        self.declare_parameter("workspace_radius", 0.0)
        self.declare_parameter("data_timeout", 1.5)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("require_clench_to_start", False)
        self.declare_parameter("auto_arm_on_start", True)
        self.declare_parameter("use_joint_state_seed", True)
        self.declare_parameter(
            "init_pose", [0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00]
        )

        self.side = str(self.get_parameter("arm_side").value).lower()
        if self.side not in ("left", "right"):
            raise ValueError("arm_side must be left|right")
        self.names = _LEFT_NAMES if self.side == "left" else _RIGHT_NAMES
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.data_timeout = float(self.get_parameter("data_timeout").value)
        rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / max(1.0, rate)

        st = str(self.get_parameter("solver_type").value).strip().lower()
        # The joint-sign flip is only meaningful for the analytic DH solver, which
        # works in the "flipped" (alpha=+90) convention. The URDF numerical solver
        # reads the original-convention URDF and already returns hardware-convention q.
        self._flip_needed = st in ("analytic_dh", "analytic", "dh")
        self.ik = make_single_arm_ik(
            self.side,
            st,
            urdf_path=str(self.get_parameter("urdf_path").value),
            ik_max_iter=int(self.get_parameter("ik_max_iter").value),
            ik_tol=float(self.get_parameter("ik_tol").value),
            ik_w_pos=float(self.get_parameter("ik_w_pos").value),
            ik_w_ori=float(self.get_parameter("ik_w_ori").value),
            ik_w_reg=float(self.get_parameter("ik_w_reg").value),
        )
        init_q_old = np.asarray(
            self.get_parameter("init_pose").value, dtype=float
        ).reshape(7)  # hardware/original convention (config)
        init_q = self._flip_q(init_q_old)  # DH/flipped convention (feed IK)
        init_q = np.clip(
            init_q, self.ik.lower_limits + 0.02, self.ik.upper_limits - 0.02
        )
        self.ik.sync_state(init_q)
        T0 = self.ik.fk(init_q)
        _ = self.ik.solve(T0)
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()
        self.q_cmd = self._flip_q(init_q)  # back to hardware convention
        self.state_q = self.q_cmd.copy()

        R = np.asarray(
            self.get_parameter("vr_to_arm_rot").value, dtype=float
        ).reshape(3, 3)
        if self._flip_needed:
            # analytic DH works in the clean base (joint1 axis = +Z); map the VR
            # delta from the original torso base into the clean base frame.
            R = _R_BASE_T @ R
        self.pose = PoseProcessor(
            vr_to_arm_rot=R,
            pos_smoothing=float(self.get_parameter("pos_smoothing").value),
            rot_smoothing=float(self.get_parameter("rot_smoothing").value),
            motion_scale=float(self.get_parameter("motion_scale").value),
            flip_pitch=bool(self.get_parameter("flip_pitch").value),
        )
        tcp = list(self.get_parameter("tcp_offset").value)
        T_ft = np.eye(4)
        T_ft[:3, :3] = Rotation.from_euler("xyz", tcp[3:]).as_matrix()
        T_ft[:3, 3] = tcp[:3]
        self._T_tcp_to_flange = np.linalg.inv(T_ft)

        self.safety = SafetyFilter(
            joint_lower_limits=self.ik.lower_limits,
            joint_upper_limits=self.ik.upper_limits,
            max_joint_vel=float(self.get_parameter("max_joint_vel").value),
            workspace_radius=float(self.get_parameter("workspace_radius").value),
        )
        self.safety.set_initial_state(init_q, self.robot_init_pos)

        qos = _sensor_qos()
        self.cmd_pub = self.create_publisher(
            JointState, f"/{self.side}_arm/joint_commands", qos
        )
        self.create_subscription(
            PoseStamped, f"quest3/{self.side}_wrist_pose", self._on_wrist, 10
        )
        if bool(self.get_parameter("use_joint_state_seed").value):
            self.create_subscription(
                JointState,
                f"/{self.side}_arm/joint_states",
                self._on_state,
                qos,
            )

        require = bool(self.get_parameter("require_clench_to_start").value)
        auto = bool(self.get_parameter("auto_arm_on_start").value)
        self._armed = (not require) or auto
        self.create_subscription(Bool, "/teleop/armed", self._on_armed, 10)
        self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)

        self._last_vr_t = 0.0
        self._prev_t = time.monotonic()
        self.create_timer(self.dt, self._loop)
        solver_name = getattr(self.ik, "method_name", type(self.ik).__name__)
        self.get_logger().info(
            f"Astral arm teleop: {self.side} solver={solver_name} "
            f"EE0={np.round(self.robot_init_pos, 3).tolist()} "
            f"scale={self.pose.motion_scale} dry_run={self.dry_run}"
        )

    def _flip_q(self, q):
        """Map joint angles between the DH (flipped) and hardware (original) convention.

        Self-inverse: the flipped joints (per _JOINT_FLIP) have their sign negated,
        so applying it twice returns the original vector. For the URDF numerical
        solver this is a no-op (it already works in the hardware convention).
        """
        q = np.asarray(q, dtype=float).reshape(7)
        if not self._flip_needed:
            return q
        out = q.copy()
        for i, f in enumerate(_JOINT_FLIP[self.side]):
            if f:
                out[i] = -out[i]
        return out

    def _on_armed(self, _msg: Bool) -> None:
        self._armed = True

    def _on_disarm(self, _msg: Bool) -> None:
        self._armed = False

    def _on_state(self, msg: JointState) -> None:
        if len(msg.position) >= 7:
            self.state_q = np.asarray(msg.position[:7], dtype=float)

    def _on_wrist(self, msg: PoseStamped) -> None:
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            return
        self.pose.update_vr_pose(pos, Rotation.from_quat(q))
        self._last_vr_t = time.monotonic()

    def _loop(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._prev_t)
        self._prev_t = now
        if not self._armed or not self.pose.is_calibrated:
            return
        if self._last_vr_t > 0 and (now - self._last_vr_t) > self.data_timeout:
            return

        dp, dr = self.pose.process()
        T_tcp = self.pose.compute_target_pose(
            dp, dr, self.robot_init_pos, self.robot_init_rot
        )
        T_flange = T_tcp @ self._T_tcp_to_flange
        # Warm-start from last *commanded* q (after vel limit), not raw IK jump.
        try:
            self.ik.sync_state(self._flip_q(self.q_cmd), reset_branch=False)
        except TypeError:
            self.ik.sync_state(self._flip_q(self.q_cmd))
        sol = self.ik.solve(T_flange)
        if sol is None:
            return
        safe, _info = self.safety.filter(sol, dt)
        # IK/safety work in the flipped convention; convert back to hardware.
        self.q_cmd = self._flip_q(safe)
        if self.dry_run:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.names)
        msg.position = self.q_cmd.tolist()
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstralTeleopArmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


if __name__ == "__main__":
    main()

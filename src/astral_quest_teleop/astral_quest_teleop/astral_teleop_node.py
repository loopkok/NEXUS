"""Astral dual-arm Quest teleop node.

Subscribes ``quest3/{left,right}_wrist_pose`` (frame ``robot_world``, already
axis-aligned to Astral) and publishes BEST_EFFORT ``JointState`` commands to
``/left_arm/joint_commands`` / ``/right_arm/joint_commands``.

Per-arm ``vr_to_arm_rot_*`` maps Quest deltas. Dual-arm IK uses
``AstralIKBridge.solve_dual`` (left/right in parallel).

solver_type:
  analytic_dh     — package ik.analytic (Modified DH, arm base)
  urdf_numerical  — package ik.urdf_solver (Pinocchio LM, parallel)
"""

from __future__ import annotations

import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from astral_quest_teleop.ik import make_ik_solver
from astral_quest_teleop.pose_processor import PoseProcessor
from astral_quest_teleop.safety_filter import SafetyFilter

# Prefer joint names from control package when installed.
try:
    from astral_robot_control.joint_layout import (
        LEFT_ARM_JOINT_NAMES,
        RIGHT_ARM_JOINT_NAMES,
    )
except ImportError:  # pragma: no cover
    LEFT_ARM_JOINT_NAMES = [
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_elbow_roll",
        "left_elbow_pitch",
        "left_forearm_roll",
        "left_wrist_pitch",
        "left_wrist_roll",
    ]
    RIGHT_ARM_JOINT_NAMES = [
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_elbow_roll",
        "right_elbow_pitch",
        "right_forearm_roll",
        "right_wrist_pitch",
        "right_wrist_roll",
    ]

# Joints whose positive direction was flipped for alpha=+90 MDH (see analytic.py).
# DH/IK use the flipped convention; hardware / old MJCF use SolidWorks signs.
# Same map as astral_teleop_arm_node._JOINT_FLIP.
_JOINT_FLIP = {
    "left": [False, True, True, True, False, False, False],
    "right": [False, True, False, True, False, False, False],
}


def _sensor_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def _finite_or_none(v) -> Optional[float]:
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(x):
        return None
    return x


class _ArmChannel:
    __slots__ = (
        "side",
        "key",
        "joint_names",
        "pose",
        "safety",
        "robot_init_pos",
        "robot_init_rot",
        "q_cmd",
        "last_vr_t",
        "vr_ok",
        "cmd_pub",
        "state_q",
    )

    def __init__(self, side: str):
        self.side = side  # left|right
        self.key = "L" if side == "left" else "R"
        self.joint_names = (
            list(LEFT_ARM_JOINT_NAMES)
            if side == "left"
            else list(RIGHT_ARM_JOINT_NAMES)
        )
        self.pose: Optional[PoseProcessor] = None
        self.safety: Optional[SafetyFilter] = None
        self.robot_init_pos = np.zeros(3)
        self.robot_init_rot = np.eye(3)
        self.q_cmd = np.zeros(7)
        self.last_vr_t = 0.0
        self.vr_ok = False
        self.cmd_pub = None
        self.state_q = np.zeros(7)


class AstralTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_teleop")

        # ---- params ----
        self.declare_parameter("arm_side", "both")  # left|right|both
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("solver_type", "analytic_dh")
        self.declare_parameter("urdf_path", "")  # empty → package-local RobotMain URDF
        self.declare_parameter("ik_max_iter", 20)
        self.declare_parameter("ik_tol", 1e-8)
        self.declare_parameter("ik_w_pos", 1.0)
        self.declare_parameter("ik_w_ori", 0.3)
        self.declare_parameter("ik_w_reg", 1e-4)

        # Fallback map (row-major). Prefer per-arm vr_to_arm_rot_*.
        self.declare_parameter(
            "robot_world_to_base_rot",
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        )
        # Default I: mocap robot_world is already Astral base_link.
        _I9 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.declare_parameter("vr_to_arm_rot_left", list(_I9))
        self.declare_parameter("vr_to_arm_rot_right", list(_I9))
        self.declare_parameter("pos_smoothing", 0.8)
        self.declare_parameter("rot_smoothing", 0.8)
        self.declare_parameter("motion_scale", 0.65)
        self.declare_parameter("flip_pitch", False)
        self.declare_parameter("tcp_offset", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.declare_parameter("max_joint_vel", 4.0)
        self.declare_parameter("workspace_radius", 0.0)  # 0 = disabled
        self.declare_parameter("workspace_z_min", float("-inf"))
        self.declare_parameter("workspace_z_max", float("inf"))
        self.declare_parameter("workspace_x_min", float("-inf"))
        self.declare_parameter("workspace_x_max", float("inf"))
        self.declare_parameter("workspace_y_min", float("-inf"))
        self.declare_parameter("workspace_y_max", float("inf"))
        self.declare_parameter("data_timeout", 1.5)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("require_clench_to_start", False)
        self.declare_parameter("auto_arm_on_start", True)

        self.declare_parameter(
            "init_pose_left", [0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00]
        )
        self.declare_parameter(
            "init_pose_right", [0.32, -0.11, 0.53, -0.80, 0.28, 0.00, 0.00]
        )
        self.declare_parameter("use_joint_state_seed", True)

        side_cfg = str(self.get_parameter("arm_side").value).strip().lower()
        if side_cfg not in ("left", "right", "both"):
            raise ValueError("arm_side must be left|right|both")
        self._sides = (
            ["left", "right"] if side_cfg == "both" else [side_cfg]
        )

        rate = float(self.get_parameter("control_rate").value)
        self.dt = 1.0 / max(1.0, rate)
        self.data_timeout = float(self.get_parameter("data_timeout").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.use_state_seed = bool(
            self.get_parameter("use_joint_state_seed").value
        )
        solver_type = str(self.get_parameter("solver_type").value).strip().lower()
        # Flip only for analytic DH (alpha=+90 convention). URDF numerical already
        # returns hardware-convention q.
        self._flip_needed = solver_type in ("analytic_dh", "analytic", "dh")

        def _rot9(name: str, fallback: np.ndarray) -> np.ndarray:
            flat = list(self.get_parameter(name).value)
            if len(flat) != 9:
                return fallback
            return np.asarray(flat, dtype=float).reshape(3, 3)

        R_fallback = _rot9("robot_world_to_base_rot", np.eye(3))
        R_left = _rot9("vr_to_arm_rot_left", R_fallback)
        R_right = _rot9("vr_to_arm_rot_right", R_fallback)
        R_by_side = {"left": R_left, "right": R_right}

        tcp = list(self.get_parameter("tcp_offset").value)
        self._T_flange_to_tcp = np.eye(4)
        self._T_flange_to_tcp[:3, :3] = Rotation.from_euler(
            "xyz", tcp[3:6]
        ).as_matrix()
        self._T_flange_to_tcp[:3, 3] = np.asarray(tcp[:3], dtype=float)
        self._T_tcp_to_flange = np.linalg.inv(self._T_flange_to_tcp)

        # ---- IK ----
        self.ik = make_ik_solver(
            solver_type,
            urdf_path=str(self.get_parameter("urdf_path").value),
            ik_max_iter=int(self.get_parameter("ik_max_iter").value),
            ik_tol=float(self.get_parameter("ik_tol").value),
            ik_w_pos=float(self.get_parameter("ik_w_pos").value),
            ik_w_ori=float(self.get_parameter("ik_w_ori").value),
            ik_w_reg=float(self.get_parameter("ik_w_reg").value),
        )
        self.get_logger().info(
            f"IK: {self.ik.method_name} flip_q={self._flip_needed}"
        )

        # ---- per-arm channels ----
        qos = _sensor_qos()
        self.arms: Dict[str, _ArmChannel] = {}
        init_dh_by_side: Dict[str, np.ndarray] = {}
        for side in self._sides:
            ch = _ArmChannel(side)
            lo, hi = self.ik.joint_limits(ch.key)
            ch.pose = PoseProcessor(
                vr_to_arm_rot=R_by_side[side],
                pos_smoothing=float(self.get_parameter("pos_smoothing").value),
                rot_smoothing=float(self.get_parameter("rot_smoothing").value),
                motion_scale=float(self.get_parameter("motion_scale").value),
                flip_pitch=bool(self.get_parameter("flip_pitch").value),
            )
            ch.safety = SafetyFilter(
                joint_lower_limits=lo,
                joint_upper_limits=hi,
                max_joint_vel=float(self.get_parameter("max_joint_vel").value),
                workspace_radius=float(
                    self.get_parameter("workspace_radius").value
                ),
                workspace_z_min=_finite_or_none(
                    self.get_parameter("workspace_z_min").value
                ),
                workspace_z_max=_finite_or_none(
                    self.get_parameter("workspace_z_max").value
                ),
                workspace_x_min=_finite_or_none(
                    self.get_parameter("workspace_x_min").value
                ),
                workspace_x_max=_finite_or_none(
                    self.get_parameter("workspace_x_max").value
                ),
                workspace_y_min=_finite_or_none(
                    self.get_parameter("workspace_y_min").value
                ),
                workspace_y_max=_finite_or_none(
                    self.get_parameter("workspace_y_max").value
                ),
            )
            init_key = f"init_pose_{side}"
            # Config init_pose_* is hardware / original (SolidWorks) convention.
            init_hw = np.asarray(
                self.get_parameter(init_key).value, dtype=float
            ).reshape(7)
            init_dh = self._flip_q(init_hw, side)
            lo_i, hi_i = np.asarray(lo, float), np.asarray(hi, float)
            init_dh = np.clip(init_dh, lo_i + 0.02, hi_i - 0.02)
            init_dh_by_side[side] = init_dh
            ch.q_cmd = self._flip_q(init_dh, side)  # publish HW convention
            ch.state_q = ch.q_cmd.copy()
            ch.cmd_pub = self.create_publisher(
                JointState, f"/{side}_arm/joint_commands", qos
            )
            self.create_subscription(
                PoseStamped,
                f"quest3/{side}_wrist_pose",
                lambda msg, s=side: self._on_wrist(s, msg),
                10,
            )
            if self.use_state_seed:
                self.create_subscription(
                    JointState,
                    f"/{side}_arm/joint_states",
                    lambda msg, s=side: self._on_state(s, msg),
                    qos,
                )
            self.arms[side] = ch

        # Seed IK + FK init targets (DH / solver convention)
        q14 = np.zeros(14)
        if "left" in self.arms:
            q14[0:7] = init_dh_by_side["left"]
        if "right" in self.arms:
            q14[7:14] = init_dh_by_side["right"]
        self.ik.sync_state(q14)
        for side, ch in self.arms.items():
            q_dh = init_dh_by_side[side]
            T = self.ik.fk(ch.key, q_dh)
            _ = self.ik.solve(ch.key, T)
            ch.robot_init_pos = T[:3, 3].copy()
            ch.robot_init_rot = T[:3, :3].copy()
            ch.safety.set_initial_state(q_dh, ch.robot_init_pos)
            self.get_logger().info(
                f"[{side}] EE0="
                f"{np.round(ch.robot_init_pos, 3).tolist()} "
                f"(solver={self.ik.method_name})"
            )

        require = bool(self.get_parameter("require_clench_to_start").value)
        auto = bool(self.get_parameter("auto_arm_on_start").value)
        self._armed = (not require) or auto
        self.create_subscription(Bool, "/teleop/armed", self._on_armed, 10)
        self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)

        self._prev_t = time.monotonic()
        self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(
            f"Astral teleop ready: sides={self._sides} rate={rate:.0f}Hz "
            f"dry_run={self.dry_run} armed={self._armed} "
            f"motion_scale={float(self.get_parameter('motion_scale').value)} "
            f"vr_to_arm_rot (default I = robot_world≡base_link)"
        )

    # ------------------------------------------------------------------ I/O
    def _flip_q(self, q: np.ndarray, side: str) -> np.ndarray:
        """Map between DH (flipped) and hardware (original) joint conventions.

        Self-inverse. No-op when ``solver_type`` is ``urdf_numerical``.
        """
        q = np.asarray(q, dtype=float).reshape(7)
        if not self._flip_needed:
            return q
        out = q.copy()
        for i, f in enumerate(_JOINT_FLIP[side]):
            if f:
                out[i] = -out[i]
        return out

    def _on_armed(self, msg: Bool) -> None:
        if msg.data and not self._armed:
            self._armed = True
            self.get_logger().info("Teleop ARMED")

    def _on_disarm(self, msg: Bool) -> None:
        if msg.data:
            self._armed = False
            for ch in self.arms.values():
                ch.pose.reset()
            self.get_logger().info("Teleop DISARMED / recalibrate on next wrist")

    def _on_wrist(self, side: str, msg: PoseStamped) -> None:
        ch = self.arms[side]
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )
        quat = np.array(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ],
            dtype=float,
        )
        n = float(np.linalg.norm(quat))
        if n < 1e-9:
            return
        rot = Rotation.from_quat(quat / n)
        if ch.pose.is_calibrated:
            jump = float(np.linalg.norm(pos - ch.pose.vr_init_pos))
            if jump > 2.0:
                self.get_logger().warning(
                    f"[{side}] VR jump {jump:.2f}m — recalibrate"
                )
                ch.pose.reset()
                # state_q is hardware convention; FK needs solver convention.
                q_dh = self._flip_q(ch.state_q, side)
                T = self.ik.fk(ch.key, q_dh)
                ch.robot_init_pos = T[:3, 3].copy()
                ch.robot_init_rot = T[:3, :3].copy()
                ch.q_cmd = ch.state_q.copy()
                ch.safety.set_initial_state(q_dh, ch.robot_init_pos)
        ch.pose.update_vr_pose(pos, rot)
        ch.last_vr_t = time.monotonic()
        ch.vr_ok = True

    def _on_state(self, side: str, msg: JointState) -> None:
        ch = self.arms[side]
        if len(msg.position) >= 7:
            ch.state_q = np.asarray(msg.position[:7], dtype=float)

    def _publish_cmd(self, ch: _ArmChannel, q7: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ch.joint_names)
        msg.position = [float(x) for x in q7.tolist()]
        if self.dry_run:
            self.get_logger().info(
                f"[dry_run] /{ch.side}_arm/joint_commands "
                f"{np.round(q7, 3).tolist()}",
                throttle_duration_sec=1.0,
            )
            return
        ch.cmd_pub.publish(msg)

    # ------------------------------------------------------------------ loop
    def _control_loop(self) -> None:
        now = time.monotonic()
        dt = max(1e-4, now - self._prev_t)
        self._prev_t = now

        if not self._armed:
            return

        # Keep IK seed coherent across arms (solver / DH convention).
        q14 = self.ik.q_full
        for side, ch in self.arms.items():
            q_hw = ch.state_q if self.use_state_seed else ch.q_cmd
            q_dh = self._flip_q(q_hw, side)
            if side == "left":
                q14[0:7] = q_dh
            else:
                q14[7:14] = q_dh
        self.ik.sync_state(q14)

        # Build targets, then solve L/R in parallel (URDF LM / DH).
        pending: Dict[str, tuple] = {}
        for side, ch in self.arms.items():
            if not ch.vr_ok:
                continue
            if self.data_timeout > 0 and (now - ch.last_vr_t) > self.data_timeout:
                self.get_logger().warning(
                    f"[{side}] VR timeout — hold",
                    throttle_duration_sec=2.0,
                )
                continue

            delta_pos, delta_rot = ch.pose.process(dt)
            T_tcp = ch.pose.compute_target_pose(
                delta_pos,
                delta_rot,
                ch.robot_init_pos,
                ch.robot_init_rot,
            )
            T_flange = T_tcp @ self._T_tcp_to_flange
            flange_pos = T_flange[:3, 3].copy()
            clamped = ch.safety.check_workspace(flange_pos)
            if not np.allclose(flange_pos, clamped):
                T_flange[:3, 3] = clamped
            pending[side] = (ch, T_flange)

        if not pending:
            return

        targets = {ch.key: T for ch, T in pending.values()}
        sols = self.ik.solve_dual(targets)

        for side, (ch, T_flange) in pending.items():
            sol = sols.get(ch.key)
            if sol is None:
                self.get_logger().warning(
                    f"[{side}] IK fail target="
                    f"{np.round(T_flange[:3, 3], 3).tolist()}",
                    throttle_duration_sec=1.0,
                )
                continue

            # IK + safety in solver (DH) convention; publish hardware q.
            safe_q, info = ch.safety.filter(sol, dt)
            if info.get("collision"):
                continue
            ch.q_cmd = self._flip_q(safe_q, side)
            self._publish_cmd(ch, ch.q_cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AstralTeleopNode()
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

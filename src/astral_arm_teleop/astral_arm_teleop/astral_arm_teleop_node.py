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
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

from astral_arm_teleop.ik.factory import make_single_arm_ik
from astral_arm_teleop.latency_meter import LatencyMeter, stamp_age_ms
from astral_arm_teleop.pose_processor import PoseProcessor
from astral_arm_teleop.safety_filter import SafetyFilter

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
        depth=20,
        durability=DurabilityPolicy.VOLATILE,
    )


class AstralTeleopArmNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_arm_teleop_arm")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("solver_type", "urdf_numerical")
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
        self.declare_parameter("max_joint_vel", 4.0)
        self.declare_parameter("workspace_radius", 0.0)
        # Watchdog: disarm when VR data is older than this. 1.5s tolerates
        # brief VR link jitter; while teleop keeps publishing the frozen
        # target the driver keeps tracking it, and only after teleop stops
        # does the driver's own command_timeout_s (1.5) engage the hold.
        # <= 0 disables the watchdog (not recommended).
        self.declare_parameter("data_timeout", 1.5)
        self.declare_parameter("dry_run", False)
        self.declare_parameter("require_clench_to_start", False)
        self.declare_parameter("auto_arm_on_start", True)
        self.declare_parameter(
            "require_start_signal",
            False,
        )
        self.declare_parameter("use_joint_state_seed", True)
        self.declare_parameter(
            "init_pose", [0.32, 0.11, -0.53, -0.80, 0.28, 0.00, 0.00]
        )
        # Flat 7*N joint-space via points (hardware convention), visited in
        # order before init_pose. Default [0.0] is a typed DOUBLE_ARRAY
        # placeholder (empty [] would be inferred as BYTE_ARRAY in rclpy).
        # Length < 7 means "no via points".
        self.declare_parameter(
            "init_waypoints",
            [0.0],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )
        self.declare_parameter("move_to_init_pose", True)
        self.declare_parameter("init_speed_percent", 10)  # of max_joint_vel
        self.declare_parameter("init_arrive_tol", 0.05)  # rad
        self.declare_parameter("init_timeout", 15.0)
        self.declare_parameter("print_latency", True)
        self.declare_parameter("latency_print_interval", 2.0)
        self.declare_parameter("publish_tune", True)

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
        self._urdf_path = str(self.get_parameter("urdf_path").value)
        self._vr_to_arm_yaml = np.asarray(
            self.get_parameter("vr_to_arm_rot").value, dtype=float
        ).reshape(3, 3)
        self.ik = make_single_arm_ik(
            self.side,
            st,
            urdf_path=self._urdf_path,
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
        self._init_q_hw = self.q_cmd.copy()
        self._homing_path = self._parse_init_waypoints() + [self._init_q_hw.copy()]
        self._homing_i = 0
        self.state_q = self.q_cmd.copy()
        self._got_state = False
        self._homing = bool(self.get_parameter("move_to_init_pose").value) and (
            not self.dry_run
        )
        vmax = float(self.get_parameter("max_joint_vel").value)
        pct = max(1.0, float(self.get_parameter("init_speed_percent").value))
        self._init_joint_vel = vmax * (pct / 100.0)
        self._init_arrive_tol = float(self.get_parameter("init_arrive_tol").value)
        self._init_timeout = float(self.get_parameter("init_timeout").value)
        self._homing_t0 = 0.0
        self._homing_last_log = 0.0
        self._homing_started = False
        self._homing_seeded = False

        R = self._vr_to_arm_yaml.copy()
        if self._flip_needed:
            # analytic DH works in the clean base (joint1 axis = +Z); map the VR
            # delta from the original torso base into the clean base frame.
            R = _R_BASE_T @ R
        self._require_start = bool(self.get_parameter("require_start_signal").value)
        self.pose = PoseProcessor(
            vr_to_arm_rot=R,
            pos_smoothing=float(self.get_parameter("pos_smoothing").value),
            rot_smoothing=float(self.get_parameter("rot_smoothing").value),
            motion_scale=float(self.get_parameter("motion_scale").value),
            flip_pitch=bool(self.get_parameter("flip_pitch").value),
            auto_calibrate=not self._require_start,
        )
        tcp = list(self.get_parameter("tcp_offset").value)
        T_ft = np.eye(4)
        T_ft[:3, :3] = Rotation.from_euler("xyz", tcp[3:]).as_matrix()
        T_ft[:3, 3] = tcp[:3]
        self._T_flange_to_tcp = T_ft
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
        self._publish_tune = bool(self.get_parameter("publish_tune").value)
        self._tune_pubs = {}
        if self._publish_tune:
            tune_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
            )
            prefix = f"/teleop/{self.side}/tune"
            self._tune_pubs = {
                "vr": self.create_publisher(PoseStamped, f"{prefix}/ee_vr", tune_qos),
                "filt": self.create_publisher(
                    PoseStamped, f"{prefix}/ee_filt", tune_qos
                ),
                "cmd": self.create_publisher(PoseStamped, f"{prefix}/ee_cmd", tune_qos),
                "xyz": self.create_publisher(
                    Float64MultiArray, f"{prefix}/xyz", tune_qos
                ),
            }
            self.get_logger().info(
                f"tune topics: /teleop/{self.side}/tune/ee_{{vr,filt,cmd}} + xyz"
            )
        self.create_subscription(
            PoseStamped, f"quest3/{self.side}_wrist_pose", self._on_wrist, 10
        )
        if self._homing or bool(self.get_parameter("use_joint_state_seed").value):
            self.create_subscription(
                JointState,
                f"/{self.side}_arm/joint_states",
                self._on_state,
                qos,
            )

        require = bool(self.get_parameter("require_clench_to_start").value)
        auto = bool(self.get_parameter("auto_arm_on_start").value)
        if self._require_start:
            # Wait for /teleop/start (or ~/start): do not arm or auto-zero on the
            # first VR pose. The user triggers start once their hand is placed.
            self._armed = False
        else:
            self._armed = (not require) or auto
        # Why teleop was last disarmed: None = never/freshly started,
        # "operator" = web/CLI pause (resume via /teleop/armed allowed),
        # "fault" = VR watchdog (must re-start to re-capture vr_init).
        self._disarm_reason: Optional[str] = None
        self.create_subscription(Bool, "/teleop/armed", self._on_armed, 10)
        self.create_subscription(Bool, "/teleop/disarm", self._on_disarm, 10)
        self.create_subscription(Bool, "/teleop/start", self._on_start, 10)
        self.create_service(Trigger, "~/start", self._start_srv)

        self._last_vr_t = 0.0
        self._last_vr_stamp = None
        self._prev_t = time.monotonic()
        self._print_latency = bool(self.get_parameter("print_latency").value)
        self._lat = LatencyMeter(
            float(self.get_parameter("latency_print_interval").value)
        )
        self.create_timer(self.dt, self._loop)
        solver_name = getattr(self.ik, "method_name", type(self.ik).__name__)
        n_via = max(0, len(self._homing_path) - 1)
        home_msg = (
            f"homing {n_via} via + init at {self._init_joint_vel:.2f} rad/s "
            f"({self.get_parameter('init_speed_percent').value}%)"
            if self._homing
            else "homing off"
        )
        self.get_logger().info(
            f"Astral arm teleop: {self.side} solver={solver_name} "
            f"EE0={np.round(self.robot_init_pos, 3).tolist()} "
            f"scale={self.pose.motion_scale} dry_run={self.dry_run} {home_msg}"
        )
        if self._require_start:
            self.get_logger().warn(
                f"[{self.side}] require_start_signal=true: waiting for "
                "/teleop/start (Bool true) or ~/start service. Place your hand "
                "at the initial pose, then send start to capture vr_init and arm."
            )
        self.add_on_set_parameters_callback(self._on_set_parameters)

    def _on_set_parameters(self, params):
        result = SetParametersResult(successful=True)
        solver_req = None
        lm = {}
        try:
            for p in params:
                name = p.name
                if name == "pos_smoothing":
                    self.pose.set_pos_smoothing(float(p.value))
                elif name == "rot_smoothing":
                    self.pose.set_rot_smoothing(float(p.value))
                elif name == "motion_scale":
                    self.pose.set_motion_scale(float(p.value))
                elif name == "flip_pitch":
                    self.pose.set_flip_pitch(bool(p.value))
                elif name == "max_joint_vel":
                    self.safety.max_joint_vel = float(p.value)
                elif name == "workspace_radius":
                    self.safety.workspace_radius = float(p.value)
                elif name == "data_timeout":
                    self.data_timeout = float(p.value)
                elif name == "dry_run":
                    self.dry_run = bool(p.value)
                elif name == "print_latency":
                    self._print_latency = bool(p.value)
                elif name == "solver_type":
                    solver_req = str(p.value).strip().lower()
                elif name == "ik_max_iter":
                    lm["max_iter"] = int(p.value)
                elif name == "ik_tol":
                    lm["tol"] = float(p.value)
                elif name == "ik_w_pos":
                    lm["w_pos"] = float(p.value)
                elif name == "ik_w_ori":
                    lm["w_ori"] = float(p.value)
                elif name == "ik_w_reg":
                    lm["w_reg"] = float(p.value)
                elif name in (
                    "arm_side",
                    "urdf_path",
                    "init_pose",
                    "init_waypoints",
                    "vr_to_arm_rot",
                    "tcp_offset",
                    "control_rate",
                    "move_to_init_pose",
                    "init_speed_percent",
                ):
                    result.successful = False
                    result.reason = f"{name} cannot be changed at runtime"
                    return result
            if solver_req is not None:
                self._rebuild_solver(solver_req)
            if lm:
                self._apply_ik_lm(lm)
        except Exception as exc:  # noqa: BLE001
            result.successful = False
            result.reason = str(exc)
            return result
        return result

    def _apply_ik_lm(self, lm: dict) -> None:
        ik = self.ik
        if hasattr(ik, "set_lm_params"):
            ik.set_lm_params(**lm)
            return
        # DH closed-form: LM weights do not apply.

    def _rebuild_solver(self, solver_type: str) -> None:
        st = solver_type.strip().lower()
        if st in ("analytic", "dh"):
            st = "analytic_dh"
        if st in ("urdf", "numerical"):
            st = "urdf_numerical"
        if st not in ("analytic_dh", "urdf_numerical"):
            raise ValueError(f"solver_type must be analytic_dh|urdf_numerical, got {st}")
        q_hw = np.asarray(self.q_cmd, dtype=float).reshape(7)
        self._flip_needed = st == "analytic_dh"
        self.ik = make_single_arm_ik(
            self.side,
            st,
            urdf_path=self._urdf_path,
            ik_max_iter=int(self.get_parameter("ik_max_iter").value),
            ik_tol=float(self.get_parameter("ik_tol").value),
            ik_w_pos=float(self.get_parameter("ik_w_pos").value),
            ik_w_ori=float(self.get_parameter("ik_w_ori").value),
            ik_w_reg=float(self.get_parameter("ik_w_reg").value),
        )
        R = self._vr_to_arm_yaml.copy()
        if self._flip_needed:
            R = _R_BASE_T @ R
        self.pose.R_vr_to_arm = R
        self.pose.R_world_to_base = R
        q_ik = self._flip_q(q_hw)
        q_ik = np.clip(
            q_ik, self.ik.lower_limits + 0.02, self.ik.upper_limits - 0.02
        )
        self.ik.sync_state(q_ik)
        T0 = self.ik.fk(q_ik)
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()
        self.q_cmd = q_hw
        self.safety.joint_lower = np.asarray(self.ik.lower_limits, dtype=float).copy()
        self.safety.joint_upper = np.asarray(self.ik.upper_limits, dtype=float).copy()
        self.safety.set_initial_state(q_ik, self.robot_init_pos)
        if self.pose.vr_current_pos is not None and self.pose.vr_current_rot is not None:
            self.pose.set_vr_zero_point(
                self.pose.vr_current_pos, self.pose.vr_current_rot
            )
        else:
            self.pose.reset()
        name = getattr(self.ik, "method_name", type(self.ik).__name__)
        self.get_logger().info(
            f"switched solver → {name}; VR zero reset, hold pose then move"
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

    def _parse_init_waypoints(self) -> list:
        """Hardware-convention via points from the flat ``init_waypoints`` array."""
        raw = np.asarray(
            self.get_parameter("init_waypoints").value, dtype=float
        ).ravel()
        if raw.size < 7:
            return []
        if raw.size % 7 != 0:
            raise ValueError(
                f"init_waypoints length {raw.size} is not a multiple of 7"
            )
        out = []
        lo = self.ik.lower_limits + 0.02
        hi = self.ik.upper_limits - 0.02
        for row in raw.reshape(-1, 7):
            q_ik = np.clip(self._flip_q(row), lo, hi)
            q_hw = self._flip_q(q_ik)
            if not np.allclose(q_hw, row, atol=1e-3):
                self.get_logger().warn(
                    f"[{self.side}] init_waypoints clipped to joint limits: "
                    f"{np.round(row, 3).tolist()} → {np.round(q_hw, 3).tolist()}"
                )
            out.append(q_hw)
        return out

    def _homing_target(self) -> np.ndarray:
        return self._homing_path[self._homing_i]

    def _on_armed(self, msg: Bool) -> None:
        # Only Bool(true) arms; a stray/default Bool(false) must not arm.
        if not msg.data:
            return
        if not self.pose.is_calibrated:
            # Also covers the web monitor's latched /teleop/armed residue
            # reaching a freshly started (uncalibrated) node.
            self.get_logger().warn(
                f"[{self.side}] armed ignored: not calibrated — "
                "send /teleop/start to capture vr_init first"
            )
            return
        if self._disarm_reason == "fault":
            self.get_logger().warn(
                f"[{self.side}] armed ignored: disarmed by VR watchdog — "
                "re-send /teleop/start to re-calibrate and resume"
            )
            return
        self._armed = True
        self._disarm_reason = None
        self.get_logger().info(f"[{self.side}] armed via /teleop/armed")

    def _on_disarm(self, _msg: Bool) -> None:
        self._armed = False
        # A fault reason sticks until a proper /teleop/start; an operator
        # pause must not downgrade it (pause→resume would bypass re-centering).
        if self._disarm_reason != "fault":
            self._disarm_reason = "operator"

    def _on_start(self, msg: Bool) -> None:
        if msg.data:
            self._start_teleop()

    def _start_srv(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        ok, message = self._start_teleop()
        resp.success = ok
        resp.message = message
        return resp

    def _start_teleop(self):
        """Capture vr_init from the current VR pose and arm teleop.

        Triggered by /teleop/start (global, both arms) or ~/start (per arm).
        Re-sending re-captures the zero from the current pose (re-center).
        """
        if self._homing:
            msg = "homing in progress; wait for init pose, then send start"
            self.get_logger().warn(f"[{self.side}] start: {msg}")
            return False, msg
        if self.pose.vr_current_pos is None:
            msg = "no VR wrist pose yet; start Quest stream, place hand, then send start"
            self.get_logger().warn(f"[{self.side}] start: {msg}")
            return False, msg
        if not self.pose.calibrate_from_current():
            msg = "calibrate failed (no VR pose)"
            self.get_logger().warn(f"[{self.side}] start: {msg}")
            return False, msg
        self._armed = True
        self._disarm_reason = None
        self.get_logger().warn(
            f"[{self.side}] START: vr_init captured from current pose, teleop armed"
        )
        return True, "started"

    def _on_state(self, msg: JointState) -> None:
        if len(msg.position) >= 7:
            self.state_q = np.asarray(msg.position[:7], dtype=float)
            self._got_state = True

    def _on_wrist(self, msg: PoseStamped) -> None:
        if self._homing:
            return
        pos = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]
        # Reject non-finite input first: NaN passes the norm check below
        # (NaN > 0.1 is False) and would permanently poison the EMA in
        # PoseProcessor and the SafetyFilter state downstream.
        if not np.isfinite(pos).all() or not np.isfinite(q).all():
            return
        if abs(np.linalg.norm(q) - 1.0) > 0.1:
            return
        self.pose.update_vr_pose(pos, Rotation.from_quat(q))
        self._last_vr_t = time.monotonic()
        self._last_vr_stamp = msg.header.stamp
        if self._print_latency:
            age = stamp_age_ms(msg.header.stamp)
            if 0.0 <= age < 5000.0:
                self._lat.add("vr_rx", age)

    def _loop(self) -> None:
        now = time.monotonic()
        dt = max(1e-3, now - self._prev_t)
        self._prev_t = now
        if self._homing:
            self._homing_tick(now, dt)
            return
        if not self._armed or not self.pose.is_calibrated:
            self._maybe_log_latency()
            return
        # VR stream stale: disarm instead of silently holding the last pose.
        # Resuming requires a fresh /teleop/start, which also re-captures
        # vr_init and resets the EMA (avoids a stale-state jump on reconnect).
        # data_timeout <= 0 disables the watchdog. Logs once per event: after
        # disarm the loop exits at the not-armed check above.
        if self.data_timeout > 0 and (
            self._last_vr_t <= 0.0 or (now - self._last_vr_t) > self.data_timeout
        ):
            self._armed = False
            self._disarm_reason = "fault"
            self.get_logger().error(
                f"[{self.side}] VR data timeout (>{self.data_timeout:.2f}s) — "
                "disarmed; re-send /teleop/start to resume"
            )
            self._maybe_log_latency()
            return

        t_loop = time.perf_counter()
        if self._print_latency and self._last_vr_t > 0:
            self._lat.add("vr_age", (now - self._last_vr_t) * 1000.0)

        dp, dr = self.pose.process(dt)
        T_tcp = self.pose.compute_target_pose(
            dp, dr, self.robot_init_pos, self.robot_init_rot
        )
        T_flange = T_tcp @ self._T_tcp_to_flange
        # Warm-start from last *commanded* q (after vel limit), not raw IK jump.
        try:
            self.ik.sync_state(self._flip_q(self.q_cmd), reset_branch=False)
        except TypeError:
            self.ik.sync_state(self._flip_q(self.q_cmd))
        t_ik = time.perf_counter()
        sol = self.ik.solve(T_flange)
        if self._print_latency:
            self._lat.add("ik", (time.perf_counter() - t_ik) * 1000.0)
        if sol is None:
            if self._print_latency:
                self._lat.count("ik_fail")
            self._publish_tune_poses(T_tcp)
            self._maybe_log_latency()
            return
        safe, _info = self.safety.filter(sol, dt)
        # IK/safety work in the flipped convention; convert back to hardware.
        self.q_cmd = self._flip_q(safe)
        if self._print_latency:
            self._lat.add("loop", (time.perf_counter() - t_loop) * 1000.0)
        self._publish_tune_poses(T_tcp)
        if self.dry_run:
            self._maybe_log_latency()
            return
        self._publish_q()
        self._maybe_log_latency()

    def _publish_q(self) -> None:
        msg = JointState()
        msg.header.stamp = (
            self._last_vr_stamp
            if self._last_vr_stamp is not None
            else self.get_clock().now().to_msg()
        )
        msg.name = list(self.names)
        msg.position = self.q_cmd.tolist()
        self.cmd_pub.publish(msg)

    def _finish_homing(self, now: float, reason: str) -> None:
        self.q_cmd = self._init_q_hw.copy()
        self.safety.set_initial_state(self._flip_q(self.q_cmd), self.robot_init_pos)
        self.pose.reset()
        self._homing = False
        elapsed = now - self._homing_t0 if self._homing_t0 else 0.0
        self.get_logger().warn(
            f"[{self.side}] Initial pose reached ({reason}, {elapsed:.1f}s). "
            "Hold VR still, then move."
        )

    def _homing_tick(self, now: float, dt: float) -> None:
        """Slow joint-space approach: init_waypoints in order, then init_pose."""
        if not self._homing_started:
            self._homing_started = True
            self._homing_t0 = now
            self._homing_last_log = now
            self._homing_i = 0
            n = len(self._homing_path)
            self.get_logger().warn(
                f"[{self.side}] Homing {n} segment(s) at "
                f"{self._init_joint_vel:.2f} rad/s; first target="
                f"{np.round(self._homing_target(), 3).tolist()}"
            )

        elapsed = now - self._homing_t0
        dt = min(float(dt), 0.05)
        if not self._homing_seeded:
            if not self._got_state:
                if elapsed < 3.0:
                    return
                self.get_logger().warn(
                    f"[{self.side}] No joint_states after 3s; homing from zeros"
                )
                self.q_cmd = np.zeros(7, dtype=float)
            else:
                self.q_cmd = self.state_q.copy()
            self.safety.set_initial_state(
                self._flip_q(self.q_cmd), self.robot_init_pos
            )
            self._homing_seeded = True

        target = self._homing_target()
        last = self._homing_i >= len(self._homing_path) - 1
        err = target - self.q_cmd
        max_abs = float(np.max(np.abs(err)))
        if now - self._homing_last_log >= 2.0:
            kind = "init" if last else f"via[{self._homing_i}]"
            self.get_logger().info(
                f"[{self.side}] Homing {kind}: error={max_abs:.3f} rad, "
                f"elapsed={elapsed:.1f}s"
            )
            self._homing_last_log = now

        if max_abs < self._init_arrive_tol:
            self.q_cmd = target.copy()
            if last:
                self._finish_homing(now, "arrived")
                self._publish_q()
                return
            self._homing_i += 1
            self.get_logger().warn(
                f"[{self.side}] Via {self._homing_i}/{len(self._homing_path)-1} "
                f"reached; next={np.round(self._homing_target(), 3).tolist()}"
            )
            self._publish_q()
            return
        if elapsed >= self._init_timeout:
            self.get_logger().warn(
                f"[{self.side}] Init pose timeout ({self._init_timeout:.0f}s), "
                f"error={max_abs:.3f} rad — VR zero uses current q"
            )
            T0 = self.ik.fk(self._flip_q(self.q_cmd))
            self.robot_init_pos = T0[:3, 3].copy()
            self.robot_init_rot = T0[:3, :3].copy()
            self._init_q_hw = self.q_cmd.copy()
            self._finish_homing(now, "timeout")
            return

        max_d = self._init_joint_vel * dt
        self.q_cmd = self.q_cmd + np.clip(err, -max_d, max_d)
        self._publish_q()

    def _pose_msg(self, T: np.ndarray, stamp) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = f"{self.side}_ik"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
            float(T[0, 3]),
            float(T[1, 3]),
            float(T[2, 3]),
        )
        q = Rotation.from_matrix(T[:3, :3]).as_quat()
        (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ) = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
        return msg

    def _publish_tune_poses(self, T_tcp: np.ndarray) -> None:
        if not self._publish_tune or not self._tune_pubs:
            return
        stamp = (
            self._last_vr_stamp
            if self._last_vr_stamp is not None
            else self.get_clock().now().to_msg()
        )
        T_vr = self.pose.compute_target_pose(
            self.pose.last_raw_delta_pos,
            self.pose.last_raw_delta_rot,
            self.robot_init_pos,
            self.robot_init_rot,
        )
        T_cmd = self.ik.fk(self._flip_q(self.q_cmd)) @ self._T_flange_to_tcp
        self._tune_pubs["vr"].publish(self._pose_msg(T_vr, stamp))
        self._tune_pubs["filt"].publish(self._pose_msg(T_tcp, stamp))
        self._tune_pubs["cmd"].publish(self._pose_msg(T_cmd, stamp))
        q_vr = Rotation.from_matrix(T_vr[:3, :3]).as_quat()
        q_fi = Rotation.from_matrix(T_tcp[:3, :3]).as_quat()
        q_cmd = Rotation.from_matrix(T_cmd[:3, :3]).as_quat()
        packed = Float64MultiArray()
        packed.data = [
            float(T_vr[0, 3]),
            float(T_vr[1, 3]),
            float(T_vr[2, 3]),
            float(T_tcp[0, 3]),
            float(T_tcp[1, 3]),
            float(T_tcp[2, 3]),
            float(T_cmd[0, 3]),
            float(T_cmd[1, 3]),
            float(T_cmd[2, 3]),
            *map(float, q_vr),
            *map(float, q_fi),
            *map(float, q_cmd),
        ]
        self._tune_pubs["xyz"].publish(packed)

    def _maybe_log_latency(self) -> None:
        if not self._print_latency or not self._lat.has_samples():
            return
        if not self._lat.should_print():
            return
        self.get_logger().info(
            f"[Latency][{self.side}] {self._lat.format_and_reset()}"
        )


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

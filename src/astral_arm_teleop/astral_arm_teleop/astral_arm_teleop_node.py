#!/usr/bin/env python3
"""Single-arm Astral teleop (Nero layout).

Quest wrist → PoseProcessor → IK (``geometric`` default; ``analytic_dh`` /
``urdf_numerical`` selectable) → SafetyFilter → ``/{side}_arm/joint_commands``.

With ``use_human_elbow`` (default on, geometric only), the Quest IOBT
shoulder/elbow from ``quest3/body_joints`` pins the arm angle psi_ref so the
robot elbow plane follows the human's.

Use two instances (left + right) for dual-arm (process-level parallel).
"""

from __future__ import annotations

import json
import math
import time
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
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
from std_msgs.msg import Bool, Float64MultiArray, String
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
    """腕位 / 关节 / 指令：只留最新一帧，避免 IK 跟不上时把旧样本排队。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


class AstralTeleopArmNode(Node):
    def __init__(self) -> None:
        super().__init__("astral_arm_teleop_arm")
        self.declare_parameter("arm_side", "left")
        self.declare_parameter("control_rate", 50.0)
        self.declare_parameter("solver_type", "geometric")
        self.declare_parameter("urdf_path", "")  # empty → astral_robot.pin.urdf
        self.declare_parameter("ik_max_iter", 20)
        self.declare_parameter("ik_tol", 1e-8)
        self.declare_parameter("ik_w_pos", 1.0)
        self.declare_parameter("ik_w_ori", 0.3)
        self.declare_parameter("ik_w_reg", 1e-4)
        # Joint4 URDF upper=0 is fully stretched (elbow singularity).
        # <0 caps IK/safety below that; >=0 keeps the URDF limit.
        self.declare_parameter("ik_q4_max", -0.45)
        self.declare_parameter("ik_w_limit", 0.12)
        # Per-solve joint step box around q_prev (rad). 0 = off. Blocks IK branch jumps.
        self.declare_parameter("ik_dq_max", 0.0)
        # Pull toward init_pose when redundant. 0 = off.
        self.declare_parameter("ik_w_pref", 0.0)
        # One-sided elbow fold: penalize q4 straighter than ik_q4_fold.
        self.declare_parameter("ik_w_fold", 0.015)
        self.declare_parameter("ik_q4_fold", -1.20)
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
        # Radial soft wall at the elbow-straight singularity (geometric
        # solver only): the wrist target is clamped to l_se + l_ew -
        # reach_margin from the shoulder center. At full extension the
        # elbow circle degenerates and q4 hits its limit, so without the
        # clamp IK returns None and the arm visibly catches. <=0 disables.
        self.declare_parameter("reach_margin", 0.01)
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
        # HOME park 逐拍贴实测 joint_states（fresh 且与 q_cmd 接近时）：电机
        # 失能/使能晚到（急停后）时 q_cmd 不"内部空跑"领先机器人，等使能后
        # 从真实位姿继续走收回零位，避免猛扑。启动 init 归位**不**用（保持改前
        # 开环推进）。
        self.declare_parameter("homing_track_state", True)
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
        # Human arm-angle prior from Quest body tracking (geometric solver
        # only): body_joints {side}-arm-upper/-lower give the human upper-arm
        # direction -> psi_ref, so the robot elbow plane follows the human's.
        # Falls back to pure continuity when the body stream is stale.
        self.declare_parameter("use_human_elbow", True)
        self.declare_parameter("human_elbow_weight", 2.0)
        self.declare_parameter("human_elbow_timeout", 0.3)
        self.declare_parameter("human_elbow_smoothing_tau", 0.15)
        # Straightness gate: when the human arm is nearly straight (elbow
        # within ~sin*upper-arm-length of the shoulder-wrist line), psi is
        # unobservable and IOBT bias would swivel the robot elbow to a
        # garbage angle. 0.15 ~= elbow 4 cm off the line on a 28 cm upper
        # arm; normal bent-arm gestures have sin >= 0.5.
        self.declare_parameter("human_elbow_min_sin", 0.15)
        # 逃逸迟滞（仅 geometric）：局部窗连续空 esc 帧才全局逃逸，逃逸后
        # home 窗连续可行 ret 帧才返回。roll 大角度腕限位边界闪空时防肩抽。
        self.declare_parameter("ik_escape_after_frames", 4)
        self.declare_parameter("ik_return_after_frames", 20)

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
            ik_q4_max=float(self.get_parameter("ik_q4_max").value),
            ik_w_limit=float(self.get_parameter("ik_w_limit").value),
            ik_dq_max=float(self.get_parameter("ik_dq_max").value),
            ik_w_pref=float(self.get_parameter("ik_w_pref").value),
            ik_w_fold=float(self.get_parameter("ik_w_fold").value),
            ik_q4_fold=float(self.get_parameter("ik_q4_fold").value),
        )
        self.reach_margin = float(self.get_parameter("reach_margin").value)
        self._use_human_elbow = bool(self.get_parameter("use_human_elbow").value)
        self._human_elbow_weight = float(self.get_parameter("human_elbow_weight").value)
        self._human_elbow_timeout = float(
            self.get_parameter("human_elbow_timeout").value
        )
        self._human_elbow_tau = float(
            self.get_parameter("human_elbow_smoothing_tau").value
        )
        self._human_elbow_min_sin = float(
            self.get_parameter("human_elbow_min_sin").value
        )
        self._apply_human_elbow_cfg()
        # Body-tracking prior state (Quest IOBT, robot_body/hips frame).
        self._body_names: Optional[List[str]] = None
        self._elbow_dir_vr: Optional[np.ndarray] = None  # EMA-smoothed unit dir
        self._elbow_dir_t: float = 0.0
        self._body_subs = None
        # Arm-angle escape watcher state (geometric solver; see main loop).
        self._esc_prev_active = False
        init_q_old = np.asarray(
            self.get_parameter("init_pose").value, dtype=float
        ).reshape(7)  # hardware/original convention (config)
        init_q = self._flip_q(init_q_old)  # DH/flipped convention (feed IK)
        init_q = np.clip(
            init_q, self.ik.lower_limits + 0.02, self.ik.upper_limits - 0.02
        )
        if hasattr(self.ik, "set_lm_params"):
            self.ik.set_lm_params(q_pref=init_q)
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
        self._homing_last_base = None
        self._homing_started = False
        self._homing_seeded = False
        # "init" = 启动归位 init_waypoints→init_pose；"park" = HOME 按钮
        # init_pose→init_waypoints→零位。决定 _finish_homing 的终态目标。
        self._homing_mode = "init"
        self._track_homing_state = bool(
            self.get_parameter("homing_track_state").value
        )

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
            PoseStamped, f"quest3/{self.side}_wrist_pose", self._on_wrist, qos
        )
        self._ensure_body_subs()
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
        # HITL takeover: re-anchor the robot origin to the current measured
        # joint state AND the VR zero to the current hand pose, then arm.
        self.create_service(Trigger, "~/reanchor", self._reanchor_srv)
        # HOME / park-to-zero: init_pose → init_waypoints → 零位（双臂同启用
        # /teleop/home 一次性信号，单臂用 ~/home 服务）。VOLATILE 语义同
        # /teleop/start：晚启动节点不得被历史 HOME 信号误触发。
        self.create_subscription(Bool, "/teleop/home", self._on_home, 10)
        self.create_service(Trigger, "~/home", self._home_srv)

        self._state_t: Optional[float] = None
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
        ee0_r = float(np.linalg.norm(self.robot_init_pos))
        ws_r = float(self.safety.workspace_radius)
        self.get_logger().info(
            f"Astral arm teleop: {self.side} solver={solver_name} "
            f"EE0={np.round(self.robot_init_pos, 3).tolist()} "
            f"r={ee0_r:.3f}m ws_r={ws_r:.3f}m "
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
                elif name == "reach_margin":
                    self.reach_margin = float(p.value)
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
                elif name == "ik_dq_max":
                    lm["dq_max"] = float(p.value)
                elif name == "ik_w_pref":
                    lm["w_pref"] = float(p.value)
                elif name == "ik_w_fold":
                    lm["w_fold"] = float(p.value)
                elif name == "ik_q4_fold":
                    lm["q4_fold"] = float(p.value)
                elif name == "ik_q4_max":
                    lm["q4_max"] = float(p.value)
                elif name == "ik_w_limit":
                    lm["w_limit"] = float(p.value)
                elif name == "use_human_elbow":
                    self._use_human_elbow = bool(p.value)
                    self._ensure_body_subs()
                elif name == "human_elbow_weight":
                    self._human_elbow_weight = float(p.value)
                    self._apply_human_elbow_cfg()
                elif name == "human_elbow_timeout":
                    self._human_elbow_timeout = float(p.value)
                elif name == "human_elbow_smoothing_tau":
                    self._human_elbow_tau = float(p.value)
                elif name == "human_elbow_min_sin":
                    self._human_elbow_min_sin = float(p.value)
                elif name in ("ik_escape_after_frames", "ik_return_after_frames"):
                    cont = getattr(self.ik, "continuity", None)
                    if cont is not None:
                        attr = (
                            "escape_after_frames"
                            if name == "ik_escape_after_frames"
                            else "return_after_frames"
                        )
                        setattr(cont, attr, max(1, int(p.value)))
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
        if hasattr(ik, "lower_limits") and hasattr(ik, "upper_limits"):
            self.safety.joint_lower = np.asarray(ik.lower_limits, dtype=float).copy()
            self.safety.joint_upper = np.asarray(ik.upper_limits, dtype=float).copy()
        # DH closed-form: LM weights do not apply.

    def _rebuild_solver(self, solver_type: str) -> None:
        st = solver_type.strip().lower()
        if st in ("analytic", "dh"):
            st = "analytic_dh"
        if st in ("urdf", "numerical"):
            st = "urdf_numerical"
        if st in ("swe", "poe", "geometric_arm_angle"):
            st = "geometric"
        if st not in ("analytic_dh", "geometric", "urdf_numerical"):
            raise ValueError(
                f"solver_type must be analytic_dh|geometric|urdf_numerical, got {st}"
            )
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
            ik_q4_max=float(self.get_parameter("ik_q4_max").value),
            ik_w_limit=float(self.get_parameter("ik_w_limit").value),
            ik_dq_max=float(self.get_parameter("ik_dq_max").value),
            ik_w_pref=float(self.get_parameter("ik_w_pref").value),
            ik_w_fold=float(self.get_parameter("ik_w_fold").value),
            ik_q4_fold=float(self.get_parameter("ik_q4_fold").value),
            escape_after_frames=int(
                self.get_parameter("ik_escape_after_frames").value
            ),
            return_after_frames=int(
                self.get_parameter("ik_return_after_frames").value
            ),
        )
        self._apply_human_elbow_cfg()
        if self._use_human_elbow and not hasattr(self.ik, "arm_angle_from_elbow_dir"):
            self.get_logger().warn(
                f"[{self.side}] human elbow prior requires the geometric solver; "
                f"disabled for {st}"
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
        if hasattr(self.ik, "set_lm_params"):
            self.ik.set_lm_params(q_pref=self._flip_q(self._init_q_hw))
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

    def _on_disarm(self, msg: Bool) -> None:
        # /teleop/disarm is a *level* signal: only Bool(true) disarms. Several
        # owners publish to it (web pause = True, astral_policy_inference opens
        # the gate with False on IDLE/HUMAN), so a False must not disarm an
        # armed teleop that a re-anchor / ~/start just armed.
        if not msg.data:
            return
        self._armed = False
        # A fault reason sticks until a proper /teleop/start; an operator
        # pause must not downgrade it (pause→resume would bypass re-centering).
        if self._disarm_reason != "fault":
            self._disarm_reason = "operator"
        if self._homing:
            # Operator disarm while a homing/park is running must cancel it:
            # otherwise the node keeps streaming the stale joint trajectory
            # (e.g. the slow startup init move), which would override a later
            # driver ~/home / ~/ready zero once motion mode returns to POSITION
            # (症状: 阻尼释放后点一键就绪，臂回到阻尼前位姿). The arm simply
            # holds its last commanded pose; a later /teleop/start re-captures.
            self._homing = False
            self._homing_started = False
            self.get_logger().warn(
                f"[{self.side}] homing cancelled by disarm — hold current pose"
            )

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

    def _on_home(self, msg: Bool) -> None:
        # One-shot level guard (mirrors /teleop/start): only Bool(true) triggers.
        if msg.data:
            self._go_home()

    def _home_srv(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self._go_home()
        resp.success = ok
        resp.message = message
        return resp

    def _go_home(self):
        """HOME / park-to-zero：从当前位姿慢速走 init_pose → init_waypoints →
        零位（硬件约定），到零位后保持（关节目标=0）。

        用于把遥操（或急停恢复后）的臂安全收回零位。动作前先 disarm（若在
        遥操/armed），期间忽略 VR/start；结束后要再遥操需重新 /teleop/start。
        与启动归位共用同一条慢速 joint-space 轨迹机（_homing_*），仅终态与
        轨迹点不同。
        """
        if self._homing:
            msg = "homing/park already in progress; wait until it finishes"
            self.get_logger().warn(f"[{self.side}] home: {msg}")
            return False, msg
        self._armed = False
        if self._disarm_reason != "fault":
            self._disarm_reason = "operator"
        self._homing_mode = "park"
        self._homing_path = (
            [self._init_q_hw.copy()]
            + self._parse_init_waypoints()
            + [np.zeros(7, dtype=float)]
        )
        self._homing_i = 0
        self._homing_started = False
        self._homing_seeded = False
        self._homing = True
        self._homing_last_log = 0.0
        self._homing_last_base = None
        self.pose.reset()
        self.get_logger().warn(
            f"[{self.side}] HOME: disarm + park to zero via init_pose → "
            f"{len(self._parse_init_waypoints())} via → zero "
            f"({self._init_joint_vel:.2f} rad/s)"
        )
        return True, "HOME 已启动 (init_pose → init_waypoints → 零位)"

    def _on_state(self, msg: JointState) -> None:
        if len(msg.position) >= 7:
            self.state_q = np.asarray(msg.position[:7], dtype=float)
            self._got_state = True
            self._state_t = time.monotonic()

    def _reanchor_srv(
        self, _req: Trigger.Request, resp: Trigger.Response
    ) -> Trigger.Response:
        ok, message = self._reanchor_teleop()
        resp.success = ok
        resp.message = message
        return resp

    def _reanchor_teleop(self):
        """Re-anchor robot origin + VR zero to the *current* state/pose, then arm.

        Used for HITL takeover: after a policy has moved the arm away from the
        startup pose, ``_start_teleop`` alone would command toward the stale
        startup ``robot_init`` and jump. This re-derives ``robot_init_pos/rot``
        from the latest measured joint state (fallback: last commanded q) and
        captures ``vr_init`` from the current VR pose, so subsequent motion is
        purely incremental from where the robot actually is.

        Returns ``(ok, message)``; on failure nothing is re-anchored/armed.
        """
        if self._homing:
            msg = "homing in progress; wait for init pose, then re-anchor"
            self.get_logger().warn(f"[{self.side}] reanchor: {msg}")
            return False, msg
        if self.pose.vr_current_pos is None or self.pose.vr_current_rot is None:
            msg = "no VR wrist pose yet; start Quest stream, place hand, then re-anchor"
            self.get_logger().warn(f"[{self.side}] reanchor: {msg}")
            return False, msg
        state_fresh = self._got_state and (
            self.data_timeout <= 0.0
            or (self._state_t is not None and time.monotonic() - self._state_t <= self.data_timeout)
        )
        q_hw = np.asarray(self.state_q if state_fresh else self.q_cmd, dtype=float).reshape(7)
        q_ik = np.clip(
            self._flip_q(q_hw), self.ik.lower_limits + 0.02, self.ik.upper_limits - 0.02
        )
        self.ik.sync_state(q_ik)
        T0 = self.ik.fk(q_ik)
        self.robot_init_pos = T0[:3, 3].copy()
        self.robot_init_rot = T0[:3, :3].copy()
        self.q_cmd = q_hw
        self.safety.set_initial_state(q_ik, self.robot_init_pos)
        if not self.pose.calibrate_from_current():
            msg = "re-anchor failed (no VR pose for zero capture)"
            self.get_logger().warn(f"[{self.side}] reanchor: {msg}")
            return False, msg
        self._armed = True
        self._disarm_reason = None
        src = "measured joint state" if state_fresh else "last commanded q"
        self.get_logger().warn(
            f"[{self.side}] REANCHOR: robot origin ← FK({src}) "
            f"{np.round(self.robot_init_pos, 3).tolist()}, vr_init ← current pose, armed"
        )
        return True, "re-anchored + armed"


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

    # ---- Human elbow prior (Quest body tracking -> geometric arm angle) ----

    def _apply_human_elbow_cfg(self) -> None:
        cont = getattr(self.ik, "continuity", None)
        if cont is not None and hasattr(cont, "w_psi_ref"):
            cont.w_psi_ref = self._human_elbow_weight

    def _ensure_body_subs(self) -> None:
        if not self._use_human_elbow or self._body_subs is not None:
            return
        if not hasattr(self.ik, "arm_angle_from_elbow_dir"):
            self.get_logger().warn(
                f"[{self.side}] use_human_elbow=true but solver "
                f"{getattr(self.ik, 'method_name', '?')} has no arm-angle prior "
                "support (geometric only) — ignoring"
            )
            self._use_human_elbow = False
            return
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._body_subs = (
            self.create_subscription(
                String, "quest3/body_joint_names", self._on_body_names, latched
            ),
            self.create_subscription(
                PoseArray, "quest3/body_joints", self._on_body_joints, _sensor_qos()
            ),
        )
        self.get_logger().info(
            f"[{self.side}] human elbow prior on: quest3/body_joints "
            f"({self.side}-arm-upper/-lower), w={self._human_elbow_weight}"
        )

    def _on_body_names(self, msg: String) -> None:
        try:
            names = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(names, list) and names:
            self._body_names = [str(n) for n in names]

    def _on_body_joints(self, msg: PoseArray) -> None:
        names = self._body_names
        if not names or len(msg.poses) != len(names):
            return
        # {side}-arm-upper joint sits at the shoulder ball, {side}-arm-lower
        # at the elbow (Meta IOBT skeleton; aliases tolerated).
        i_up = i_lo = -1
        for alias_up, alias_lo in (
            (f"{self.side}-arm-upper", f"{self.side}-arm-lower"),
            (f"{self.side}-upper-arm", f"{self.side}-lower-arm"),
            (f"{self.side}-shoulder", f"{self.side}-elbow"),
        ):
            if alias_up in names and alias_lo in names:
                i_up, i_lo = names.index(alias_up), names.index(alias_lo)
                break
        if i_up < 0:
            return
        pu, pl = msg.poses[i_up].position, msg.poses[i_lo].position
        d = np.array([pl.x - pu.x, pl.y - pu.y, pl.z - pu.z], dtype=float)
        if not np.isfinite(d).all():
            return
        n = float(np.linalg.norm(d))
        if n < 0.05 or n > 1.0:  # implausible human upper-arm length
            return
        d /= n
        now = time.monotonic()
        if self._elbow_dir_vr is None or self._elbow_dir_t <= 0.0:
            self._elbow_dir_vr = d
        else:
            dt_e = min(0.5, max(1e-3, now - self._elbow_dir_t))
            a = math.exp(-dt_e / max(1e-3, self._human_elbow_tau))
            v = a * self._elbow_dir_vr + (1.0 - a) * d
            nv = float(np.linalg.norm(v))
            if nv > 1e-6:  # keep last good dir if EMA degenerates
                self._elbow_dir_vr = v / nv
        self._elbow_dir_t = now

    def _human_psi_ref(self, T_flange: np.ndarray) -> Optional[float]:
        """Arm-angle prior (rad) from the latest human elbow direction."""
        fn = getattr(self.ik, "arm_angle_from_elbow_dir", None)
        d = self._elbow_dir_vr
        if fn is None or d is None or self._elbow_dir_t <= 0.0:
            return None
        if (time.monotonic() - self._elbow_dir_t) > self._human_elbow_timeout:
            return None
        # Same rotation as wrist deltas (direction only: no zero-point/scale).
        d_arm = self.pose.R_vr_to_arm @ d
        try:
            psi = fn(T_flange, d_arm, min_sin=self._human_elbow_min_sin)
        except Exception:  # noqa: BLE001 - never let the prior break teleop
            return None
        if psi is None and self._print_latency:
            # Fresh body data but prior gated (nearly straight human arm or
            # degenerate target) — visible in the [Latency] line.
            self._lat.count("psi_off")
        return psi

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
        # Sphere in the IK frame (*_base_link origin). radius<=0 disables.
        p_req = T_flange[:3, 3].copy()
        r_req = float(np.linalg.norm(p_req))
        if self._print_latency:
            self._lat.add("ee_r", r_req * 1000.0, unit="mm")
        p_ws = self.safety.check_workspace(p_req)
        if not np.allclose(p_ws, p_req, atol=1e-9):
            T_flange[:3, 3] = p_ws
            T_tcp = T_flange @ self._T_flange_to_tcp
            if self._print_latency:
                self._lat.count("ws_clip")
        # Soft wall at full elbow extension: clamp the wrist target to the
        # reachable sphere around the shoulder center. Beyond it the elbow
        # circle degenerates, q4 hits its limit, IK fails, and the arm
        # catches; clamping keeps IK well-posed (hand feels a soft wall).
        clamp_reach = getattr(self.ik, "clamp_wrist_reach", None)
        if clamp_reach is not None and self.reach_margin > 0.0:
            p_clamped, clipped = clamp_reach(p_req, self.reach_margin)
            if clipped:
                T_flange[:3, 3] = p_clamped
                T_tcp = T_flange @ self._T_flange_to_tcp
                p_req = p_clamped
                if self._print_latency:
                    self._lat.count("reach_clip")
        # Warm-start from last *commanded* q (after vel limit), not raw IK jump.
        try:
            self.ik.sync_state(self._flip_q(self.q_cmd), reset_branch=False)
        except TypeError:
            self.ik.sync_state(self._flip_q(self.q_cmd))
        t_ik = time.perf_counter()
        psi_ref = self._human_psi_ref(T_flange) if self._use_human_elbow else None
        if psi_ref is None:
            sol = self.ik.solve(T_flange)
        else:
            sol = self.ik.solve(T_flange, psi_ref=psi_ref)
        if self._print_latency:
            self._lat.add("ik", (time.perf_counter() - t_ik) * 1000.0)
        # Arm-angle escape: the local psi window went empty for
        # ik_escape_after_frames (wrist-limit boundary during a big roll) and
        # the solver jumped to a globally feasible psi — the visible shoulder
        # swing. Count it and log once per escape; return is debounced by
        # ik_return_after_frames.
        esc_state = getattr(self.ik, "_state", None)
        if esc_state is not None and esc_state.esc_active:
            if self._esc_prev_active is not True:
                self._esc_prev_active = True
                if self._print_latency:
                    self._lat.count("psi_escape")
                self.get_logger().warn(
                    f"[{self.side}] arm-angle escape: wrist limit swallowed the "
                    f"local psi window during a big roll; shoulder swings "
                    f"once, returns after ik_return_after_frames of feasible "
                    f"home psi"
                )
        else:
            self._esc_prev_active = False
        if sol is None:
            if self._print_latency:
                self._lat.count("ik_fail")
            self._publish_tune_poses(T_tcp)
            self._maybe_log_latency()
            return
        if self._print_latency:
            Terr = self.ik.fk(sol)
            sat = float(np.linalg.norm(Terr[:3, 3] - T_flange[:3, 3]))
            if sat > 0.008:
                self._lat.count("ik_sat")
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
        mode = self._homing_mode
        self._homing_mode = "init"
        if mode == "park":
            if reason == "arrived":
                # 到零位：终态 q=0，并把机器人原点锚到零位 FK（后续 start/
                # re-anchor 从实际位姿重新标定前不会用旧 init 目标）
                self.q_cmd = np.zeros(7, dtype=float)
                T0 = self.ik.fk(self._flip_q(self.q_cmd))
                self.robot_init_pos = T0[:3, 3].copy()
                self.robot_init_rot = T0[:3, :3].copy()
            # reason=timeout → 保持当前 q_cmd（未到零位绝不能硬发零目标）
            self.safety.set_initial_state(
                self._flip_q(self.q_cmd), self.robot_init_pos
            )
            self.pose.reset()
            self._homing = False
            elapsed = now - self._homing_t0 if self._homing_t0 else 0.0
            verb = "Parked at zero" if reason == "arrived" else "Park interrupted"
            self.get_logger().warn(
                f"[{self.side}] {verb} ({reason}, {elapsed:.1f}s). "
                "Arms hold; send /teleop/start to resume teleop"
            )
            return
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

        # base = q_cmd（启动 init 归位保持改前开环推进：web 启动已回退自动
        # enable，启动归位就按原样逐拍朝轨迹推进，不因实测没动而卡住）。
        # HOME park 才用反馈钳制：电机正常跟随时从*实测位姿*迈步，避免使能
        # 晚到/失能时 q_cmd"内部空跑"领先实体，收回零位时不会猛扑。
        base = self.q_cmd
        if (
            self._homing_mode == "park"
            and self._track_homing_state
            and self._got_state
            and self._state_t is not None
        ):
            state_fresh = self.data_timeout <= 0.0 or (
                now - self._state_t <= max(self.data_timeout, 1.0)
            )
            if state_fresh:
                sdiff = float(np.max(np.abs(self.state_q - self.q_cmd)))
                if sdiff < 0.6:
                    base = np.asarray(self.state_q, dtype=float).reshape(7)

        target = self._homing_target()
        last = self._homing_i >= len(self._homing_path) - 1
        err = target - base
        max_abs = float(np.max(np.abs(err)))
        if now - self._homing_last_log >= 2.0:
            if self._homing_mode == "park" and last:
                kind = "zero"
            elif self._homing_mode == "park":
                kind = f"park[{self._homing_i}]"
            else:
                kind = "init" if last else f"via[{self._homing_i}]"
            stuck = ""
            if base is not self.q_cmd and self._homing_last_base is not None:
                moved = float(np.max(np.abs(base - self._homing_last_base)))
                if moved < 1e-4 and max_abs > self._init_arrive_tol:
                    stuck = " — 机器人没在动（电机未使能/堵转？）"
            self.get_logger().info(
                f"[{self.side}] Homing {kind}: error={max_abs:.3f} rad, "
                f"elapsed={elapsed:.1f}s{stuck}"
            )
            self._homing_last_log = now
            self._homing_last_base = base.copy()

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
            if self._homing_mode == "park":
                self.get_logger().warn(
                    f"[{self.side}] Home timeout ({self._init_timeout:.0f}s), "
                    f"error={max_abs:.3f} rad — hold at current q"
                )
            else:
                # 启动 init 归位保持改前措辞与语义：超时即用当前 q 作 VR 零点。
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
        self.q_cmd = base + np.clip(err, -max_d, max_d)
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

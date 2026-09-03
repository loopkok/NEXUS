"""Centralised configuration: topic names, thresholds, ports, presets loader.

All topic names live here so the monitor never hardcodes ROS contracts in
multiple places. Values can be overridden via environment variables for
testing without touching code.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field


# --- Web server -------------------------------------------------------------
WEB_HOST = os.environ.get("ASTRAL_WEB_MONITOR_HOST", "0.0.0.0")
WEB_PORT = int(os.environ.get("ASTRAL_WEB_MONITOR_PORT", "8080"))

# --- WebSocket push ---------------------------------------------------------
WS_PUSH_HZ = float(os.environ.get("ASTRAL_WEB_MONITOR_PUSH_HZ", "30"))
# 环形缓冲（全量，供 GET /api/v1/logs 下载）；WS ui_state 只推尾部以免撑爆帧。
WS_LOG_TAIL_LINES = int(os.environ.get("ASTRAL_WEB_MONITOR_LOG_TAIL", "8000"))
WS_LOG_PUSH_LINES = int(os.environ.get("ASTRAL_WEB_MONITOR_LOG_PUSH", "800"))


def log_tail_for_push(lines: list[str], n: int | None = None) -> list[str]:
    """WS 实时推送切片：只带环形缓冲的最后 n 行。"""
    keep = WS_LOG_PUSH_LINES if n is None else n
    if keep <= 0:
        return []
    return lines[-keep:]

# --- Stale detection --------------------------------------------------------
STALE_THRESHOLD_S = float(os.environ.get("ASTRAL_WEB_MONITOR_STALE_S", "2.0"))

# --- Launch subprocess ------------------------------------------------------
STARTUP_WARMUP_S = float(os.environ.get("ASTRAL_WEB_MONITOR_WARMUP_S", "2.0"))
STOP_SIGINT_TIMEOUT_S = float(os.environ.get("ASTRAL_WEB_MONITOR_STOP_TIMEOUT_S", "30.0"))
STOP_SIGKILL_GRACE_S = float(os.environ.get("ASTRAL_WEB_MONITOR_KILL_GRACE_S", "5.0"))

# --- Rate counter EWMA -----------------------------------------------------
RATE_EWMA_ALPHA = float(os.environ.get("ASTRAL_WEB_MONITOR_RATE_ALPHA", "0.7"))

# --- Health inspection ------------------------------------------------------
# Expected command-rate floor (Hz) per entity, used by the health panel to flag
# a stream as "slow"/"dead". Read-only — the monitor never enforces these.
# Override via env as comma-separated name=hz pairs.
_DEFAULT_EXPECTED = {
    "left_arm_cmd": 30.0,
    "right_arm_cmd": 30.0,
    "left_gripper_cmd": 30.0,
    "right_hand_cmd": 30.0,
}


def _parse_expected() -> dict[str, float]:
    raw = os.environ.get("ASTRAL_WEB_MONITOR_EXPECTED_HZ", "")
    out = dict(_DEFAULT_EXPECTED)
    if not raw:
        return out
    for part in raw.split(","):
        part = part.strip()
        if "=" in part:
            k, v = part.split("=", 1)
            try:
                out[k.strip()] = float(v)
            except ValueError:
                pass
    return out


EXPECTED_RATES_HZ: dict[str, float] = _parse_expected()

# --- ROS topic names (read-only subscriptions) -----------------------------
# Grouped by entity. Each entry: (state_topic, command_topic, cmd_kind).
# command_topic is "" when there is no command stream to monitor for that
# entity. cmd_kind: "joint_state" (JointState) or "float64" (std_msgs/Float64)
# — grippers stream a unitless close ratio on /{side}_gripper/command (the
# driver owns the rad mapping), so their command topic is Float64.
@dataclass(frozen=True)
class TopicPair:
    state: str
    command: str = ""
    cmd_kind: str = "joint_state"


TOPICS: dict[str, TopicPair] = {
    "left_arm":      TopicPair("/left_arm/joint_states",      "/left_arm/joint_commands"),
    "right_arm":     TopicPair("/right_arm/joint_states",     "/right_arm/joint_commands"),
    "left_gripper":  TopicPair("/left_gripper/joint_states", "/left_gripper/command", "float64"),
    "right_hand":    TopicPair("/right_hand/joint_states",    "/right_hand/joint_commands"),
    # Head (astral_teleop/head_teleop_node → driver). No cmd-rate floor on
    # purpose: head commands legitimately idle at 0 Hz before start / when the
    # right controller is not streaming, so a floor would flag false "slow".
    "head":          TopicPair("/head/joint_states",          "/head/joint_commands"),
    "full_body":     TopicPair("/astral/joint_states",        ""),
}

# Hand namespace is configurable because wujihand uses {hand_name} not "hand".
HAND_NAME = os.environ.get("ASTRAL_WEB_MONITOR_HAND_NAME", "right_hand")
# Rebuild right_hand topics with the configured hand name.
_right_hand_state = f"/{HAND_NAME}/joint_states"
_right_hand_cmd = f"/{HAND_NAME}/joint_commands"
TOPICS["right_hand"] = TopicPair(_right_hand_state, _right_hand_cmd)

# --- Latency / pipeline metrics (read-only subscriptions) -------------------
# Mocap wrist poses: header.stamp = host wall-clock packet arrival. The monitor
# computes stamp age = now - stamp as a proxy for mocap→ROS pipeline delay.
MOCAP_WRIST_TOPICS = {
    "left":  "/quest3/left_wrist_pose",
    "right": "/quest3/right_wrist_pose",
}
# IK solver status: std_msgs/String with "OK dt=X.XXms" or "FAILED". The monitor
# parses the solve time for the latency panel.
IK_STATUS_TOPICS = {
    "left":  "/ik_solver_left/ik_status",
    "right": "/ik_solver_right/ik_status",
}
# Expected mocap wrist rate (Hz) for health "slow" flagging.
MOCAP_EXPECTED_HZ = float(os.environ.get("ASTRAL_WEB_MONITOR_MOCAP_HZ", "60.0"))

# --- Control topics (publish, pause/resume) ---------------------------------
# These already exist in astral_arm_teleop; publishing to them is the only
# write path the monitor uses. It never touches hardware topics.
TOPIC_ARMED = "/teleop/armed"
TOPIC_DISARM = "/teleop/disarm"
# One-shot "capture vr_init + arm" trigger for astral_arm_teleop when
# require_start_signal is true. Deliberately volatile (not latched) so a
# late-joining arm node does not auto-start from a stale start signal.
TOPIC_START = "/teleop/start"
# One-shot HOME / park-to-zero trigger (both arm nodes): 当前位姿 →
# init_pose → init_waypoints → 零位。同样 volatile——晚启动节点不得被
# 历史 HOME 信号误触发（与 /teleop/start 同理）。
TOPIC_HOME = "/teleop/home"

# --- Driver ROS services (call, robot hardware mode) -----------------------
# These services already exist on astral_robot_control's driver node. Calling
# them is non-intrusive (the driver owns the SDK/hardware; the monitor just
# invokes its already-exposed services). Override the node name via env if the
# driver is remapped.
DRIVER_NODE = os.environ.get("ASTRAL_WEB_MONITOR_DRIVER_NODE", "astral_robot_driver")
DRIVER_SRV_READY = f"/{DRIVER_NODE}/ready"
# 上电不回零（遥操启动/恢复归位路径用，避免与 teleop 轨迹抢目标）
DRIVER_SRV_ENABLE = f"/{DRIVER_NODE}/enable"
DRIVER_SRV_HOME = f"/{DRIVER_NODE}/home"
DRIVER_SRV_ESTOP = f"/{DRIVER_NODE}/estop"
DRIVER_SRV_DAMPING = f"/{DRIVER_NODE}/damping"
DRIVER_SRV_POSITION = f"/{DRIVER_NODE}/position"

# --- Driver-enable wait after launch (start/restart/home) -------------------
# 启动带 driver 的预设后，monitor 等待 ~/enable 服务可用并调用，确保电机
# 使能不静默失败（急停后 driver auto_ready 可能没确认上电）。总预算上限。
DRIVER_ENABLE_WAIT_S = float(os.environ.get("ASTRAL_WEB_MONITOR_ENABLE_WAIT_S", "12.0"))

# --- Video return (quest3_video_streamer) runtime gate ----------------------
# The streamer exposes a SetBool master switch and a latched String topic with
# the active camera subset (empty = all). It also publishes a latched JSON
# gate_state snapshot. The monitor only calls/publishes/subscribes these —
# the streamer owns the cameras. Override the node name via env if remapped.
VIDEO_NODE = os.environ.get("ASTRAL_WEB_MONITOR_VIDEO_NODE", "quest3_video_streamer")
VIDEO_SRV_PUSH = f"/{VIDEO_NODE}/set_push_enabled"
VIDEO_TOPIC_CAMERAS = f"/{VIDEO_NODE}/active_cameras"
VIDEO_TOPIC_GATE_STATE = f"/{VIDEO_NODE}/gate_state"
# Per-camera JPEG preview topics published by the streamer (CompressedImage).
VIDEO_TOPIC_PREVIEW = f"/{VIDEO_NODE}/preview"  # + "/{label}"

# --- astral_data_collect（VLA 数据采集）-------------------------------------
# 录制控制（String 命令：start/stop/discard/next/pause/resume）+ 任务文本
# （latched，应用于下一段）+ 状态镜像（latched JSON，含状态/段号/时长/流率）。
DC_TOPIC_CONTROL = "/data_collect/control"
DC_TOPIC_TASK = "/data_collect/task"
DC_TOPIC_STATE = "/data_collect/state"
# 允许的录制命令（web 端按钮白名单，防注入任意字符串）
DC_COMMANDS = ("start", "stop", "discard", "next", "pause", "resume")

# --- Orphan process detection ----------------------------------------------
# Regex fragment matched against the full command line of running processes
# to detect a leftover teleop launch before starting a new one.
BACKEND_MATCH = "ros2 launch"


def _presets_path() -> str:
    """Resolve presets.yaml from the installed share directory or source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("astral_web_monitor")
        return os.path.join(share, "config", "presets.yaml")
    except Exception:
        # Fallback: source tree (dev mode before colcon install)
        return os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "config", "presets.yaml",
        )


def video_params_path() -> str | None:
    """Resolve quest3_video_streamer/config/params.yaml (share, else source tree).

    Deprecated for the video card (the streamer's gate_state is now the
    authoritative camera list); kept for external tooling.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("quest3_video_streamer")
        return os.path.join(share, "config", "params.yaml")
    except Exception:
        pass
    # Source-tree fallback: walk up from this file looking for a sibling
    # quest3_video_streamer package (workspace src/ layout).
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(6):
        candidate = os.path.join(here, "quest3_video_streamer", "config", "params.yaml")
        if os.path.isfile(candidate):
            return candidate
        here = os.path.dirname(here)
    return None

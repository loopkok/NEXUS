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
WS_LOG_TAIL_LINES = int(os.environ.get("ASTRAL_WEB_MONITOR_LOG_TAIL", "500"))

# --- Stale detection --------------------------------------------------------
STALE_THRESHOLD_S = float(os.environ.get("ASTRAL_WEB_MONITOR_STALE_S", "2.0"))

# --- Launch subprocess ------------------------------------------------------
STARTUP_WARMUP_S = float(os.environ.get("ASTRAL_WEB_MONITOR_WARMUP_S", "2.0"))
STOP_SIGINT_TIMEOUT_S = float(os.environ.get("ASTRAL_WEB_MONITOR_STOP_TIMEOUT_S", "30.0"))
STOP_SIGKILL_GRACE_S = float(os.environ.get("ASTRAL_WEB_MONITOR_KILL_GRACE_S", "5.0"))

# --- Rate counter EWMA -----------------------------------------------------
RATE_EWMA_ALPHA = float(os.environ.get("ASTRAL_WEB_MONITOR_RATE_ALPHA", "0.7"))

# --- ROS topic names (read-only subscriptions) -----------------------------
# Grouped by entity. Each entry: (state_topic, command_topic).
# command_topic is "" when there is no command stream to monitor for that entity.
@dataclass(frozen=True)
class TopicPair:
    state: str
    command: str = ""


TOPICS: dict[str, TopicPair] = {
    "left_arm":      TopicPair("/left_arm/joint_states",      "/left_arm/joint_commands"),
    "right_arm":     TopicPair("/right_arm/joint_states",     "/right_arm/joint_commands"),
    "left_gripper":  TopicPair("/left_gripper/joint_states", "/left_gripper/joint_commands"),
    "right_hand":    TopicPair("/right_hand/joint_states",    "/right_hand/joint_commands"),
    "full_body":     TopicPair("/astral/joint_states",        ""),
}

# Hand namespace is configurable because wujihand uses {hand_name} not "hand".
HAND_NAME = os.environ.get("ASTRAL_WEB_MONITOR_HAND_NAME", "right_hand")
# Rebuild right_hand topics with the configured hand name.
_right_hand_state = f"/{HAND_NAME}/joint_states"
_right_hand_cmd = f"/{HAND_NAME}/joint_commands"
TOPICS["right_hand"] = TopicPair(_right_hand_state, _right_hand_cmd)

# --- Control topics (publish, pause/resume) ---------------------------------
# These already exist in astral_arm_teleop; publishing to them is the only
# write path the monitor uses. It never touches hardware topics.
TOPIC_ARMED = "/teleop/armed"
TOPIC_DISARM = "/teleop/disarm"

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

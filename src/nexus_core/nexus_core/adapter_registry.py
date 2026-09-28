"""Capabilities of adapters that can be composed into a NEXUS profile.

This registry describes the stable plugin boundary. Robot selection is a
composition of these adapters, not a separate ``robot`` switch in the core.
Hardware-specific ROS launch and protocol code stays behind each adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class Adapter:
    name: str
    kinds: frozenset[str]
    scope: str = "component"
    feedback: frozenset[str] = frozenset({"measured"})
    joint_names: Callable[[str, str], list[str] | None] | None = None
    lifecycle_namespace: str | None = None
    supports_home: bool = False


def _arm_joints(prefix: str, side: str) -> list[str]:
    return [f"{side}_{joint}" for joint in prefix.split(",")]


def _xhand_joints(_kind: str, side: str) -> list[str]:
    suffixes = ("thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
                "index_bend_joint", "index_joint1", "index_joint2",
                "mid_joint1", "mid_joint2", "ring_joint1", "ring_joint2",
                "pinky_joint1", "pinky_joint2")
    return [f"{side}_hand_{joint}" for joint in suffixes]


def _wuji_joints(_kind: str, side: str) -> list[str]:
    return [f"{side}_finger{finger}_joint{joint}"
            for finger in range(1, 6) for joint in range(1, 5)]


def _astral_joints(kind: str, side: str) -> list[str] | None:
    if kind == "arm":
        return _arm_joints("shoulder_pitch,shoulder_roll,elbow_roll,elbow_pitch,forearm_roll,wrist_pitch,wrist_roll", side)
    if kind == "gripper":
        return [f"{side}_gripper"]
    return None


def _nero_joints(kind: str, side: str) -> list[str] | None:
    return [f"{side}_joint{i}" for i in range(1, 8)] if kind == "arm" else None


DRIVERS = {
    "astral_sdk": Adapter("astral_sdk", frozenset({"arm", "gripper"}), "assembly",
                          frozenset({"measured", "command_echo"}), _astral_joints,
                          "/astral_robot_driver", True),
    "nero_can": Adapter("nero_can", frozenset({"arm"}), joint_names=_nero_joints,
                        supports_home=True),
    "xhand_serial": Adapter("xhand_serial", frozenset({"hand"}), joint_names=_xhand_joints),
    "wuji_serial": Adapter("wuji_serial", frozenset({"hand"}), joint_names=_wuji_joints),
}

IK_ADAPTERS = {
    "astral_geometric": frozenset({"arm"}),
    "nero_analytic": frozenset({"arm"}),
}

RETARGETERS = {
    "pinch_gripper": frozenset({"gripper"}),
    "wuji_hand": frozenset({"hand"}),
    "xhand_dexpilot": frozenset({"hand"}),
}

INPUT_SOURCES = frozenset({"quest3", "wuji_glove"})
CAMERA_SOURCES = frozenset({"quest3_video_streamer", "orbbec"})


def known_adapter(registry: dict, name: object, family: str):
    if not isinstance(name, str) or name not in registry:
        raise ValueError(f"unknown {family} adapter {name!r}")
    return registry[name]

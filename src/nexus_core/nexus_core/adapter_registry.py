"""Discoverable adapter contracts for NEXUS.

Hardware packages register adapters with Python entry points. Installing a new
ROS 2 package therefore extends these registries without editing nexus_core.
Entry-point values are factories returning one of the immutable specs below.
Launch callbacks are imported only when a profile is assembled, so profile
validation and dataset tools do not need the ROS launch runtime installed.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib import import_module, metadata
from typing import Any, Callable, Iterable


@dataclass(frozen=True)
class Adapter:
    name: str
    kinds: frozenset[str]
    scope: str = "component"
    feedback: frozenset[str] = frozenset({"measured"})
    joint_names: Callable[[str, str | None], list[str] | None] | None = None
    lifecycle_namespace: str | None = None
    supports_home: bool = False
    launcher: str | None = None
    preflight: str | None = None
    validate_config: str | Callable[[dict[str, Any]], None] | None = None
    home_target: Callable[[dict[str, Any], str, str | None, int], list[float] | None] | None = None
    home_tolerance: float = 0.05


@dataclass(frozen=True)
class IKAdapter:
    name: str
    kinds: frozenset[str]
    launcher: str
    validate_config: str | Callable[[dict[str, Any]], None] | None = None


@dataclass(frozen=True)
class RetargeterAdapter:
    name: str
    kinds: frozenset[str]
    scope: str = "component"
    launcher: str = ""
    validate_config: str | Callable[[dict[str, Any]], None] | None = None


@dataclass(frozen=True)
class InputAdapter:
    name: str
    semantics: frozenset[str]
    launcher: str
    preflight: str | None = None
    validate_config: str | Callable[[dict[str, Any]], None] | None = None


@dataclass(frozen=True)
class CameraAdapter:
    name: str
    launcher: str
    preflight: str | None = None
    validate_config: str | Callable[[dict[str, Any]], None] | None = None
    validate_camera: str | Callable[[dict[str, Any]], None] | None = None


def _arm_joints(prefix: str, side: str | None) -> list[str]:
    label = side or "arm"
    return [f"{label}_{joint}" for joint in prefix.split(",")]


def _xhand_joints(_kind: str, side: str | None) -> list[str]:
    suffixes = ("thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
                "index_bend_joint", "index_joint1", "index_joint2",
                "mid_joint1", "mid_joint2", "ring_joint1", "ring_joint2",
                "pinky_joint1", "pinky_joint2")
    return [f"{side or 'hand'}_hand_{joint}" for joint in suffixes]


def _wuji_joints(_kind: str, side: str | None) -> list[str]:
    return [f"{side or 'hand'}_finger{finger}_joint{joint}"
            for finger in range(1, 6) for joint in range(1, 5)]


def _astral_joints(kind: str, side: str | None) -> list[str] | None:
    if kind == "arm":
        return _arm_joints("shoulder_pitch,shoulder_roll,elbow_roll,elbow_pitch,forearm_roll,wrist_pitch,wrist_roll", side)
    if kind == "gripper":
        return [f"{side or 'gripper'}_gripper"]
    return None


def _nero_joints(kind: str, side: str | None) -> list[str] | None:
    label = side or "arm"
    return [f"{label}_joint{i}" for i in range(1, 8)] if kind == "arm" else None


def _validate_quest_camera(row: dict[str, Any]) -> None:
    if row.get("mode") not in ("v4l2", "webcam", "ros"):
        raise ValueError("mode must be v4l2, webcam, or ros")
    capture_topic = row.get("capture_topic")
    if not isinstance(capture_topic, str) or not capture_topic.startswith("/"):
        raise ValueError("capture_topic must be an absolute ROS topic")
    if row["mode"] == "ros" and not row.get("topic"):
        raise ValueError("ROS camera mode requires topic")


def _builtin_registries() -> dict[str, dict[str, Any]]:
    return {
        "driver": {
            "astral_sdk": Adapter("astral_sdk", frozenset({"arm", "gripper"}), "assembly",
                                  frozenset({"measured", "command_echo"}), _astral_joints,
                                  "/astral_robot_driver", True,
                                  "nexus_core.builtin_launchers:launch_astral_driver",
                                  "nexus_core.builtin_launchers:preflight_astral"),
            "nero_can": Adapter("nero_can", frozenset({"arm"}), joint_names=_nero_joints,
                                supports_home=True,
                                launcher="nexus_core.builtin_launchers:launch_nero_driver",
                                preflight="nexus_core.builtin_launchers:preflight_nero"),
            "xhand_serial": Adapter("xhand_serial", frozenset({"hand"}), joint_names=_xhand_joints,
                                    launcher="nexus_core.builtin_launchers:launch_xhand_driver",
                                    preflight="nexus_core.builtin_launchers:preflight_xhand"),
            "wuji_serial": Adapter("wuji_serial", frozenset({"hand"}), joint_names=_wuji_joints,
                                   launcher="nexus_core.builtin_launchers:launch_wuji_driver",
                                   preflight="nexus_core.builtin_launchers:preflight_wuji"),
        },
        "ik": {
            "astral_geometric": IKAdapter("astral_geometric", frozenset({"arm"}),
                "nexus_core.builtin_launchers:launch_astral_ik"),
            "nero_analytic": IKAdapter("nero_analytic", frozenset({"arm"}),
                "nexus_core.builtin_launchers:launch_nero_ik"),
        },
        "retargeter": {
            "pinch_gripper": RetargeterAdapter("pinch_gripper", frozenset({"gripper"}),
                launcher="nexus_core.builtin_launchers:launch_pinch"),
            "wuji_hand": RetargeterAdapter("wuji_hand", frozenset({"hand"}),
                launcher="nexus_core.builtin_launchers:launch_wuji_retargeter"),
            "xhand_dexpilot": RetargeterAdapter("xhand_dexpilot", frozenset({"hand"}),
                scope="assembly", launcher="nexus_core.builtin_launchers:launch_xhand_retargeter"),
        },
        "input": {
            "quest3": InputAdapter("quest3", frozenset({"wrist", "hand", "controller_joy",
                "body_joints", "body_joint_names"}),
                "nexus_core.builtin_launchers:launch_quest3_input"),
            "wuji_glove": InputAdapter("wuji_glove", frozenset({"hand"}),
                "nexus_core.builtin_launchers:launch_wuji_glove_input",
                "nexus_core.builtin_launchers:preflight_wuji_glove"),
        },
        "camera": {
            "quest3_video_streamer": CameraAdapter("quest3_video_streamer",
                "nexus_core.builtin_launchers:launch_quest_camera",
                validate_camera=_validate_quest_camera),
            "orbbec": CameraAdapter("orbbec", "nexus_core.builtin_launchers:launch_orbbec_camera",
                "nexus_core.builtin_launchers:preflight_orbbec"),
        },
    }


_ENTRY_POINT_GROUPS = {
    "driver": "nexus.driver_adapters",
    "ik": "nexus.ik_adapters",
    "retargeter": "nexus.retargeter_adapters",
    "input": "nexus.input_adapters",
    "camera": "nexus.camera_adapters",
}
_INPUT_SEMANTICS = frozenset({"wrist", "hand", "controller_joy",
                              "body_joints", "body_joint_names"})


def _entry_points() -> list[Any]:
    found = metadata.entry_points()
    if hasattr(found, "select"):
        return [ep for group in _ENTRY_POINT_GROUPS.values()
                for ep in found.select(group=group)]
    return [ep for group in _ENTRY_POINT_GROUPS.values() for ep in found.get(group, ())]


def discover_plugins(entry_points: Iterable[Any] | None = None) -> dict[str, dict[str, Any]]:
    """Return built-in and installed plugin specs; reject ambiguous registrations."""
    registries = _builtin_registries()
    group_to_kind = {group: kind for kind, group in _ENTRY_POINT_GROUPS.items()}
    expected_types = {"driver": Adapter, "ik": IKAdapter,
                      "retargeter": RetargeterAdapter,
                      "input": InputAdapter, "camera": CameraAdapter}
    for ep in _entry_points() if entry_points is None else entry_points:
        kind = group_to_kind.get(ep.group)
        if kind is None:
            continue
        if ep.name in registries[kind]:
            raise RuntimeError(f"duplicate NEXUS {kind} adapter entry point {ep.name!r}")
        factory = ep.load()
        spec = factory() if callable(factory) else factory
        if not isinstance(spec, expected_types[kind]):
            raise RuntimeError(f"NEXUS {kind} adapter {ep.name!r} returned "
                               f"{type(spec).__name__}, expected {expected_types[kind].__name__}")
        if getattr(spec, "name", None) != ep.name:
            raise RuntimeError(f"NEXUS {kind} adapter {ep.name!r} returned a mismatched name")
        if not getattr(spec, "launcher", None):
            raise RuntimeError(f"NEXUS {kind} adapter {ep.name!r} has no launcher")
        if kind in ("driver", "ik", "retargeter") and not spec.kinds:
            raise RuntimeError(f"NEXUS {kind} adapter {ep.name!r} declares no component kinds")
        if kind in ("driver", "retargeter") and spec.scope not in ("component", "assembly"):
            raise RuntimeError(f"NEXUS {kind} adapter {ep.name!r} has invalid scope {spec.scope!r}")
        if kind == "input" and (not spec.semantics or not spec.semantics <= _INPUT_SEMANTICS):
            raise RuntimeError(f"NEXUS input adapter {ep.name!r} declares invalid input semantics")
        registries[kind][ep.name] = spec
    all_names = [name for registry in registries.values() for name in registry]
    duplicates = sorted({name for name in all_names if all_names.count(name) > 1})
    if duplicates:
        raise RuntimeError(f"NEXUS adapter names must be globally unique: {duplicates}")
    return registries


REGISTRIES = discover_plugins()
DRIVERS: dict[str, Adapter] = REGISTRIES["driver"]
IK_ADAPTERS: dict[str, IKAdapter] = REGISTRIES["ik"]
RETARGETERS: dict[str, RetargeterAdapter] = REGISTRIES["retargeter"]
INPUT_ADAPTERS: dict[str, InputAdapter] = REGISTRIES["input"]
CAMERA_ADAPTERS: dict[str, CameraAdapter] = REGISTRIES["camera"]
INPUT_SOURCES = frozenset(INPUT_ADAPTERS)
CAMERA_SOURCES = frozenset(CAMERA_ADAPTERS)


def load_target(target: str | Callable[..., Any]) -> Callable[..., Any]:
    """Resolve a ``module:callable`` adapter hook lazily at ROS launch time."""
    if callable(target):
        return target
    if not isinstance(target, str) or ":" not in target:
        raise ValueError(f"invalid NEXUS adapter target {target!r}; expected module:callable")
    module_name, attribute = target.split(":", 1)
    result = getattr(import_module(module_name), attribute)
    if not callable(result):
        raise TypeError(f"NEXUS adapter target {target!r} is not callable")
    return result


def known_adapter(registry: dict, name: object, family: str):
    if not isinstance(name, str) or name not in registry:
        raise ValueError(f"unknown {family} adapter {name!r}")
    return registry[name]

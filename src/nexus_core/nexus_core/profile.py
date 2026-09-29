"""Versioned, robot-independent NEXUS assembly profiles.

Profiles compose registered input, camera, IK, retargeting and driver adapters.
The ordered component list is the source of truth for ROS joint names and the
state/action layout frozen into every dataset and model manifest.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .adapter_registry import (
    CAMERA_ADAPTERS,
    DRIVERS,
    IK_ADAPTERS,
    INPUT_ADAPTERS,
    RETARGETERS,
    load_target,
)

_IDENT = re.compile(r"^[a-z][a-z0-9_]*$")
_FEEDBACK = {"measured", "command_echo"}


class ProfileError(ValueError):
    pass


@dataclass(frozen=True)
class Component:
    name: str
    kind: str
    joints: tuple[str, ...]
    lower: tuple[float, ...]
    upper: tuple[float, ...]
    driver: str
    feedback: str
    ik: str | None
    side: str | None = None
    input_channel: str | None = None
    retargeter: str | None = None
    unit: str = "rad"
    teleop: dict[str, Any] | None = None
    frame_transforms: dict[str, tuple[float, ...]] | None = None

    @property
    def dim(self) -> int:
        return len(self.joints)


class Profile:
    def __init__(self, raw: dict[str, Any], path: str = "", *, validate_plugins: bool = True):
        self.raw = raw
        self.path = path
        self.validate_plugins = validate_plugins
        self._components: tuple[Component, ...] = ()
        self._validate()

    @classmethod
    def load(cls, path: str | Path, *, validate_plugins: bool = True) -> "Profile":
        p = Path(path).expanduser().resolve()
        with p.open(encoding="utf-8") as fh:
            raw = json.load(fh)
        return cls(raw, str(p), validate_plugins=validate_plugins)

    def _validate_teleop(self, name: str, row: Any) -> dict[str, Any]:
        if not isinstance(row, dict):
            raise ProfileError(f"{name}: teleop settings must be an object")
        if not row.get("arm_base_frame"):
            raise ProfileError(f"{name}: teleop arm_base_frame is required")
        rot = row.get("vr_to_arm_rot")
        # Optional legacy field: only adapters that truly command a tool TCP
        # should set this. Physical model-frame geometry belongs in frames.
        offset = row.get("tcp_offset", [0.0] * 6)
        if not isinstance(rot, list) or len(rot) != 9 or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in rot):
            raise ProfileError(f"{name}: teleop vr_to_arm_rot must have 9 finite values")
        axes = [rot[i:i + 3] for i in (0, 3, 6)]
        dot = lambda a, b: sum(x * y for x, y in zip(a, b))
        cross = (axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1],
                 axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2],
                 axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0])
        if (any(abs(dot(axis, axis) - 1.0) > 1e-3 for axis in axes)
                or any(abs(dot(axes[i], axes[j])) > 1e-3
                       for i in range(3) for j in range(i + 1, 3))
                or abs(dot(cross, axes[2])) < 0.999):
            raise ProfileError(f"{name}: teleop vr_to_arm_rot is not orthogonal")
        if not isinstance(offset, list) or len(offset) != 6 or not all(
                isinstance(v, (int, float)) and math.isfinite(v) for v in offset):
            raise ProfileError(f"{name}: teleop tcp_offset must have 6 finite values")
        if not isinstance(row.get("motion_scale"), (int, float)) or not 0 < row["motion_scale"] <= 2:
            raise ProfileError(f"{name}: teleop motion_scale must be in (0,2]")
        return row

    def _validate(self) -> None:
        d = self.raw
        if d.get("schema_version") not in (1, 2):
            raise ProfileError("profile schema_version must be 1 or 2")
        if d.get("schema_version") == 2 and "robot" in d:
            raise ProfileError("schema v2 composes adapters and must not declare a robot selector")
        for field in ("profile_id", "instance"):
            if not _IDENT.fullmatch(str(d.get(field, ""))):
                raise ProfileError(f"invalid {field}: {d.get(field)!r}")

        inputs = d.get("inputs")
        if not isinstance(inputs, dict) or not inputs:
            raise ProfileError("inputs must be a nonempty object")
        for channel, row in inputs.items():
            if not _IDENT.fullmatch(str(channel)) or not isinstance(row, dict):
                raise ProfileError(f"invalid input channel {channel!r}")
            for semantic, spec in row.items():
                if semantic not in ("wrist", "hand", "controller_joy", "body_joints", "body_joint_names"):
                    raise ProfileError(f"{channel}: unsupported input semantic {semantic}")
                if d.get("schema_version") == 2 and not isinstance(spec, dict):
                    raise ProfileError(f"{channel}.{semantic}: schema v2 inputs must declare source and topic")
                source = spec if isinstance(spec, str) else spec.get("source") if isinstance(spec, dict) else None
                if (source != "none" and self.validate_plugins
                        and (not isinstance(source, str) or source not in INPUT_ADAPTERS)):
                    raise ProfileError(f"{channel}.{semantic}: unsupported input source {source!r}")
                input_adapter = INPUT_ADAPTERS.get(source) if isinstance(source, str) else None
                if input_adapter and semantic not in input_adapter.semantics:
                    raise ProfileError(f"{channel}.{semantic}: input adapter {source} does not provide this semantic")
                if isinstance(spec, dict):
                    topic = spec.get("topic")
                    if not isinstance(topic, str) or not topic.startswith("/"):
                        raise ProfileError(f"{channel}.{semantic}: topic must be an absolute ROS topic")
                    if semantic in ("wrist", "hand", "controller_joy", "body_joints") and spec.get("frame_policy") != "stable":
                        raise ProfileError(f"{channel}.{semantic}: frame_policy must be stable")
        settings = d.get("input_settings", {})
        if not isinstance(settings, dict):
            raise ProfileError("input_settings must be an object")
        if settings.get("landmark_preprocess", "raw") not in ("raw", "mano"):
            raise ProfileError("input_settings.landmark_preprocess must be raw or mano")
        wrist_mapping = settings.get("quest3_wrist_pose_mapping")
        if wrist_mapping is not None:
            if not isinstance(wrist_mapping, dict) or wrist_mapping.get("mode") != "per_side":
                raise ProfileError(
                    "input_settings.quest3_wrist_pose_mapping.mode must be per_side"
                )
            for side in ("left", "right"):
                rotation = wrist_mapping.get(f"{side}_rotation")
                frame_id = wrist_mapping.get(f"{side}_frame_id")
                if (not isinstance(rotation, list) or len(rotation) != 9
                        or not all(isinstance(v, (int, float)) and math.isfinite(v)
                                   for v in rotation)):
                    raise ProfileError(
                        f"quest3 wrist {side} rotation must have 9 finite values"
                    )
                matrix = [rotation[i:i + 3] for i in (0, 3, 6)]
                row_dot = lambda a, b: sum(x * y for x, y in zip(a, b))
                cross = (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1],
                         matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2],
                         matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0])
                if (any(abs(row_dot(row, row) - 1.0) > 1e-3 for row in matrix)
                        or any(abs(row_dot(matrix[i], matrix[j])) > 1e-3
                               for i in range(3) for j in range(i + 1, 3))
                        or abs(abs(row_dot(cross, matrix[2])) - 1.0) > 1e-3):
                    raise ProfileError(f"quest3 wrist {side} rotation must be orthogonal")
                if not isinstance(frame_id, str) or not frame_id.strip():
                    raise ProfileError(f"quest3 wrist {side} frame_id must be nonempty")

        rows = d.get("components")
        if not isinstance(rows, list) or not rows:
            raise ProfileError("components must be a nonempty ordered list")
        components: list[Component] = []
        seen_names: set[str] = set()
        seen_joints: set[str] = set()
        legacy_teleop = d.get("teleop", {})
        for row in rows:
            if not isinstance(row, dict):
                raise ProfileError("each component must be an object")
            name = str(row.get("name", ""))
            kind = str(row.get("kind", ""))
            joints = row.get("joints")
            if not _IDENT.fullmatch(name) or name in seen_names:
                raise ProfileError(f"duplicate or invalid component name {name!r}")
            if not _IDENT.fullmatch(kind):
                raise ProfileError(f"invalid component kind for {name}")
            if not isinstance(joints, list) or not joints or any(
                    not isinstance(j, str) or not j for j in joints):
                raise ProfileError(f"{name}: joints must be a nonempty string list")
            if len(set(joints)) != len(joints) or seen_joints.intersection(joints):
                raise ProfileError(f"{name}: joint names must be unique across the assembly")
            lower, upper = row.get("lower"), row.get("upper")
            if (not isinstance(lower, list) or not isinstance(upper, list)
                    or len(lower) != len(joints) or len(upper) != len(joints)):
                raise ProfileError(f"{name}: joint limit dimensions differ")
            try:
                lo = tuple(float(v) for v in lower)
                hi = tuple(float(v) for v in upper)
            except (ValueError, TypeError) as exc:
                raise ProfileError(f"{name}: invalid joint limits") from exc
            if any(not math.isfinite(a) or not math.isfinite(b) or a >= b
                   for a, b in zip(lo, hi)):
                raise ProfileError(f"{name}: invalid joint limits")
            frame_transforms_raw = row.get("frame_transforms", {})
            if not isinstance(frame_transforms_raw, dict):
                raise ProfileError(f"{name}: frame_transforms must be an object")
            frame_transforms: dict[str, tuple[float, ...]] = {}
            for transform_name, pose in frame_transforms_raw.items():
                if (not isinstance(transform_name, str) or not transform_name
                        or not isinstance(pose, list) or len(pose) != 6):
                    raise ProfileError(f"{name}: frame transform must be a named six-value pose")
                try:
                    values = tuple(float(v) for v in pose)
                except (ValueError, TypeError) as exc:
                    raise ProfileError(f"{name}: invalid frame transform {transform_name}") from exc
                if not all(math.isfinite(v) for v in values):
                    raise ProfileError(f"{name}: non-finite frame transform {transform_name}")
                frame_transforms[transform_name] = values

            driver = str(row.get("driver", ""))
            driver_adapter = DRIVERS.get(driver)
            if driver_adapter is None and self.validate_plugins:
                raise ProfileError(f"{name}: unknown driver adapter {driver!r}")
            if driver_adapter and kind not in driver_adapter.kinds:
                raise ProfileError(f"{name}: driver {driver} cannot drive component kind {kind}")
            feedback = str(row.get("feedback", ""))
            if (feedback not in _FEEDBACK
                    or (driver_adapter and feedback not in driver_adapter.feedback)):
                raise ProfileError(f"{name}: feedback {feedback!r} is unsupported by {driver}")
            unit = str(row.get("unit", "rad"))
            if unit not in ("rad", "m"):
                raise ProfileError(f"{name}: unit must be rad or m")

            side = row.get("side")
            if side is None:
                legacy_side = name.split("_", 1)[0]
                side = legacy_side if legacy_side in ("left", "right") else None
            if side is not None and (not isinstance(side, str) or not _IDENT.fullmatch(side)):
                raise ProfileError(f"{name}: invalid side {side!r}")
            expected_joints = (driver_adapter.joint_names(kind, side)
                               if driver_adapter and driver_adapter.joint_names else None)
            if expected_joints is not None and joints != expected_joints:
                raise ProfileError(f"{name}: joint names/order do not match {driver} adapter")
            input_channel = row.get("input_channel", side)
            if input_channel is not None and input_channel not in inputs:
                raise ProfileError(f"{name}: unknown input channel {input_channel!r}")

            ik = row.get("ik")
            if kind == "arm":
                if (not isinstance(ik, str) or not ik
                        or (self.validate_plugins and ik not in IK_ADAPTERS)
                        or (ik in IK_ADAPTERS and kind not in IK_ADAPTERS[ik].kinds)):
                    raise ProfileError(f"{name}: unknown or incompatible IK adapter {ik!r}")
            elif ik is not None:
                raise ProfileError(f"{name}: IK adapter is only valid for an arm")

            retargeter = row.get("retargeter")
            if retargeter is not None:
                if (not isinstance(retargeter, str) or not retargeter
                        or (self.validate_plugins and retargeter not in RETARGETERS)
                        or (retargeter in RETARGETERS and kind not in RETARGETERS[retargeter].kinds)):
                    raise ProfileError(f"{name}: unknown or incompatible retargeter {retargeter!r}")
            elif kind != "arm" and d.get("schema_version") == 2:
                raise ProfileError(f"{name}: schema v2 non-arm component requires a retargeter adapter")
            teleop = row.get("teleop")
            if teleop is None and side is not None:
                teleop = legacy_teleop.get(side)
            if kind == "arm" and d.get("schema_version") == 2 and teleop is None:
                raise ProfileError(f"{name}: schema v2 arm requires component teleop settings")
            if teleop is not None:
                teleop = self._validate_teleop(name, teleop)
            if kind == "arm" and input_channel is None:
                raise ProfileError(f"{name}: arm requires input_channel")

            components.append(Component(
                name, kind, tuple(joints), lo, hi, driver, feedback, ik,
                side, input_channel, retargeter, unit, teleop, frame_transforms))
            seen_names.add(name)
            seen_joints.update(joints)

        cameras = d.get("cameras")
        if not isinstance(cameras, list) or not cameras:
            raise ProfileError("at least one camera is required")
        roles: set[str] = set()
        devices: set[str] = set()
        for camera in cameras:
            if not isinstance(camera, dict):
                raise ProfileError("each camera must be an object")
            role = str(camera.get("role", ""))
            device = str(camera.get("device", ""))
            source = camera.get("source")
            if not _IDENT.fullmatch(role) or role in roles or not device or device in devices:
                raise ProfileError(f"duplicate or invalid camera role/device: {role}/{device}")
            if self.validate_plugins and (not isinstance(source, str) or source not in CAMERA_ADAPTERS):
                raise ProfileError(f"{role}: unknown camera source {source!r}")
            camera_adapter = CAMERA_ADAPTERS.get(source) if isinstance(source, str) else None
            validator = camera_adapter.validate_camera if camera_adapter else None
            if validator and d.get("schema_version") == 2:
                try:
                    load_target(validator)(camera)
                except Exception as exc:
                    raise ProfileError(f"{role}: {exc}") from exc
            if "topic" in camera and (not isinstance(camera["topic"], str)
                                       or not camera["topic"].startswith("/")):
                raise ProfileError(f"{role}: source topic must be absolute")
            if "capture_topic" in camera and (not isinstance(camera["capture_topic"], str)
                                               or not camera["capture_topic"].startswith("/")):
                raise ProfileError(f"{role}: capture_topic must be an absolute ROS topic")
            roles.add(role)
            devices.add(device)

        adapter_config = d.get("adapter_config", {})
        if not isinstance(adapter_config, dict):
            raise ProfileError("adapter_config must be an object")
        config_registries = (DRIVERS, IK_ADAPTERS, RETARGETERS,
                             INPUT_ADAPTERS, CAMERA_ADAPTERS)
        config_adapters = {name: adapter for registry in config_registries
                           for name, adapter in registry.items()}
        unknown_config = set(adapter_config) - set(config_adapters) if self.validate_plugins else set()
        if unknown_config:
            raise ProfileError(f"unknown adapter_config entries: {sorted(unknown_config)}")
        for name, config in adapter_config.items():
            if not isinstance(config, dict):
                raise ProfileError(f"adapter_config.{name} must be an object")
            adapter = config_adapters.get(name)
            validator = adapter.validate_config if adapter else None
            if validator:
                try:
                    load_target(validator)(config)
                except Exception as exc:
                    raise ProfileError(f"adapter_config.{name}: {exc}") from exc

        dataset = d.get("dataset", {})
        if not isinstance(dataset, dict) or dataset.get("action_source") not in ("next_state", "command"):
            raise ProfileError("dataset.action_source must be next_state or command")
        if not isinstance(dataset.get("fps"), int) or dataset["fps"] < 1:
            raise ProfileError("dataset.fps must be a positive integer")
        policy = d.get("policy", {})
        camera_map = policy.get("camera_map", {})
        if not isinstance(camera_map, dict) or not camera_map or any(
                role not in roles for role in camera_map.values()):
            raise ProfileError("policy.camera_map must reference configured camera roles")
        self._components = tuple(components)

    @property
    def profile_id(self) -> str:
        return self.raw["profile_id"]

    @property
    def instance(self) -> str:
        return self.raw["instance"]

    @property
    def namespace(self) -> str:
        return f"/nexus/{self.instance}"

    @property
    def digest(self) -> str:
        payload = json.dumps(self.raw, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
        return hashlib.sha256(payload.encode("utf-8")).hexdigest()

    @property
    def components(self) -> tuple[Component, ...]:
        return self._components

    @property
    def dimension(self) -> int:
        return sum(c.dim for c in self.components)

    def component(self, name: str) -> Component:
        for component in self.components:
            if component.name == name:
                return component
        raise ProfileError(f"unknown component {name}")

    def teleop_config(self, component: str) -> dict[str, Any]:
        spec = self.component(component)
        if spec.teleop is not None:
            return spec.teleop
        raise ProfileError(f"{component}: no teleop mapping configured")

    def adapter_config(self, adapter: str) -> dict[str, Any]:
        """Return this adapter's config, with a read-only schema-v1 fallback."""
        configured = self.raw.get("adapter_config", {}).get(adapter)
        if configured is not None:
            return configured
        hardware = self.raw.get("hardware", {})
        if adapter == "astral_sdk":
            return hardware
        if adapter == "nero_can":
            return {"channels": hardware.get("can", {}),
                    "home_pose": hardware.get("home_pose", {})}
        if adapter == "xhand_serial":
            return {"serials": hardware.get("xhand_serial", {})}
        if adapter == "wuji_serial":
            return {"serials": hardware.get("wuji_serial", {})}
        if adapter == "wuji_glove":
            return {"serials": hardware.get("wuji_glove", {})}
        return {}

    def input_spec(self, channel: str, semantic: str) -> dict[str, Any] | None:
        raw = self.raw["inputs"].get(channel, {}).get(semantic)
        if raw is None:
            return None
        if isinstance(raw, str):
            side = channel
            defaults = {
                "wrist": f"/quest3/{side}_wrist_pose",
                "hand": f"/hand_landmarks/{side}",
                "controller_joy": f"/quest3/{side}_controller_joy",
                "body_joints": "/quest3/body_joints",
                "body_joint_names": "/quest3/body_joint_names",
            }
            return {"source": raw, "topic": defaults[semantic],
                    "frame_policy": "stable" if semantic in (
                        "wrist", "hand", "controller_joy", "body_joints") else "none"}
        return raw

    def topic(self, component: str, leaf: str) -> str:
        if component not in {c.name for c in self.components}:
            raise ProfileError(f"unknown component {component}")
        if leaf not in ("joint_states", "joint_commands"):
            raise ProfileError(f"unknown component topic {leaf}")
        return f"{self.namespace}/components/{component}/{leaf}"

    def candidate_topic(self, source: str, component: str) -> str:
        if source not in ("teleop", "policy", "playback"):
            raise ProfileError(f"unknown command source {source}")
        self.component(component)
        return f"{self.namespace}/candidates/{source}/{component}/joint_commands"

    def frozen_schema(self) -> dict[str, Any]:
        return {
            "schema_version": 2,
            "profile_id": self.profile_id,
            "profile_sha256": self.digest,
            "components": [{"name": c.name, "kind": c.kind, "joints": list(c.joints),
                            "dim": c.dim, "feedback": c.feedback, "unit": c.unit}
                           for c in self.components],
            "cameras": [c["role"] for c in self.raw["cameras"]],
            "camera_map": dict(self.raw["policy"]["camera_map"]),
            "fps": self.raw["dataset"]["fps"],
            "action_source": self.raw["dataset"]["action_source"],
            "state_dim": self.dimension,
            "action_dim": self.dimension,
        }


def verify_model_manifest(profile: Profile, manifest: dict[str, Any]) -> None:
    expected = profile.frozen_schema()
    for key in ("profile_sha256", "components", "camera_map", "fps", "action_source",
                "state_dim", "action_dim"):
        if manifest.get(key) != expected[key]:
            raise ProfileError(f"model manifest mismatch: {key}")

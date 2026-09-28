"""Versioned, hardware-independent NEXUS assembly profile.

The canonical JSON encoding is also the immutable identity stored with every
episode and model. No ROS import is required, so profiles can be checked on a
training host before a robot is connected.
"""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

_IDENT = re.compile(r"^[a-z][a-z0-9_]*$")
_KINDS = {"arm", "hand", "gripper", "head", "waist"}
_FEEDBACK = {"measured", "command_echo"}
_INPUTS = {"quest3", "wuji_glove", "none"}


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

    @property
    def dim(self) -> int:
        return len(self.joints)


class Profile:
    def __init__(self, raw: dict[str, Any], path: str = ""):
        self.raw = raw
        self.path = path
        self._validate()

    @classmethod
    def load(cls, path: str | Path) -> "Profile":
        p = Path(path).expanduser().resolve()
        with p.open(encoding="utf-8") as fh:
            raw = json.load(fh)
        return cls(raw, str(p))

    def _validate(self) -> None:
        d = self.raw
        if d.get("schema_version") != 1:
            raise ProfileError("profile schema_version must be 1")
        for field in ("profile_id", "instance"):
            if not _IDENT.fullmatch(str(d.get(field, ""))):
                raise ProfileError(f"invalid {field}: {d.get(field)!r}")
        inputs = d.get("inputs")
        if not isinstance(inputs, dict):
            raise ProfileError("inputs must be an object")
        for side in ("left", "right"):
            row = inputs.get(side, {})
            if row.get("wrist", "none") not in _INPUTS or row.get("hand", "none") not in _INPUTS:
                raise ProfileError(f"unsupported input source for {side}")
            if row.get("wrist") != "quest3" or row.get("hand") not in ("quest3", "wuji_glove"):
                raise ProfileError(f"{side}: built-in teleop requires Quest 3 wrist and a hand landmark source")
        if d.get("robot") not in ("astral", "nero"):
            raise ProfileError("robot must be astral or nero for built-in launch")
        input_settings = d.get("input_settings", {})
        if input_settings.get("landmark_preprocess") not in ("raw", "mano"):
            raise ProfileError("input_settings.landmark_preprocess must be raw or mano")
        rows = d.get("components")
        if not isinstance(rows, list) or not rows:
            raise ProfileError("components must be a nonempty ordered list")
        components: list[Component] = []
        seen_names: set[str] = set()
        seen_joints: set[str] = set()
        for row in rows:
            name = str(row.get("name", ""))
            kind = str(row.get("kind", ""))
            joints = row.get("joints")
            if not _IDENT.fullmatch(name) or name in seen_names:
                raise ProfileError(f"duplicate or invalid component name {name!r}")
            if kind not in _KINDS:
                raise ProfileError(f"invalid component kind for {name}")
            if not isinstance(joints, list) or not joints or any(not isinstance(j, str) or not j for j in joints):
                raise ProfileError(f"{name}: joints must be a nonempty string list")
            if len(set(joints)) != len(joints) or seen_joints.intersection(joints):
                raise ProfileError(f"{name}: joint names must be unique across the assembly")
            lower, upper = row.get("lower"), row.get("upper")
            if not isinstance(lower, list) or not isinstance(upper, list) or len(lower) != len(joints) or len(upper) != len(joints):
                raise ProfileError(f"{name}: joint limit dimensions differ")
            try:
                lo = tuple(float(v) for v in lower)
                hi = tuple(float(v) for v in upper)
            except (ValueError, TypeError) as exc:
                raise ProfileError(f"{name}: invalid joint limits") from exc
            if any(a >= b or not -1e6 < a < 1e6 or not -1e6 < b < 1e6 for a, b in zip(lo, hi)):
                raise ProfileError(f"{name}: invalid joint limits")
            driver = str(row.get("driver", ""))
            if not _IDENT.fullmatch(driver):
                raise ProfileError(f"{name}: missing driver adapter")
            feedback = str(row.get("feedback", ""))
            if feedback not in _FEEDBACK:
                raise ProfileError(f"{name}: feedback must be measured or command_echo")
            allowed_driver = {"astral": {"astral_sdk", "wuji_serial"},
                              "nero": {"nero_can", "xhand_serial"}}[d["robot"]]
            if driver not in allowed_driver:
                raise ProfileError(f"{name}: driver {driver} incompatible with {d['robot']}")
            if (driver == "nero_can" and kind != "arm") or (
                    driver == "xhand_serial" and kind != "hand") or (
                    driver == "wuji_serial" and kind != "hand") or (
                    driver == "astral_sdk" and kind not in ("arm", "gripper")):
                raise ProfileError(f"{name}: component kind incompatible with driver {driver}")
            side = name.split("_", 1)[0]
            if side not in ("left", "right") or name not in (f"{side}_arm", f"{side}_ee"):
                raise ProfileError(f"{name}: built-in assembly requires left/right arm or ee")
            expected = None
            if driver == "nero_can":
                expected = [f"{side}_joint{i}" for i in range(1, 8)]
            elif driver == "xhand_serial":
                expected = [f"{side}_hand_{joint}" for joint in (
                    "thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
                    "index_bend_joint", "index_joint1", "index_joint2",
                    "mid_joint1", "mid_joint2", "ring_joint1", "ring_joint2",
                    "pinky_joint1", "pinky_joint2")]
            elif driver == "wuji_serial":
                expected = [f"{side}_finger{f}_joint{j}"
                            for f in range(1, 6) for j in range(1, 5)]
            elif driver == "astral_sdk" and kind == "arm":
                expected = [f"{side}_{joint}" for joint in (
                    "shoulder_pitch", "shoulder_roll", "elbow_roll", "elbow_pitch",
                    "forearm_roll", "wrist_pitch", "wrist_roll")]
            elif driver == "astral_sdk" and kind == "gripper":
                expected = [f"{side}_gripper"]
            if expected is None or joints != expected:
                raise ProfileError(f"{name}: joint names/order do not match {driver} adapter")
            ik = row.get("ik")
            if kind == "arm" and (not isinstance(ik, str) or not _IDENT.fullmatch(ik)):
                raise ProfileError(f"{name}: arm requires an IK adapter")
            if kind == "arm" and ik != {"astral": "astral_geometric", "nero": "nero_analytic"}[d["robot"]]:
                raise ProfileError(f"{name}: IK adapter incompatible with robot")
            components.append(Component(name, kind, tuple(joints), lo, hi, driver, feedback, ik))
            seen_names.add(name)
            seen_joints.update(joints)
        if not any(c.kind == "arm" for c in components):
            raise ProfileError("at least one arm is required")
        if {c.name for c in components} != {"left_arm", "right_arm", "left_ee", "right_ee"}:
            raise ProfileError("built-in assembly requires left/right arm and ee components")
        if d["robot"] == "nero" and any(inputs[side]["hand"] != "quest3" for side in ("left", "right")):
            raise ProfileError("Nero XHand built-in retargeting requires Quest 3 hand landmarks")
        cameras = d.get("cameras")
        if not isinstance(cameras, list) or not cameras:
            raise ProfileError("at least one camera is required")
        roles: set[str] = set()
        devices: set[str] = set()
        for camera in cameras:
            role = str(camera.get("role", ""))
            device = str(camera.get("device", ""))
            if not _IDENT.fullmatch(role) or role in roles or not device or device in devices:
                raise ProfileError(f"duplicate or invalid camera role/device: {role}/{device}")
            roles.add(role)
            devices.add(device)
            if camera.get("source") not in ("quest3_video_streamer", "orbbec"):
                raise ProfileError(f"{role}: unsupported camera source")
            expected_source = "quest3_video_streamer" if d["robot"] == "astral" else "orbbec"
            if camera["source"] != expected_source:
                raise ProfileError(f"{role}: camera source incompatible with {d['robot']} launch")
        hardware = d.get("hardware")
        if not isinstance(hardware, dict):
            raise ProfileError("hardware must be an object")
        if d["robot"] == "astral" and not hardware.get("control_board_ip"):
            raise ProfileError("astral control_board_ip is required")
        if d["robot"] == "nero":
            for key in ("can", "xhand_serial"):
                pair = hardware.get(key)
                if not isinstance(pair, dict) or any(not pair.get(side) for side in ("left", "right")):
                    raise ProfileError(f"nero hardware.{key} requires left/right identifiers")
                if pair["left"] == pair["right"]:
                    raise ProfileError(f"nero hardware.{key} duplicates device identifier")
            home = hardware.get("home_pose", {})
            for side in ("left", "right"):
                target = home.get(side)
                if not isinstance(target, list) or len(target) != 7 or any(
                        not isinstance(v, (int, float)) or not math.isfinite(v) for v in target):
                    raise ProfileError(f"nero hardware.home_pose.{side} must have 7 finite angles")
        teleop = d.get("teleop")
        if not isinstance(teleop, dict):
            raise ProfileError("teleop must be an object")
        for side in ("left", "right"):
            row = teleop.get(side)
            if not isinstance(row, dict) or not row.get("arm_base_frame"):
                raise ProfileError(f"teleop.{side}.arm_base_frame is required")
            rot = row.get("vr_to_arm_rot")
            offset = row.get("tcp_offset")
            if not isinstance(rot, list) or len(rot) != 9 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in rot):
                raise ProfileError(f"teleop.{side}.vr_to_arm_rot must be 9 finite values")
            axes = [rot[i:i + 3] for i in (0, 3, 6)]
            dot = lambda a, b: sum(x * y for x, y in zip(a, b))
            cross = (axes[0][1] * axes[1][2] - axes[0][2] * axes[1][1],
                     axes[0][2] * axes[1][0] - axes[0][0] * axes[1][2],
                     axes[0][0] * axes[1][1] - axes[0][1] * axes[1][0])
            if any(abs(dot(axis, axis) - 1.0) > 1e-3 for axis in axes) or any(
                    abs(dot(axes[i], axes[j])) > 1e-3 for i in range(3) for j in range(i + 1, 3)) or abs(dot(cross, axes[2])) < 0.999:
                # The existing Nero VR convention deliberately includes a
                # reflection; its processor conjugates rotations by this
                # orthogonal basis matrix rather than using it as an SO(3) pose.
                raise ProfileError(f"teleop.{side}.vr_to_arm_rot is not orthogonal")
            if not isinstance(offset, list) or len(offset) != 6 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in offset):
                raise ProfileError(f"teleop.{side}.tcp_offset must be 6 finite values")
            if not isinstance(row.get("motion_scale"), (int, float)) or not 0 < row["motion_scale"] <= 2:
                raise ProfileError(f"teleop.{side}.motion_scale must be in (0,2]")
        dataset = d.get("dataset", {})
        if dataset.get("action_source") not in ("next_state", "command"):
            raise ProfileError("dataset.action_source must be next_state or command")
        if not isinstance(dataset.get("fps"), int) or dataset["fps"] < 1:
            raise ProfileError("dataset.fps must be a positive integer")
        policy = d.get("policy", {})
        camera_map = policy.get("camera_map", {})
        if not isinstance(camera_map, dict) or not camera_map or any(role not in roles for role in camera_map.values()):
            raise ProfileError("policy.camera_map must reference configured camera roles")
        self.components = tuple(components)

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
    def dimension(self) -> int:
        return sum(c.dim for c in self.components)

    def component(self, name: str) -> Component:
        for component in self.components:
            if component.name == name:
                return component
        raise ProfileError(f"unknown component {name}")

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
                            "dim": c.dim, "feedback": c.feedback} for c in self.components],
            "cameras": [c["role"] for c in self.raw["cameras"]],
            "camera_map": dict(self.raw["policy"]["camera_map"]),
            "fps": self.raw["dataset"]["fps"],
            "action_source": self.raw["dataset"]["action_source"],
            "state_dim": self.dimension,
            "action_dim": self.dimension,
        }


def verify_model_manifest(profile: Profile, manifest: dict[str, Any]) -> None:
    expected = profile.frozen_schema()
    for key in ("profile_sha256", "components", "camera_map", "fps", "action_source", "state_dim", "action_dim"):
        if manifest.get(key) != expected[key]:
            raise ProfileError(f"model manifest mismatch: {key}")

#!/usr/bin/env python3
"""Regenerate built-in profiles from the checked-in URDF and driver contracts."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "src/nexus_core/profiles"


def urdf_limits(path, names):
    model = ET.parse(ROOT / path).getroot()
    joints = {j.get("name"): j for j in model.findall("joint")}
    pairs = []
    for name in names:
        joint = joints[name]
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"missing limit for {name}")
        pairs.append((float(limit.get("lower")), float(limit.get("upper"))))
    return [p[0] for p in pairs], [p[1] for p in pairs]


def component(name, kind, joints, lower, upper, driver, feedback, ik=None):
    row = dict(name=name, kind=kind, joints=joints, lower=lower,
               upper=upper, driver=driver, feedback=feedback)
    if ik is not None:
        row["ik"] = ik
    return row


def write(name, profile):
    DEST.mkdir(parents=True, exist_ok=True)
    (DEST / name).write_text(json.dumps(profile, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def astral():
    model = "src/astral_robot_description/urdf/astral_robot.pin.urdf"
    left_native = [f"left_joint{i}" for i in range(1, 8)]
    right_native = [f"right_joint{i}" for i in range(1, 8)]
    left_names = ["left_shoulder_pitch", "left_shoulder_roll", "left_elbow_roll",
                  "left_elbow_pitch", "left_forearm_roll", "left_wrist_pitch", "left_wrist_roll"]
    right_names = [n.replace("left_", "right_") for n in left_names]
    lo_l, hi_l = urdf_limits(model, left_native)
    lo_r, hi_r = urdf_limits(model, right_native)
    arms = [component("left_arm", "arm", left_names, lo_l, hi_l,
                      "astral_sdk", "measured", "astral_geometric"),
            component("right_arm", "arm", right_names, lo_r, hi_r,
                      "astral_sdk", "measured", "astral_geometric")]
    wuji = [f"finger{f}_joint{j}" for f in range(1, 6) for j in range(1, 5)]
    wlo, whi = urdf_limits("src/wujihand_retargeting/urdf/wujihand_right.urdf", wuji)
    base = {
        "schema_version": 1, "instance": "astral", "robot": "astral",
        "hardware": {"control_board_ip": "192.168.10.2",
                     "wuji_serial": {"right": "SET_RIGHT_WUJI_SERIAL"}},
        "input_settings": {"landmark_preprocess": "raw"},
        "teleop": {side: {"arm_base_frame": f"{side}_base_link",
                          "vr_to_arm_rot": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                          "motion_scale": 1.0, "tcp_offset": [0.0] * 6}
                   for side in ("left", "right")},
        "inputs": {"left": {"wrist": "quest3", "hand": "quest3"},
                   "right": {"wrist": "quest3", "hand": "quest3"}},
        "components": arms,
        "cameras": [
            {"role": "base", "source": "quest3_video_streamer", "device": "realsense_base"},
            {"role": "left_wrist", "source": "quest3_video_streamer", "device": "usb_left_wrist"},
            {"role": "right_wrist", "source": "quest3_video_streamer", "device": "usb_right_wrist"}],
        "dataset": {"fps": 30, "action_source": "next_state"},
        "policy": {"camera_map": {"base_0_rgb": "base", "left_wrist_0_rgb": "left_wrist",
                                   "right_wrist_0_rgb": "right_wrist"}},
    }
    # The SDK driver's JointState contract uses left_gripper/right_gripper,
    # and its physical angle range is 0..0.8 rad (0.8=open).
    gripper = component("left_ee", "gripper", ["left_gripper"],
                        [0.0], [0.8], "astral_sdk", "command_echo")
    right_wuji = component("right_ee", "hand", [f"right_{n}" for n in wuji], wlo, whi,
                           "wuji_serial", "measured")
    mixed = json.loads(json.dumps(base))
    mixed["profile_id"] = "astral_gripper_wuji"
    mixed["components"] = arms + [gripper, right_wuji]
    write("astral_gripper_wuji.json", mixed)
    glove = json.loads(json.dumps(mixed))
    glove["profile_id"] = "astral_gripper_wuji_glove"
    glove["inputs"]["right"]["hand"] = "wuji_glove"
    glove["hardware"]["wuji_glove"] = {"right": "SET_RIGHT_GLOVE_SERIAL"}
    write("astral_gripper_wuji_glove.json", glove)
    dual = json.loads(json.dumps(base))
    dual["profile_id"] = "astral_dual_gripper"
    dual["components"] = arms + [gripper, component("right_ee", "gripper",
        ["right_gripper"], [0.0], [0.8], "astral_sdk", "command_echo")]
    write("astral_dual_gripper.json", dual)


def nero():
    model = "src/xhand_nero_description/urdf/xhand_nero_description.urdf"
    hand_base = ["thumb_bend_joint", "thumb_rota_joint1", "thumb_rota_joint2",
                 "index_bend_joint", "index_joint1", "index_joint2",
                 "mid_joint1", "mid_joint2", "ring_joint1", "ring_joint2",
                 "pinky_joint1", "pinky_joint2"]
    components = []
    for side in ("left", "right"):
        arm_names = [f"{side}_joint{i}" for i in range(1, 8)]
        # The merged visualization URDF applies a J2 frame offset. The live
        # CAN/analytic-IK contract instead uses NeroParams.default() limits.
        # Keep these physical command limits aligned with the unchanged IK.
        alo = [-2.705261, -1.745330, -2.757621, -1.012291,
               -2.757621, -0.733039, -1.570797]
        ahi = [2.705261, 1.745330, 2.757621, 2.146755,
               2.757621, 0.959932, 1.570797]
        components.append(component(f"{side}_arm", "arm", arm_names,
                                    alo, ahi, "nero_can", "measured", "nero_analytic"))
    for side in ("left", "right"):
        hand_names = [f"{side}_hand_{n}" for n in hand_base]
        hlo, hhi = urdf_limits(model, hand_names)
        components.append(component(f"{side}_ee", "hand", hand_names,
                                    hlo, hhi, "xhand_serial", "measured"))
    profile = {
        "schema_version": 1, "profile_id": "nero_dual_xhand", "instance": "nero",
        "robot": "nero", "inputs": {side: {"wrist": "quest3", "hand": "quest3"}
                                     for side in ("left", "right")},
        "input_settings": {
            "landmark_preprocess": "mano",
            "quest3_wrist_pose_mapping": {
                "mode": "per_side",
                "left_rotation": [0.0, 1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0],
                "right_rotation": [0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0],
                "left_frame_id": "nero_left_wrist_mapped",
                "right_frame_id": "nero_right_wrist_mapped",
            },
        },
        "adapter_config": {"nero_can": {
            "home_tolerance_rad": 0.05,
            "channels": {"left": "can_nero_left", "right": "can_nero_right"},
            "home_pose": {
                "left": [-0.405, 1.281, -0.957, 1.311, 2.682, -0.314, -0.163],
                "right": [0.405, 1.281, 0.957, 1.311, -2.682, 0.314, -0.163]}}},
        "hardware": {"xhand_serial": {"left": "/dev/ttyUSB0", "right": "/dev/ttyUSB1"}},
        "teleop": {"left": {"arm_base_frame": "left_arm_base",
                            "vr_to_arm_rot": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                            "motion_scale": 0.65, "tcp_offset": [0.0] * 6},
                   "right": {"arm_base_frame": "right_arm_base",
                             "vr_to_arm_rot": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                             "motion_scale": 0.65, "tcp_offset": [0.0] * 6}},
        "components": components,
        "cameras": [
            {"role": "base", "source": "orbbec", "device": "CP02653000VE"},
            {"role": "left_wrist", "source": "orbbec", "device": "CV284600002L"},
            {"role": "right_wrist", "source": "orbbec", "device": "CV28460000FV"}],
        "dataset": {"fps": 30, "action_source": "next_state"},
        "policy": {"camera_map": {"base_0_rgb": "base", "left_wrist_0_rgb": "left_wrist",
                                   "right_wrist_0_rgb": "right_wrist"}},
    }
    write("nero_dual_xhand.json", profile)


if __name__ == "__main__":
    astral()
    nero()

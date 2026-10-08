"""Launch hooks for adapters shipped with NEXUS.

These are ordinary plugin implementations. New hardware packages register
their own ``module:callable`` hooks through Python entry points and never need
to add a hardware branch to ``system.launch.py``.
"""

from __future__ import annotations

import json
import os
import re

import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def _node(package: str, executable: str, name: str, params=None, remaps=None,
          namespace=None):
    return Node(package=package, executable=executable, name=name,
                namespace=namespace, output="screen", parameters=params or [],
                remappings=remaps or [])


def _safe(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]", "_", value)


def _node_name(prefix: str, component) -> str:
    return f"{prefix}_{_safe(component.name)}"


def _astral_control_board_ip(profile, required: bool) -> str:
    value = str(profile.adapter_config("astral_sdk").get("control_board_ip", "")).strip()
    if value.startswith("${") and value.endswith("}"):
        value = os.environ.get(value[2:-1], "").strip()
    if required and (not value or value.startswith("SET_")):
        raise RuntimeError(
            "set ASTRAL_CONTROL_BOARD_IP for Astral hardware startup"
        )
    return value


def preflight_astral(profile, components) -> None:
    _astral_control_board_ip(profile, required=True)
    from astral_robot_control.joint_layout import ASTRAL_SDK_IDS_AVAILABLE
    if not ASTRAL_SDK_IDS_AVAILABLE:
        raise RuntimeError(
            "Astral hardware startup requires astral_robot_sdk with arm motor IDs; "
            "install the robot SDK or use dry_run:=true"
        )


def preflight_nero(profile, components) -> None:
    config = profile.adapter_config("nero_can").get("channels", {})
    for component in components:
        channel = config.get(component.side, "")
        if not channel or not Path("/sys/class/net", channel).exists():
            raise RuntimeError(f"Nero {component.side} CAN interface {channel or '<missing>'} absent")


def preflight_xhand(profile, components) -> None:
    config = profile.adapter_config("xhand_serial").get("serials", {})
    for component in components:
        port = config.get(component.side, "")
        if not port or not Path(port).exists():
            raise RuntimeError(f"XHand {component.side} serial device {port or '<missing>'} absent")


def preflight_wuji(profile, components) -> None:
    config = profile.adapter_config("wuji_serial").get("serials", {})
    for component in components:
        serial = config.get(component.side, "")
        if not serial or str(serial).startswith("SET_"):
            raise RuntimeError(f"configure adapter_config.wuji_serial.serials.{component.side}")


def preflight_wuji_glove(profile, channels) -> None:
    config = profile.adapter_config("wuji_glove").get("serials", {})
    for channel in channels:
        component = next((c for c in profile.components
                          if c.input_channel == channel and c.side), None)
        side = component.side if component else channel
        serial = config.get(side, "")
        if not serial or str(serial).startswith("SET_"):
            raise RuntimeError(f"configure adapter_config.wuji_glove.serials.{side}")


def preflight_orbbec(profile, cameras) -> None:
    from nero_dual_data_collect.camera_manager import HAS_ORBBEC, _get_device_list
    if not HAS_ORBBEC:
        raise RuntimeError("pyorbbecsdk unavailable for configured Orbbec cameras")
    connected = {str(device["serial"]) for device in _get_device_list()}
    missing = [row["device"] for row in cameras if row["device"] not in connected]
    if missing:
        raise RuntimeError(f"configured Orbbec serials absent: {missing}")


def launch_astral_driver(profile, components, path: str, dry_run: bool):
    ns = profile.namespace
    arms = {side: next((c for c in components if c.kind == "arm" and c.side == side), None)
            for side in ("left", "right")}
    grippers = {side: next((c for c in components if c.kind == "gripper" and c.side == side), None)
                for side in ("left", "right")}
    astral_config = profile.adapter_config("astral_sdk")
    cfg = _share("astral_robot_control", "config", "astral_robot.yaml")
    params = {
        "control_board_ip": _astral_control_board_ip(profile, required=not dry_run),
        "dry_run": dry_run, "auto_ready": False,
        "enable_full_body_cmd": False, "enable_head_cmd": False,
        "enable_gripper_cmd": any(grippers.values()), "enable_gripper_ratio_cmd": False,
        "left_gripper_open_rad": float(astral_config.get("gripper_open_rad", 2.5)),
        "right_gripper_open_rad": float(astral_config.get("gripper_open_rad", 2.5)),
        "left_gripper_closed_rad": float(astral_config.get("gripper_closed_rad", 0.0)),
        "right_gripper_closed_rad": float(astral_config.get("gripper_closed_rad", 0.0)),
        "left_arm_ns": f"{ns}/components/{arms['left'].name}" if arms["left"] else f"{ns}/unused/left_arm",
        "right_arm_ns": f"{ns}/components/{arms['right'].name}" if arms["right"] else f"{ns}/unused/right_arm",
        "left_gripper_ns": f"{ns}/components/{grippers['left'].name}" if grippers['left'] else f"{ns}/unused/left_gripper",
        "right_gripper_ns": f"{ns}/components/{grippers['right'].name}" if grippers['right'] else f"{ns}/unused/right_gripper",
    }
    return [_node("astral_robot_control", "astral_robot_driver", "astral_robot_driver", [cfg, params])]


def launch_nero_driver(profile, components, path: str, dry_run: bool):
    # Resolve every arm before creating launch actions; reject malformed
    # tolerance settings before any driver can connect to a CAN interface.
    from .nero_home_config import home_tolerances
    for component in components:
        home_tolerances(profile.adapter_config("nero_can"), component.side or "left", component.dim)
    return [_node("nexus_core", "nexus_nero_driver", _node_name("driver", component), [{
        "profile_file": path, "component": component.name, "side": component.side or "left",
        "dry_run": dry_run,
        "home_timeout": float(profile.adapter_config("nero_can").get("home_timeout", 20.0)),
        "home_mode_timeout": float(profile.adapter_config("nero_can").get("home_mode_timeout", 1.0)),
        "home_speed_percent": int(profile.adapter_config("nero_can").get("home_speed_percent", 10)),
    }]) for component in components]


def launch_xhand_driver(profile, components, path: str, dry_run: bool):
    actions = []
    for component in components:
        side = component.side
        if dry_run:
            actions.append(_node("nexus_core", "nexus_fake_driver", _node_name("fake", component), [{
                "profile_file": path, "component": component.name,
            }]))
        else:
            config = profile.adapter_config("xhand_serial").get("serials", {})
            actions.append(_node("xhand_control_ros2", "xhand_control_ros2_node",
                                 _node_name("xhand", component), [
                                     _share("xhand_control_ros2", "config", "xhand_config.yaml"),
                                     {"port_name": config[side], "update_rate": 100.0},
                                 ], namespace=f"{side}_hand"))
        actions.append(_node("nexus_core", "nexus_joint_bridge", _node_name("xhand_bridge", component), [{
            "profile_file": path, "component": component.name, "side": side,
            "adapter_mode": "xhand", "candidate_only": dry_run,
        }]))
    return actions


def launch_wuji_driver(profile, components, path: str, dry_run: bool):
    actions = []
    for component in components:
        side = component.side
        if dry_run:
            actions.append(_node("nexus_core", "nexus_fake_driver", _node_name("fake", component), [{
                "profile_file": path, "component": component.name,
            }]))
        else:
            serial = profile.adapter_config("wuji_serial").get("serials", {})[side]
            actions.append(_node("wujihand_driver", "wujihand_driver_node",
                                 _node_name("wuji", component), [{
                                     "hand_side": side, "serial_number": serial, "auto_enable": False,
                                 }], namespace=f"{side}_hand"))
        actions.append(_node("nexus_core", "nexus_joint_bridge", _node_name("wuji_bridge", component), [{
            "profile_file": path, "component": component.name, "side": side,
            "adapter_mode": "wuji", "candidate_only": dry_run,
        }]))
    return actions


def launch_astral_ik(profile, components, path: str, dry_run: bool):
    component = components[0]
    side = component.side
    cfg = _share("astral_arm_teleop", "config", f"astral_arm_teleop_{side}.yaml")
    teleop = profile.teleop_config(component.name)
    # ROS YAML keys target node names. NEXUS gives components instance-local
    # names, so load the legacy preset as a parameter dictionary explicitly.
    with open(cfg, encoding="utf-8") as stream:
        preset = yaml.safe_load(stream)[f"astral_arm_teleop_{side}"]["ros__parameters"]
    ns = profile.namespace
    remaps = [
        (f"/{side}_arm/joint_commands", profile.candidate_topic("teleop", component.name)),
        (f"/{side}_arm/joint_states", profile.topic(component.name, "joint_states")),
        (f"quest3/{side}_wrist_pose", f"{ns}/input/{component.input_channel}/wrist_pose"),
        ("quest3/body_joints", f"{ns}/input/body/body_joints"),
        ("quest3/body_joint_names", f"{ns}/input/body/body_joint_names"),
        ("/teleop/start", f"{ns}/control/teleop_start"),
        ("/teleop/disarm", f"{ns}/control/teleop_disarm"),
        ("/teleop/armed", f"{ns}/control/teleop_armed"),
    ]
    return [_node("astral_arm_teleop", "astral_arm_teleop_node",
                  _node_name("teleop", component), [preset, {
                      "control_rate": float(teleop.get("control_rate", 100.0)),
                      "arm_side": side, "require_start_signal": True, "move_to_init_pose": False,
                      "reanchor_require_measured": True,
                      "motion_scale": float(teleop["motion_scale"]),
                      "vr_to_arm_rot": teleop["vr_to_arm_rot"], "tcp_offset": teleop["tcp_offset"],
                  }], remaps)]


def launch_nero_ik(profile, components, path: str, dry_run: bool):
    component = components[0]
    ns = profile.namespace
    remaps = [("/teleop/start", f"{ns}/control/teleop_start"),
              ("/teleop/disarm", f"{ns}/control/teleop_disarm")]
    return [_node("nexus_core", "nexus_nero_teleop", _node_name("teleop", component), [{
        "profile_file": path, "component": component.name, "side": component.side or "left",
        "control_rate": float(profile.teleop_config(component.name).get("control_rate", 100.0)),
    }], remaps + [("/teleop/armed", f"{ns}/control/teleop_armed")])]


def launch_pinch(profile, components, path: str, dry_run: bool):
    component = components[0]
    channel = component.input_channel or component.side
    ns = profile.namespace
    return [
        _node("astral_gripper_teleop", "pinch_gripper_node",
              _node_name("pinch", component), [{
                  "hand_side": component.side, "landmark_topic": f"{ns}/input/{channel}/hand_landmarks",
                  "controller_joy_topic": f"{ns}/input/{channel}/controller_joy",
                  "command_topic": f"{ns}/legacy/teleop/{component.side}_gripper_ratio",
                  "disarm_topic": f"{ns}/control/teleop_disarm",
                  "arm_topic": f"{ns}/control/teleop_armed",
              }]),
        _node("nexus_core", "nexus_joint_bridge", _node_name("gripper_bridge", component), [{
            "profile_file": path, "component": component.name,
            "side": component.side, "adapter_mode": "astral_gripper",
        }]),
    ]


def launch_wuji_retargeter(profile, components, path: str, dry_run: bool):
    component = components[0]
    side = component.side
    channel = component.input_channel or side
    ns = profile.namespace
    return [_node("wujihand_retargeting", "wujihand_retarget_node",
                  _node_name("retarget", component), [{
                      "hand_side": side, "input_topic": "/hand_landmarks", "viz": False,
                      "retarget_backend": "dexpilot" if dry_run else "official",
                  }], [
                      (f"/hand_landmarks/{side}", f"{ns}/input/{channel}/hand_landmarks"),
                      (f"/{side}_hand/joint_commands", f"{ns}/legacy/{side}_wuji_candidate"),
                  ])]


def launch_xhand_retargeter(profile, components, path: str, _dry_run: bool):
    if not components:
        return []
    ns = profile.namespace
    params = [_share("xhand_retargeting", "config", "retargeting_params.yaml"), {
        "input_topic": "/hand_landmarks", "viz": False,
        "dry_run": False, "require_clench_to_start": False,
    }]
    remaps = []
    for component in components:
        side = component.side
        channel = component.input_channel or side
        remaps.extend([
            (f"/hand_landmarks/{side}", f"{ns}/input/{channel}/hand_landmarks"),
            (f"/{side}_hand/xhand_command", f"{ns}/legacy/{side}_xhand_candidate"),
        ])
    remaps.extend([("/teleop/disarm", f"{ns}/control/teleop_disarm"),
                   ("/teleop/armed", f"{ns}/control/teleop_armed")])
    return [_node("xhand_retargeting", "xhand_dex_retargeting_node", "nexus_xhand_retargeter",
                  params, remaps)]


def launch_quest3_input(profile, channels, path: str, dry_run: bool):
    remaps = []
    for channel in channels:
        for semantic, default_topic in (
            ("wrist", f"/quest3/{channel}_wrist_pose"),
            ("hand", f"/hand_landmarks/{channel}"),
            ("controller_joy", f"/quest3/{channel}_controller_joy"),
            ("body_joints", "/quest3/body_joints"),
            ("body_joint_names", "/quest3/body_joint_names"),
        ):
            spec = profile.input_spec(channel, semantic)
            if spec is None:
                continue
            if spec["source"] == "quest3":
                remaps.append((default_topic, spec["topic"]))
            elif semantic == "hand":
                remaps.append((default_topic, f"{profile.namespace}/unused/input/{channel}/hand_landmarks"))
    quest_cfg = _share("quest3_hand_mocap", "config", "quest3_mocap.yaml")
    input_settings = profile.raw.get("input_settings", {})
    mapping = input_settings.get("quest3_wrist_pose_mapping", {})
    quest_params = {
        "arm_side": "both",
        "landmark_preprocess": input_settings.get("landmark_preprocess", "raw"),
        "wrist_pose_mapping_mode": mapping.get("mode", "global"),
    }
    if mapping.get("mode") == "per_side":
        for side in ("left", "right"):
            quest_params[f"{side}_wrist_to_arm_rot"] = mapping[f"{side}_rotation"]
            quest_params[f"{side}_wrist_frame_id"] = mapping[f"{side}_frame_id"]
    return [_node("quest3_hand_mocap", "quest3_udp_mocap", "nexus_quest3_input", [
        quest_cfg, quest_params
    ], remaps)]


def launch_wuji_glove_input(profile, channels, path: str, dry_run: bool):
    if dry_run:
        return []
    glove_cfg = profile.adapter_config("wuji_glove").get("serials", {})
    actions = []
    for channel in channels:
        component = next((c for c in profile.components
                          if c.input_channel == channel and c.side), None)
        side = component.side if component else channel
        actions.append(_node("wuji_glove", "wuji_glove_mocap", f"nexus_glove_{channel}", [{
            "hand_side": side, "publish_rate": 50.0, "ema_alpha": 0.7,
            "sn": glove_cfg[side], "output_topic": "hand_landmarks",
        }], namespace=f"wuji_glove_{channel}"))
    return actions


def launch_quest_camera(profile, cameras, path: str, dry_run: bool):
    overrides = {
        camera["role"]: {
            **{"source": camera["mode"]},
            **{key: camera[key] for key in ("device", "topic", "preset", "force_mjpg")
               if key in camera},
        }
        for camera in cameras
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share("quest3_video_streamer", "launch", "multi_camera.launch.py")),
        launch_arguments={"cameras": ",".join(c["role"] for c in cameras),
                          "camera_overrides": json.dumps(overrides)}.items())]


def launch_orbbec_camera(profile, cameras, path: str, dry_run: bool):
    return [_node("nexus_core", "nexus_orbbec_camera", "nexus_orbbec_camera", [{
        "profile_file": profile.path,
    }])]

"""Assemble a NEXUS system by composing registered adapters from one profile.

The core launch owns profile loading, shared ROS contracts, recorder and policy
services. Hardware-specific nodes live in adapter launchers below; there is no
robot-family switch in the assembly path.
"""

from __future__ import annotations

import os
import re
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nexus_core.adapter_registry import DRIVERS
from nexus_core.profile import Profile


def _share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def _node(package: str, executable: str, name: str, params=None, remaps=None,
          namespace=None):
    return Node(package=package, executable=executable, name=name,
                namespace=namespace, output="screen", parameters=params or [],
                remappings=remaps or [])


def _safe(value: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]", "_", value)


def _component_node_name(prefix: str, component) -> str:
    return f"{prefix}_{_safe(component.name)}"


def _source(spec: dict | None) -> str | None:
    return spec.get("source") if spec else None


def _selected_sources(profile: Profile) -> set[str]:
    return {
        spec["source"]
        for channel in profile.raw["inputs"]
        for semantic in profile.raw["inputs"][channel]
        if (spec := profile.input_spec(channel, semantic)) is not None
        and spec["source"] != "none"
    }


def _preflight(profile: Profile, dry_run: bool, with_cameras: bool) -> None:
    if dry_run:
        return
    for component in profile.components:
        side = component.side
        config = profile.adapter_config(component.driver)
        if component.driver == "astral_sdk":
            address = config.get("control_board_ip", "")
            if not address or str(address).startswith("SET_"):
                raise RuntimeError("configure adapter_config.astral_sdk.control_board_ip")
        elif component.driver == "nero_can":
            channel = config.get("channels", {}).get(side, "")
            if not channel or not Path("/sys/class/net", channel).exists():
                raise RuntimeError(f"Nero {side} CAN interface {channel or '<missing>'} absent")
        elif component.driver == "xhand_serial":
            port = config.get("serials", {}).get(side, "")
            if not port or not Path(port).exists():
                raise RuntimeError(f"XHand {side} serial device {port or '<missing>'} absent")
        elif component.driver == "wuji_serial":
            serial = config.get("serials", {}).get(side, "")
            if not serial or str(serial).startswith("SET_"):
                raise RuntimeError(f"configure adapter_config.wuji_serial.serials.{side}")
    for channel in profile.raw["inputs"]:
        hand = profile.input_spec(channel, "hand")
        if _source(hand) == "wuji_glove":
            side = next((c.side for c in profile.components if c.input_channel == channel and c.side), channel)
            serial = profile.adapter_config("wuji_glove").get("serials", {}).get(side, "")
            if not serial or str(serial).startswith("SET_"):
                raise RuntimeError(f"configure adapter_config.wuji_glove.serials.{side}")
    if with_cameras and any(c["source"] == "orbbec" for c in profile.raw["cameras"]):
        from nero_dual_data_collect.camera_manager import HAS_ORBBEC, _get_device_list
        if not HAS_ORBBEC:
            raise RuntimeError("pyorbbecsdk unavailable for configured Orbbec cameras")
        connected = {str(device["serial"]) for device in _get_device_list()}
        missing = [c["device"] for c in profile.raw["cameras"]
                   if c["source"] == "orbbec" and c["device"] not in connected]
        if missing:
            raise RuntimeError(f"configured Orbbec serials absent: {missing}")


def _launch_inputs(profile: Profile, dry_run: bool):
    actions = []
    sources = _selected_sources(profile)
    ns = profile.namespace
    if "quest3" in sources:
        remaps = []
        for channel in profile.raw["inputs"]:
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
                    remaps.append((default_topic, f"{ns}/unused/input/{channel}/hand_landmarks"))
        quest_cfg = _share("quest3_hand_mocap", "config", "quest3_mocap.yaml")
        actions.append(_node("quest3_hand_mocap", "quest3_udp_mocap", "nexus_quest3_input",
                             [quest_cfg, {"arm_side": "both",
                                          "landmark_preprocess": profile.raw.get("input_settings", {}).get(
                                              "landmark_preprocess", "raw")}], remaps))
    if "wuji_glove" in sources and not dry_run:
        glove_cfg = profile.adapter_config("wuji_glove").get("serials", {})
        for channel in profile.raw["inputs"]:
            hand = profile.input_spec(channel, "hand")
            if _source(hand) != "wuji_glove":
                continue
            side = next((c.side for c in profile.components
                         if c.input_channel == channel and c.side), channel)
            actions.append(_node("wuji_glove", "wuji_glove_mocap", f"nexus_glove_{channel}", [{
                "hand_side": side, "publish_rate": 50.0, "ema_alpha": 0.7,
                "sn": glove_cfg[side], "output_topic": "hand_landmarks",
            }], namespace=f"wuji_glove_{channel}"))
    return actions


def _launch_astral_driver(profile: Profile, components, path: str, dry_run: bool):
    ns = profile.namespace
    arms = {side: next((c for c in components if c.kind == "arm" and c.side == side), None)
            for side in ("left", "right")}
    grippers = {side: next((c for c in components if c.kind == "gripper" and c.side == side), None)
                for side in ("left", "right")}
    cfg = _share("astral_robot_control", "config", "astral_robot.yaml")
    params = {
        "control_board_ip": profile.adapter_config("astral_sdk")["control_board_ip"],
        "dry_run": dry_run, "auto_ready": False,
        "enable_full_body_cmd": False, "enable_head_cmd": False,
        "enable_gripper_cmd": any(grippers.values()), "enable_gripper_ratio_cmd": False,
        "left_arm_ns": f"{ns}/components/{arms['left'].name}" if arms["left"] else f"{ns}/unused/left_arm",
        "right_arm_ns": f"{ns}/components/{arms['right'].name}" if arms["right"] else f"{ns}/unused/right_arm",
        "left_gripper_ns": f"{ns}/components/{grippers['left'].name}" if grippers["left"] else f"{ns}/unused/left_gripper",
        "right_gripper_ns": f"{ns}/components/{grippers['right'].name}" if grippers["right"] else f"{ns}/unused/right_gripper",
    }
    return [_node("astral_robot_control", "astral_robot_driver", "astral_robot_driver", [cfg, params])]


def _launch_nero_driver(profile: Profile, component, path: str, dry_run: bool):
    return [_node("nexus_core", "nexus_nero_driver", _component_node_name("driver", component), [{
        "profile_file": path, "component": component.name, "side": component.side or "left",
        "dry_run": dry_run,
    }])]


def _launch_xhand_driver(profile: Profile, component, path: str, dry_run: bool):
    side = component.side
    actions = []
    if dry_run:
        actions.append(_node("nexus_core", "nexus_fake_driver", _component_node_name("fake", component), [{
            "profile_file": path, "component": component.name,
        }]))
    else:
        config = profile.adapter_config("xhand_serial").get("serials", {})
        actions.append(_node("xhand_control_ros2", "xhand_control_ros2_node",
                             _component_node_name("xhand", component), [
                                 _share("xhand_control_ros2", "config", "xhand_config.yaml"),
                                 {"port_name": config[side], "update_rate": 100.0},
                             ], namespace=f"{side}_hand"))
    actions.append(_node("nexus_core", "nexus_joint_bridge", _component_node_name("xhand_bridge", component), [{
        "profile_file": path, "component": component.name, "side": side,
        "adapter_mode": "xhand", "candidate_only": dry_run,
    }]))
    return actions


def _launch_wuji_driver(profile: Profile, component, path: str, dry_run: bool):
    side = component.side
    actions = []
    if dry_run:
        actions.append(_node("nexus_core", "nexus_fake_driver", _component_node_name("fake", component), [{
            "profile_file": path, "component": component.name,
        }]))
    else:
        serial = profile.adapter_config("wuji_serial").get("serials", {})[side]
        actions.append(_node("wujihand_driver", "wujihand_driver_node",
                             _component_node_name("wuji", component), [{
                                 "hand_side": side, "serial_number": serial, "auto_enable": False,
                             }], namespace=f"{side}_hand"))
    actions.append(_node("nexus_core", "nexus_joint_bridge", _component_node_name("wuji_bridge", component), [{
        "profile_file": path, "component": component.name, "side": side,
        "adapter_mode": "wuji", "candidate_only": dry_run,
    }]))
    return actions


DRIVER_LAUNCHERS = {
    "astral_sdk": _launch_astral_driver,
    "nero_can": _launch_nero_driver,
    "xhand_serial": _launch_xhand_driver,
    "wuji_serial": _launch_wuji_driver,
}


def _launch_astral_ik(profile: Profile, component, path: str):
    side = component.side
    cfg = _share("astral_arm_teleop", "config", f"astral_arm_teleop_{side}.yaml")
    teleop = profile.teleop_config(component.name)
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
                  _component_node_name("teleop", component), [cfg, {
                      "arm_side": side, "require_start_signal": True, "move_to_init_pose": False,
                      "reanchor_require_measured": True,
                      "motion_scale": float(teleop["motion_scale"]),
                      "vr_to_arm_rot": teleop["vr_to_arm_rot"], "tcp_offset": teleop["tcp_offset"],
                  }], remaps)]


def _launch_nero_ik(profile: Profile, component, path: str):
    ns = profile.namespace
    remaps = [("/teleop/start", f"{ns}/control/teleop_start"),
              ("/teleop/disarm", f"{ns}/control/teleop_disarm")]
    return [_node("nexus_core", "nexus_nero_teleop", _component_node_name("teleop", component), [{
        "profile_file": path, "component": component.name, "side": component.side or "left",
    }], remaps + [("/teleop/armed", f"{ns}/control/teleop_armed")])]


IK_LAUNCHERS = {
    "astral_geometric": _launch_astral_ik,
    "nero_analytic": _launch_nero_ik,
}


def _launch_pinch(profile: Profile, component, path: str):
    channel = component.input_channel or component.side
    ns = profile.namespace
    return [
        _node("astral_gripper_teleop", "pinch_gripper_node",
              _component_node_name("pinch", component), [{
                  "hand_side": component.side, "landmark_topic": f"{ns}/input/{channel}/hand_landmarks",
                  "controller_joy_topic": f"{ns}/input/{channel}/controller_joy",
                  "command_topic": f"{ns}/legacy/teleop/{component.side}_gripper_ratio",
                  "disarm_topic": f"{ns}/control/teleop_disarm",
                  "arm_topic": f"{ns}/control/teleop_armed",
              }]),
        _node("nexus_core", "nexus_joint_bridge", _component_node_name("gripper_bridge", component), [{
            "profile_file": path, "component": component.name,
            "side": component.side, "adapter_mode": "astral_gripper",
        }]),
    ]


def _launch_wuji_retargeter(profile: Profile, component, path: str):
    side = component.side
    channel = component.input_channel or side
    ns = profile.namespace
    return [_node("wujihand_retargeting", "wujihand_retarget_node",
                  _component_node_name("retarget", component), [{
                      "hand_side": side, "input_topic": "/hand_landmarks", "viz": False,
                  }], [
                      (f"/hand_landmarks/{side}", f"{ns}/input/{channel}/hand_landmarks"),
                      (f"/{side}_hand/joint_commands", f"{ns}/legacy/{side}_wuji_candidate"),
                  ])]


def _launch_xhand_retargeter(profile: Profile, components):
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
    remaps.extend([
        ("/teleop/disarm", f"{ns}/control/teleop_disarm"),
        ("/teleop/armed", f"{ns}/control/teleop_armed"),
    ])
    return [_node("xhand_retargeting", "xhand_dex_retargeting_node", "nexus_xhand_retargeter",
                  params, remaps)]


RETARGETER_LAUNCHERS = {
    "pinch_gripper": _launch_pinch,
    "wuji_hand": _launch_wuji_retargeter,
}


def _launch_cameras(profile: Profile):
    sources = {camera["source"] for camera in profile.raw["cameras"]}
    actions = []
    if "quest3_video_streamer" in sources:
        cameras = [c for c in profile.raw["cameras"]
                   if c["source"] == "quest3_video_streamer"]
        overrides = {
            camera["role"]: {
                **{"source": camera["mode"]},
                **{key: camera[key] for key in ("device", "topic", "preset", "force_mjpg")
                   if key in camera},
            }
            for camera in cameras
        }
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _share("quest3_video_streamer", "launch", "multi_camera.launch.py")),
            launch_arguments={"cameras": ",".join(c["role"] for c in cameras),
                              "camera_overrides": json.dumps(overrides)}.items()))
    if "orbbec" in sources:
        actions.append(_node("nexus_core", "nexus_orbbec_camera", "nexus_orbbec_camera", [{
            "profile_file": profile.path,
        }]))
    return actions


def _setup(context):
    profile_name = LaunchConfiguration("profile").perform(context).strip()
    if not profile_name or (Path(profile_name).name != profile_name and not Path(profile_name).is_file()):
        raise RuntimeError("profile must be a built-in name or an existing JSON file")
    path = Path(profile_name)
    if not path.is_file():
        path = Path(_share("nexus_core", "profiles", f"{profile_name}.json"))
    profile = Profile.load(path)
    if profile.raw.get("schema_version") != 2:
        raise RuntimeError("system launch requires a schema v2 assembly profile; v1 remains readable for old episode metadata")
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() in ("true", "1", "yes")
    with_cameras = LaunchConfiguration("with_cameras").perform(context).lower() in ("true", "1", "yes")
    with_recording = LaunchConfiguration("with_recording").perform(context).lower() in ("true", "1", "yes")
    with_policy = LaunchConfiguration("with_policy").perform(context).lower() in ("true", "1", "yes")
    _preflight(profile, dry_run, with_cameras)
    actions = [LogInfo(msg=f"NEXUS profile={profile.profile_id} instance={profile.instance} "
                           f"sha256={profile.digest} dry_run={dry_run}"),
               _node("nexus_core", "nexus_command_mux", "nexus_command_mux",
                     [{"profile_file": str(path)}]),
               _node("nexus_core", "nexus_driver_manager", "nexus_driver_manager",
                     [{"profile_file": str(path)}]),
               _node("nexus_core", "nexus_input_bridge", "nexus_input_bridge",
                     [{"profile_file": str(path)}])]
    actions.extend(_launch_inputs(profile, dry_run))

    # Assembly scoped adapters (such as Astral's single bilateral SDK process)
    # start once, while component scoped adapters start for each component.
    started_assemblies = set()
    for component in profile.components:
        adapter = DRIVER_LAUNCHERS.get(component.driver)
        if adapter is None:
            raise RuntimeError(f"no launch plugin registered for driver {component.driver}")
        if DRIVERS[component.driver].scope == "assembly":
            if component.driver in started_assemblies:
                continue
            group = [c for c in profile.components if c.driver == component.driver]
            actions.extend(adapter(profile, group, str(path), dry_run))
            started_assemblies.add(component.driver)
        else:
            actions.extend(adapter(profile, component, str(path), dry_run))

    for component in profile.components:
        if component.ik is None:
            continue
        if component.ik not in IK_LAUNCHERS:
            raise RuntimeError(f"no launch plugin registered for IK adapter {component.ik}")
        actions.extend(IK_LAUNCHERS[component.ik](profile, component, str(path)))

    for component in profile.components:
        if component.retargeter == "xhand_dexpilot":
            continue
        if component.retargeter:
            launcher = RETARGETER_LAUNCHERS.get(component.retargeter)
            if launcher is None:
                raise RuntimeError(f"no launch plugin registered for retargeter {component.retargeter}")
            actions.extend(launcher(profile, component, str(path)))
    xhand_components = [c for c in profile.components if c.retargeter == "xhand_dexpilot"]
    actions.extend(_launch_xhand_retargeter(profile, xhand_components))

    if with_cameras:
        actions.extend(_launch_cameras(profile))
    if with_recording:
        actions.append(_node("astral_data_collect", "data_collect_node", "nexus_data_collect", [{
            "profile_file": str(path),
            "save_root": LaunchConfiguration("data_root").perform(context),
            "session": LaunchConfiguration("session").perform(context),
        }], [("/teleop/start", f"{profile.namespace}/control/teleop_start"),
              ("/teleop/disarm", f"{profile.namespace}/control/teleop_disarm"),
              ("/data_collect/control", f"{profile.namespace}/data/collect/control"),
              ("/data_collect/task", f"{profile.namespace}/data/collect/task"),
              ("/data_collect/session", f"{profile.namespace}/data/collect/session"),
              ("/data_collect/state", f"{profile.namespace}/data/collect/state")]))
    if with_policy:
        actions.append(_node("nexus_core", "nexus_policy", "nexus_policy", [{
            "profile_file": str(path),
            "model_manifest": LaunchConfiguration("model_manifest").perform(context),
            "backend_type": LaunchConfiguration("backend_type").perform(context),
            "model": LaunchConfiguration("model").perform(context),
            "host": LaunchConfiguration("server_host").perform(context),
            "port": int(LaunchConfiguration("server_port").perform(context)),
            "replay_path": LaunchConfiguration("replay_path").perform(context),
        }], [("/policy_inference/state", f"{profile.namespace}/policy/state"),
              ("/policy_inference/cmd", f"{profile.namespace}/policy/cmd"),
              ("/policy_inference/task", f"{profile.namespace}/policy/task"),
              ("/teleop/disarm", f"{profile.namespace}/control/teleop_disarm")]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="astral_gripper_wuji"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("with_cameras", default_value="false"),
        DeclareLaunchArgument("with_recording", default_value="true"),
        DeclareLaunchArgument("with_policy", default_value="true"),
        DeclareLaunchArgument("data_root", default_value="~/nexus_data"),
        DeclareLaunchArgument("session", default_value="default_task"),
        DeclareLaunchArgument("model_manifest", default_value=""),
        DeclareLaunchArgument("backend_type", default_value="remote"),
        DeclareLaunchArgument("model", default_value="act"),
        DeclareLaunchArgument("server_host", default_value="127.0.0.1"),
        DeclareLaunchArgument("server_port", default_value="8001"),
        DeclareLaunchArgument("replay_path", default_value=""),
        OpaqueFunction(function=_setup),
    ])

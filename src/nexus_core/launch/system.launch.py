"""One validated NEXUS assembly per ROS graph.

Examples:
  ros2 launch nexus_core system.launch.py profile:=astral_gripper_wuji dry_run:=true
  ros2 launch nexus_core system.launch.py profile:=nero_dual_xhand dry_run:=false with_cameras:=true
"""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from nexus_core.profile import Profile


def _share(package: str, *parts: str) -> str:
    return os.path.join(get_package_share_directory(package), *parts)


def _node(package: str, executable: str, name: str, params=None, remaps=None,
          namespace=None):
    return Node(package=package, executable=executable, name=name,
                namespace=namespace, output="screen", parameters=params or [],
                remappings=remaps or [])


def _setup(context):
    profile_name = LaunchConfiguration("profile").perform(context).strip()
    if not profile_name or Path(profile_name).name != profile_name and not Path(profile_name).is_file():
        raise RuntimeError("profile must be a built-in name or an existing JSON file")
    path = Path(profile_name)
    if not path.is_file():
        path = Path(_share("nexus_core", "profiles", f"{profile_name}.json"))
    profile = Profile.load(path)
    dry_run = LaunchConfiguration("dry_run").perform(context).lower() in ("true", "1", "yes")
    with_cameras = LaunchConfiguration("with_cameras").perform(context).lower() in ("true", "1", "yes")
    with_recording = LaunchConfiguration("with_recording").perform(context).lower() in ("true", "1", "yes")
    with_policy = LaunchConfiguration("with_policy").perform(context).lower() in ("true", "1", "yes")
    ns = profile.namespace
    if not dry_run and profile.raw["robot"] == "astral":
        for spec in profile.components:
            if spec.driver == "wuji_serial":
                side = spec.name.split("_", 1)[0]
                serial = profile.raw["hardware"].get("wuji_serial", {}).get(side, "")
                if not serial or serial.startswith("SET_"):
                    raise RuntimeError(f"configure hardware.wuji_serial.{side} before real launch")
            side = spec.name.split("_", 1)[0]
            if profile.raw["inputs"][side]["hand"] == "wuji_glove":
                glove_sn = profile.raw["hardware"].get("wuji_glove", {}).get(side, "")
                if not glove_sn or glove_sn.startswith("SET_"):
                    raise RuntimeError(f"configure hardware.wuji_glove.{side} before real launch")
    if not dry_run and profile.raw["robot"] == "nero":
        for side, channel in profile.raw["hardware"]["can"].items():
            if not Path("/sys/class/net", channel).exists():
                raise RuntimeError(f"Nero {side} CAN interface {channel} absent")
        for side, port in profile.raw["hardware"]["xhand_serial"].items():
            if not Path(port).exists():
                raise RuntimeError(f"XHand {side} device {port} absent")
    if with_cameras and profile.raw["robot"] == "nero":
        from nero_dual_data_collect.camera_manager import HAS_ORBBEC, _get_device_list
        if not HAS_ORBBEC:
            raise RuntimeError("pyorbbecsdk unavailable for Nero cameras")
        connected = {str(device["serial"]) for device in _get_device_list()}
        missing = [camera["device"] for camera in profile.raw["cameras"]
                   if camera["source"] == "orbbec" and camera["device"] not in connected]
        if missing:
            raise RuntimeError(f"Nero camera serials absent: {missing}")
    actions = [LogInfo(msg=f"NEXUS profile={profile.profile_id} instance={profile.instance} "
                           f"sha256={profile.digest} dry_run={dry_run}"),
               _node("nexus_core", "nexus_command_mux", "nexus_command_mux",
                     [{"profile_file": str(path)}]),
               _node("nexus_core", "nexus_input_bridge", "nexus_input_bridge",
                     [{"profile_file": str(path)}])]

    quest_cfg = _share("quest3_hand_mocap", "config", "quest3_mocap.yaml")
    quest_remaps = [(f"/hand_landmarks/{side}", f"{ns}/unused/quest3_{side}_hand")
                    for side in ("left", "right")
                    if profile.raw["inputs"][side]["hand"] == "wuji_glove"]
    actions.append(_node("quest3_hand_mocap", "quest3_udp_mocap", "quest3_udp_mocap",
                         [quest_cfg, {"arm_side": "both", "landmark_preprocess":
                                      profile.raw["input_settings"]["landmark_preprocess"]}],
                         quest_remaps))
    for side in ("left", "right"):
        if profile.raw["inputs"][side]["hand"] == "wuji_glove" and not dry_run:
            actions.append(_node("wuji_glove", "wuji_glove_mocap", f"wuji_glove_{side}",
                                 [{"hand_side": side, "publish_rate": 50.0,
                                   "ema_alpha": 0.7,
                                   "sn": profile.raw["hardware"]["wuji_glove"][side],
                                   "output_topic": "hand_landmarks"}]))

    if profile.raw["robot"] == "astral":
        actions += _astral(profile, str(path), dry_run)
    elif profile.raw["robot"] == "nero":
        actions += _nero(profile, str(path), dry_run)
    if with_cameras:
        if profile.raw["robot"] == "astral":
            actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
                _share("quest3_video_streamer", "launch", "multi_camera.launch.py"))))
        else:
            actions.append(_node("nexus_core", "nexus_orbbec_camera", "nexus_orbbec_camera",
                                 [{"profile_file": str(path)}]))
    if with_recording:
        actions.append(_node("astral_data_collect", "data_collect_node", "data_collect",
                             [{"profile_file": str(path),
                               "save_root": LaunchConfiguration("data_root").perform(context),
                               "session": LaunchConfiguration("session").perform(context)}]))
    if with_policy:
        actions.append(_node("nexus_core", "nexus_policy", "nexus_policy",
                             [{"profile_file": str(path),
                               "model_manifest": LaunchConfiguration("model_manifest").perform(context),
                               "backend_type": LaunchConfiguration("backend_type").perform(context),
                               "model": LaunchConfiguration("model").perform(context),
                               "host": LaunchConfiguration("server_host").perform(context),
                               "port": int(LaunchConfiguration("server_port").perform(context)),
                               "replay_path": LaunchConfiguration("replay_path").perform(context)}]))
    return actions


def _astral(profile: Profile, path: str, dry_run: bool):
    ns = profile.namespace
    ee = {s: profile.component(f"{s}_ee") for s in ("left", "right")}
    driver_cfg = _share("astral_robot_control", "config", "astral_robot.yaml")
    gripper_ns = {s: f"{ns}/components/{s}_ee" if ee[s].kind == "gripper"
                  else f"{ns}/unused/{s}_gripper" for s in ("left", "right")}
    actions = [_node("astral_robot_control", "astral_robot_driver", "astral_robot_driver",
                     [driver_cfg, {
                         "control_board_ip": profile.raw["hardware"]["control_board_ip"],
                         "dry_run": dry_run, "auto_ready": False,
                         "enable_full_body_cmd": False, "enable_head_cmd": False,
                         "enable_gripper_cmd": any(c.kind == "gripper" for c in ee.values()),
                         "enable_gripper_ratio_cmd": False,
                         "left_arm_ns": f"{ns}/components/left_arm",
                         "right_arm_ns": f"{ns}/components/right_arm",
                         "left_gripper_ns": gripper_ns["left"],
                         "right_gripper_ns": gripper_ns["right"],
                     }])]
    for side in ("left", "right"):
        teleop_cfg = _share("astral_arm_teleop", "config", f"astral_arm_teleop_{side}.yaml")
        remaps = [
            (f"/{side}_arm/joint_commands", profile.candidate_topic("teleop", f"{side}_arm")),
            (f"/{side}_arm/joint_states", profile.topic(f"{side}_arm", "joint_states")),
            (f"quest3/{side}_wrist_pose", f"{ns}/input/{side}/wrist_pose"),
            ("quest3/body_joints", f"{ns}/input/body_joints"),
            ("quest3/body_joint_names", f"{ns}/input/body_joint_names"),
        ]
        actions.append(_node("astral_arm_teleop", "astral_arm_teleop_node",
                             f"astral_arm_teleop_{side}", [teleop_cfg, {
                                 "require_start_signal": True, "move_to_init_pose": False,
                                 "reanchor_require_measured": True,
                                 "motion_scale": float(profile.raw["teleop"][side]["motion_scale"]),
                                 "vr_to_arm_rot": profile.raw["teleop"][side]["vr_to_arm_rot"],
                                 "tcp_offset": profile.raw["teleop"][side]["tcp_offset"],
                             }], remaps))
        spec = ee[side]
        if spec.kind == "gripper":
            actions.append(_node("astral_gripper_teleop", "pinch_gripper_node",
                                 f"pinch_gripper_{side}", [{
                                     "hand_side": side,
                                     "landmark_topic": f"{ns}/input/{side}/hand_landmarks",
                                     "controller_joy_topic": f"{ns}/input/{side}/controller_joy",
                                     "command_topic": f"{ns}/legacy/teleop/{side}_gripper_ratio",
                                 }]))
            actions.append(_node("nexus_core", "nexus_joint_bridge", f"gripper_bridge_{side}",
                                 [{"profile_file": path, "component": spec.name,
                                   "side": side, "adapter_mode": "astral_gripper"}]))
        elif spec.driver == "wuji_serial":
            actions.append(_node("wujihand_retargeting", "wujihand_retarget_node",
                                 f"wuji_retarget_{side}", [{"hand_side": side,
                                      "input_topic": "/hand_landmarks", "viz": False}], [
                                      (f"/hand_landmarks/{side}", f"{ns}/input/{side}/hand_landmarks"),
                                      (f"/{side}_hand/joint_commands",
                                       f"{ns}/legacy/{side}_wuji_candidate")]))
            if dry_run:
                actions.append(_node("nexus_core", "nexus_fake_driver", f"fake_{spec.name}",
                                     [{"profile_file": path, "component": spec.name}]))
                # Retarget candidate still needs its Wuji name conversion.
                actions.append(_node("nexus_core", "nexus_joint_bridge", f"wuji_bridge_{side}",
                                     [{"profile_file": path, "component": spec.name,
                                       "side": side, "adapter_mode": "wuji", "candidate_only": True}]))
            else:
                actions.append(_node("wujihand_driver", "wujihand_driver_node",
                                     f"wuji_driver_{side}", [{"hand_side": side,
                                         "serial_number": profile.raw["hardware"]["wuji_serial"][side],
                                         "auto_enable": False}],
                                     namespace=f"{side}_hand"))
                actions.append(_node("nexus_core", "nexus_joint_bridge", f"wuji_bridge_{side}",
                                     [{"profile_file": path, "component": spec.name,
                                       "side": side, "adapter_mode": "wuji"}]))
        else:
            raise RuntimeError(f"unsupported Astral end effector: {spec.driver}")
    return actions


def _nero(profile: Profile, path: str, dry_run: bool):
    ns = profile.namespace
    xhand_cfg = _share("xhand_control_ros2", "config", "xhand_config.yaml")
    retarget_cfg = _share("xhand_retargeting", "config", "retargeting_params.yaml")
    actions = [_node("xhand_retargeting", "xhand_dex_retargeting_node",
                     "xhand_dex_retargeting", [retarget_cfg, {
                         "input_topic": "/hand_landmarks", "viz": False,
                         "dry_run": False, "require_clench_to_start": False,
                     }], [
                         (f"/hand_landmarks/{side}", f"{ns}/input/{side}/hand_landmarks")
                         for side in ("left", "right")
                     ] + [
                         (f"/{side}_hand/xhand_command", f"{ns}/legacy/{side}_xhand_candidate")
                         for side in ("left", "right")
                     ])]
    for side in ("left", "right"):
        actions.extend([
            _node("nexus_core", "nexus_nero_driver", f"nero_driver_{side}",
                  [{"profile_file": path, "side": side, "dry_run": dry_run}]),
            _node("nexus_core", "nexus_nero_teleop", f"nero_teleop_{side}",
                  [{"profile_file": path, "side": side}]),
            _node("nexus_core", "nexus_joint_bridge", f"xhand_bridge_{side}",
                  [{"profile_file": path, "component": f"{side}_ee",
                    "side": side, "adapter_mode": "xhand", "candidate_only": dry_run}]),
        ])
        if dry_run:
            actions.append(_node("nexus_core", "nexus_fake_driver", f"fake_{side}_ee",
                                 [{"profile_file": path, "component": f"{side}_ee"}]))
        else:
            actions.append(_node("xhand_control_ros2", "xhand_control_ros2_node",
                                 "xhand_control", [xhand_cfg, {
                                     "port_name": profile.raw["hardware"]["xhand_serial"][side],
                                     "update_rate": 100.0,
                                 }], namespace=f"{side}_hand"))
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

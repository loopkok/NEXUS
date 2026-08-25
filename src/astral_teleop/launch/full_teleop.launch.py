"""Whole-robot teleop: dual arms + left gripper + right Wuji hand.

One Quest3 mocap. Right Wuji input is quest3 or glove (not both on
hand_landmarks/right).

Usage:
  # Quest3 wrists + left pinch + Quest3 right hand
  ros2 launch astral_teleop full_teleop.launch.py \\
    with_arm_driver:=true with_hand_driver:=true \\
    right_hand_source:=quest3 retarget_backend:=wuji_retargeting

  # Same, but right Wuji from glove
  ros2 launch astral_teleop full_teleop.launch.py \\
    with_arm_driver:=true with_hand_driver:=true \\
    right_hand_source:=glove
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _opt(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _setup(context, *args, **kwargs):
    quest_pkg = get_package_share_directory("quest3_hand_mocap")
    teleop_pkg = get_package_share_directory("astral_arm_teleop")
    retarget_pkg = get_package_share_directory("wujihand_retargeting")
    glove_pkg = get_package_share_directory("wuji_glove")
    hand_ctrl_pkg = get_package_share_directory("wujihand_control")

    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    right_src = _opt(context, "right_hand_source").lower()
    if right_src not in ("quest3", "glove", "none"):
        raise RuntimeError("right_hand_source must be quest3|glove|none")

    backend = _opt(context, "retarget_backend")
    if not backend:
        backend = "wuji_retargeting" if right_src == "quest3" else "official"

    wuji_right = _opt(context, "wuji_lib_config_right")
    if not wuji_right and right_src == "quest3" and backend == "wuji_retargeting":
        wuji_right = os.path.join(
            retarget_pkg, "config", "retarget_wuji_lib_quest3_right.yaml"
        )

    mocap_extra = {
        "arm_side": "both",
        "viz": False,
        "landmark_preprocess": "raw",
        "enable_xhand_pinky_adapt": False,
        "publish_landmarks_left": True,
        "publish_landmarks_right": right_src != "glove",
    }
    protocol = _opt(context, "protocol")
    if protocol:
        mocap_extra["protocol"] = protocol
    convert = _opt(context, "convert_to_robot")
    if convert:
        mocap_extra["convert_to_robot"] = convert.lower() in ("true", "1", "yes")

    mocap = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        emulate_tty=True,
        parameters=[quest_cfg, mocap_extra],
    )

    arms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg, "launch", "astral_dual_arm_teleop.launch.py")
        ),
        launch_arguments={
            "with_mocap": "false",
            "with_driver": _opt(context, "with_arm_driver") or "false",
            "dry_run": _opt(context, "dry_run") or "false",
            "control_board_ip": _opt(context, "control_board_ip"),
            "solver_type": _opt(context, "solver_type"),
            "urdf_path": _opt(context, "urdf_path"),
            "protocol": protocol,
            "convert_to_robot": convert,
            "require_start_signal": _opt(context, "require_start_signal"),
        }.items(),
    )

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("astral_gripper_teleop"),
                "launch",
                "gripper_teleop.launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("with_gripper")),
        launch_arguments={"hand_side": "left"}.items(),
    )

    glove = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(glove_pkg, "launch", "wuji_glove_mocap.launch.py")
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration("right_hand_source"), "' == 'glove'"]
            )
        ),
        launch_arguments={
            "hand_side": "right",
            "sn": LaunchConfiguration("glove_sn"),
        }.items(),
    )

    use_wuji = right_src != "none"
    wuji = []
    if use_wuji:
        wuji.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        retarget_pkg, "launch", "wujihand_retarget.launch.py"
                    )
                ),
                launch_arguments={
                    "hand_side": "right",
                    "retarget_backend": backend,
                    "hand_model": _opt(context, "hand_model") or "wuji_hand",
                    "wuji_lib_config_right": wuji_right,
                }.items(),
            )
        )
        wuji.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        hand_ctrl_pkg, "launch", "wujihand_drivers.launch.py"
                    )
                ),
                condition=IfCondition(LaunchConfiguration("with_hand_driver")),
                launch_arguments={
                    "hand_side": "right",
                    "right_serial": LaunchConfiguration("right_serial"),
                    "filter_cutoff_freq": LaunchConfiguration("filter_cutoff_freq"),
                }.items(),
            )
        )

    return [mocap, arms, gripper, glove, *wuji]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "right_hand_source",
                default_value="quest3",
                description="quest3 | glove | none — right Wuji input",
            ),
            DeclareLaunchArgument(
                "retarget_backend",
                default_value="",
                description="empty → wuji_retargeting if quest3 else official",
            ),
            DeclareLaunchArgument(
                "hand_model",
                default_value=os.environ.get("GLOVE_HAND_MODEL", "wuji_hand"),
            ),
            DeclareLaunchArgument(
                "wuji_lib_config_right",
                default_value="",
                description="override wuji_retargeting yaml for the right hand",
            ),
            DeclareLaunchArgument(
                "glove_sn",
                default_value=os.environ.get("GLOVE_HAND_SN", ""),
            ),
            DeclareLaunchArgument("right_serial", default_value=""),
            DeclareLaunchArgument("filter_cutoff_freq", default_value="10.0"),
            DeclareLaunchArgument(
                "with_gripper",
                default_value="true",
                description="Left Quest pinch → left gripper",
            ),
            DeclareLaunchArgument(
                "with_arm_driver",
                default_value="false",
                description="astral_robot_control (arms + left gripper)",
            ),
            DeclareLaunchArgument(
                "with_hand_driver",
                default_value="false",
                description="wujihand_driver for the right Wuji hand",
            ),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument(
                "control_board_ip",
                default_value=os.environ.get("ASTRAL_BOARD_IP", "192.168.10.2"),
            ),
            DeclareLaunchArgument("protocol", default_value=""),
            DeclareLaunchArgument("convert_to_robot", default_value=""),
            DeclareLaunchArgument("solver_type", default_value=""),
            DeclareLaunchArgument("urdf_path", default_value=""),
            DeclareLaunchArgument(
                "require_start_signal",
                default_value="",
                description=(
                    "empty → yaml. true → wait for /teleop/start to capture vr_init "
                    "and arm (use after placing hand at initial pose)."
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )

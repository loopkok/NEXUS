"""Quest3 mocap + Astral dual-arm teleop (no hardware driver).

Usage:
  ros2 launch astral_quest_teleop astral_quest_teleop.launch.py \\
    dry_run:=true

  ros2 launch astral_quest_teleop astral_quest_teleop.launch.py \\
    solver_type:=urdf_numerical
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    teleop_pkg = get_package_share_directory("astral_quest_teleop")
    quest_pkg = get_package_share_directory("quest3_hand_mocap")
    default_cfg = os.path.join(teleop_pkg, "config", "astral_teleop.yaml")
    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_cfg),
            DeclareLaunchArgument(
                "solver_type",
                default_value="analytic_dh",
                description="analytic_dh | urdf_numerical",
            ),
            DeclareLaunchArgument("arm_side", default_value="both"),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("control_rate", default_value="150.0"),
            DeclareLaunchArgument(
                "protocol",
                default_value="tcp_wired",
                description="quest3: udp | tcp_wired | tcp_wireless",
            ),
            Node(
                package="quest3_hand_mocap",
                executable="quest3_udp_mocap",
                name="quest3_udp_mocap",
                output="screen",
                parameters=[
                    quest_cfg,
                    {
                        "protocol": LaunchConfiguration("protocol"),
                        "arm_side": "both",
                        "convert_to_robot": True,
                        "enable_xhand_pinky_adapt": False,
                        "landmark_preprocess": "raw",
                        "viz": False,
                    },
                ],
            ),
            Node(
                package="astral_quest_teleop",
                executable="astral_teleop_node",
                name="astral_teleop",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "solver_type": LaunchConfiguration("solver_type"),
                        "arm_side": LaunchConfiguration("arm_side"),
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "control_rate": ParameterValue(
                            LaunchConfiguration("control_rate"), value_type=float
                        ),
                    },
                ],
            ),
        ]
    )

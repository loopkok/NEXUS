"""Dual Astral arm teleop — Nero layout (2× IK + 2× teleop).

  quest3_udp_mocap
  ik_solver_left / ik_solver_right   (optional CPU isolation; teleop has in-process IK)
  astral_teleop_arm ×2               (left_base_link / right_base_link DH)
  astral_robot_control driver        (optional, with_driver:=true)

Usage:
  ros2 launch astral_quest_teleop astral_dual_arm_teleop.launch.py dry_run:=true
  ros2 launch astral_quest_teleop astral_dual_arm_teleop.launch.py \\
    with_driver:=true control_board_ip:=192.168.10.2
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    teleop_pkg = get_package_share_directory("astral_quest_teleop")
    control_pkg = get_package_share_directory("astral_robot_control")
    quest_pkg = get_package_share_directory("quest3_hand_mocap")
    cfg_l = os.path.join(teleop_pkg, "config", "astral_teleop_left.yaml")
    cfg_r = os.path.join(teleop_pkg, "config", "astral_teleop_right.yaml")
    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    drivers_launch = os.path.join(control_pkg, "launch", "astral_drivers.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument("protocol", default_value="tcp_wired"),
            DeclareLaunchArgument(
                "convert_to_robot",
                default_value="true",
                description="true=Unity→robot_world (X left Y back Z up)",
            ),
            DeclareLaunchArgument(
                "with_driver",
                default_value="false",
                description="Also launch astral_robot_control driver",
            ),
            DeclareLaunchArgument(
                "control_board_ip",
                default_value=os.environ.get("ASTRAL_BOARD_IP", "192.168.10.2"),
            ),
            DeclareLaunchArgument(
                "solver_type",
                default_value="analytic_dh",
                description="analytic_dh | urdf_numerical",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value="",
                description="For urdf_numerical; empty → astral_robot.pin.urdf",
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
                        "convert_to_robot": ParameterValue(
                            LaunchConfiguration("convert_to_robot"),
                            value_type=bool,
                        ),
                    },
                ],
            ),
            Node(
                package="astral_quest_teleop",
                executable="ik_solver_node",
                name="ik_solver_left",
                output="screen",
                parameters=[{"arm_side": "left", "solver_type": "analytic_dh"}],
            ),
            Node(
                package="astral_quest_teleop",
                executable="ik_solver_node",
                name="ik_solver_right",
                output="screen",
                parameters=[{"arm_side": "right", "solver_type": "analytic_dh"}],
            ),
            Node(
                package="astral_quest_teleop",
                executable="astral_teleop_arm_node",
                name="astral_teleop_left",
                output="screen",
                parameters=[
                    cfg_l,
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "solver_type": LaunchConfiguration("solver_type"),
                        "urdf_path": LaunchConfiguration("urdf_path"),
                    },
                ],
            ),
            Node(
                package="astral_quest_teleop",
                executable="astral_teleop_arm_node",
                name="astral_teleop_right",
                output="screen",
                parameters=[
                    cfg_r,
                    {
                        "dry_run": ParameterValue(
                            LaunchConfiguration("dry_run"), value_type=bool
                        ),
                        "solver_type": LaunchConfiguration("solver_type"),
                        "urdf_path": LaunchConfiguration("urdf_path"),
                    },
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(drivers_launch),
                condition=IfCondition(LaunchConfiguration("with_driver")),
                launch_arguments={
                    "dry_run": LaunchConfiguration("dry_run"),
                    "control_board_ip": LaunchConfiguration("control_board_ip"),
                }.items(),
            ),
        ]
    )

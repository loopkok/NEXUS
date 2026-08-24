"""Dual Astral arm teleop — 2× arm_node (+ optional driver).

  quest3_udp_mocap          (optional, with_mocap:=true)
  astral_arm_teleop_arm ×2
  astral_robot_control driver (optional, with_driver:=true)

Arm-only. Gripper / dexterous hand are composed by astral_teleop, not here.
Solver / protocol / convert_to_robot come from yaml unless you pass a launch override.

Usage:
  ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py dry_run:=true
  ros2 launch astral_arm_teleop astral_dual_arm_teleop.launch.py \\
    with_driver:=true control_board_ip:=192.168.10.2
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _opt(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _launch_setup(context, *args, **kwargs):
    teleop_pkg = get_package_share_directory("astral_arm_teleop")
    control_pkg = get_package_share_directory("astral_robot_control")
    quest_pkg = get_package_share_directory("quest3_hand_mocap")
    cfg_l = os.path.join(teleop_pkg, "config", "astral_arm_teleop_left.yaml")
    cfg_r = os.path.join(teleop_pkg, "config", "astral_arm_teleop_right.yaml")
    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    drivers_launch = os.path.join(control_pkg, "launch", "astral_drivers.launch.py")

    teleop_extra = {
        "dry_run": _opt(context, "dry_run").lower() in ("true", "1", "yes"),
    }
    solver_type = _opt(context, "solver_type")
    if solver_type:
        teleop_extra["solver_type"] = solver_type
    urdf_path = _opt(context, "urdf_path")
    if urdf_path:
        teleop_extra["urdf_path"] = urdf_path

    mocap_extra = {"arm_side": "both"}
    protocol = _opt(context, "protocol")
    if protocol:
        mocap_extra["protocol"] = protocol
    convert = _opt(context, "convert_to_robot")
    if convert:
        mocap_extra["convert_to_robot"] = convert.lower() in ("true", "1", "yes")

    with_mocap = _opt(context, "with_mocap").lower() in ("true", "1", "yes")
    actions = []
    if with_mocap:
        actions.append(
            Node(
                package="quest3_hand_mocap",
                executable="quest3_udp_mocap",
                name="quest3_udp_mocap",
                output="screen",
                parameters=[quest_cfg, mocap_extra],
            )
        )

    actions.extend(
        [
            Node(
                package="astral_arm_teleop",
                executable="ik_solver_node",
                name="ik_solver_left",
                output="screen",
                parameters=[{"arm_side": "left", "solver_type": "analytic_dh"}],
            ),
            Node(
                package="astral_arm_teleop",
                executable="ik_solver_node",
                name="ik_solver_right",
                output="screen",
                parameters=[{"arm_side": "right", "solver_type": "analytic_dh"}],
            ),
            Node(
                package="astral_arm_teleop",
                executable="astral_arm_teleop_node",
                name="astral_arm_teleop_left",
                output="screen",
                parameters=[cfg_l, teleop_extra],
            ),
            Node(
                package="astral_arm_teleop",
                executable="astral_arm_teleop_node",
                name="astral_arm_teleop_right",
                output="screen",
                parameters=[cfg_r, teleop_extra],
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
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "with_mocap",
                default_value="true",
                description="Start quest3_udp_mocap (false when a parent launch owns it)",
            ),
            DeclareLaunchArgument("dry_run", default_value="false"),
            DeclareLaunchArgument(
                "protocol",
                default_value="",
                description="empty → quest3_mocap.yaml",
            ),
            DeclareLaunchArgument(
                "convert_to_robot",
                default_value="",
                description="empty → yaml",
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
                default_value="",
                description="empty → astral_arm_teleop_{left,right}.yaml",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value="",
                description="empty → yaml / astral_robot.pin.urdf",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

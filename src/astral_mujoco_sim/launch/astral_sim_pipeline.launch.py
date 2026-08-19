"""Quest3 → Astral dual teleop → MuJoCo sim.

Do NOT start astral_robot_control alongside this launch.

Node config lives in yaml (not launch defaults):
  astral_teleop_{left,right}.yaml  — solver_type, smoothing, …
  quest3_mocap.yaml                — protocol, convert_to_robot, …
  astral_mujoco_sim.yaml           — viewer, mjcf, …

Launch arguments are optional overrides (empty → yaml).
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_astral_urdf() -> str:
    """Installed share → source tree → empty (teleop uses its own default)."""
    try:
        share = get_package_share_directory("astral_robot_description")
        p = os.path.join(share, "urdf", "astral_robot.pin.urdf")
        if os.path.isfile(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(
            os.path.join(
                here, "..", "..", "astral_robot_description", "urdf", "astral_robot.pin.urdf"
            )
        ),
        os.path.normpath(
            os.path.join(
                here,
                "..",
                "..",
                "..",
                "src",
                "astral_robot_description",
                "urdf",
                "astral_robot.pin.urdf",
            )
        ),
    ]
    for p in candidates:
        if os.path.isfile(p):
            return p
    return ""


def _opt(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context).strip()


def _launch_setup(context, *args, **kwargs):
    teleop_pkg = get_package_share_directory("astral_quest_teleop")
    sim_pkg = get_package_share_directory("astral_mujoco_sim")
    quest_pkg = get_package_share_directory("quest3_hand_mocap")

    cfg_l = os.path.join(teleop_pkg, "config", "astral_teleop_left.yaml")
    cfg_r = os.path.join(teleop_pkg, "config", "astral_teleop_right.yaml")
    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    sim_cfg = os.path.join(sim_pkg, "config", "astral_mujoco_sim.yaml")

    teleop_extra = {"dry_run": False}
    solver_type = _opt(context, "solver_type")
    if solver_type:
        teleop_extra["solver_type"] = solver_type
    urdf_arg = _opt(context, "urdf_path") or _resolve_astral_urdf()
    if urdf_arg:
        teleop_extra["urdf_path"] = urdf_arg

    mocap_extra = {"arm_side": "both"}
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
        parameters=[quest_cfg, mocap_extra],
    )

    sim_extra = {}
    viewer = _opt(context, "enable_viewer")
    if viewer:
        sim_extra["enable_viewer"] = viewer.lower() in ("true", "1", "yes")

    sim_params = [sim_cfg]
    if sim_extra:
        sim_params.append(sim_extra)

    return [
        mocap,
        Node(
            package="astral_quest_teleop",
            executable="astral_teleop_arm_node",
            name="astral_teleop_left",
            output="screen",
            parameters=[cfg_l, teleop_extra],
        ),
        Node(
            package="astral_quest_teleop",
            executable="astral_teleop_arm_node",
            name="astral_teleop_right",
            output="screen",
            parameters=[cfg_r, teleop_extra],
        ),
        Node(
            package="astral_mujoco_sim",
            executable="astral_mujoco_sim_node",
            name="astral_mujoco_sim_node",
            output="screen",
            parameters=sim_params,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "protocol",
                default_value="",
                description="empty → quest3_mocap.yaml; else udp | tcp_wired | tcp_wireless",
            ),
            DeclareLaunchArgument(
                "convert_to_robot",
                default_value="",
                description="empty → yaml; true=Unity → robot_world",
            ),
            DeclareLaunchArgument(
                "enable_viewer",
                default_value="",
                description="empty → astral_mujoco_sim.yaml",
            ),
            DeclareLaunchArgument(
                "solver_type",
                default_value="",
                description="empty → astral_teleop_{left,right}.yaml",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value="",
                description="empty → astral_robot.pin.urdf (still passed so DH↔URDF hot-switch works)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

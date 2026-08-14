"""Quest3 → Astral dual teleop → MuJoCo sim.

Do NOT start astral_robot_control alongside this launch.

solver_type:
  analytic_dh     — default (Nero closed-form, arm base)
  urdf_numerical  — Pinocchio LM on astral_robot.pin.urdf (matches MJCF)
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_astral_urdf() -> str:
    """Installed share → source tree → empty (teleop uses its own default)."""
    try:
        share = get_package_share_directory("astral_robot_description")
        p = os.path.join(share, "urdf", "astral_robot.pin.urdf")
        if os.path.isfile(p):
            return p
    except Exception:  # noqa: BLE001
        pass
    # workspace source fallback (package not built yet)
    here = os.path.dirname(os.path.abspath(__file__))
    # .../src/astral_mujoco_sim/launch → .../src/astral_robot_description/...
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


def _launch_setup(context, *args, **kwargs):
    teleop_pkg = get_package_share_directory("astral_quest_teleop")
    sim_pkg = get_package_share_directory("astral_mujoco_sim")
    quest_pkg = get_package_share_directory("quest3_hand_mocap")

    cfg_l = os.path.join(teleop_pkg, "config", "astral_teleop_left.yaml")
    cfg_r = os.path.join(teleop_pkg, "config", "astral_teleop_right.yaml")
    quest_cfg = os.path.join(quest_pkg, "config", "quest3_mocap.yaml")
    sim_cfg = os.path.join(sim_pkg, "config", "astral_mujoco_sim.yaml")

    solver_type = LaunchConfiguration("solver_type").perform(context)
    urdf_arg = LaunchConfiguration("urdf_path").perform(context).strip()
    if not urdf_arg and solver_type.strip().lower() in (
        "urdf_numerical",
        "urdf",
        "numerical",
    ):
        urdf_arg = _resolve_astral_urdf()
        if not urdf_arg:
            raise FileNotFoundError(
                "astral_robot.pin.urdf not found. Build the description package:\n"
                "  colcon build --packages-select astral_robot_description --symlink-install\n"
                "  source install/setup.bash\n"
                "Or pass urdf_path:=/absolute/path/to/astral_robot.pin.urdf"
            )

    teleop_extra = {
        "dry_run": False,
        "solver_type": solver_type,
    }
    if urdf_arg:
        teleop_extra["urdf_path"] = urdf_arg

    return [
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
            parameters=[
                sim_cfg,
                {
                    "enable_viewer": ParameterValue(
                        LaunchConfiguration("enable_viewer"), value_type=bool
                    ),
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("protocol", default_value="tcp_wired"),
            DeclareLaunchArgument(
                "convert_to_robot",
                default_value="true",
                description="true=Unity→robot_world (X left Y back Z up)",
            ),
            DeclareLaunchArgument("enable_viewer", default_value="true"),
            DeclareLaunchArgument(
                "solver_type",
                default_value="analytic_dh",
                description="analytic_dh | urdf_numerical",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value="",
                description="URDF for urdf_numerical; empty → astral_robot.pin.urdf",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )

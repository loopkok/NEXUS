"""Launch the profile-selected NEXUS MuJoCo driver by itself."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("profile_file", description="Absolute NEXUS profile JSON path"),
        DeclareLaunchArgument("enable_viewer", default_value="true"),
        DeclareLaunchArgument("realtime", default_value="true"),
        DeclareLaunchArgument("simulation_mode", default_value="kinematic"),
        Node(
            package="nero_mujoco_sim",
            executable="nero_mujoco_sim_node",
            name="nero_mujoco_sim",
            output="screen",
            parameters=[{
                "profile_file": LaunchConfiguration("profile_file"),
                "enable_viewer": LaunchConfiguration("enable_viewer"),
                "realtime": LaunchConfiguration("realtime"),
                "simulation_mode": LaunchConfiguration("simulation_mode"),
            }],
        ),
    ])

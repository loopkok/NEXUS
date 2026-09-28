#!/usr/bin/env python3
"""Launch file for data collection during teleoperation.

Launches the data collection node alongside the existing teleop system.
Use this in addition to teleop_full_dual.launch.py.

Usage:
    # 全系统采集（双臂双手三相机）
    ros2 launch nero_dual_data_collect data_collect.launch.py task_name:=my_task

    # 只采集左手+左臂+2个相机（去掉右手腕相机）
    ros2 launch nero_dual_data_collect data_collect.launch.py \
        task_name:=my_task collect_left:=true collect_right:=false \
        camera_configs:="[{'id':'cam_0','serial':'CP02653000VE','width':1280,'height':720,'fps':30},{'id':'cam_1','serial':'CV28460000FV','width':640,'height':480,'fps':30}]"
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("nero_dual_data_collect")
    config_path = os.path.join(pkg_dir, "config", "data_collect.yaml")

    # ---- Launch arguments ----
    task_name_arg = DeclareLaunchArgument(
        "task_name", default_value="default_task",
        description="Task name for organizing collected data."
    )
    data_path_arg = DeclareLaunchArgument(
        "data_base_path", default_value="~/xnero_data",
        description="Root directory for storing collected data."
    )
    collect_left_arg = DeclareLaunchArgument(
        "collect_left", default_value="true",
        description="Collect left arm + left hand data."
    )
    collect_right_arg = DeclareLaunchArgument(
        "collect_right", default_value="true",
        description="Collect right arm + right hand data."
    )
    use_cam_0_arg = DeclareLaunchArgument(
        "use_cam_0", default_value="true",
        description="Collect overhead Gemini 335 (cam_0)."
    )
    use_cam_1_arg = DeclareLaunchArgument(
        "use_cam_1", default_value="true",
        description="Collect left wrist Gemini 305 (cam_1)."
    )
    use_cam_2_arg = DeclareLaunchArgument(
        "use_cam_2", default_value="true",
        description="Collect right wrist Gemini 305 (cam_2)."
    )

    # ---- Data collection node ----
    data_collect_node = Node(
        package="nero_dual_data_collect",
        executable="data_collect_node",
        name="data_collect_node",
        output="screen",
        parameters=[
            config_path,
            {
                "task_name": LaunchConfiguration("task_name"),
                "data_base_path": LaunchConfiguration("data_base_path"),
                "collect_left": LaunchConfiguration("collect_left"),
                "collect_right": LaunchConfiguration("collect_right"),
                "use_cam_0": LaunchConfiguration("use_cam_0"),
                "use_cam_1": LaunchConfiguration("use_cam_1"),
                "use_cam_2": LaunchConfiguration("use_cam_2"),
            },
        ],
    )

    return LaunchDescription([
        task_name_arg,
        data_path_arg,
        collect_left_arg,
        collect_right_arg,
        use_cam_0_arg,
        use_cam_1_arg,
        use_cam_2_arg,
        data_collect_node,
    ])

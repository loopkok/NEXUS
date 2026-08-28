"""启动数据采集：采集节点 + 键盘控制器。

前置条件：
  1. 遥操作链路已运行（astral_teleop / astral_robot_control / ...）
  2. quest3_video_streamer 已运行且 collect_tap: true（默认开）

用法：
  ros2 launch astral_data_collect data_collect.launch.py \
      session:=pick_place end_effector_right:=wuji include_head:=true

schema 相关参数会直接覆盖 config/data_collect.yaml 的同名字段。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    share = get_package_share_directory("astral_data_collect")
    default_params = os.path.join(share, "config", "data_collect.yaml")

    args = [
        DeclareLaunchArgument("session", default_value="default_task"),
        DeclareLaunchArgument("save_root", default_value="~/astral_data"),
        DeclareLaunchArgument("arms", default_value="left,right",
                              description="启用的臂侧，逗号分隔：left,right / left / right"),
        DeclareLaunchArgument("end_effector_left", default_value="gripper"),
        DeclareLaunchArgument("end_effector_right", default_value="gripper"),
        DeclareLaunchArgument("include_waist", default_value="false"),
        DeclareLaunchArgument("include_head", default_value="false"),
        DeclareLaunchArgument("dataset_fps", default_value="30"),
        DeclareLaunchArgument("action_source", default_value="next_state"),
        DeclareLaunchArgument("keyboard", default_value="true",
                              description="是否同启键盘控制器节点"),
    ]

    collect_node = Node(
        package="astral_data_collect",
        executable="data_collect_node",
        name="data_collect",
        output="screen",
        parameters=[
            default_params,
            {
                "session": LaunchConfiguration("session"),
                "save_root": LaunchConfiguration("save_root"),
                "arms": LaunchConfiguration("arms"),
                "end_effector_left": LaunchConfiguration("end_effector_left"),
                "end_effector_right": LaunchConfiguration("end_effector_right"),
                "include_waist": LaunchConfiguration("include_waist"),
                "include_head": LaunchConfiguration("include_head"),
                "dataset_fps": LaunchConfiguration("dataset_fps"),
                "action_source": LaunchConfiguration("action_source"),
            },
        ],
    )

    keyboard_node = Node(
        package="astral_data_collect",
        executable="keyboard_controller",
        name="data_collect_keyboard",
        output="screen",
        condition=IfCondition(LaunchConfiguration("keyboard")),
    )

    return LaunchDescription([*args, collect_node, keyboard_node])

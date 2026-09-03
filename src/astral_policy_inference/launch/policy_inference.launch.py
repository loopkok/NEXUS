"""astral_policy_inference launch: policy_node + optional policy_keyboard.

用法：
  ros2 launch astral_policy_inference policy_inference.launch.py \
      params_file:=<path> backend_type:=openpi host:=192.168.x.x \
      keyboard:=true

顶层 robot/schema 参数通过 `policy.robot.<key>` 形如 --ros-args -p arms:="['left']"
在 launch 里用 extra_args 传入（见 config yaml 注释）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_params() -> str:
    return os.path.join(
        get_package_share_directory("astral_policy_inference"),
        "config",
        "policy_inference.yaml",
    )


def _node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    backend_type = LaunchConfiguration("backend_type").perform(context)
    host = LaunchConfiguration("host").perform(context)
    port = LaunchConfiguration("port").perform(context)
    checkpoint_dir = LaunchConfiguration("checkpoint_dir").perform(context)
    cmd_topic = LaunchConfiguration("cmd_topic").perform(context)
    parameters = [params_file]
    if backend_type:
        parameters.append(
            {
                "backend_type": backend_type,
                "host": host,
                "port": int(port),
                "checkpoint_dir": checkpoint_dir,
                "cmd_topic": cmd_topic,
            }
        )
    node = Node(
        package="astral_policy_inference",
        executable="policy_node",
        name="policy_node",
        output="screen",
        parameters=parameters,
    )
    kb = LaunchConfiguration("keyboard").perform(context).lower() in ("1", "true")
    actions = [node]
    if kb:
        actions.append(
            Node(
                package="astral_policy_inference",
                executable="policy_keyboard",
                name="policy_keyboard",
                output="screen",
                parameters=[{"cmd_topic": cmd_topic, "state_topic": LaunchConfiguration("state_topic")}],
            )
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=_default_params()),
            DeclareLaunchArgument("backend_type", default_value=""),
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="8000"),
            DeclareLaunchArgument("checkpoint_dir", default_value=""),
            DeclareLaunchArgument("keyboard", default_value="false"),
            DeclareLaunchArgument("cmd_topic", default_value="/policy_inference/cmd"),
            DeclareLaunchArgument("state_topic", default_value="/policy_inference/state"),
            OpaqueFunction(function=_node),
        ]
    )

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_udp_port = "9000"
    default_arm_side = "both"
    default_right_serial = "/dev/ttyUSB1"
    default_left_serial = "/dev/ttyUSB0"

    udp_port = LaunchConfiguration("udp_port")
    protocol = LaunchConfiguration("protocol")
    arm_side = LaunchConfiguration("arm_side")
    right_serial = LaunchConfiguration("right_serial")
    left_serial = LaunchConfiguration("left_serial")
    enable_thumb_fix = LaunchConfiguration("enable_thumb_fix")

    retargeting_config = os.path.join(
        get_package_share_directory("xhand_retargeting"),
        "config", "retargeting_params.yaml",
    )

    xhand_config_path = os.path.join(
        get_package_share_directory("xhand_control_ros2"),
        "config",
        "xhand_config.yaml",
    )

    quest3_udp_mocap_node = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        emulate_tty=True,
        parameters=[retargeting_config],
    )

    xhand_dex_retargeting_node = Node(
        package="xhand_retargeting",
        executable="xhand_dex_retargeting_node",
        name="xhand_dex_retargeting_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            retargeting_config,
            {"enable_thumb_fix": enable_thumb_fix},
        ],
    )

    # 根据 arm_side 自动启用对应的手：left→左手，right→右手，both→双手
    right_hand_node = Node(
        package="xhand_control_ros2",
        executable="xhand_control_ros2_node",
        name="xhand_control_node",
        namespace="right_hand",
        output="screen",
        condition=IfCondition(PythonExpression(["'", arm_side, "' != 'left'"])),
        parameters=[
            xhand_config_path,
            {"port_name": right_serial},
        ],
    )

    left_hand_node = Node(
        package="xhand_control_ros2",
        executable="xhand_control_ros2_node",
        name="xhand_control_node",
        namespace="left_hand",
        output="screen",
        condition=IfCondition(PythonExpression(["'", arm_side, "' != 'right'"])),
        parameters=[
            xhand_config_path,
            {"port_name": left_serial},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("udp_port", default_value=default_udp_port),
        DeclareLaunchArgument("protocol", default_value="tcp_wired"),
        DeclareLaunchArgument("arm_side", default_value=default_arm_side),
        DeclareLaunchArgument("right_serial", default_value=default_right_serial),
        DeclareLaunchArgument("left_serial", default_value=default_left_serial),
        DeclareLaunchArgument("enable_thumb_fix", default_value="false"),
        DeclareLaunchArgument("require_clench_to_start", default_value="true"),
        quest3_udp_mocap_node,
        xhand_dex_retargeting_node,
        right_hand_node,
        left_hand_node,
    ])

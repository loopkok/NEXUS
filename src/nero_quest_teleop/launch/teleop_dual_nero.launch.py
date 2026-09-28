import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nero_teleop_dir = get_package_share_directory("nero_quest_teleop")

    # ---- Launch arguments ----
    left_can_arg = DeclareLaunchArgument(
        "left_can", default_value="can_nero_left",
        description="CAN channel for the left Nero arm",
    )
    right_can_arg = DeclareLaunchArgument(
        "right_can", default_value="can_nero_right",
        description="CAN channel for the right Nero arm",
    )
    control_rate_arg = DeclareLaunchArgument(
        "control_rate", default_value="100.0",
        description="Control loop rate in Hz",
    )
    motion_scale_arg = DeclareLaunchArgument(
        "motion_scale", default_value="0.65",
        description="VR-to-robot position scale factor",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="Whether to launch RViz",
    )
    udp_port_arg = DeclareLaunchArgument(
            "udp_port", default_value="9000",
        description="UDP port for Quest3 hand mocap data",
    )

    config_file_left = os.path.join(nero_teleop_dir, "config", "nero_teleop_left.yaml")
    config_file_right = os.path.join(nero_teleop_dir, "config", "nero_teleop_right.yaml")

    # ---- Quest3 mocap node (publishes BOTH left and right wrist) ----
    quest3_mocap_node = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        parameters=[retargeting_config],
    )

    # ---- IK solver nodes ----
    ik_solver_left = Node(
        package="nero_quest_teleop",
        executable="ik_solver_node",
        name="ik_solver_left",
        output="screen",
        parameters=[config_file_left],
    )

    ik_solver_right = Node(
        package="nero_quest_teleop",
        executable="ik_solver_node",
        name="ik_solver_right",
        output="screen",
        parameters=[config_file_right],
    )

    # ---- Left arm teleop ----
    left_arm_node = Node(
        package="nero_quest_teleop",
        executable="nero_teleop_node",
        name="nero_teleop_left",
        output="screen",
        parameters=[
            config_file_left,
            {
            },
        ],
    )

    # ---- Right arm teleop ----
    right_arm_node = Node(
        package="nero_quest_teleop",
        executable="nero_teleop_node",
        name="nero_teleop_right",
        output="screen",
        parameters=[
            config_file_right,
            {
            },
        ],
    )

    # ---- RViz ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    hts_reminder = LogInfo(
        msg="\n"
            "\033[1;33m"
            "╔══════════════════════════════════════════════════════════╗\n"
            "║  ⚠️  Quest 3 HTS 双手初始摆放姿势                          ║\n"
            "╠══════════════════════════════════════════════════════════╣\n"
            "║                                                          ║\n"
            "║  左手：整体向身体中间倾斜，手掌朝前偏内，肘部内收           ║\n"
            "║  右手：向右侧倾斜，手掌指向斜右侧，手腕指向身体中间         ║\n"
            "║                                                          ║\n"
            "║  简单记：左手往中间倒，右手向外斜但手腕指向中间            ║\n"
            "║                                                          ║\n"
            "╚══════════════════════════════════════════════════════════╝"
            "\033[0m"
    )

    return LaunchDescription([
        left_can_arg,
        right_can_arg,
        control_rate_arg,
        motion_scale_arg,
        rviz_arg,
        udp_port_arg,
            hts_reminder,
        quest3_mocap_node,
        ik_solver_left,
        #ik_solver_right,
        left_arm_node,
        #right_arm_node,
        #rviz_node,
    ])

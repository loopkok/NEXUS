#!/usr/bin/env python3
"""Full right-side teleop: Quest3 + right XHand + right Nero arm.

Launches:
  1. quest3_udp_mocap          — VR hand/wrist data (both hands from single UDP stream)
  2. xhand_control_ros2_node   — right XHand serial driver
  3. xhand_dex_retargeting_node — landmark → joint retargeting
  4. nero_teleop_node          — right arm teleop
  5. ik_solver_node            — IK solver for right arm
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nero_teleop_dir = get_package_share_directory("nero_quest_teleop")
    xhand_retargeting_dir = get_package_share_directory("xhand_retargeting")
    config_right = os.path.join(nero_teleop_dir, "config", "nero_teleop_right.yaml")
    retargeting_config = os.path.join(xhand_retargeting_dir, "config", "retargeting_params.yaml")

    # ---- Launch arguments ----
    udp_port_arg = DeclareLaunchArgument("udp_port", default_value="9000")
    protocol_arg = DeclareLaunchArgument("protocol", default_value="tcp_wired")
    right_can_arg = DeclareLaunchArgument("right_can", default_value="can_nero_right")
    right_serial_arg = DeclareLaunchArgument("right_serial", default_value="/dev/ttyUSB1")
    smoothing_alpha_arg = DeclareLaunchArgument("smoothing_alpha", default_value="0.7")
    enable_thumb_fix_arg = DeclareLaunchArgument("enable_thumb_fix", default_value="true")
    clench_start_arg = DeclareLaunchArgument("require_clench_to_start", default_value="true")
    dry_run_arg = DeclareLaunchArgument("dry_run", default_value="false")

    # ---- Quest3 UDP mocap (publishes both hands) ----
    quest3_node = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        parameters=[retargeting_config],
    )

    # ---- Right XHand driver ----
    xhand_right = Node(
        package="xhand_control_ros2",
        executable="xhand_control_ros2_node",
        name="xhand_control_right",
        namespace="right_hand",
        output="screen",
        parameters=[{
            "port_name": LaunchConfiguration("right_serial"),
            "update_rate": 100.0,
        }],
    )

    # ---- XHand retargeting ----
    retarget_node = Node(
        package="xhand_retargeting",
        executable="xhand_dex_retargeting_node",
        name="xhand_dex_retargeting",
        output="screen",
        parameters=[
            retargeting_config,
            {
                "smoothing_alpha": LaunchConfiguration("smoothing_alpha"),
                "enable_thumb_fix": LaunchConfiguration("enable_thumb_fix"),
                "require_clench_to_start": LaunchConfiguration("require_clench_to_start"),
            },
        ],
    )

    # ---- IK solver (right arm) ----
    ik_solver_node = Node(
        package="nero_quest_teleop",
        executable="ik_solver_node",
        name="ik_solver_right",
        output="screen",
        parameters=[config_right],
    )

    # ---- Right arm teleop ----
    nero_right = Node(
        package="nero_quest_teleop",
        executable="nero_teleop_node",
        name="nero_teleop_right",
        output="screen",
        parameters=[
            config_right,
            {
            },
        ],
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
        udp_port_arg, protocol_arg, right_can_arg, right_serial_arg,
        control_rate_arg, motion_scale_arg, pos_smoothing_arg,
        max_joint_vel_arg, solver_type_arg,
        smoothing_alpha_arg, enable_thumb_fix_arg, clench_start_arg, dry_run_arg,
        hts_reminder,
        quest3_node, xhand_right, retarget_node,
        ik_solver_node, nero_right,
    ])

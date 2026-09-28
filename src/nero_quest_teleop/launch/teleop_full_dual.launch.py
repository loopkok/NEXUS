#!/usr/bin/env python3
"""Full dual-side teleop: Quest3 + both XHands + both Nero arms.

Launches:
  1. quest3_udp_mocap           — VR hand/wrist data (single UDP stream, both hands)
  2. xhand_control_ros2_node ×2 — left + right XHand serial drivers
  3. xhand_dex_retargeting_node — landmark → joint retargeting (both hands)
  4. nero_teleop_node ×2        — left + right arm teleop
  5. ik_solver_node ×2          — IK solvers for both arms

Startup order respects dependencies:
  Quest3 → Retargeting → Hand drivers
  IK solvers → Arm nodes
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nero_teleop_dir = get_package_share_directory("nero_quest_teleop")
    xhand_retargeting_dir = get_package_share_directory("xhand_retargeting")
    config_left = os.path.join(nero_teleop_dir, "config", "nero_teleop_left.yaml")
    config_right = os.path.join(nero_teleop_dir, "config", "nero_teleop_right.yaml")
    retargeting_config = os.path.join(xhand_retargeting_dir, "config", "retargeting_params.yaml")

    # ---- Launch arguments ----
    udp_port_arg = DeclareLaunchArgument("udp_port", default_value="9000")
    protocol_arg = DeclareLaunchArgument("protocol", default_value="tcp_wired")
    left_can_arg = DeclareLaunchArgument("left_can", default_value="can_nero_left")
    right_can_arg = DeclareLaunchArgument("right_can", default_value="can_nero_right")
    left_serial_arg = DeclareLaunchArgument("left_serial", default_value="/dev/ttyUSB0")
    right_serial_arg = DeclareLaunchArgument("right_serial", default_value="/dev/ttyUSB1")
    control_rate_arg = DeclareLaunchArgument("control_rate", default_value="100.0")
    motion_scale_arg = DeclareLaunchArgument("motion_scale", default_value="0.65")
    pos_smoothing_arg = DeclareLaunchArgument("pos_smoothing", default_value="0.5")
    max_joint_vel_arg = DeclareLaunchArgument("max_joint_vel", default_value="0.05")
    solver_type_arg = DeclareLaunchArgument("solver_type", default_value="analytic_dh")
    smoothing_alpha_arg = DeclareLaunchArgument("smoothing_alpha", default_value="0.7")
    enable_thumb_fix_arg = DeclareLaunchArgument("enable_thumb_fix", default_value="false")
    clench_start_arg = DeclareLaunchArgument("require_clench_to_start", default_value="true")
    rviz_arg = DeclareLaunchArgument("rviz", default_value="false")

    # ---- Quest3 UDP mocap (publishes BOTH hands) ----
    quest3_node = Node(
        package="quest3_hand_mocap",
        executable="quest3_udp_mocap",
        name="quest3_udp_mocap",
        output="screen",
        parameters=[retargeting_config],
    )

    # ---- Left XHand driver ----
    xhand_left = Node(
        package="xhand_control_ros2",
        executable="xhand_control_ros2_node",
        name="xhand_control_left",
        namespace="left_hand",
        output="screen",
        parameters=[{
            "port_name": LaunchConfiguration("left_serial"),
            "update_rate": 100.0,
        }],
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

    # ---- IK solver (left arm) ----
    ik_solver_left = Node(
        package="nero_quest_teleop",
        executable="ik_solver_node",
        name="ik_solver_left",
        output="screen",
        parameters=[config_left],
    )

    # ---- IK solver (right arm) ----
    ik_solver_right = Node(
        package="nero_quest_teleop",
        executable="ik_solver_node",
        name="ik_solver_right",
        output="screen",
        parameters=[config_right],
    )

    # ---- Left arm teleop ----
    nero_left = Node(
        package="nero_quest_teleop",
        executable="nero_teleop_node",
        name="nero_teleop_left",
        output="screen",
        parameters=[
            config_left,
            {
            },
        ],
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

    # ---- RViz (optional) ----
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    # ---- HTS posture reminder ----
    hts_reminder = [
        LogInfo(
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
        ),
    ]

    return LaunchDescription([
        # Arguments
        udp_port_arg, protocol_arg, left_can_arg, right_can_arg,
            left_serial_arg, right_serial_arg,
        control_rate_arg, motion_scale_arg, pos_smoothing_arg,
        max_joint_vel_arg, solver_type_arg,
        smoothing_alpha_arg, enable_thumb_fix_arg,
        clench_start_arg, rviz_arg,
        # HTS posture reminder
        *hts_reminder,
        # Nodes (order: data source → processing → control)
        quest3_node,
        xhand_left, 
        xhand_right,
        retarget_node,
        ik_solver_left, 
        ik_solver_right,
        nero_left, 
        nero_right,
        #rviz_node,
    ])

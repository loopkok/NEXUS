import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('xhand_control_ros2'),
        'config',
        'xhand_config.yaml'
    )

    # 左手节点
    left_hand_node = Node(
        package='xhand_control_ros2',
        executable='xhand_control_ros2_node',
        name='xhand_control_node',
        namespace='left_hand',   # 左手的命名空间
        output='screen',
        parameters=[
            config_file_path,
            {'port_name': '/dev/ttyUSB0'} # 指定左手串口
        ],
    )

    # 右手节点
    right_hand_node = Node(
        package='xhand_control_ros2',
        executable='xhand_control_ros2_node',
        name='xhand_control_node',
        namespace='right_hand',  # 右手的命名空间
        output='screen',
        parameters=[
            config_file_path,
            {'port_name': '/dev/ttyUSB1'} # 指定右手串口
        ],
    )

    return LaunchDescription([
        left_hand_node,
        right_hand_node
    ])

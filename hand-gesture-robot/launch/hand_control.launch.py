#!/usr/bin/env python3
"""
Launch file for hand gesture control node.

This launch file starts the hand recognition node that:
1. Captures video from webcam
2. Detects hand gestures using MediaPipe
3. Publishes velocity commands to /cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_hand_gesture = get_package_share_directory('hand_gesture_robot')
    config_file = os.path.join(pkg_hand_gesture, 'config', 'robot_params.yaml')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_id = LaunchConfiguration('camera_id', default='0')
    show_preview = LaunchConfiguration('show_preview', default='true')
    max_linear_vel = LaunchConfiguration('max_linear_vel', default='0.22')
    max_angular_vel = LaunchConfiguration('max_angular_vel', default='2.84')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true (set to false for real robot)'
    )

    declare_camera_id = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Camera device ID (usually 0 for built-in webcam)'
    )

    declare_show_preview = DeclareLaunchArgument(
        'show_preview',
        default_value='true',
        description='Show camera preview window with gesture visualization'
    )

    declare_max_linear_vel = DeclareLaunchArgument(
        'max_linear_vel',
        default_value='0.22',
        description='Maximum linear velocity in m/s'
    )

    declare_max_angular_vel = DeclareLaunchArgument(
        'max_angular_vel',
        default_value='2.84',
        description='Maximum angular velocity in rad/s'
    )

    # Hand drive node
    hand_drive_node = Node(
        package='hand_recognition',
        executable='hand_drive_node',
        name='hand_drive_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_id': camera_id,
            'show_preview': show_preview,
            'max_linear_vel': max_linear_vel,
            'max_angular_vel': max_angular_vel,
            'publish_rate': 15.0,
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
            ('drive_enabled', '/drive_enabled'),
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_camera_id,
        declare_show_preview,
        declare_max_linear_vel,
        declare_max_angular_vel,
        hand_drive_node,
    ])


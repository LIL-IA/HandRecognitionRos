#!/usr/bin/env python3
"""
Launch file for the complete hand gesture controlled robot system.

This launch file starts the hand gesture recognition node that publishes
velocity commands to /cmd_vel topic for robot control.

Usage:
    ros2 launch hand_gesture_robot full_system.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    # Get package directories
    pkg_hand_gesture = get_package_share_directory('hand_gesture_robot')

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

    # Include hand control launch
    hand_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_hand_gesture,
                'launch',
                'hand_control.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'camera_id': camera_id,
            'show_preview': show_preview,
            'max_linear_vel': max_linear_vel,
            'max_angular_vel': max_angular_vel,
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_camera_id,
        declare_show_preview,
        declare_max_linear_vel,
        declare_max_angular_vel,
        hand_control_launch,
    ])


#!/usr/bin/env python3
"""Khởi chạy mô phỏng và node bám quỹ đạo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory('ackermann_description')
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('gz_args', default_value='-r empty.sdf'),
        DeclareLaunchArgument('trajectory', default_value='circle'),
        DeclareLaunchArgument('radius', default_value='1.0'),
        DeclareLaunchArgument('speed', default_value='0.12'),
        DeclareLaunchArgument('laps', default_value='1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'display.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items(),
        ),
        Node(
            package='ackermann_description',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen',
            parameters=[
                os.path.join(package, 'config', 'trajectory.yaml'),
                {
                    'trajectory': ParameterValue(LaunchConfiguration('trajectory'), value_type=str),
                    'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
                    'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
                    'laps': ParameterValue(LaunchConfiguration('laps'), value_type=int),
                },
            ],
        ),
    ])

#!/usr/bin/env python3
"""Khởi chạy mô phỏng tùy chọn và bộ bám quỹ đạo."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = get_package_share_directory('ackermann_bringup')
    navigation = get_package_share_directory('ackermann_navigation')
    arguments = [('start_sim', 'true'),
                 ('gz_args', '-r ' + os.path.join(package, 'worlds', 'gps_world.sdf')),
                 ('trajectory', 'circle'),
                 ('radius', '1.0'), ('speed', '0.12'), ('laps', '1'), ('use_sim_time', 'true')]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in arguments],
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'trajectory.launch.py')),
            launch_arguments={name: LaunchConfiguration(name) for name in
                              ['trajectory', 'radius', 'speed', 'laps', 'use_sim_time']}.items()),
    ])

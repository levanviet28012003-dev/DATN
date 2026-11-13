#!/usr/bin/env python3
"""Khởi chạy bộ bám quỹ đạo, dùng nguồn /odom có sẵn."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory('ackermann_navigation')
    arguments = [('use_sim_time', 'true'), ('trajectory', 'circle'), ('radius', '1.0'),
                 ('speed', '0.12'), ('laps', '1')]
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in arguments],
        Node(package='ackermann_navigation', executable='trajectory_controller',
             name='trajectory_controller', output='screen', parameters=[
                 os.path.join(package, 'config', 'trajectory.yaml'), {
                     'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                     'trajectory': ParameterValue(LaunchConfiguration('trajectory'), value_type=str),
                     'radius': ParameterValue(LaunchConfiguration('radius'), value_type=float),
                     'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
                     'laps': ParameterValue(LaunchConfiguration('laps'), value_type=int),
                 }]),
    ])

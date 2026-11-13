#!/usr/bin/env python3
"""Khởi chạy map_server, AMCL và quản lý lifecycle định vị."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory('ackermann_localization')
    params = os.path.join(package, 'config', 'amcl.yaml')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(package, 'maps', 'nav_map.yaml')),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, clock, {'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, clock]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[clock, {
                 'autostart': True, 'node_names': ['map_server', 'amcl'],
             }]),
    ])

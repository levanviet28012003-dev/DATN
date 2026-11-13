#!/usr/bin/env python3
"""Khởi chạy Gazebo, mô hình xe và bridge ROS 2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory('ackermann_bringup')
    description = get_package_share_directory('ackermann_description')
    gazebo = get_package_share_directory('ros_gz_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r empty.sdf',
                              description='Tham số Gazebo; thêm -s để chạy không giao diện.'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', os.path.dirname(description)
                               + os.pathsep + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(description)
                               + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()),
        Node(package='ros_gz_sim', executable='create', name='spawn_ackermann', output='screen',
             arguments=['-topic', 'robot_description', '-name', 'ackermann_car',
                        '-x', '0.0', '-y', '0.0', '-z', '0.04']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
             output='screen', parameters=[{
                 'config_file': os.path.join(package, 'config', 'bridge.yaml'), 'use_sim_time': True,
             }]),
    ])

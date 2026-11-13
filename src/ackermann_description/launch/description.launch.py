#!/usr/bin/env python3
"""Nạp URDF và xuất TF của xe; dùng chung cho mô phỏng và xe thật."""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory('ackermann_description')
    xacro_file = os.path.join(package, 'urdf', 'ackermann.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        LogInfo(msg=['Loading coloured STL model: ', xacro_file]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{
                 'robot_description': robot_description,
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])

#!/usr/bin/env python3
"""Mở RViz hiển thị số GPS; mặc định dùng Gazebo đang chạy."""
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
    package = get_package_share_directory('ackermann_bringup')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    # Terminal VS Code Snap có thể mang đường dẫn GTK không tương thích với RViz.
    rviz_environment = {key: '' for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR')
                        if '/snap/' in os.environ.get(key, '')}
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='false',
                              description='Mở cả Gazebo khi chưa chạy mô phỏng.'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Dùng đồng hồ mô phỏng Gazebo.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Mở cửa sổ RViz xem tọa độ GPS.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim'))),
        Node(package='ackermann_sensors', executable='gps_visualizer', name='gps_visualizer',
             output='screen', parameters=[clock]),
        Node(package='rviz2', executable='rviz2', name='gps_rviz', output='screen',
             arguments=['-d', os.path.join(package, 'rviz', 'gps.rviz')],
             parameters=[clock], additional_env=rviz_environment,
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

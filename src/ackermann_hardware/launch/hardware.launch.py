#!/usr/bin/env python3
"""Chạy cầu nối UART giữa Pi4 và STM32."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ackermann_hardware'),
                          'config', 'stm32_bridge.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial0',
                              description='Cổng UART nối STM32'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel',
                              description='Topic lệnh vận tốc đưa xuống STM32'),
        Node(
            package='ackermann_hardware',
            executable='stm32_bridge',
            name='stm32_bridge',
            output='screen',
            parameters=[params, {'port': LaunchConfiguration('port')}],
            remappings=[('cmd_vel', LaunchConfiguration('cmd_vel_topic'))],
        ),
    ])

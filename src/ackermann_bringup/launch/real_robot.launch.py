#!/usr/bin/env python3
"""Khung khởi chạy xe thật; hiện chỉ xuất mô hình, chưa có UART và driver cảm biến."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launches = [('ackermann_description', 'description.launch.py'),
                ('ackermann_hardware', 'hardware.launch.py'),
                ('ackermann_sensors', 'sensors.launch.py')]
    return LaunchDescription([
        LogInfo(msg='Khung xe thật: hiện chỉ có robot_state_publisher; chưa nhận lệnh điều khiển STM32.'),
        *[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory(package), 'launch', filename)),
            launch_arguments={'use_sim_time': 'false'}.items()) for package, filename in launches],
    ])

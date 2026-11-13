#!/usr/bin/env python3
"""Khung khởi chạy giao tiếp UART STM32; chưa có node phần cứng."""
from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='[ackermann_hardware] Chưa triển khai giao tiếp UART STM32; launch này chưa kết nối phần cứng.'),
    ])

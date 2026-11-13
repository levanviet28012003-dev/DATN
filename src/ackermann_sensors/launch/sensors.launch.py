#!/usr/bin/env python3
"""Khung khởi chạy driver cảm biến thật; chưa có node phần cứng."""
from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='[ackermann_sensors] Chưa triển khai driver cảm biến thật; launch này chưa kết nối phần cứng.'),
    ])

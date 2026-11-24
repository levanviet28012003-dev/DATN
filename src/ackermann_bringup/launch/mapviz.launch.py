#!/usr/bin/env python3
"""Mapviz trên nền ảnh vệ tinh, gốc local_xy cố định tại mốc của world.

Trên mô phỏng, bản đồ SLAM (frame map) bắt đầu tại mốc world, hướng Đông, nên initialize_origin
đặt đúng điểm đó để ảnh vệ tinh, GPS và vị trí xe chồng khớp nhau. Trên xe thật, frame map của SLAM
không gắn với hướng Bắc; việc khớp GPS với map thuộc GPS Waypoint Converter (chưa có).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    latitude = float(LaunchConfiguration('latitude').perform(context))
    longitude = float(LaunchConfiguration('longitude').perform(context))
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    return [
        Node(package='swri_transform_util', executable='initialize_origin.py', name='initialize_origin',
             output='screen', parameters=[{
                 'use_sim_time': use_sim_time,
                 'local_xy_frame': 'map',
                 'local_xy_origin': 'datum',
                 # [vĩ độ, kinh độ, độ cao, hướng trục X theo ENU (độ)]
                 'local_xy_origins': [latitude, longitude, 0.0, 0.0],
             }], remappings=[('fix', '/gps/fix')]),
        Node(package='mapviz', executable='mapviz', name='mapviz', output='screen',
             parameters=[{'use_sim_time': use_sim_time,
                          'config': LaunchConfiguration('config').perform(context)}]),
    ]


def generate_launch_description():
    package = get_package_share_directory('ackermann_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('latitude', description='Vĩ độ gốc frame map (datum).'),
        DeclareLaunchArgument('longitude', description='Kinh độ gốc frame map (datum).'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('config', default_value=os.path.join(package, 'config', 'mapviz_outdoor.mvc')),
        OpaqueFunction(function=launch_nodes),
    ])

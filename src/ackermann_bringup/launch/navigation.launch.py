#!/usr/bin/env python3
"""Mô phỏng sân trong phòng với kiến trúc SLAM, Nav2, RViz và công cụ thả vật cản.

  Gazebo ─► sim_sensors (/wheel_odom, /imu/data chỉ gyro, /gps/fix)
         ─► ekf_odom ─► TF odom → base_link;  /scan ─► slam_toolbox ─► /map, TF map → odom ─► Nav2
  Vị trí thật của Gazebo chỉ ra /ground_truth/odom để so sai số.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items())


def launch_setup(context):
    package = get_package_share_directory('ackermann_bringup')
    world = os.path.join(package, 'worlds', 'nav_world.sdf')
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    sim = {'use_sim_time': True}
    rviz_environment = {key: '' for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR')
                        if '/snap/' in os.environ.get(key, '')}
    actions = [
        include('ackermann_bringup', 'simulation.launch.py', {'gz_args': ('-r -s ' if headless else '-r ') + world}),
        include('ackermann_bringup', 'sim_sensors.launch.py', {}),
        include('ackermann_localization', 'slam_localization.launch.py', {'use_sim_time': 'true'}),
        include('ackermann_navigation', 'nav2.launch.py', {
            'controller': LaunchConfiguration('controller').perform(context), 'use_sim_time': 'true'}),
        # Điểm bấm ở frame map (bản đồ SLAM bắt đầu tại gốc world), so với vị trí thật của xe.
        Node(package='ackermann_bringup', executable='obstacle_spawner', name='obstacle_spawner',
             output='screen', parameters=[sim, {'world': 'nav_world'}],
             remappings=[('/odom', '/ground_truth/odom')]),
    ]
    if LaunchConfiguration('rviz').perform(context) == 'true':
        actions += [
            Node(package='ackermann_sensors', executable='gps_visualizer', name='gps_visualizer',
                 output='screen', parameters=[sim, {'text_height': 0.18, 'text_color': [0.05, 0.1, 0.65]}]),
            Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                 arguments=['-d', os.path.join(package, 'rviz', 'navigation.rviz')],
                 parameters=[sim], additional_env=rviz_environment),
        ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='rpp', choices=['rpp', 'mppi']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_setup),
    ])

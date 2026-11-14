#!/usr/bin/env python3
"""Ghép mô phỏng, định vị, Nav2, RViz và công cụ thả vật cản."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory('ackermann_bringup')
    localization = get_package_share_directory('ackermann_localization')
    navigation = get_package_share_directory('ackermann_navigation')
    world = os.path.join(package, 'worlds', 'nav_world.sdf')
    gz_args = [PythonExpression([
        "'-r -s ' if '", LaunchConfiguration('headless'), "' == 'true' else '-r '"]), world]
    rviz_environment = {key: '' for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR')
                        if '/snap/' in os.environ.get(key, '')}
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='rpp', choices=['rpp', 'mppi']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'simulation.launch.py')),
            launch_arguments={'gz_args': gz_args}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(localization, 'launch', 'localization.launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(navigation, 'launch', 'nav2.launch.py')),
            launch_arguments={'controller': LaunchConfiguration('controller'), 'use_sim_time': 'true'}.items()),
        Node(package='ackermann_bringup', executable='obstacle_spawner', name='obstacle_spawner',
             output='screen', parameters=[{'use_sim_time': True, 'world': 'nav_world'}]),
        Node(package='ackermann_sensors', executable='gps_visualizer', name='gps_visualizer',
             output='screen', parameters=[{'use_sim_time': True, 'text_height': 0.18,
                                          'text_color': [0.05, 0.1, 0.65]}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(package, 'rviz', 'navigation.rviz')],
             parameters=[{'use_sim_time': True}], additional_env=rviz_environment,
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])

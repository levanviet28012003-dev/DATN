#!/usr/bin/env python3
"""Khởi chạy các server Nav2; nguồn odometry, Lidar và định vị do bringup cung cấp."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    package = get_package_share_directory('ackermann_navigation')
    common = os.path.join(package, 'config', 'nav2_params.yaml')
    controller = ParameterFile([
        os.path.join(package, 'config', 'controller_'), LaunchConfiguration('controller'), '.yaml'])
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='rpp', choices=['rpp', 'mppi']),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[common, controller, clock],
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[common, controller, clock]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[common, clock]),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[common, clock, {
                 'default_nav_to_pose_bt_xml': os.path.join(
                     package, 'behavior_trees', 'navigate_to_pose_ackermann.xml'),
                 'default_nav_through_poses_bt_xml': os.path.join(
                     package, 'behavior_trees', 'navigate_through_poses_ackermann.xml'),
             }]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[common, clock],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[clock, {
                 'autostart': True, 'node_names': ['controller_server', 'planner_server',
                     'behavior_server', 'bt_navigator', 'velocity_smoother'],
             }]),
    ])

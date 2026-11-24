#!/usr/bin/env python3
"""Định vị bằng SLAM (kiến trúc 2026-11-23): EKF local và slam_toolbox chế độ online async.

  /wheel_odom + /imu/data (gyro) ─► ekf_odom ─► /odom, TF odom → base_link
  /scan + TF odom → base_link ─► slam_toolbox ─► /map, TF map → odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package = get_package_share_directory('ackermann_localization')
    clock = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(package, 'config', 'slam_toolbox.yaml'),
                              description='File tham số slam_toolbox.'),
        Node(package='robot_localization', executable='ekf_node', name='ekf_odom', output='screen',
             parameters=[os.path.join(package, 'config', 'ekf_local.yaml'), clock],
             remappings=[('odometry/filtered', '/odom')]),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             output='screen', parameters=[LaunchConfiguration('slam_params_file'), clock]),
    ])

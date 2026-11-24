#!/usr/bin/env python3
"""Khởi chạy các server Nav2; nguồn odometry, Lidar và định vị do bringup cung cấp.

Định vị do slam_toolbox (map → odom) và ekf_odom (odom → base_link) cung cấp.
profile:=indoor: global costmap lấy bản đồ SLAM /map (lớp tĩnh, sân trong phòng nhỏ);
profile:=outdoor nạp thêm nav2_outdoor.yaml (global costmap cửa sổ trượt 50 m, dung sai đích 1,5 m).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def parameter_files(package, controller, profile):
    """Danh sách file tham số theo thứ tự nạp; file sau ghi đè file trước."""
    files = [os.path.join(package, 'config', 'nav2_params.yaml'),
             os.path.join(package, 'config', f'controller_{controller}.yaml')]
    if profile == 'outdoor':
        files.append(os.path.join(package, 'config', 'nav2_outdoor.yaml'))
    return files


def launch_nodes(context):
    package = get_package_share_directory('ackermann_navigation')
    files = parameter_files(package, LaunchConfiguration('controller').perform(context),
                            LaunchConfiguration('profile').perform(context))
    clock = {'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}
    params = files + [clock]
    return [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=params, remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=params),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=params),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=params + [{
                 'default_nav_to_pose_bt_xml': os.path.join(
                     package, 'behavior_trees', 'navigate_to_pose_ackermann.xml'),
                 'default_nav_through_poses_bt_xml': os.path.join(
                     package, 'behavior_trees', 'navigate_through_poses_ackermann.xml'),
             }]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=params,
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[clock, {
                 'autostart': True, 'node_names': ['controller_server', 'planner_server',
                     'behavior_server', 'bt_navigator', 'velocity_smoother'],
             }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='rpp', choices=['rpp', 'mppi']),
        DeclareLaunchArgument('profile', default_value='indoor', choices=['indoor', 'outdoor']),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        OpaqueFunction(function=launch_nodes),
    ])

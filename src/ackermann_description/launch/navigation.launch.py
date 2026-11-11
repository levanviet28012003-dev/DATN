#!/usr/bin/env python3
"""Khởi chạy mô phỏng trong sân có vật cản, định vị AMCL và Nav2 để xe tự đi tới điểm đích."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    package = get_package_share_directory('ackermann_description')
    world = os.path.join(package, 'worlds', 'nav_world.sdf')
    sim_time = {'use_sim_time': True}

    # Tham số chung và file của bộ điều khiển được chọn: controller_rpp.yaml hoặc controller_mppi.yaml.
    common_params = os.path.join(package, 'config', 'nav2_params.yaml')
    controller_params = ParameterFile(
        [os.path.join(package, 'config', 'controller_'), LaunchConfiguration('controller'), '.yaml'])

    # Thêm -s để chạy Gazebo không giao diện khi headless:=true.
    gz_args = [
        PythonExpression(["'-r -s ' if '", LaunchConfiguration('headless'), "' == 'true' else '-r '"]),
        world,
    ]

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'display.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[common_params, sim_time,
                    {'yaml_filename': os.path.join(package, 'maps', 'nav_map.yaml')}],
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[common_params, sim_time],
    )
    # Lệnh của bộ điều khiển đi qua velocity_smoother rồi mới tới /cmd_vel.
    controller_server = Node(
        package='nav2_controller', executable='controller_server', name='controller_server', output='screen',
        parameters=[common_params, controller_params, sim_time],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    # File của bộ điều khiển cũng chọn kiểu đường cho bộ lập đường (Reeds-Shepp cho RPP, Dubins cho MPPI).
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
        parameters=[common_params, controller_params, sim_time],
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
        parameters=[common_params, sim_time],
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
        parameters=[common_params, sim_time, {
            'default_nav_to_pose_bt_xml':
                os.path.join(package, 'behavior_trees', 'navigate_to_pose_ackermann.xml'),
            'default_nav_through_poses_bt_xml':
                os.path.join(package, 'behavior_trees', 'navigate_through_poses_ackermann.xml'),
        }],
    )
    velocity_smoother = Node(
        package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
        output='screen',
        parameters=[common_params, sim_time],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    # Tên hai node quản lý giữ theo mặc định của Nav2 để bảng Navigation 2 trong RViz hiện đúng trạng thái.
    lifecycle_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[sim_time, {'autostart': True, 'node_names': ['map_server', 'amcl']}],
    )
    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[sim_time, {'autostart': True, 'node_names': [
            'controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother']}],
    )

    # Thả chướng ngại vật tại điểm bấm bằng công cụ Publish Point của RViz (/clicked_point).
    obstacle_spawner = Node(
        package='ackermann_description', executable='obstacle_spawner', name='obstacle_spawner',
        output='screen',
        parameters=[sim_time, {'world': 'nav_world'}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'),
                                      'rviz', 'nav2_default_view.rviz')],
        parameters=[sim_time],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('controller', default_value='rpp', choices=['rpp', 'mppi'],
                              description='Bộ điều khiển bám đường: rpp (Regulated Pure Pursuit) hoặc mppi.'),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='true để chạy Gazebo không có giao diện.'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='Mở RViz để xem bản đồ và chọn điểm đích.'),
        simulation,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        lifecycle_localization,
        lifecycle_navigation,
        obstacle_spawner,
        rviz,
    ])

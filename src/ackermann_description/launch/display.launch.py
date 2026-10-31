#!/usr/bin/env python3

import os

import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # ==========================================
    # 1. PACKAGE PATH
    # ==========================================

    pkg_description = get_package_share_directory(
        'ackermann_description'
    )

    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim'
    )

    # ==========================================
    # 2. LOAD ROBOT DESCRIPTION
    # ==========================================

    xacro_file = os.path.join(
        pkg_description,
        'urdf',
        'ackermann.urdf.xacro'
    )

    robot_description = xacro.process_file(
        xacro_file
    ).toxml()

    # ==========================================
    # 3. START ROBOT STATE PUBLISHER
    # ==========================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # ==========================================
    # 4. START GAZEBO
    # ==========================================

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': LaunchConfiguration('gz_args')
        }.items()
    )

    # ==========================================
    # 5. SPAWN ACKERMANN VEHICLE
    # ==========================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ackermann',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'ackermann_car',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.04'
        ]
    )

    # ==========================================
    # 6. LAUNCH DESCRIPTION
    # ==========================================

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_description, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value='-r empty.sdf',
                              description='Tham số Gazebo; thêm -s để chạy không có giao diện.'),
        LogInfo(msg=['Loading coloured STL model: ', xacro_file]),
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            os.path.dirname(pkg_description) + os.pathsep
            + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ),
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.dirname(pkg_description) + os.pathsep
            + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ),
        gazebo,
        bridge,
        robot_state_publisher,
        spawn_robot
    ])

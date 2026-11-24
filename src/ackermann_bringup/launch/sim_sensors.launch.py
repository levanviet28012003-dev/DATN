#!/usr/bin/env python3
"""Biến cảm biến lý tưởng của Gazebo thành dữ liệu giống xe thật (chỉ dùng cho mô phỏng).

  /sim/wheel_odom ─► covariance_setter ─► /wheel_odom   (như odometry encoder)
  /sim/imu        ─► covariance_setter ─► /imu/data     (chỉ gyro, không hướng tuyệt đối: như điện thoại)
  /sim/gps/fix    ─► gps_noise_injector ─► /gps/fix     (GPS điện thoại: 1 Hz, trễ 0,5 s, nhảy vài mét)

Trên xe thật, cầu nối điện thoại và node odometry encoder phát thẳng /imu/data, /gps/fix, /wheel_odom.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_nodes(context):
    sim = {'use_sim_time': True}
    gps_noise = LaunchConfiguration('gps_noise').perform(context) == 'true'
    noise = {} if gps_noise else {'white_sigma': 0.0, 'bias_sigma': 0.0, 'vertical_sigma': 0.0}
    return [
        Node(package='ackermann_sensors', executable='covariance_setter', name='imu_covariance',
             output='screen', parameters=[sim, {
                 'message_type': 'imu', 'input_topic': '/sim/imu', 'output_topic': '/imu/data',
                 'orientation_variance': [-1.0, -1.0, -1.0]}]),
        Node(package='ackermann_sensors', executable='covariance_setter', name='wheel_odom_covariance',
             output='screen', parameters=[sim, {
                 'message_type': 'odometry', 'input_topic': '/sim/wheel_odom',
                 'output_topic': '/wheel_odom'}]),
        Node(package='ackermann_sensors', executable='gps_noise_injector', name='gps_noise_injector',
             output='screen', parameters=[sim, noise, {
                 'input_topic': '/sim/gps/fix', 'output_topic': '/gps/fix',
                 'seed': int(LaunchConfiguration('gps_seed').perform(context))}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gps_noise', default_value='true', choices=['true', 'false'],
                              description='false: GPS vẫn 1 Hz, trễ 0,5 s nhưng không có sai số vị trí.'),
        DeclareLaunchArgument('gps_seed', default_value='-1',
                              description='Hạt giống nhiễu GPS; -1 là ngẫu nhiên mỗi lần chạy.'),
        OpaqueFunction(function=launch_nodes),
    ])

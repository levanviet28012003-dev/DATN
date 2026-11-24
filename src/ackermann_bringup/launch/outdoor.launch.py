#!/usr/bin/env python3
"""Mô phỏng ngoài trời (đoạn đường Bách khoa) với kiến trúc SLAM, Nav2, Mapviz trên ảnh vệ tinh.

Luồng dữ liệu (CLAUDE.md, 2026-11-23):
  Gazebo ─► sim_sensors ─► /wheel_odom, /imu/data (chỉ gyro) ─► ekf_odom ─► TF odom → base_link
         ─► /scan ─► slam_toolbox ─► /map, TF map → odom ─► Nav2 (lập đường ở map, rolling 50 m)
         ─► /gps/fix (GPS điện thoại giả lập): chỉ hiển thị; chưa có GPS Waypoint Converter
         ─► /ground_truth/odom (chỉ để so sai số, không phát TF)

Bản đồ SLAM bắt đầu tại chỗ xe xuất phát (gốc world, hướng Đông), nên frame map trùng world của
Gazebo. Gốc Mapviz và ảnh vệ tinh lấy từ <spherical_coordinates> của world.
"""
import os
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def world_origin(world_path):
    """Đọc (vĩ độ, kinh độ, độ cao) từ <spherical_coordinates> của file SDF; bắt buộc ENU."""
    world = ElementTree.parse(world_path).getroot().find('world')
    coordinates = world.find('spherical_coordinates') if world is not None else None
    if coordinates is None:
        raise RuntimeError(f'{world_path} không có <spherical_coordinates>; GPS sẽ không hoạt động.')
    if coordinates.findtext('world_frame_orientation', 'ENU') != 'ENU':
        raise RuntimeError('World phải dùng hướng ENU để frame map trùng world.')
    if float(coordinates.findtext('heading_deg', '0')) != 0.0:
        raise RuntimeError('heading_deg của world phải bằng 0 để trục X là hướng Đông.')
    return (float(coordinates.findtext('latitude_deg')), float(coordinates.findtext('longitude_deg')),
            float(coordinates.findtext('elevation', '0')))


SATELLITE_DIR = os.path.join(os.path.expanduser('~'), '.cache', 'ackermann_ws', 'satellite_ground')


def satellite_ground(latitude, longitude, directory=SATELLITE_DIR):
    """Đường dẫn model.sdf ảnh vệ tinh nếu đã tạo cho đúng mốc này, kèm lý do nếu không dùng được."""
    model, meta = os.path.join(directory, 'model.sdf'), os.path.join(directory, 'meta.yaml')
    if not (os.path.isfile(model) and os.path.isfile(meta)):
        return None, 'chưa tạo ảnh vệ tinh'
    values = {}
    with open(meta, encoding='utf-8') as stream:
        for line in stream:
            key, _, value = line.partition(':')
            values[key.strip()] = value.strip()
    try:
        same = (abs(float(values['latitude']) - latitude) < 1e-9
                and abs(float(values['longitude']) - longitude) < 1e-9)
    except (KeyError, ValueError):
        return None, f'{meta} hỏng'
    if not same:
        return None, f"ảnh vệ tinh tạo cho mốc {values['latitude']}, {values['longitude']}, khác mốc world"
    return model, None


def include(package, filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(package), 'launch', filename)),
        launch_arguments=arguments.items())


def launch_setup(context):
    package = get_package_share_directory('ackermann_bringup')
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    rviz = LaunchConfiguration('rviz').perform(context) == 'true'
    mapviz = LaunchConfiguration('mapviz').perform(context) == 'true'
    latitude, longitude, _ = world_origin(world)
    sim = {'use_sim_time': True}
    rviz_environment = {key: '' for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR')
                        if '/snap/' in os.environ.get(key, '')}

    actions = [
        LogInfo(msg=f'Mốc world: vĩ độ {latitude}, kinh độ {longitude}.'),
        include('ackermann_bringup', 'simulation.launch.py', {'gz_args': ('-r -s ' if headless else '-r ') + world}),
        include('ackermann_bringup', 'sim_sensors.launch.py', {
            'gps_noise': LaunchConfiguration('gps_noise').perform(context),
            'gps_seed': LaunchConfiguration('gps_seed').perform(context)}),
        include('ackermann_localization', 'slam_localization.launch.py', {'use_sim_time': 'true'}),
        include('ackermann_navigation', 'nav2.launch.py', {
            'controller': LaunchConfiguration('controller').perform(context),
            'profile': 'outdoor', 'use_sim_time': 'true'}),
        Node(package='ackermann_bringup', executable='obstacle_spawner', name='obstacle_spawner',
             output='screen', parameters=[sim, {'world': 'outdoor_world'}],
             # Điểm bấm ở frame map, trùng world; so khoảng cách với vị trí thật, không phải /odom.
             remappings=[('/odom', '/ground_truth/odom')]),
        # Chữ tọa độ GPS điện thoại cho Mapviz và marker cho RViz.
        Node(package='ackermann_sensors', executable='gps_visualizer', name='gps_text',
             output='screen', parameters=[sim, {
                 'fix_topic': '/gps/fix', 'marker_topic': '/gps/marker', 'text_topic': '/gps/text',
                 'label': 'GPS điện thoại', 'text_height': 0.4, 'text_color': [0.9, 0.1, 0.1]}]),
    ]
    if LaunchConfiguration('satellite').perform(context) == 'true':
        model, reason = satellite_ground(latitude, longitude)
        if model is None:
            actions.append(LogInfo(msg=f'Không dán ảnh vệ tinh lên Gazebo: {reason}. Tạo bằng: python3 '
                                       'src/ackermann_bringup/scripts/generate_satellite_ground.py'))
        else:
            # Ảnh có tâm tại mốc world nên đặt model tại gốc; chỉ có visual, không ảnh hưởng vật lý.
            actions.append(Node(package='ros_gz_sim', executable='create', name='spawn_satellite_ground',
                                output='screen', arguments=['-file', model, '-name', 'satellite_ground',
                                                            '-x', '0', '-y', '0', '-z', '0']))
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                            arguments=['-d', os.path.join(package, 'rviz', 'outdoor.rviz')],
                            parameters=[sim], additional_env=rviz_environment))
    if mapviz:
        actions.append(include('ackermann_bringup', 'mapviz.launch.py', {
            'latitude': str(latitude), 'longitude': str(longitude), 'use_sim_time': 'true'}))
    return actions


def generate_launch_description():
    package = get_package_share_directory('ackermann_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(package, 'worlds', 'outdoor_world.sdf')),
        DeclareLaunchArgument('controller', default_value='rpp', choices=['rpp', 'mppi']),
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('mapviz', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('gps_noise', default_value='true', choices=['true', 'false'],
                              description='false: GPS vẫn 1 Hz và trễ 0,5 s nhưng không có sai số vị trí.'),
        DeclareLaunchArgument('satellite', default_value='true', choices=['true', 'false'],
                              description='Dán ảnh vệ tinh (generate_satellite_ground.py) lên mặt đất Gazebo.'),
        DeclareLaunchArgument('gps_seed', default_value='-1',
                              description='Hạt giống nhiễu GPS; -1 là ngẫu nhiên mỗi lần chạy.'),
        OpaqueFunction(function=launch_setup),
    ])

#!/usr/bin/env python3
"""Sinh hai world Gazebo: sân trong phòng (nav_world) và đoạn đường Bách khoa (outdoor_world).

Chạy: python3 src/ackermann_bringup/scripts/generate_world.py
Kết quả: ackermann_bringup/worlds. Bản đồ do slam_toolbox lập trong lúc chạy, không sinh sẵn.
Chỉ dùng thư viện chuẩn. Sửa bố cục trong obstacles() hoặc outdoor_obstacles() rồi chạy lại.
"""

import argparse
from collections import namedtuple
from pathlib import Path

# Đơn vị mét. Gốc tọa độ (0, 0) là nơi đặt xe, phải luôn trống.
HALF_X = 4.0           # Nửa chiều dài sân theo trục X
HALF_Y = 3.0           # Nửa chiều rộng sân theo trục Y
WALL_THICKNESS = 0.10
WALL_HEIGHT = 0.40     # Cao hơn mặt phẳng quét của Lidar (khoảng 0,10 m)
SPAWN_CLEARANCE = 0.5  # Bán kính trống quanh gốc của sân trong phòng

# Mốc địa lý minh họa, không phải vị trí đo thật; +X Đông, +Y Bắc, +Z lên.
GPS_LATITUDE_DEG = 10.0
GPS_LONGITUDE_DEG = 106.0
GPS_ELEVATION_M = 0.0

# Mốc của world ngoài trời: vị trí xe xuất phát (gốc world) trên bản đồ thật, xe hướng Đông.
# Đặt ở tim đường nội bộ Đông–Tây trong khuôn viên ĐH Bách khoa Hà Nội (giữa tòa nhà cao phía bắc
# và sân lát gạch trước thư viện), chọn và đo trên ảnh Esri zoom 19 ngày 2026-11-18. Mặt đường
# rộng khoảng 10 m: y từ −5 đến +5 m quanh mốc, chạy thẳng về phía Đông hơn 300 m; phía bắc là hàng
# cây (tán tới y ≈ +4), phía nam là sân gạch (từ y ≈ −5,5). Đổi mốc: sửa hai hằng số, chạy lại script,
# build lại, chạy generate_satellite_ground.py; launch ngoài trời đọc mốc từ file world đã sinh.
OUTDOOR_LATITUDE_DEG = 21.0049949
OUTDOOR_LONGITUDE_DEG = 105.843951
OUTDOOR_ELEVATION_M = 0.0
OUTDOOR_GROUND_SIZE = 200.0      # Cạnh mặt đất vuông, m
OUTDOOR_OBSTACLE_HEIGHT = 0.80   # Cao hơn mặt phẳng quét Lidar (khoảng 0,10 m so với mặt đất)
OUTDOOR_SPAWN_CLEARANCE = 2.0    # Bán kính trống quanh gốc để xe xuất phát và quay đầu
# Các điểm đích mẫu (x Đông, y Bắc, m) trên mặt đường để thử điều hướng; phải trống trong bán kính
# 1,5 m và mỗi đích cách đích trước dưới 24 m (global costmap 50 m).
OUTDOOR_DEMO_GOALS = [(22.0, -1.0), (42.0, -1.0), (20.0, 2.0), (0.0, 0.0)]
OUTDOOR_ROAD_HALF_WIDTH = 5.0    # Nửa bề rộng mặt đường quanh trục y = 0, m

TREE_COLOR = '0.25 0.45 0.20 1'
CAR_COLOR = '0.20 0.35 0.70 1'
CONE_COLOR = '0.95 0.45 0.05 1'
BIKE_COLOR = '0.15 0.15 0.15 1'
BARRIER_COLOR = '0.90 0.90 0.20 1'

WALL_COLOR = '0.55 0.55 0.58 1'
OBJECT_COLOR = '0.90 0.45 0.10 1'

# Hình hộp: tâm (x, y), kích thước (sx, sy). Hình trụ: tâm (x, y), bán kính r.
Box = namedtuple('Box', 'name x y sx sy color')
Cylinder = namedtuple('Cylinder', 'name x y r color')


def obstacles():
    """Trả về toàn bộ vật cản tĩnh của sân: tường bao, tường ngăn và vật cản rời."""
    t = WALL_THICKNESS
    return [
        # Tường bao quanh sân 8 m × 6 m.
        Box('wall_north', 0.0, HALF_Y, 2 * HALF_X + t, t, WALL_COLOR),
        Box('wall_south', 0.0, -HALF_Y, 2 * HALF_X + t, t, WALL_COLOR),
        Box('wall_east', HALF_X, 0.0, t, 2 * HALF_Y + t, WALL_COLOR),
        Box('wall_west', -HALF_X, 0.0, t, 2 * HALF_Y + t, WALL_COLOR),
        # Hai tường ngăn đứng tách khỏi tường bao để xe đi vòng được, không tạo phòng cụt.
        # Xe chỉ chạy tiến và cần khoảng 1,5 m bề ngang để quay đầu 180°, nên khe giữa đầu tường ngăn
        # và tường bao rộng từ 1,8 m; các lối đi thẳng khác rộng từ 1,0 m.
        Box('divider_west', -1.6, -0.2, t, 1.8, WALL_COLOR),
        Box('divider_north', 1.4, 1.0, 1.2, t, WALL_COLOR),
        # Vật cản rời; hai hộp b và c đặt sát tường bao để không tạo khe hẹp.
        # Góc đông nam để trống làm chỗ cho xe xoay trở.
        Box('box_a', -2.8, 1.5, 0.4, 0.4, OBJECT_COLOR),
        Box('box_b', 3.65, 2.0, 0.6, 0.3, OBJECT_COLOR),
        Box('box_c', -3.75, -1.6, 0.4, 0.6, OBJECT_COLOR),
        Cylinder('cylinder_a', -0.2, 1.7, 0.25, OBJECT_COLOR),
        Cylinder('cylinder_b', 0.4, -1.5, 0.20, OBJECT_COLOR),
    ]


def outdoor_obstacles():
    """Vật cản của world ngoài trời theo đoạn đường Bách khoa quanh mốc.

    Thân cây đặt theo hàng cây thật trên ảnh vệ tinh (ước lượng từ tán cây, sai vài mét). Ô tô đỗ,
    cọc tiêu, xe máy và rào chắn là vật cản giả định đặt trên mặt đường để thử né tránh; ngoài đời
    không có. Mỗi đoạn giữa hai đích mẫu liên tiếp vẫn còn lối đi rộng từ 2,3 m.
    """
    trees = [(-30.0, 8.0), (-18.0, 8.0), (-6.0, 8.0), (30.0, 9.0), (40.0, 8.0), (50.0, 8.0),
             (-17.0, -12.5), (3.0, -14.5), (38.0, -12.5)]
    shapes = [Cylinder(f'tree_{i}', x, y, 0.3, TREE_COLOR) for i, (x, y) in enumerate(trees)]
    shapes += [
        # Ô tô đỗ lệch về làn nam, chắn đường thẳng từ gốc tới đích (22, −1).
        Box('parked_car', 12.0, -1.4, 4.2, 1.8, CAR_COLOR),
        # Hàng cọc tiêu ngang giữa đường, chắn đường thẳng giữa (42, −1) và (20, 2).
        *[Cylinder(f'cone_{i}', 30.0, y, 0.15, CONE_COLOR) for i, y in enumerate((-2.5, -1.5, -0.5, 0.5))],
        # Xe máy đỗ sát mép nam.
        *[Box(f'motorbike_{i}', x, -4.0, 0.7, 1.9, BIKE_COLOR) for i, x in enumerate((35.0, 36.0, 37.0))],
        # Rào chắn thi công ở làn bắc phía trước đích (42, −1).
        Box('barrier', 46.0, 1.5, 0.4, 3.0, BARRIER_COLOR),
    ]
    return shapes


def shape_distance(shape, x, y):
    """Khoảng cách từ điểm (x, y) tới mép vật cản; âm nếu điểm nằm trong vật cản."""
    if isinstance(shape, Box):
        dx = max(abs(x - shape.x) - shape.sx / 2, 0.0)
        dy = max(abs(y - shape.y) - shape.sy / 2, 0.0)
        if dx == 0.0 and dy == 0.0:
            return -min(shape.sx / 2 - abs(x - shape.x), shape.sy / 2 - abs(y - shape.y))
        return (dx * dx + dy * dy) ** 0.5
    return ((x - shape.x) ** 2 + (y - shape.y) ** 2) ** 0.5 - shape.r


def _shape_sdf(shape, height=WALL_HEIGHT):
    """Sinh cặp collision và visual của một vật cản trong link chung."""
    if isinstance(shape, Box):
        geometry = f'<box><size>{shape.sx:g} {shape.sy:g} {height:g}</size></box>'
    else:
        geometry = f'<cylinder><radius>{shape.r:g}</radius><length>{height:g}</length></cylinder>'
    pose = f'<pose>{shape.x:g} {shape.y:g} {height / 2:g} 0 0 0</pose>'
    material = (f'<material><ambient>{shape.color}</ambient><diffuse>{shape.color}</diffuse>'
                '<specular>0.1 0.1 0.1 1</specular></material>')
    return (f'        <collision name="{shape.name}_collision">{pose}<geometry>{geometry}</geometry></collision>\n'
            f'        <visual name="{shape.name}_visual">{pose}<geometry>{geometry}</geometry>{material}</visual>\n')


def world_sdf(shapes=None, name='nav_world',
              origin=(GPS_LATITUDE_DEG, GPS_LONGITUDE_DEG, GPS_ELEVATION_M),
              height=WALL_HEIGHT, ground_size=100, ground_color='0.8 0.8 0.8 1'):
    """Sinh world Fortress có GPS, IMU và plugin Sensors (cần cho Lidar)."""
    shapes = obstacles() if shapes is None else shapes
    latitude, longitude, elevation = origin
    body = ''.join(_shape_sdf(shape, height) for shape in shapes)
    sensors = '''    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
'''
    arena = f'''    <model name="arena">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
''' if shapes else ''
    return f'''<?xml version="1.0"?>
<!-- File được sinh bởi scripts/generate_world.py. Không sửa tay; sửa script rồi chạy lại. -->
<sdf version="1.8">
  <world name="{name}">
    <!-- Mốc WGS84 minh họa; cần dùng cùng mốc khi ghép định vị GPS. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{latitude}</latitude_deg>
      <longitude_deg>{longitude}</longitude_deg>
      <elevation>{elevation}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- Khi world tự khai báo plugin, Gazebo không nạp bộ mặc định nên phải liệt kê đủ. -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-navsat-system" name="ignition::gazebo::systems::NavSat"/>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
{sensors}    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>{ground_size:g} {ground_size:g}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>{ground_size:g} {ground_size:g}</size></plane></geometry>
          <material><ambient>{ground_color}</ambient><diffuse>{ground_color}</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>
{arena}  </world>
</sdf>
'''


def outdoor_world_sdf():
    """World ngoài trời: mốc địa lý là vị trí xuất phát thật."""
    return world_sdf(outdoor_obstacles(), name='outdoor_world',
                     origin=(OUTDOOR_LATITUDE_DEG, OUTDOOR_LONGITUDE_DEG, OUTDOOR_ELEVATION_M),
                     height=OUTDOOR_OBSTACLE_HEIGHT, ground_size=OUTDOOR_GROUND_SIZE,
                     ground_color='0.45 0.55 0.35 1')


def write_outputs(worlds_dir):
    """Ghi hai world vào thư mục worlds."""
    worlds_dir = Path(worlds_dir)
    worlds_dir.mkdir(parents=True, exist_ok=True)
    (worlds_dir / 'nav_world.sdf').write_text(world_sdf(), encoding='utf-8')
    (worlds_dir / 'outdoor_world.sdf').write_text(outdoor_world_sdf(), encoding='utf-8')


def main():
    package_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worlds-dir', type=Path, default=package_dir / 'worlds')
    args = parser.parse_args()
    write_outputs(args.worlds_dir)
    print(f'Đã sinh nav_world.sdf và outdoor_world.sdf vào {args.worlds_dir}')


if __name__ == '__main__':
    main()

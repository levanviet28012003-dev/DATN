#!/usr/bin/env python3
"""Sinh world Gazebo và bản đồ Nav2 từ cùng một danh sách vật cản.

Chạy: python3 src/ackermann_bringup/scripts/generate_world.py
Kết quả: ackermann_bringup/worlds và ackermann_localization/maps.
Chỉ dùng thư viện chuẩn. Sửa bố cục trong hàm obstacles() rồi chạy lại.
"""

import argparse
from collections import namedtuple
from pathlib import Path

# Đơn vị mét. Gốc tọa độ (0, 0) là nơi đặt xe, phải luôn trống.
RESOLUTION = 0.05      # Kích thước một ô bản đồ
HALF_X = 4.0           # Nửa chiều dài sân theo trục X
HALF_Y = 3.0           # Nửa chiều rộng sân theo trục Y
WALL_THICKNESS = 0.10
WALL_HEIGHT = 0.40     # Cao hơn mặt phẳng quét của Lidar (khoảng 0,10 m)
MARGIN = 0.20          # Phần bản đồ thừa ra ngoài tường bao

OCCUPIED, FREE, UNKNOWN = 0, 254, 205   # Giá trị điểm ảnh theo quy ước của map_server

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


def map_origin():
    """Tọa độ góc dưới trái của bản đồ trong hệ map."""
    return -(HALF_X + MARGIN), -(HALF_Y + MARGIN)


def map_size():
    """Số ô theo chiều ngang và chiều dọc."""
    return (round(2 * (HALF_X + MARGIN) / RESOLUTION),
            round(2 * (HALF_Y + MARGIN) / RESOLUTION))


def world_to_cell(x, y):
    """Đổi tọa độ mét sang chỉ số (cột, hàng); hàng 0 là hàng dưới cùng."""
    origin_x, origin_y = map_origin()
    return int((x - origin_x) // RESOLUTION), int((y - origin_y) // RESOLUTION)


def _overlaps(shape, x_min, y_min, x_max, y_max):
    """Kiểm tra vật cản có chồng lên ô hay không; chỉ chạm cạnh thì không tính."""
    eps = 1e-6
    if isinstance(shape, Box):
        return (x_min < shape.x + shape.sx / 2 - eps and x_max > shape.x - shape.sx / 2 + eps
                and y_min < shape.y + shape.sy / 2 - eps and y_max > shape.y - shape.sy / 2 + eps)
    # Hình trụ: so khoảng cách từ tâm tới điểm gần nhất của ô.
    dx = shape.x - min(max(shape.x, x_min), x_max)
    dy = shape.y - min(max(shape.y, y_min), y_max)
    return dx * dx + dy * dy < (shape.r - eps) ** 2


def rasterize(shapes=None):
    """Tạo lưới chiếm chỗ: grid[hàng][cột], hàng 0 ở dưới cùng (y nhỏ nhất)."""
    shapes = obstacles() if shapes is None else shapes
    width, height = map_size()
    origin_x, origin_y = map_origin()
    limit_x = HALF_X + WALL_THICKNESS / 2
    limit_y = HALF_Y + WALL_THICKNESS / 2
    grid = []
    for row in range(height):
        y_min = origin_y + row * RESOLUTION
        line = []
        for col in range(width):
            x_min = origin_x + col * RESOLUTION
            x_mid, y_mid = x_min + RESOLUTION / 2, y_min + RESOLUTION / 2
            if abs(x_mid) > limit_x or abs(y_mid) > limit_y:
                line.append(UNKNOWN)   # Ngoài tường bao, Lidar không nhìn thấy
            elif any(_overlaps(s, x_min, y_min, x_min + RESOLUTION, y_min + RESOLUTION) for s in shapes):
                line.append(OCCUPIED)
            else:
                line.append(FREE)
        grid.append(line)
    return grid


def pgm_bytes(grid):
    """Mã hóa lưới thành ảnh PGM nhị phân; hàng đầu của ảnh là hàng trên cùng của bản đồ."""
    header = f'P5\n{len(grid[0])} {len(grid)}\n255\n'.encode('ascii')
    return header + b''.join(bytes(line) for line in reversed(grid))


def map_yaml(image_name='nav_map.pgm'):
    """Nội dung file mô tả bản đồ cho nav2_map_server."""
    origin_x, origin_y = map_origin()
    return (f'image: {image_name}\n'
            'mode: trinary\n'
            f'resolution: {RESOLUTION}\n'
            f'origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]\n'
            'negate: 0\n'
            'occupied_thresh: 0.65\n'
            'free_thresh: 0.25\n')


def _shape_sdf(shape):
    """Sinh cặp collision và visual của một vật cản trong link chung."""
    if isinstance(shape, Box):
        geometry = f'<box><size>{shape.sx:g} {shape.sy:g} {WALL_HEIGHT:g}</size></box>'
    else:
        geometry = f'<cylinder><radius>{shape.r:g}</radius><length>{WALL_HEIGHT:g}</length></cylinder>'
    pose = f'<pose>{shape.x:g} {shape.y:g} {WALL_HEIGHT / 2:g} 0 0 0</pose>'
    material = (f'<material><ambient>{shape.color}</ambient><diffuse>{shape.color}</diffuse>'
                '<specular>0.1 0.1 0.1 1</specular></material>')
    return (f'        <collision name="{shape.name}_collision">{pose}<geometry>{geometry}</geometry></collision>\n'
            f'        <visual name="{shape.name}_visual">{pose}<geometry>{geometry}</geometry>{material}</visual>\n')


def world_sdf(shapes=None):
    """Sinh world cho Gazebo Fortress, có plugin Sensors để Lidar hoạt động."""
    shapes = obstacles() if shapes is None else shapes
    body = ''.join(_shape_sdf(shape) for shape in shapes)
    return f'''<?xml version="1.0"?>
<!-- File được sinh bởi scripts/generate_world.py. Không sửa tay; sửa script rồi chạy lại. -->
<sdf version="1.8">
  <world name="nav_world">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <!-- Khi world tự khai báo plugin, Gazebo không nạp bộ mặc định nên phải liệt kê đủ. -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sun">
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
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
        </visual>
      </link>
    </model>
    <model name="arena">
      <static>true</static>
      <link name="link">
{body}      </link>
    </model>
  </world>
</sdf>
'''


def write_outputs(worlds_dir, maps_dir):
    """Ghi world và bản đồ đồng bộ vào hai package tương ứng."""
    worlds_dir, maps_dir = Path(worlds_dir), Path(maps_dir)
    worlds_dir.mkdir(parents=True, exist_ok=True)
    maps_dir.mkdir(parents=True, exist_ok=True)
    grid = rasterize()
    (worlds_dir / 'nav_world.sdf').write_text(world_sdf(), encoding='utf-8')
    (maps_dir / 'nav_map.pgm').write_bytes(pgm_bytes(grid))
    (maps_dir / 'nav_map.yaml').write_text(map_yaml(), encoding='utf-8')
    return grid


def main():
    package_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--worlds-dir', type=Path, default=package_dir / 'worlds')
    parser.add_argument('--maps-dir', type=Path,
                        default=package_dir.parent / 'ackermann_localization' / 'maps')
    args = parser.parse_args()
    grid = write_outputs(args.worlds_dir, args.maps_dir)
    print(f'Đã sinh world và bản đồ {len(grid[0])} × {len(grid)} ô, độ phân giải {RESOLUTION} m.')
    print(f'World: {args.worlds_dir}; bản đồ: {args.maps_dir}')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Tạo mặt đất Gazebo dán ảnh vệ tinh quanh mốc của world ngoài trời.

Tải ô ảnh (Web Mercator) của Esri World Imagery, ghép, cắt đúng một hình vuông `size` mét có tâm
là mốc world (gốc tọa độ, nơi xe xuất phát), rồi sinh model Gazebo chỉ có phần hiển thị: một mặt
phẳng dán ảnh nằm trên mặt đất. Vì frame map (datum = mốc world) và Mapviz dùng cùng mốc, ảnh trong
Gazebo trùng ảnh nền Mapviz; vật cản và xe hiện cùng chỗ trên cả hai.

Chạy (cần mạng):
    python3 src/ackermann_bringup/scripts/generate_satellite_ground.py
    python3 src/ackermann_bringup/scripts/generate_satellite_ground.py --size 200 --zoom 18

Kết quả ở ~/.cache/ackermann_ws/satellite_ground/ (không đưa vào git: ảnh thuộc Esri và các nhà
cung cấp, chỉ dùng cho học tập/thử nghiệm, phải ghi nguồn). outdoor.launch.py tự thả model nếu
meta.yaml khớp mốc của world. Sửa mốc trong generate_world.py thì phải chạy lại script này.
"""

import argparse
import io
import math
from pathlib import Path
import sys
import urllib.request
import xml.etree.ElementTree as ElementTree

TILE_SIZE = 256
EARTH_CIRCUMFERENCE = 40075016.686   # m, tại xích đạo, theo bán kính Web Mercator
ESRI_URL = 'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}'
ATTRIBUTION = 'Imagery: Esri, Maxar, Earthstar Geographics, and the GIS User Community'
USER_AGENT = 'ackermann_ws-satellite-ground/1.0'
OUTPUT_DIR = Path.home() / '.cache' / 'ackermann_ws' / 'satellite_ground'
CACHE_DIR = Path.home() / '.cache' / 'ackermann_ws' / 'tiles' / 'esri'
GROUND_HEIGHT = 0.005   # m, cao hơn mặt phẳng đất của world để không nhấp nháy (z-fighting)


def global_pixel(latitude, longitude, zoom):
    """Tọa độ điểm ảnh toàn cầu (x sang Đông, y xuống Nam) của một điểm WGS84 ở mức zoom."""
    scale = TILE_SIZE * 2 ** zoom
    x = (longitude + 180.0) / 360.0 * scale
    siny = math.sin(math.radians(latitude))
    y = (0.5 - math.log((1 + siny) / (1 - siny)) / (4 * math.pi)) * scale
    return x, y


def meters_per_pixel(latitude, zoom):
    """Kích thước thật của một điểm ảnh Web Mercator tại vĩ độ đã cho."""
    return EARTH_CIRCUMFERENCE * math.cos(math.radians(latitude)) / (TILE_SIZE * 2 ** zoom)


def crop_window(latitude, longitude, zoom, size_m):
    """Khung cắt (trái, trên, phải, dưới) theo điểm ảnh toàn cầu, tâm là mốc, cạnh size_m mét."""
    cx, cy = global_pixel(latitude, longitude, zoom)
    half = size_m / 2.0 / meters_per_pixel(latitude, zoom)
    return cx - half, cy - half, cx + half, cy + half


def tile_range(window):
    """Các chỉ số ô (x0..x1, y0..y1) phủ khung cắt."""
    left, top, right, bottom = window
    return (int(math.floor(left / TILE_SIZE)), int(math.floor((right - 1e-9) / TILE_SIZE)),
            int(math.floor(top / TILE_SIZE)), int(math.floor((bottom - 1e-9) / TILE_SIZE)))


def texture_size(native_pixels, limit=4096):
    """Cạnh ảnh dán (lũy thừa của 2, không nhỏ hơn ảnh gốc, tối đa limit) để GPU xử lý tốt."""
    size = 256
    while size < native_pixels and size < limit:
        size *= 2
    return size


def obj_quad(size_m, height=GROUND_HEIGHT):
    """Lưới OBJ một hình vuông nằm ngang, tâm tại gốc; UV (0,0) ở góc Tây Nam, (1,1) ở Đông Bắc."""
    h = size_m / 2.0
    return ('# Sinh bởi generate_satellite_ground.py: mặt đất dán ảnh vệ tinh, +X Đông, +Y Bắc.\n'
            f'v {-h:.4f} {-h:.4f} {height}\nv {h:.4f} {-h:.4f} {height}\n'
            f'v {h:.4f} {h:.4f} {height}\nv {-h:.4f} {h:.4f} {height}\n'
            'vt 0 0\nvt 1 0\nvt 1 1\nvt 0 1\nvn 0 0 1\n'
            'f 1/1/1 2/2/1 3/3/1\nf 1/1/1 3/3/1 4/4/1\n')


def model_sdf(mesh_path, texture_path):
    return f'''<?xml version="1.0"?>
<!-- Sinh bởi generate_satellite_ground.py. {ATTRIBUTION}. -->
<sdf version="1.8">
  <model name="satellite_ground">
    <static>true</static>
    <link name="link">
      <!-- Chỉ có visual: xe vẫn chạy trên ground_plane của world; Lidar quét ngang nên không thấy. -->
      <visual name="imagery">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>{mesh_path}</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>{texture_path}</albedo_map>
            <roughness>1.0</roughness>
            <metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
'''


def world_origin(world_path):
    """(vĩ độ, kinh độ) từ <spherical_coordinates> của world."""
    coordinates = ElementTree.parse(world_path).getroot().find('world/spherical_coordinates')
    if coordinates is None:
        raise RuntimeError(f'{world_path} không có <spherical_coordinates>.')
    return float(coordinates.findtext('latitude_deg')), float(coordinates.findtext('longitude_deg'))


def fetch_tile(zoom, x, y):
    """Byte JPEG của một ô ảnh; lưu cache để không tải lại."""
    path = CACHE_DIR / str(zoom) / str(y) / f'{x}.jpg'
    if path.exists():
        return path.read_bytes()
    request = urllib.request.Request(ESRI_URL.format(z=zoom, y=y, x=x), headers={'User-Agent': USER_AGENT})
    with urllib.request.urlopen(request, timeout=20) as response:
        data = response.read()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return data


def is_placeholder(image):
    """Ô 'Map data not yet available' của Esri: gần như một màu xám, chỉ có ít chữ."""
    histogram = image.convert('L').histogram()
    return max(histogram) > 0.85 * sum(histogram)


def best_zoom(latitude, longitude, highest=19, lowest=15):
    """Mức zoom cao nhất mà ô chứa mốc là ảnh thật."""
    from PIL import Image
    for zoom in range(highest, lowest - 1, -1):
        x, y = global_pixel(latitude, longitude, zoom)
        tile = Image.open(io.BytesIO(fetch_tile(zoom, int(x // TILE_SIZE), int(y // TILE_SIZE))))
        if not is_placeholder(tile):
            return zoom
        print(f'Zoom {zoom} chưa có ảnh tại mốc, thử mức thấp hơn.')
    raise RuntimeError('Không có ảnh vệ tinh tại mốc này.')


def build_texture(latitude, longitude, zoom, size_m):
    from PIL import Image
    window = crop_window(latitude, longitude, zoom, size_m)
    x0, x1, y0, y1 = tile_range(window)
    count = (x1 - x0 + 1) * (y1 - y0 + 1)
    print(f'Tải {count} ô ảnh zoom {zoom} ({meters_per_pixel(latitude, zoom):.2f} m/điểm ảnh)...')
    mosaic = Image.new('RGB', ((x1 - x0 + 1) * TILE_SIZE, (y1 - y0 + 1) * TILE_SIZE))
    placeholders = 0
    for ty in range(y0, y1 + 1):
        for tx in range(x0, x1 + 1):
            tile = Image.open(io.BytesIO(fetch_tile(zoom, tx, ty))).convert('RGB')
            placeholders += is_placeholder(tile)
            mosaic.paste(tile, ((tx - x0) * TILE_SIZE, (ty - y0) * TILE_SIZE))
    if placeholders:
        print(f'Cảnh báo: {placeholders}/{count} ô chưa có ảnh ở zoom {zoom}; giảm --zoom nếu thấy ô xám.')
    left, top, right, bottom = window
    box = (left - x0 * TILE_SIZE, top - y0 * TILE_SIZE, right - x0 * TILE_SIZE, bottom - y0 * TILE_SIZE)
    side = texture_size(right - left)
    # Ảnh: hàng trên cùng là phía Bắc, cột trái là phía Tây (khớp UV của obj_quad).
    return mosaic.transform((side, side), Image.EXTENT, box, Image.BICUBIC)


def main():
    package_dir = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--world', type=Path, default=package_dir / 'worlds' / 'outdoor_world.sdf',
                        help='World lấy mốc tọa độ (mặc định outdoor_world.sdf).')
    parser.add_argument('--size', type=float, default=160.0, help='Cạnh vùng ảnh, m (mặc định 160).')
    parser.add_argument('--zoom', type=int, default=0, help='Mức zoom; 0 = tự chọn mức cao nhất có ảnh.')
    parser.add_argument('--output', type=Path, default=OUTPUT_DIR)
    args = parser.parse_args()
    if not 10.0 <= args.size <= 1000.0:
        parser.error('--size phải từ 10 đến 1000 m.')
    latitude, longitude = world_origin(args.world)
    print(f'Mốc world: {latitude}, {longitude}; vùng {args.size:g} × {args.size:g} m.')
    zoom = args.zoom or best_zoom(latitude, longitude)
    texture = build_texture(latitude, longitude, zoom, args.size)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    texture_path, mesh_path = output / 'satellite.png', output / 'ground.obj'
    texture.save(texture_path)
    mesh_path.write_text(obj_quad(args.size), encoding='utf-8')
    (output / 'model.sdf').write_text(model_sdf(mesh_path, texture_path), encoding='utf-8')
    (output / 'meta.yaml').write_text(
        f'latitude: {latitude!r}\nlongitude: {longitude!r}\nsize: {args.size!r}\nzoom: {zoom}\n'
        f'meters_per_pixel: {meters_per_pixel(latitude, zoom):.4f}\nattribution: "{ATTRIBUTION}"\n',
        encoding='utf-8')
    print(f'Đã tạo {output}/model.sdf (ảnh {texture.size[0]} px, {ATTRIBUTION}).')
    return 0


if __name__ == '__main__':
    sys.exit(main())

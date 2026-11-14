"""Kiểm tra world Gazebo và bản đồ Nav2 được sinh từ cùng một danh sách vật cản."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree


package_dir = Path(__file__).resolve().parents[1]
maps_dir = package_dir.parent / 'ackermann_localization' / 'maps'
source = package_dir / 'scripts' / 'generate_world.py'
spec = importlib.util.spec_from_file_location('generate_world', source)
world = importlib.util.module_from_spec(spec)
spec.loader.exec_module(world)


class WorldMapTests(unittest.TestCase):
    def setUp(self):
        self.grid = world.rasterize()

    def cell(self, x, y):
        col, row = world.world_to_cell(x, y)
        return self.grid[row][col]

    def test_map_size_and_origin_cover_the_arena(self):
        self.assertEqual(world.map_size(), (168, 128))
        self.assertEqual((len(self.grid[0]), len(self.grid)), (168, 128))
        self.assertEqual(world.map_origin(), (-4.2, -3.2))

    def test_spawn_area_is_free(self):
        # Xe được đặt tại gốc tọa độ; cần trống trong bán kính 0,5 m để xuất phát.
        steps = 20
        for i in range(-steps, steps + 1):
            for j in range(-steps, steps + 1):
                x, y = i * 0.025, j * 0.025
                if math.hypot(x, y) <= 0.5:
                    self.assertEqual(self.cell(x, y), world.FREE, f'Ô tại ({x}, {y}) không trống')

    def test_obstacles_are_occupied_and_outside_is_unknown(self):
        for shape in world.obstacles():
            self.assertEqual(self.cell(shape.x, shape.y), world.OCCUPIED, shape.name)
        self.assertEqual(self.cell(4.15, 0.0), world.UNKNOWN)
        self.assertEqual(self.cell(0.0, -3.15), world.UNKNOWN)
        # Sát mép trong của tường bao vẫn là vùng trống.
        self.assertEqual(self.cell(3.90, 0.0), world.FREE)

    def test_cylinder_is_rasterized_as_a_disc(self):
        cylinder = next(s for s in world.obstacles() if s.name == 'cylinder_a')
        self.assertEqual(self.cell(cylinder.x + cylinder.r - 0.01, cylinder.y), world.OCCUPIED)
        # Góc của hình vuông ngoại tiếp nằm ngoài hình tròn.
        self.assertEqual(self.cell(cylinder.x + cylinder.r - 0.01, cylinder.y + cylinder.r - 0.01), world.FREE)

    def test_world_declares_every_obstacle_and_the_sensors_plugin(self):
        root = ElementTree.fromstring(world.world_sdf())
        plugins = {plugin.get('name') for plugin in root.iter('plugin')}
        self.assertIn('ignition::gazebo::systems::Sensors', plugins)
        self.assertIn('ignition::gazebo::systems::Physics', plugins)
        arena = next(model for model in root.iter('model') if model.get('name') == 'arena')
        names = {collision.get('name') for collision in arena.iter('collision')}
        self.assertEqual(names, {f'{shape.name}_collision' for shape in world.obstacles()})

    def test_committed_files_match_the_generator(self):
        # Nếu ca này lỗi: chạy lại scripts/generate_world.py sau khi sửa bố cục.
        self.assertEqual((package_dir / 'worlds' / 'nav_world.sdf').read_text(encoding='utf-8'), world.world_sdf())
        self.assertEqual((package_dir / 'worlds' / 'gps_world.sdf').read_text(encoding='utf-8'),
                         world.world_sdf([], name='gps_world', lidar=False))
        self.assertEqual((maps_dir / 'nav_map.pgm').read_bytes(), world.pgm_bytes(self.grid))
        self.assertEqual((maps_dir / 'nav_map.yaml').read_text(encoding='utf-8'), world.map_yaml())

    def test_generator_writes_world_and_map_to_separate_packages(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary)
            worlds = target / 'bringup' / 'worlds'
            maps = target / 'localization' / 'maps'
            grid = world.write_outputs(worlds, maps)
            self.assertEqual((worlds / 'nav_world.sdf').read_text(), world.world_sdf())
            self.assertEqual((worlds / 'gps_world.sdf').read_text(),
                             world.world_sdf([], name='gps_world', lidar=False))
            self.assertEqual((maps / 'nav_map.pgm').read_bytes(), world.pgm_bytes(grid))
            self.assertEqual((maps / 'nav_map.yaml').read_text(), world.map_yaml())
            self.assertFalse((worlds.parent / 'maps').exists())


if __name__ == '__main__':
    unittest.main()

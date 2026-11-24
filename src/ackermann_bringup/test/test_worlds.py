"""Kiểm tra hai world Gazebo sinh từ generate_world.py: sân trong phòng và đoạn đường Bách khoa."""

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ElementTree


package_dir = Path(__file__).resolve().parents[1]
source = package_dir / 'scripts' / 'generate_world.py'
spec = importlib.util.spec_from_file_location('generate_world', source)
world = importlib.util.module_from_spec(spec)
spec.loader.exec_module(world)


def arena_collisions(root):
    arena = next(model for model in root.iter('model') if model.get('name') == 'arena')
    return {collision.get('name') for collision in arena.iter('collision')}


class WorldTests(unittest.TestCase):
    def test_indoor_spawn_area_is_clear(self):
        # Xe xuất phát tại gốc; slam_toolbox đặt gốc bản đồ tại đây.
        for shape in world.obstacles():
            self.assertGreater(world.shape_distance(shape, 0.0, 0.0), world.SPAWN_CLEARANCE, shape.name)

    def test_indoor_obstacles_stay_inside_the_walls(self):
        for shape in world.obstacles():
            self.assertLessEqual(abs(shape.x), world.HALF_X, shape.name)
            self.assertLessEqual(abs(shape.y), world.HALF_Y, shape.name)

    def test_world_declares_every_obstacle_and_the_sensors_plugin(self):
        root = ElementTree.fromstring(world.world_sdf())
        plugins = {plugin.get('name') for plugin in root.iter('plugin')}
        for name in ['Sensors', 'NavSat', 'Imu', 'Physics']:
            self.assertIn(f'ignition::gazebo::systems::{name}', plugins)
        self.assertEqual(arena_collisions(root), {f'{shape.name}_collision' for shape in world.obstacles()})

    def test_outdoor_spawn_and_demo_goals_are_clear(self):
        for shape in world.outdoor_obstacles():
            self.assertGreater(world.shape_distance(shape, 0.0, 0.0), world.OUTDOOR_SPAWN_CLEARANCE,
                               shape.name)
            for goal in world.OUTDOOR_DEMO_GOALS:
                self.assertGreater(world.shape_distance(shape, *goal), 1.5, (shape.name, goal))

    def test_outdoor_demo_goals_are_on_the_road_and_reachable(self):
        previous = (0.0, 0.0)
        for goal in world.OUTDOOR_DEMO_GOALS:
            self.assertLessEqual(abs(goal[1]), world.OUTDOOR_ROAD_HALF_WIDTH - 1.5, goal)
            # Global costmap ngoài trời rộng 50 m quanh xe.
            self.assertLess(math.dist(previous, goal), 24.0, goal)
            previous = goal

    def test_outdoor_world_has_sensors_and_its_own_origin(self):
        root = ElementTree.fromstring(world.outdoor_world_sdf())
        plugins = {plugin.get('name') for plugin in root.iter('plugin')}
        for name in ['Sensors', 'NavSat', 'Imu', 'Physics']:
            self.assertIn(f'ignition::gazebo::systems::{name}', plugins)
        coordinates = root.find('world/spherical_coordinates')
        self.assertEqual(float(coordinates.findtext('latitude_deg')), world.OUTDOOR_LATITUDE_DEG)
        self.assertEqual(float(coordinates.findtext('longitude_deg')), world.OUTDOOR_LONGITUDE_DEG)
        self.assertEqual(coordinates.findtext('world_frame_orientation'), 'ENU')
        self.assertEqual(arena_collisions(root),
                         {f'{shape.name}_collision' for shape in world.outdoor_obstacles()})

    def test_shape_distance(self):
        box = world.Box('b', 0.0, 0.0, 2.0, 1.0, '')
        self.assertAlmostEqual(world.shape_distance(box, 2.0, 0.0), 1.0)
        self.assertAlmostEqual(world.shape_distance(box, 0.0, 0.0), -0.5)
        cylinder = world.Cylinder('c', 1.0, 1.0, 0.5, '')
        self.assertAlmostEqual(world.shape_distance(cylinder, 1.0, 3.0), 1.5)

    def test_committed_files_match_the_generator(self):
        # Nếu ca này lỗi: chạy lại scripts/generate_world.py sau khi sửa bố cục.
        self.assertEqual((package_dir / 'worlds' / 'nav_world.sdf').read_text(encoding='utf-8'), world.world_sdf())
        self.assertEqual((package_dir / 'worlds' / 'outdoor_world.sdf').read_text(encoding='utf-8'),
                         world.outdoor_world_sdf())
        self.assertEqual(sorted(path.name for path in (package_dir / 'worlds').glob('*.sdf')),
                         ['nav_world.sdf', 'outdoor_world.sdf'])

    def test_generator_writes_both_worlds(self):
        with tempfile.TemporaryDirectory() as temporary:
            worlds = Path(temporary) / 'worlds'
            world.write_outputs(worlds)
            self.assertEqual((worlds / 'nav_world.sdf').read_text(), world.world_sdf())
            self.assertEqual((worlds / 'outdoor_world.sdf').read_text(), world.outdoor_world_sdf())


if __name__ == '__main__':
    unittest.main()

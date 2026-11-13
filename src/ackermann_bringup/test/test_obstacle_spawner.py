"""Kiểm tra phần logic thuần của node thả chướng ngại vật."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree


source = Path(__file__).resolve().parents[1] / 'scripts' / 'obstacle_spawner.py'
spec = importlib.util.spec_from_file_location('obstacle_spawner', source)
spawner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(spawner)


class ObstacleSpawnerTests(unittest.TestCase):
    def test_sdf_is_a_static_cylinder_with_collision(self):
        root = ElementTree.fromstring(spawner.obstacle_sdf('clicked_obstacle_1', 0.15, 0.4))
        model = root.find('model')
        self.assertEqual(model.get('name'), 'clicked_obstacle_1')
        self.assertEqual(model.findtext('static'), 'true')
        cylinder = model.find('link/collision/geometry/cylinder')
        self.assertEqual(float(cylinder.findtext('radius')), 0.15)
        self.assertEqual(float(cylinder.findtext('length')), 0.4)
        self.assertIsNotNone(model.find('link/visual/geometry/cylinder'))

    def test_accepts_a_free_point_away_from_the_robot(self):
        self.assertIsNone(spawner.placement_error((1.5, 0.0), (0.0, 0.0), [(3.0, 3.0)], 0.15, 0.6))

    def test_rejects_points_on_or_near_the_robot(self):
        self.assertIn('cách xe', spawner.placement_error((0.3, 0.2), (0.0, 0.0), [], 0.15, 0.6))

    def test_rejects_overlap_with_an_existing_obstacle(self):
        self.assertIn('chồng', spawner.placement_error((1.6, 0.0), (0.0, 0.0), [(1.5, 0.1)], 0.15, 0.6))

    def test_rejects_when_robot_position_is_unknown_or_point_invalid(self):
        self.assertIn('/odom', spawner.placement_error((1.0, 0.0), None, [], 0.15, 0.6))
        self.assertIn('không hợp lệ', spawner.placement_error((float('nan'), 0.0), (0.0, 0.0), [], 0.15, 0.6))


if __name__ == '__main__':
    unittest.main()

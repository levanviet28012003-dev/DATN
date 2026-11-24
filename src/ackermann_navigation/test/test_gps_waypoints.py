"""Kiểm tra đọc/ghi danh sách điểm GPS (gps_goal --file, gps_waypoint_logger), không cần ROS graph."""
import importlib.util
import math
import os
from pathlib import Path
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gps_goal = load('gps_goal', ROOT / 'scripts' / 'gps_goal.py')
logger = load('gps_waypoint_logger', ROOT / 'scripts' / 'gps_waypoint_logger.py')
world = load('generate_world', ROOT.parent / 'ackermann_bringup' / 'scripts' / 'generate_world.py')
DEMO_FILE = ROOT / 'config' / 'gps_waypoints_hust.yaml'


class WaypointFileTests(unittest.TestCase):
    def test_parse_accepts_nav2_demo_format_and_ignores_yaw(self):
        data = {'waypoints': [{'latitude': 21.0, 'longitude': 105.8, 'yaw': 1.2},
                              {'latitude': '21.001', 'longitude': 105.8}]}
        self.assertEqual(gps_goal.parse_waypoints(data), [(21.0, 105.8), (21.001, 105.8)])

    def test_parse_rejects_bad_content(self):
        for data in [None, {}, {'waypoints': []}, {'waypoints': [{'latitude': 21.0}]},
                     {'waypoints': [{'latitude': 'a', 'longitude': 105.0}]},
                     {'waypoints': [{'latitude': 95.0, 'longitude': 105.0}]}, {'waypoints': [3]}]:
            with self.assertRaises(ValueError, msg=str(data)):
                gps_goal.parse_waypoints(data)

    def test_load_reports_missing_file_as_value_error(self):
        with self.assertRaises(ValueError):
            gps_goal.load_waypoints('/khong/ton/tai.yaml')

    def test_offset_matches_meters_per_degree_at_hust(self):
        # Bảng độ dài một độ ở vĩ độ 21°: vĩ độ 110,72 km, kinh độ 103,95 km.
        east, north = gps_goal.latlon_offset_m(21.0, 105.8, 21.00001, 105.80001)
        self.assertAlmostEqual(north, 1.1072, places=3)
        self.assertAlmostEqual(east, 1.0395, places=3)
        for a, b in zip(logger.latlon_offset_m(21.0, 105.8, 21.00001, 105.80001), (east, north)):
            self.assertAlmostEqual(a, b, places=9)

    def test_long_legs_numbers_destination_points_from_one(self):
        start = (21.0, 105.8)
        near = (21.0, 105.8 + 20 / 103950)   # khoảng 20 m về Đông
        far = (21.0, 105.8 + 50 / 103950)    # thêm khoảng 30 m
        legs = gps_goal.long_legs([start, near, far], 24.0)
        self.assertEqual([index for index, _ in legs], [3])
        self.assertAlmostEqual(legs[0][1], 30.0, delta=0.1)

    def test_logger_output_round_trips_through_gps_goal(self):
        points = [(21.004985868, 105.844162606), (21.0050129641, 105.8441433687)]
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'diem.yaml')
            logger.save_waypoints(path, points)
            loaded = gps_goal.load_waypoints(path)
            self.assertEqual(logger.read_waypoints(path), loaded)
            self.assertFalse(os.path.exists(path + '.tmp'))
            logger.save_waypoints(path, [])
            self.assertEqual(logger.read_waypoints(path), [])
        for (a, b), (c, d) in zip(points, loaded):
            self.assertLess(math.hypot(*gps_goal.latlon_offset_m(a, b, c, d)), 1e-3)

    def test_average_reports_spread(self):
        latitude, longitude, spread = logger.average_latlon([(21.0, 105.8), (21.00001, 105.8)])
        self.assertAlmostEqual(latitude, 21.000005)
        self.assertAlmostEqual(spread, 0.553, places=2)
        with self.assertRaises(ValueError):
            logger.average_latlon([])


class DemoWaypointTests(unittest.TestCase):
    def test_demo_file_matches_world_goals_and_leg_limit(self):
        with open(DEMO_FILE, encoding='utf-8') as stream:
            points = gps_goal.parse_waypoints(yaml.safe_load(stream))
        origin = (world.OUTDOOR_LATITUDE_DEG, world.OUTDOOR_LONGITUDE_DEG)
        self.assertEqual(len(points), len(world.OUTDOOR_DEMO_GOALS))
        for point, goal in zip(points, world.OUTDOOR_DEMO_GOALS):
            east, north = gps_goal.latlon_offset_m(*origin, *point)
            self.assertLess(math.hypot(east - goal[0], north - goal[1]), 0.01)
        self.assertEqual(gps_goal.long_legs(points, 24.0), [])


if __name__ == '__main__':
    unittest.main()

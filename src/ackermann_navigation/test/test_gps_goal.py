"""Kiểm tra phần thuần logic của gps_goal: phân loại điểm, hướng đích."""
import importlib.util
import math
from pathlib import Path
import unittest

source = Path(__file__).resolve().parents[1] / 'scripts' / 'gps_goal.py'
spec = importlib.util.spec_from_file_location('gps_goal', source)
gps_goal = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gps_goal)


class GpsGoalTests(unittest.TestCase):
    def test_wgs84_point_uses_x_as_longitude(self):
        # Mapviz point_click_publisher với frame wgs84: x = kinh độ, y = vĩ độ.
        self.assertEqual(gps_goal.parse_goal_point('wgs84', 106.5, 10.25), ('wgs84', 10.25, 106.5))

    def test_map_point_passes_through(self):
        self.assertEqual(gps_goal.parse_goal_point('map', 3.0, -2.0), ('map', 3.0, -2.0))

    def test_rejects_unknown_frame_and_invalid_coordinates(self):
        with self.assertRaises(ValueError):
            gps_goal.parse_goal_point('odom', 1.0, 1.0)
        with self.assertRaises(ValueError):
            gps_goal.parse_goal_point('wgs84', 200.0, 10.0)
        with self.assertRaises(ValueError):
            gps_goal.validate_latlon(float('nan'), 106.0)

    def test_goal_yaw_points_from_robot_to_goal(self):
        self.assertAlmostEqual(gps_goal.goal_yaw((0.0, 0.0), (0.0, 5.0)), math.pi / 2)
        self.assertAlmostEqual(gps_goal.goal_yaw((1.0, 1.0), (0.0, 1.0)), math.pi)
        self.assertEqual(gps_goal.goal_yaw((1.0, 1.0), (1.0, 1.0), default=0.3), 0.3)

    def test_transform_point_2d(self):
        x, y = gps_goal.transform_point_2d((1.0, 2.0, math.pi / 2), (1.0, 0.0))
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 3.0)

    def test_quaternion_round_trip(self):
        for yaw in (-3.0, -1.0, 0.0, 0.5, 3.0):
            self.assertAlmostEqual(gps_goal.quaternion_to_yaw(*gps_goal.yaw_to_quaternion(yaw)), yaw)

    def test_refresh_only_when_goal_moved_and_robot_is_far(self):
        # Đích dịch 1,5 m, xe còn cách 10 m: gửi lại.
        self.assertTrue(gps_goal.needs_refresh((10.0, 0.0), (10.0, 1.5), (0.0, 0.0), 1.0, 3.0))
        # Đích dịch ít: không gửi lại.
        self.assertFalse(gps_goal.needs_refresh((10.0, 0.0), (10.0, 0.5), (0.0, 0.0), 1.0, 3.0))
        # Xe đã sát đích: không gửi lại dù đích dịch nhiều.
        self.assertFalse(gps_goal.needs_refresh((10.0, 0.0), (10.0, 2.0), (9.0, 1.0), 1.0, 3.0))

    def test_quaternion(self):
        x, y, z, w = gps_goal.yaw_to_quaternion(math.pi / 2)
        self.assertAlmostEqual(z, math.sqrt(0.5))
        self.assertAlmostEqual(w, math.sqrt(0.5))
        self.assertEqual((x, y), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()

"""Kiểm tra mô hình sai số GPS điện thoại, không cần ROS graph."""
import importlib.util
import math
from pathlib import Path
import statistics
import unittest

from sensor_msgs.msg import NavSatFix, NavSatStatus

source = Path(__file__).resolve().parents[1] / 'scripts' / 'gps_noise_injector.py'
spec = importlib.util.spec_from_file_location('gps_noise_injector', source)
injector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(injector)


def make_fix(lat=10.0, lon=106.0):
    fix = NavSatFix()
    fix.header.frame_id = 'gps_link'
    fix.header.stamp.sec = 7
    fix.status.status = NavSatStatus.STATUS_FIX
    fix.latitude, fix.longitude, fix.altitude = lat, lon, 0.0
    return fix


class ConversionTests(unittest.TestCase):
    def test_meters_per_degree_matches_known_values(self):
        north, east = injector.meters_per_degree(0.0)
        self.assertAlmostEqual(north, 110574.3, delta=1.0)
        self.assertAlmostEqual(east, 111319.5, delta=1.0)
        north, east = injector.meters_per_degree(10.0)
        self.assertAlmostEqual(north, 110608.0, delta=2.0)
        self.assertAlmostEqual(east, 109639.0, delta=2.0)

    def test_offset_round_trip(self):
        lat, lon = injector.offset_latlon(10.0, 106.0, 30.0, -20.0)
        north, east = injector.meters_per_degree(10.0)
        self.assertAlmostEqual((lon - 106.0) * east, 30.0, places=6)
        self.assertAlmostEqual((lat - 10.0) * north, -20.0, places=6)


class NoiseModelTests(unittest.TestCase):
    def test_error_statistics_match_reported_covariance(self):
        model = injector.GpsNoiseModel(white_sigma=1.5, bias_sigma=2.5, bias_tau=5.0, seed=3)
        errors = [model.sample_error(float(t))[0] for t in range(20000)]
        self.assertAlmostEqual(statistics.fmean(errors), 0.0, delta=0.15)
        self.assertAlmostEqual(statistics.pstdev(errors), model.horizontal_sigma(), delta=0.15)
        self.assertAlmostEqual(model.covariance()[0], 1.5 ** 2 + 2.5 ** 2)

    def test_bias_drifts_slowly(self):
        # Với tau dài, hai mẫu cách nhau 1 s gần như cùng bias: sai số tương quan mạnh.
        model = injector.GpsNoiseModel(white_sigma=0.0, bias_sigma=3.0, bias_tau=120.0, seed=1)
        first = model.sample_error(0.0)[0]
        second = model.sample_error(1.0)[0]
        self.assertLess(abs(second - first), 1.0)

    def test_corrupt_keeps_header_and_sets_diagonal_covariance(self):
        model = injector.GpsNoiseModel(seed=2)
        fix = make_fix()
        noisy = model.corrupt(fix, 0.0)
        self.assertEqual(noisy.header.frame_id, 'gps_link')
        self.assertEqual(noisy.position_covariance_type, NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN)
        self.assertNotEqual((noisy.latitude, noisy.longitude), (fix.latitude, fix.longitude))
        # Đổi stamp bản sao không được làm đổi bản tin gốc.
        noisy.header.stamp.sec = 99
        self.assertEqual(fix.header.stamp.sec, 7)
        error = math.hypot(*[(a - b) * scale for a, b, scale in zip(
            (noisy.latitude, noisy.longitude), (fix.latitude, fix.longitude),
            injector.meters_per_degree(10.0))])
        self.assertLess(error, 6 * model.horizontal_sigma())

    def test_noise_off_still_reports_nonzero_covariance(self):
        model = injector.GpsNoiseModel(white_sigma=0.0, bias_sigma=0.0, vertical_sigma=0.0, seed=1)
        self.assertEqual(model.sample_error(0.0), (0.0, 0.0, 0.0))
        covariance = model.covariance()
        self.assertGreater(covariance[0], 0.0)
        self.assertGreater(covariance[8], 0.0)

    def test_rejects_negative_sigma(self):
        with self.assertRaises(ValueError):
            injector.GpsNoiseModel(white_sigma=-1.0)


class DelayLineTests(unittest.TestCase):
    def test_releases_items_after_delay_in_order(self):
        line = injector.DelayLine(0.5)
        line.push(1.0, 'a')
        line.push(2.0, 'b')
        self.assertEqual(line.pop_due(1.4), [])
        self.assertEqual(line.pop_due(1.5), ['a'])
        self.assertEqual(line.pop_due(3.0), ['b'])

    def test_rejects_negative_delay(self):
        with self.assertRaises(ValueError):
            injector.DelayLine(-0.1)


if __name__ == '__main__':
    unittest.main()

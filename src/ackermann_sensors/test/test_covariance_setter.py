"""Kiểm tra cách điền covariance cho Imu và Odometry."""
import importlib.util
from pathlib import Path
import unittest

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

source = Path(__file__).resolve().parents[1] / 'scripts' / 'covariance_setter.py'
spec = importlib.util.spec_from_file_location('covariance_setter', source)
setter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setter)


class CovarianceSetterTests(unittest.TestCase):
    def test_diagonal(self):
        self.assertEqual(setter.diagonal([1, 2]), [1.0, 0.0, 0.0, 2.0])

    def test_imu_covariance(self):
        message = setter.set_imu_covariance(Imu(), [1e-3, 2e-3, 3e-3], [1e-5] * 3, [1e-4] * 3)
        self.assertEqual(list(message.orientation_covariance)[::4], [1e-3, 2e-3, 3e-3])
        self.assertEqual(list(message.angular_velocity_covariance)[8], 1e-5)
        self.assertEqual(list(message.linear_acceleration_covariance)[4], 1e-4)

    def test_imu_without_orientation(self):
        message = setter.set_imu_covariance(Imu(), [-1.0, 0.0, 0.0], [1e-5] * 3, [1e-4] * 3)
        self.assertEqual(message.orientation_covariance[0], -1.0)

    def test_odometry_covariance(self):
        message = setter.set_odometry_covariance(Odometry(), [1.0] * 6, [1, 2, 3, 4, 5, 6])
        self.assertEqual(list(message.twist.covariance)[::7], [1, 2, 3, 4, 5, 6])
        self.assertEqual(message.pose.covariance[35], 1.0)

    def test_check_variances(self):
        self.assertEqual(setter.check_variances('x', [1, 2, 3], 3), [1.0, 2.0, 3.0])
        with self.assertRaises(ValueError):
            setter.check_variances('x', [1, 2], 3)
        with self.assertRaises(ValueError):
            setter.check_variances('x', [0.0, 1.0, 1.0], 3)
        self.assertEqual(setter.check_variances('x', [-1.0, 1.0, 1.0], 3, allow_negative_first=True)[0], -1.0)


if __name__ == '__main__':
    unittest.main()

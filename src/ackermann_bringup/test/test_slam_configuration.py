"""Kiểm tra cấu hình định vị bằng SLAM (kiến trúc 2026-11-23) khớp URDF, bridge, EKF và Nav2."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import xacro
import yaml

package_dir = Path(__file__).resolve().parents[1]
src = package_dir.parent
localization = src / 'ackermann_localization'


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


class SlamConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.slam = load_yaml(localization / 'config' / 'slam_toolbox.yaml')['slam_toolbox']['ros__parameters']
        cls.ekf_local = load_yaml(localization / 'config' / 'ekf_local.yaml')['ekf_odom']['ros__parameters']
        cls.bridge = {entry['ros_topic_name']: entry for entry in load_yaml(
            package_dir / 'config' / 'bridge.yaml')}
        cls.nav2 = load_yaml(src / 'ackermann_navigation' / 'config' / 'nav2_params.yaml')
        cls.robot = ElementTree.fromstring(xacro.process_file(
            str(src / 'ackermann_description' / 'urdf' / 'ackermann.urdf.xacro')).toxml())

    def test_slam_frames_follow_the_architecture(self):
        # slam_toolbox phát map → odom; odom → base_link do ekf_odom phát.
        self.assertEqual(self.slam['map_frame'], 'map')
        self.assertEqual(self.slam['odom_frame'], 'odom')
        self.assertEqual(self.slam['mode'], 'mapping')
        links = {link.get('name') for link in self.robot.iter('link')}
        self.assertIn(self.slam['base_frame'], links)
        self.assertEqual(self.ekf_local['world_frame'], self.slam['odom_frame'])
        self.assertEqual(self.ekf_local['base_link_frame'], self.slam['base_frame'])
        self.assertTrue(self.ekf_local['publish_tf'])

    def test_slam_reads_the_simulated_lidar(self):
        self.assertIn(self.slam['scan_topic'], self.bridge)
        lidar = next(sensor for sensor in self.robot.iter('sensor') if sensor.get('name') == 'lidar')
        self.assertEqual(self.bridge[self.slam['scan_topic']]['gz_topic_name'], lidar.findtext('topic'))
        self.assertLessEqual(self.slam['max_laser_range'], float(lidar.find('lidar/range/max').text))

    def test_slow_vehicle_adds_scans_often_enough(self):
        # 0,2 m/s: mỗi scan mới cách nhau tối đa khoảng 1 s quãng đường.
        self.assertLessEqual(self.slam['minimum_travel_distance'], 0.25)
        self.assertLessEqual(self.slam['minimum_travel_heading'], 0.25)

    def test_ekf_local_uses_gyro_rate_only(self):
        self.assertEqual(self.ekf_local['odom0'], '/wheel_odom')
        self.assertEqual(self.ekf_local['imu0'], '/imu/data')
        self.assertFalse(self.ekf_local['imu0_config'][5])   # không lấy yaw tuyệt đối
        self.assertTrue(self.ekf_local['imu0_config'][11])   # lấy tốc độ quay
        self.assertTrue(self.ekf_local['two_d_mode'])

    def test_ground_truth_is_not_odometry(self):
        self.assertNotIn('/odom', self.bridge)
        self.assertNotIn('/tf', self.bridge)
        self.assertIn('/sim/wheel_odom', self.bridge)
        self.assertIn('/sim/imu', self.bridge)

    def test_nav2_global_costmap_uses_the_slam_map(self):
        costmap = self.nav2['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(costmap['global_frame'], self.slam['map_frame'])
        self.assertIn('static_layer', costmap['plugins'])
        self.assertTrue(costmap['static_layer']['map_subscribe_transient_local'])


if __name__ == '__main__':
    unittest.main()

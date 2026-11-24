"""Kiểm tra frame, cảm biến, world và bridge IMU khớp nhau."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / 'ackermann_description'


def load_robot():
    return ET.fromstring(xacro.process_file(
        str(DESCRIPTION / 'urdf' / 'ackermann.urdf.xacro')).toxml())


class IMUConfigurationTests(unittest.TestCase):
    def test_imu_has_a_fixed_frame_aligned_with_base_link(self):
        robot = load_robot()
        link = robot.find("link[@name='imu_link']")
        self.assertIsNotNone(link)
        # Link không có khối lượng có thể bị bỏ khi chuyển URDF sang SDF, kéo theo mất cảm biến.
        self.assertGreater(float(link.find('inertial/mass').get('value')), 0.0)
        joint = robot.find("joint[@name='imu_joint']")
        self.assertEqual(joint.get('type'), 'fixed')
        self.assertEqual(joint.find('parent').get('link'), 'base_link')
        self.assertEqual(joint.find('child').get('link'), 'imu_link')
        rpy = [float(value) for value in joint.find('origin').get('rpy').split()]
        self.assertEqual(rpy, [0.0, 0.0, 0.0])

    def test_sensor_topic_and_bridge_match(self):
        sensor = load_robot().find("gazebo[@reference='imu_link']/sensor")
        self.assertEqual(sensor.get('type'), 'imu')
        self.assertGreaterEqual(float(sensor.findtext('update_rate')), 20.0)
        self.assertEqual(sensor.findtext('ignition_frame_id'), 'imu_link')
        self.assertEqual(
            sensor.findtext('imu/orientation_reference_frame/localization'), 'ENU')
        # IMU thô ra /sim/imu; covariance_setter bỏ hướng tuyệt đối và phát /imu/data như gyro điện thoại.
        bridges = yaml.safe_load((PACKAGE / 'config' / 'bridge.yaml').read_text())
        entries = [entry for entry in bridges if entry['ros_topic_name'] == '/sim/imu']
        self.assertEqual(len(entries), 1)
        bridge = entries[0]
        self.assertEqual(bridge['gz_topic_name'], sensor.findtext('topic'))
        self.assertEqual(bridge['ros_type_name'], 'sensor_msgs/msg/Imu')
        self.assertEqual(bridge['gz_type_name'], 'ignition.msgs.IMU')
        self.assertEqual(bridge['direction'], 'GZ_TO_ROS')

    def test_sensor_has_gyro_and_accel_noise(self):
        imu = load_robot().find("gazebo[@reference='imu_link']/sensor/imu")
        for group in ['angular_velocity', 'linear_acceleration']:
            for axis in 'xyz':
                with self.subTest(group=group, axis=axis):
                    noise = imu.find(f'{group}/{axis}/noise')
                    self.assertEqual(noise.get('type'), 'gaussian')
                    self.assertGreater(float(noise.findtext('stddev')), 0.0)

    def test_worlds_load_the_imu_system(self):
        for filename in ['nav_world.sdf', 'outdoor_world.sdf']:
            with self.subTest(world=filename):
                world = ET.parse(PACKAGE / 'worlds' / filename).getroot().find('world')
                imu = [plugin for plugin in world.findall('plugin')
                       if plugin.get('name') == 'ignition::gazebo::systems::Imu']
                self.assertEqual(len(imu), 1)
                self.assertEqual(imu[0].get('filename'), 'ignition-gazebo-imu-system')


if __name__ == '__main__':
    unittest.main()

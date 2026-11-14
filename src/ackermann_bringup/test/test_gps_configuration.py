"""Kiểm tra cảm biến, frame, world và bridge GPS khớp nhau."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
DESCRIPTION = PACKAGE.parent / 'ackermann_description'


class GPSConfigurationTests(unittest.TestCase):
    def test_gps_has_a_fixed_frame_on_the_robot(self):
        robot = ET.fromstring(xacro.process_file(
            str(DESCRIPTION / 'urdf' / 'ackermann.urdf.xacro')).toxml())
        self.assertIsNotNone(robot.find("link[@name='gps_link']"))
        joint = robot.find("joint[@name='gps_joint']")
        self.assertEqual(joint.get('type'), 'fixed')
        self.assertEqual(joint.find('parent').get('link'), 'base_link')
        self.assertEqual(joint.find('child').get('link'), 'gps_link')
        self.assertIsNotNone(joint.find('origin'))

    def test_sensor_topic_and_bridge_match(self):
        robot = ET.fromstring(xacro.process_file(
            str(DESCRIPTION / 'urdf' / 'ackermann.urdf.xacro')).toxml())
        sensor = robot.find("gazebo[@reference='gps_link']/sensor")
        self.assertEqual(sensor.get('type'), 'navsat')
        self.assertGreater(float(sensor.findtext('update_rate')), 0.0)
        self.assertEqual(sensor.findtext('ignition_frame_id'), 'gps_link')
        bridges = yaml.safe_load((PACKAGE / 'config' / 'bridge.yaml').read_text())
        entries = [entry for entry in bridges if entry['ros_topic_name'] == '/gps/fix']
        self.assertEqual(len(entries), 1)
        bridge = entries[0]
        self.assertEqual(bridge['gz_topic_name'], sensor.findtext('topic'))
        self.assertEqual(bridge['ros_type_name'], 'sensor_msgs/msg/NavSatFix')
        self.assertEqual(bridge['gz_type_name'], 'ignition.msgs.NavSat')
        self.assertEqual(bridge['direction'], 'GZ_TO_ROS')

    def test_worlds_have_navsat_and_the_same_geographic_origin(self):
        origins = []
        for filename in ['gps_world.sdf', 'nav_world.sdf']:
            with self.subTest(world=filename):
                world = ET.parse(PACKAGE / 'worlds' / filename).getroot().find('world')
                navsat = [plugin for plugin in world.findall('plugin')
                          if plugin.get('name') == 'ignition::gazebo::systems::NavSat']
                self.assertEqual(len(navsat), 1)
                self.assertEqual(navsat[0].get('filename'), 'ignition-gazebo-navsat-system')
                coordinates = world.find('spherical_coordinates')
                self.assertEqual(coordinates.findtext('surface_model'), 'EARTH_WGS84')
                self.assertEqual(coordinates.findtext('world_frame_orientation'), 'ENU')
                lat = float(coordinates.findtext('latitude_deg'))
                lon = float(coordinates.findtext('longitude_deg'))
                self.assertTrue(-90.0 <= lat <= 90.0)
                self.assertTrue(-180.0 <= lon <= 180.0)
                origins.append(ET.tostring(coordinates))
        self.assertEqual(origins[0], origins[1])

    def test_empty_gps_world_does_not_need_a_gpu_renderer(self):
        world = ET.parse(PACKAGE / 'worlds' / 'gps_world.sdf').getroot().find('world')
        self.assertIsNone(world.find("model[@name='arena']"))
        self.assertFalse(any(plugin.get('name') == 'ignition::gazebo::systems::Sensors'
                             for plugin in world.findall('plugin')))


if __name__ == '__main__':
    unittest.main()

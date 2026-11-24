"""Kiểm tra cấu hình mô phỏng ngoài trời khớp nhau: bridge, URDF, Nav2, Mapviz."""

import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ElementTree

import xacro
import yaml

package_dir = Path(__file__).resolve().parents[1]
src = package_dir.parent


def load_yaml(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


spec = importlib.util.spec_from_file_location('outdoor_launch', package_dir / 'launch' / 'outdoor.launch.py')
outdoor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(outdoor)


class OutdoorConfigurationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bridge = {entry['ros_topic_name']: entry for entry in load_yaml(
            package_dir / 'config' / 'bridge.yaml')}
        cls.nav2 = load_yaml(src / 'ackermann_navigation' / 'config' / 'nav2_outdoor.yaml')
        cls.mapviz = load_yaml(package_dir / 'config' / 'mapviz_outdoor.mvc')
        robot = ElementTree.fromstring(xacro.process_file(
            str(src / 'ackermann_description' / 'urdf' / 'ackermann.urdf.xacro')).toxml())
        cls.sensor_topics = {sensor.get('name'): sensor.findtext('topic') for sensor in robot.iter('sensor')}

    def test_ground_truth_is_not_used_as_odometry(self):
        # TF odom → base_link do ekf_odom phát; bridge không được phát /odom hay /tf của Gazebo.
        self.assertNotIn('/odom', self.bridge)
        self.assertNotIn('/tf', self.bridge)
        self.assertEqual(self.bridge['/ground_truth/odom']['gz_topic_name'], '/model/ackermann_car/odometry')

    def test_raw_sensors_come_from_the_robot_model(self):
        self.assertEqual(self.bridge['/sim/imu']['gz_topic_name'], self.sensor_topics['imu'])
        self.assertEqual(self.bridge['/sim/gps/fix']['gz_topic_name'], self.sensor_topics['gps'])
        self.assertEqual(self.bridge['/scan']['gz_topic_name'], self.sensor_topics['lidar'])

    def test_nav2_plans_in_map_with_a_rolling_window(self):
        # Bản đồ SLAM chỉ phủ vùng Lidar đã quét; global costmap cửa sổ trượt để đích xa vẫn lập được đường.
        self.assertEqual(self.nav2['bt_navigator']['ros__parameters']['global_frame'], 'map')
        costmap = self.nav2['global_costmap']['global_costmap']['ros__parameters']
        self.assertEqual(costmap['global_frame'], 'map')
        self.assertTrue(costmap['rolling_window'])
        self.assertNotIn('static_layer', costmap['plugins'])
        tolerance = self.nav2['controller_server']['ros__parameters']['general_goal_checker']
        self.assertGreaterEqual(tolerance['xy_goal_tolerance'], 1.0)

    def test_mapviz_topics_exist_in_the_pipeline(self):
        displays = {d['type'].split('/')[-1] + ':' + d['config'].get('topic', d['config'].get('frame', ''))
                    for d in self.mapviz['displays']}
        for expected in ['navsat:/gps/fix', 'navsat:/sim/gps/fix', 'path:/plan',
                         'laserscan:/scan', 'point_click_publisher:/gps_goal', 'pose:/gps_goal/pose']:
            self.assertIn(expected, displays)
        self.assertEqual(self.mapviz['fixed_frame'], 'map')
        click = next(d for d in self.mapviz['displays'] if d['type'].endswith('point_click_publisher'))
        self.assertEqual(click['config']['output_frame'], 'wgs84')

    def test_launch_reads_origin_from_the_world(self):
        world = package_dir / 'worlds' / 'outdoor_world.sdf'
        latitude, longitude, _ = outdoor.world_origin(world)
        generator_spec = importlib.util.spec_from_file_location(
            'generate_world', package_dir / 'scripts' / 'generate_world.py')
        generator = importlib.util.module_from_spec(generator_spec)
        generator_spec.loader.exec_module(generator)
        self.assertEqual((latitude, longitude),
                         (generator.OUTDOOR_LATITUDE_DEG, generator.OUTDOOR_LONGITUDE_DEG))


if __name__ == '__main__':
    unittest.main()

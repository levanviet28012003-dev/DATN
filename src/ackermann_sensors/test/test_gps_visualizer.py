"""Kiểm tra nội dung GPS hiển thị, không cần khởi chạy ROS graph."""
import importlib.util
from pathlib import Path
import unittest

from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker

source = Path(__file__).resolve().parents[1] / 'scripts' / 'gps_visualizer.py'
spec = importlib.util.spec_from_file_location('gps_visualizer', source)
visualizer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(visualizer)


class GPSVisualizerTests(unittest.TestCase):
    def setUp(self):
        self.fix = NavSatFix()
        self.fix.header.frame_id = 'gps_link'
        self.fix.latitude = 10.0
        self.fix.longitude = 106.00000574
        self.fix.altitude = 0.0845
        self.fix.status.status = NavSatStatus.STATUS_FIX

    def test_displays_received_coordinates_in_the_sensor_frame(self):
        marker = visualizer.make_gps_marker(self.fix)
        self.assertEqual(marker.type, Marker.TEXT_VIEW_FACING)
        self.assertEqual(marker.header.frame_id, 'gps_link')
        self.assertTrue(marker.frame_locked)
        self.assertIn('Lat:10.00000000deg', marker.text)
        self.assertIn('Lon:106.00000574deg', marker.text)
        self.assertIn('Alt:0.085m', marker.text)
        self.assertGreater(marker.scale.z, 0)
        self.assertEqual(marker.color.a, 1.0)

    def test_no_fix_does_not_display_coordinates_as_valid(self):
        self.fix.status.status = NavSatStatus.STATUS_NO_FIX
        marker = visualizer.make_gps_marker(self.fix)
        self.assertIn('CHUA_CO_DINH_VI', marker.text)
        self.assertNotIn('Lat:', marker.text)
        self.assertEqual(marker.color.g, 0.0)

    def test_invalid_coordinates_are_rejected(self):
        for latitude, longitude in [(float('nan'), 106.0), (91.0, 106.0),
                                    (10.0, float('inf')), (10.0, -181.0)]:
            self.fix.latitude, self.fix.longitude = latitude, longitude
            self.assertNotIn('Lat:', visualizer.make_gps_marker(self.fix).text)

    def test_missing_altitude_is_not_shown_as_a_measured_zero(self):
        self.fix.altitude = float('nan')
        self.assertIn('Alt:N/A', visualizer.make_gps_marker(self.fix).text)


if __name__ == '__main__':
    unittest.main()

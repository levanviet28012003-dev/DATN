#!/usr/bin/env python3
"""Hiện tọa độ nhận từ /gps/fix bằng chữ gắn với cảm biến trong RViz."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker


def make_gps_marker(fix, text_height=0.025, text_color=(1.0, 1.0, 0.0)):
    """Chữ thể hiện dữ liệu GPS; vị trí chữ lấy theo TF, không phải nghiệm định vị."""
    marker = Marker()
    marker.header.frame_id = fix.header.frame_id or 'gps_link'
    # Stamp 0 dùng TF mới nhất; frame_locked giữ chữ đi cùng cảm biến.
    marker.ns = 'gps_coordinates'
    marker.id = 0
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.frame_locked = True
    marker.pose.position.z = 0.10
    marker.pose.orientation.w = 1.0
    marker.scale.z = text_height
    marker.color.r = 1.0
    marker.color.a = 1.0

    valid = (fix.status.status >= NavSatStatus.STATUS_FIX
             and math.isfinite(fix.latitude) and -90 <= fix.latitude <= 90
             and math.isfinite(fix.longitude) and -180 <= fix.longitude <= 180)
    if valid:
        marker.color.r, marker.color.g, marker.color.b = text_color
        altitude = f'{fix.altitude:.3f}m' if math.isfinite(fix.altitude) else 'N/A'
        # Tránh lỗi mất chữ có khoảng trắng trên bản RViz Humble đang dùng.
        marker.text = (f'GPS(WGS84)\nLat:{fix.latitude:.8f}deg\n'
                       f'Lon:{fix.longitude:.8f}deg\nAlt:{altitude}')
    else:
        marker.text = 'GPS:CHUA_CO_DINH_VI'
    return marker


class GPSVisualizer(Node):
    def __init__(self):
        super().__init__('gps_visualizer')
        fix_topic = self.declare_parameter('fix_topic', '/gps/fix').value
        marker_topic = self.declare_parameter('marker_topic', '/gps/marker').value
        self.text_height = self.declare_parameter('text_height', 0.025).value
        if not math.isfinite(self.text_height) or self.text_height <= 0:
            raise ValueError('text_height phải là số hữu hạn lớn hơn 0 (m).')
        self.text_color = self.declare_parameter('text_color', [1.0, 1.0, 0.0]).value
        if len(self.text_color) != 3 or any(
                not math.isfinite(value) or not 0 <= value <= 1 for value in self.text_color):
            raise ValueError('text_color cần ba giá trị RGB trong khoảng 0 đến 1.')
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Marker, marker_topic, marker_qos)
        self.subscription = self.create_subscription(
            NavSatFix, fix_topic, self.on_fix, qos_profile_sensor_data)
        self.get_logger().info(f'Hiển thị tọa độ từ {fix_topic} qua {marker_topic}.')

    def on_fix(self, message):
        self.publisher.publish(make_gps_marker(message, self.text_height, self.text_color))


def main(args=None):
    rclpy.init(args=args)
    node = GPSVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

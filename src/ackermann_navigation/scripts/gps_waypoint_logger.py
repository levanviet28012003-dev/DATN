#!/usr/bin/env python3
"""Ghi danh sách điểm GPS để gps_goal --file đi lại.

Lái xe (tay hoặc bằng đích GPS) tới từng chỗ cần đi, dừng hẳn, bấm Enter để ghi vị trí hiện tại:
  gps_waypoint_logger --output ~/gps_waypoints.yaml

Lệnh trong terminal: Enter ghi điểm, u xóa điểm cuối, l liệt kê, q lưu và thoát. File được ghi lại
sau mỗi thay đổi nên dừng ngang (Ctrl+C) không mất điểm đã ghi.

Nguồn là GPS điện thoại /gps/fix (1 Hz, nhảy vài mét); mỗi điểm là trung bình các mẫu trong
averaging_time giây cuối (mặc định 10 s) để giảm nhiễu. Từ 2026-11-24 không còn /gps/filtered của
ekf_map (kiến trúc SLAM, GPS không tham gia định vị).
"""

import argparse
import math
import os
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix, NavSatStatus

DEFAULT_MAX_SPREAD = 5.0  # m: GPS điện thoại nhảy 3–5 m; lệch hơn nữa thì coi như xe đang chạy


def latlon_offset_m(latitude0, longitude0, latitude1, longitude1):
    """(Đông, Bắc) tính bằng mét; cùng công thức với gps_goal.latlon_offset_m."""
    a, e2 = 6378137.0, 6.69437999014e-3
    phi = math.radians((latitude0 + latitude1) / 2.0)
    w = math.sqrt(1.0 - e2 * math.sin(phi) ** 2)
    return ((longitude1 - longitude0) * a / w * math.cos(phi) * math.pi / 180.0,
            (latitude1 - latitude0) * a * (1.0 - e2) / w ** 3 * math.pi / 180.0)


def average_latlon(samples):
    """(vĩ độ, kinh độ trung bình, độ lệch lớn nhất của mẫu so với trung bình tính bằng m)."""
    if not samples:
        raise ValueError('Chưa có mẫu GPS.')
    latitude = sum(s[0] for s in samples) / len(samples)
    longitude = sum(s[1] for s in samples) / len(samples)
    spread = max(math.hypot(*latlon_offset_m(latitude, longitude, *s)) for s in samples)
    return latitude, longitude, spread


def waypoints_yaml(points):
    """Nội dung file theo định dạng gps_goal đọc (tương thích định dạng demo GPS của Nav2, bỏ yaw)."""
    lines = ['# Ghi bằng gps_waypoint_logger; đi lại bằng: ros2 run ackermann_navigation gps_goal --file <file>',
             'waypoints:']
    for latitude, longitude in points:
        lines += [f'  - latitude: {latitude:.9f}', f'    longitude: {longitude:.9f}']
    if not points:
        lines[-1] = 'waypoints: []'
    return '\n'.join(lines) + '\n'


def read_waypoints(path):
    """Điểm đã có trong file để ghi tiếp; file rỗng ('waypoints: []') hợp lệ."""
    import yaml
    try:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'Không đọc được {path}: {error}') from None
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list):
        raise ValueError(f"{path} không có danh sách 'waypoints'.")
    try:
        return [(float(item['latitude']), float(item['longitude'])) for item in data['waypoints']]
    except (TypeError, KeyError, ValueError):
        raise ValueError(f'{path} có điểm thiếu latitude/longitude dạng số.') from None


def save_waypoints(path, points):
    """Ghi qua file tạm rồi đổi tên, để file cũ không bị hỏng nếu dừng giữa chừng."""
    temporary = path + '.tmp'
    with open(temporary, 'w', encoding='utf-8') as stream:
        stream.write(waypoints_yaml(points))
    os.replace(temporary, path)


class WaypointLogger(Node):
    def __init__(self, topic, averaging_time):
        super().__init__('gps_waypoint_logger')
        self.averaging_time = averaging_time
        self.lock = threading.Lock()
        self.samples = []  # (thời điểm s, vĩ độ, kinh độ)
        self.create_subscription(NavSatFix, topic, self.on_fix, qos_profile_sensor_data)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_fix(self, message):
        if message.status.status < NavSatStatus.STATUS_FIX:
            return
        if not (math.isfinite(message.latitude) and math.isfinite(message.longitude)):
            return
        now = self.now_s()
        with self.lock:
            self.samples.append((now, message.latitude, message.longitude))
            self.samples = [s for s in self.samples if now - s[0] <= self.averaging_time]

    def current(self):
        """Vị trí trung bình trong cửa sổ, hoặc ValueError nếu không có mẫu mới."""
        now = self.now_s()
        with self.lock:
            recent = [(s[1], s[2]) for s in self.samples if now - s[0] <= self.averaging_time]
        return average_latlon(recent)


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    parser = argparse.ArgumentParser(description='Ghi danh sách điểm GPS cho gps_goal --file.')
    parser.add_argument('--output', default=os.path.expanduser('~/gps_waypoints.yaml'))
    parser.add_argument('--append', action='store_true', help='Ghi tiếp vào file đã có.')
    parser.add_argument('--topic', default='/gps/fix')
    parser.add_argument('--averaging-time', type=float, default=10.0)
    parser.add_argument('--max-spread', type=float, default=DEFAULT_MAX_SPREAD,
                        help='Mẫu lệch khỏi trung bình hơn mức này (m) thì coi như xe đang chạy.')
    parser.add_argument('--max-leg', type=float, default=24.0,
                        help='Cảnh báo khi chặng dài hơn (bằng max_goal_distance của gps_goal).')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
    output = os.path.abspath(os.path.expanduser(args.output))

    points = []
    if os.path.exists(output):
        if not args.append:
            print(f'{output} đã tồn tại; thêm --append để ghi tiếp hoặc chọn --output khác.', file=sys.stderr)
            rclpy.shutdown()
            return 2
        try:
            points = read_waypoints(output)
        except ValueError as error:
            print(error, file=sys.stderr)
            rclpy.shutdown()
            return 2

    node = WaypointLogger(args.topic, args.averaging_time)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    print(f'Ghi vào {output} ({len(points)} điểm có sẵn), nguồn {args.topic}.')
    print('Enter: ghi điểm | u: xóa điểm cuối | l: liệt kê | q: lưu và thoát')
    try:
        while True:
            command = input(f'[{len(points)} điểm] > ').strip().lower()
            if command == 'q':
                break
            if command == 'u':
                if points:
                    points.pop()
                    save_waypoints(output, points)
                    print('Đã xóa điểm cuối.')
                continue
            if command == 'l':
                for index, (latitude, longitude) in enumerate(points, start=1):
                    print(f'  {index}: {latitude:.7f}, {longitude:.7f}')
                continue
            if command:
                print('Lệnh không hiểu; Enter, u, l hoặc q.')
                continue
            try:
                latitude, longitude, spread = node.current()
            except ValueError:
                print(f'Không có {args.topic} trong {args.averaging_time:g} s qua; định vị chạy chưa?')
                continue
            if spread > args.max_spread:
                print(f'Vị trí dao động {spread:.2f} m trong {args.averaging_time:g} s; dừng xe hẳn rồi ghi lại.')
                continue
            if points:
                leg = math.hypot(*latlon_offset_m(*points[-1], latitude, longitude))
                if leg > args.max_leg:
                    print(f'Cảnh báo: chặng dài {leg:.1f} m, vượt {args.max_leg:.0f} m; gps_goal sẽ từ chối. '
                          'Bấm u rồi ghi thêm điểm ở giữa.')
            points.append((latitude, longitude))
            save_waypoints(output, points)
            print(f'Điểm {len(points)}: {latitude:.7f}, {longitude:.7f}')
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        if points or os.path.exists(output):
            save_waypoints(output, points)
            print(f'Đã lưu {len(points)} điểm vào {output}.')
        else:
            print('Chưa ghi điểm nào; không tạo file.')
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())

#!/usr/bin/env python3
"""Bám quỹ đạo tròn hoặc số 8 bằng phản hồi odometry và thuật toán Pure Pursuit."""

import bisect
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


class ReferencePath:
    def __init__(self, shape, radius, laps, origin):
        self.points = []
        self.lengths = [0.0]
        ox, oy, yaw = origin
        c, s = math.cos(yaw), math.sin(yaw)
        for i in range(600 * laps + 1):
            t = 2 * math.pi * i / 600
            if shape == 'circle':
                x, y = radius * math.sin(t), radius * (1 - math.cos(t))
            else:
                x, y = radius * math.sin(t), radius * math.sin(2 * t) / 2
                # Xoay quỹ đạo số 8 để tiếp tuyến ban đầu cùng hướng đầu xe.
                x, y = (x + y) / math.sqrt(2), (y - x) / math.sqrt(2)
            point = (ox + c*x - s*y, oy + s*x + c*y)
            if self.points:
                self.lengths.append(self.lengths[-1] + math.dist(self.points[-1], point))
            self.points.append(point)
        self.total_length = self.lengths[-1]

    def project(self, x, y, progress, window):
        # Chỉ tìm gần tiến độ hiện tại để không nhảy nhánh tại giao điểm hình số 8.
        first = max(0, bisect.bisect_left(self.lengths, max(0.0, progress - 0.1)) - 1)
        last = min(len(self.points) - 1, bisect.bisect_right(self.lengths, progress + window))
        best_distance, best_progress = float('inf'), progress
        for i in range(first, last):
            ax, ay = self.points[i]
            bx, by = self.points[i + 1]
            dx, dy = bx - ax, by - ay
            ratio = max(0.0, min(1.0, ((x-ax)*dx + (y-ay)*dy) / (dx*dx + dy*dy)))
            distance = math.hypot(x - ax - ratio*dx, y - ay - ratio*dy)
            if distance < best_distance:
                best_distance = distance
                best_progress = self.lengths[i] + ratio*(self.lengths[i+1] - self.lengths[i])
        return max(progress, best_progress), best_distance

    def target(self, progress):
        progress = max(0.0, min(self.total_length, progress))
        i = min(len(self.points) - 2, max(0, bisect.bisect_right(self.lengths, progress) - 1))
        fraction = (progress - self.lengths[i]) / (self.lengths[i+1] - self.lengths[i])
        a, b = self.points[i], self.points[i+1]
        return (a[0] + fraction*(b[0]-a[0]), a[1] + fraction*(b[1]-a[1]))


class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')
        defaults = dict(trajectory='circle', radius=1.0, laps=1, speed=0.12,
                        lookahead=0.4, goal_tolerance=0.06, max_curvature=2.5,
                        max_yaw_rate=0.5, max_tracking_error=0.75,
                        odom_timeout=1.0, startup_timeout=15.0, max_duration=240.0)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        self.validate_settings()
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_pub = self.create_publisher(Path, '/trajectory/reference', path_qos)
        self.traveled_pub = self.create_publisher(Path, '/trajectory/traveled', path_qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odometry, qos_profile_sensor_data)
        self.path = None
        self.pose = None
        self.progress = 0.0
        self.frame = None
        self.last_odom_wall = None
        self.odom_stamp = None
        self.start_stamp = None
        self.started_wall = time.monotonic()
        self.last_path_wall = 0.0
        self.last_log_wall = 0.0
        self.traveled = Path()
        self.finished = False
        self.exit_code = 0
        # Đồng hồ thực giúp phát hiện mất odometry ngay cả khi Gazebo tạm dừng.
        self.timer = self.create_timer(0.05, self.control, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Đang chờ /odom và bridge; chỉ chạy một nguồn phát /cmd_vel.')

    def validate_settings(self):
        p = self.settings
        if p['trajectory'] not in ('circle', 'figure8'):
            raise ValueError('trajectory phải là circle hoặc figure8.')
        if type(p['laps']) is not int or not 1 <= p['laps'] <= 20:
            raise ValueError('laps phải là số nguyên từ 1 đến 20.')
        for name in p:
            if name not in ('trajectory', 'laps'):
                if not math.isfinite(p[name]) or p[name] <= 0:
                    raise ValueError(f'{name} phải là số hữu hạn lớn hơn 0.')
        if p['speed'] > 0.3 or p['max_yaw_rate'] > 0.5 or p['max_curvature'] > 2.5:
            raise ValueError('Giới hạn mô hình: speed ≤ 0.3, max_yaw_rate ≤ 0.5, max_curvature ≤ 2.5.')
        minimum_radius = (1.0 if p['trajectory'] == 'circle' else 4.791) / (0.8 * p['max_curvature'])
        if p['radius'] < minimum_radius:
            raise ValueError(f'radius quá nhỏ cho {p["trajectory"]}; cần ít nhất {minimum_radius:.3f} m.')
        if p['lookahead'] > p['radius'] or p['goal_tolerance'] >= p['lookahead']:
            raise ValueError('Cần goal_tolerance < lookahead ≤ radius.')

    def on_odometry(self, message):
        if self.finished:
            return
        position, q = message.pose.pose.position, message.pose.pose.orientation
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        values = (position.x, position.y, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[2:]) < 1e-8:
            self.finish('Odometry không hợp lệ.', failed=True)
            return
        frame = message.header.frame_id or 'odom'
        if self.odom_stamp is not None and (stamp < self.odom_stamp or frame != self.frame):
            self.finish('Odometry đã đổi hệ tọa độ hoặc đặt lại thời gian; hãy chạy lại node.', failed=True)
            return
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(w*z + x*y), 1-2*(y*y + z*z))
        self.pose = (position.x, position.y, yaw)
        if self.odom_stamp != stamp:
            self.last_odom_wall = time.monotonic()
        self.odom_stamp = stamp
        self.frame = frame
        if self.path is None:
            self.path = ReferencePath(self.settings['trajectory'], self.settings['radius'],
                                      self.settings['laps'], self.pose)
            self.traveled.header.frame_id = frame
            reference = Path()
            reference.header = message.header
            reference.header.frame_id = frame
            for i, (px, py) in enumerate(self.path.points):
                pose = PoseStamped()
                pose.header = reference.header
                pose.pose.position.x, pose.pose.position.y = px, py
                a = self.path.points[max(0, i-1)]
                b = self.path.points[min(len(self.path.points)-1, i+1)]
                angle = math.atan2(b[1]-a[1], b[0]-a[0])
                pose.pose.orientation.z = math.sin(angle/2)
                pose.pose.orientation.w = math.cos(angle/2)
                reference.poses.append(pose)
            self.reference_pub.publish(reference)
            self.get_logger().info(f'Quỹ đạo {self.settings["trajectory"]}: {self.path.total_length:.2f} m, '
                                   f'{self.settings["laps"]} vòng, bắt đầu tại vị trí hiện tại.')
        if time.monotonic() - self.last_path_wall > 0.5:
            self.last_path_wall = time.monotonic()
            pose = PoseStamped()
            pose.header = message.header
            pose.header.frame_id = frame
            pose.pose = message.pose.pose
            self.traveled.header.stamp = message.header.stamp
            self.traveled.poses.append(pose)
            self.traveled.poses = self.traveled.poses[-5000:]
            self.traveled_pub.publish(self.traveled)

    def send(self, speed=0.0, yaw_rate=0.0):
        message = Twist()
        message.linear.x, message.angular.z = speed, yaw_rate
        self.command_pub.publish(message)

    def finish(self, message, failed=False):
        self.send()
        self.finished = True
        self.exit_code = int(failed)
        (self.get_logger().error if failed else self.get_logger().info)(message)

    def control(self):
        if self.finished:
            return
        p = self.settings
        now = time.monotonic()
        if self.pose is None or not self.command_pub.get_subscription_count():
            self.send()
            if now - self.started_wall > p['startup_timeout']:
                self.finish('Không nhận được /odom hoặc chưa có bridge nhận /cmd_vel.', failed=True)
            return
        if now - self.last_odom_wall > p['odom_timeout']:
            self.finish('Mất odometry hoặc Gazebo tạm dừng quá lâu; đã dừng xe.', failed=True)
            return
        if self.start_stamp is None:
            self.start_stamp = self.odom_stamp
        if self.odom_stamp - self.start_stamp > p['max_duration']:
            self.finish('Quá thời gian chạy cho phép; đã dừng xe.', failed=True)
            return
        x, y, yaw = self.pose
        self.progress, error = self.path.project(x, y, self.progress, max(0.5, p['lookahead']*2))
        remaining = self.path.total_length - self.progress
        if error > p['max_tracking_error']:
            self.finish(f'Lệch quỹ đạo {error:.2f} m; đã dừng xe.', failed=True)
            return
        if remaining <= p['goal_tolerance'] and math.dist((x, y), self.path.points[-1]) <= p['goal_tolerance']:
            self.finish('Đã hoàn thành quỹ đạo và dừng xe.')
            return
        tx, ty = self.path.target(self.progress + p['lookahead'])
        dx, dy = tx-x, ty-y
        lateral = -math.sin(yaw)*dx + math.cos(yaw)*dy
        # Pure Pursuit: độ cong bằng 2 lần độ lệch ngang chia bình phương khoảng cách mục tiêu.
        curvature = 2*lateral / max(dx*dx + dy*dy, 1e-6)
        curvature = max(-p['max_curvature'], min(p['max_curvature'], curvature))
        speed = min(p['speed'], p['max_yaw_rate']/max(abs(curvature), 1e-6), max(0.03, remaining*0.8))
        self.send(speed, speed*curvature)
        if now - self.last_log_wall > 5.0:
            self.last_log_wall = now
            self.get_logger().info(f'Tiến độ {100*self.progress/self.path.total_length:.0f}%, lệch đường {error:.3f} m.')

    def stop(self):
        for _ in range(6):
            self.send()
            time.sleep(0.05)


def main(argv=None):
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = None

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, interrupt)
    exit_code = 0
    try:
        node = TrajectoryController()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = node.exit_code
    except KeyboardInterrupt:
        pass
    except (ValueError, TypeError) as error:
        print(f'Cấu hình không hợp lệ: {error}', file=sys.stderr)
        exit_code = 2
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())

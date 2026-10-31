#!/usr/bin/env python3
"""Gửi lệnh tiến, lùi, rẽ hoặc dừng từ terminal rồi tự dừng khi hết thời gian."""

import argparse
import math
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args


def parse_command(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('forward', 'backward', 'left', 'right', 'stop', 'drive'),
                        help='Tiến, lùi, tiến rẽ trái/phải, dừng hoặc chạy tùy chỉnh.')
    parser.add_argument('--speed', type=float, default=0.15, help='Vận tốc m/s, tối đa 0.3; drive cho phép số âm.')
    parser.add_argument('--yaw-rate', type=float, default=0.3, help='Tốc độ đổi hướng rad/s, tối đa 0.5; không phải góc servo.')
    parser.add_argument('--duration', type=float, default=3.0, help='Thời gian chạy tính bằng giây, mặc định 3.')
    args = parser.parse_args(argv)
    if not all(math.isfinite(v) for v in (args.speed, args.yaw_rate, args.duration)):
        parser.error('Các giá trị phải là số hữu hạn.')
    if abs(args.speed) > 0.3 or (args.command != 'drive' and args.speed < 0):
        parser.error('--speed phải từ 0 đến 0.3 m/s; riêng drive cho phép từ -0.3 đến 0.3.')
    if abs(args.yaw_rate) > 0.5:
        parser.error('--yaw-rate phải từ -0.5 đến 0.5 rad/s.')
    if args.duration <= 0:
        parser.error('--duration phải lớn hơn 0.')

    speed = args.speed
    yaw_rate = 0.0
    if args.command == 'backward':
        speed = -speed
    elif args.command == 'left':
        yaw_rate = abs(args.yaw_rate)
    elif args.command == 'right':
        yaw_rate = -abs(args.yaw_rate)
    elif args.command == 'drive':
        yaw_rate = args.yaw_rate
    elif args.command == 'stop':
        speed = 0.0
    if speed == 0.0:
        yaw_rate = 0.0
    return args, speed, yaw_rate


class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def send(self, speed=0.0, yaw_rate=0.0):
        message = Twist()
        message.linear.x = speed
        message.angular.z = yaw_rate
        self.publisher.publish(message)

    def wait_for_bridge(self):
        # Chờ kết nối để lệnh đầu tiên không bị mất khi ROS 2 đang tìm node bridge.
        deadline = time.monotonic() + 5.0
        while self.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('Chưa có node nhận /cmd_vel. Hãy chạy display.launch.py trước.')
            rclpy.spin_once(self, timeout_sec=0.05)

    def run_command(self, speed, yaw_rate, duration):
        self.wait_for_bridge()
        self.get_logger().info(f'v={speed:.3f} m/s, tốc độ đổi hướng={yaw_rate:.3f} rad/s, {duration:g} giây.')
        # Dùng thời gian thực để vẫn có thể kết thúc lệnh khi Gazebo đang tạm dừng.
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.send(speed, yaw_rate)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

    def stop(self):
        # Gửi lặp lệnh dừng trước khi đóng node để bridge kịp chuyển sang Gazebo.
        for _ in range(6):
            self.send()
            time.sleep(0.05)


def main(argv=None):
    args, speed, yaw_rate = parse_command(remove_ros_args(args=argv)[1:])
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    node = ManualControl()

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, interrupt)
    exit_code = 0
    try:
        node.run_command(speed, yaw_rate, 0.3 if args.command == 'stop' else args.duration)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        # Bỏ qua tín hiệu lặp trong lúc gửi lệnh dừng và giải phóng node.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())

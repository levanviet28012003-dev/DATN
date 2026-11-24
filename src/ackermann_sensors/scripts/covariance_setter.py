#!/usr/bin/env python3
"""Điền covariance (đường chéo) cho Imu hoặc Odometry trước khi đưa vào robot_localization.

Bridge Gazebo Fortress để mọi covariance bằng 0; robot_localization coi 0 là đo tuyệt đối chính xác.
Driver thật nào thiếu covariance cũng dùng được node này.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu


def diagonal(variances):
    """Ma trận vuông dạng hàng có đường chéo là variances."""
    size = len(variances)
    matrix = [0.0] * (size * size)
    for i, value in enumerate(variances):
        matrix[i * size + i] = float(value)
    return matrix


def check_variances(name, values, size, allow_negative_first=False):
    if len(values) != size:
        raise ValueError(f'{name} cần {size} phần tử, nhận {len(values)}.')
    for i, value in enumerate(values):
        if not math.isfinite(value):
            raise ValueError(f'{name} có giá trị không hữu hạn.')
        if value <= 0 and not (allow_negative_first and value < 0):
            raise ValueError(f'{name} phải dương (phương sai), trừ -1 để báo không có dữ liệu.')
    return [float(v) for v in values]


def set_imu_covariance(message, orientation, angular_velocity, linear_acceleration):
    """Ghi covariance vào Imu. orientation[0] < 0 nghĩa là IMU không có hướng (theo REP-145)."""
    if orientation[0] < 0:
        message.orientation_covariance = [-1.0] + [0.0] * 8
    else:
        message.orientation_covariance = diagonal(orientation)
    message.angular_velocity_covariance = diagonal(angular_velocity)
    message.linear_acceleration_covariance = diagonal(linear_acceleration)
    return message


def set_odometry_covariance(message, pose, twist):
    message.pose.covariance = diagonal(pose)
    message.twist.covariance = diagonal(twist)
    return message


class CovarianceSetter(Node):
    def __init__(self):
        super().__init__('covariance_setter')
        kind = self.declare_parameter('message_type', 'imu').value
        input_topic = self.declare_parameter('input_topic', '/sim/imu').value
        output_topic = self.declare_parameter('output_topic', '/imu/data').value
        if kind == 'imu':
            self.orientation = check_variances(
                'orientation_variance', self.declare_parameter(
                    'orientation_variance', [4e-4, 4e-4, 4e-4]).value, 3, allow_negative_first=True)
            self.angular_velocity = check_variances(
                'angular_velocity_variance', self.declare_parameter(
                    'angular_velocity_variance', [2.5e-5, 2.5e-5, 2.5e-5]).value, 3)
            self.linear_acceleration = check_variances(
                'linear_acceleration_variance', self.declare_parameter(
                    'linear_acceleration_variance', [4e-4, 4e-4, 4e-4]).value, 3)
            message_class, callback = Imu, self.on_imu
        elif kind == 'odometry':
            self.pose = check_variances('pose_variance', self.declare_parameter(
                'pose_variance', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).value, 6)
            self.twist = check_variances('twist_variance', self.declare_parameter(
                'twist_variance', [4e-4, 1e-3, 1e-6, 1e-6, 1e-6, 2.5e-3]).value, 6)
            message_class, callback = Odometry, self.on_odometry
        else:
            raise ValueError("message_type phải là 'imu' hoặc 'odometry'.")
        # Phát reliable để khớp mọi kiểu subscriber; nhận best effort để khớp mọi kiểu publisher.
        self.publisher = self.create_publisher(message_class, output_topic, 10)
        self.create_subscription(message_class, input_topic, callback, qos_profile_sensor_data)
        self.get_logger().info(f'Điền covariance {kind}: {input_topic} -> {output_topic}.')

    def on_imu(self, message):
        self.publisher.publish(set_imu_covariance(
            message, self.orientation, self.angular_velocity, self.linear_acceleration))

    def on_odometry(self, message):
        self.publisher.publish(set_odometry_covariance(message, self.pose, self.twist))


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CovarianceSetter()
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

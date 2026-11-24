#!/usr/bin/env python3
"""Biến GPS lý tưởng của Gazebo thành GPS thật: giảm tần số, trễ, bias trôi chậm, có covariance.

Mặc định giống GPS điện thoại gắn trên xe (kiến trúc 2026-11-22): 1 Hz, trễ 0,5 s, lệch trôi chậm
2,5 m (tau 120 s) cộng rung 1,5 m, tức nhảy vài mét. Đây là giá trị giả định; đo iPhone thật rồi sửa.

Sai số ngang mỗi trục = bias Gauss-Markov bậc 1 (trôi từ từ, hằng số thời gian bias_tau)
+ nhiễu trắng. Covariance báo ra bằng phương sai lý thuyết của tổng hai thành phần, giống độ chính
xác thiết bị báo (bán kính 68%). Mặc định stamp là thời điểm phát (restamp), như driver NMEA gắn
stamp lúc nhận câu: độ trễ đi thẳng vào sai số, đúng như xe thật.
"""

from collections import deque
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import NavSatFix, NavSatStatus

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
MIN_HORIZONTAL_SIGMA = 0.3   # m
MIN_VERTICAL_SIGMA = 1.0     # m


def copy_fix(fix):
    """Bản sao độc lập của NavSatFix, để đổi stamp hay tọa độ không ảnh hưởng bản tin gốc."""
    result = NavSatFix()
    result.header.stamp.sec = fix.header.stamp.sec
    result.header.stamp.nanosec = fix.header.stamp.nanosec
    result.header.frame_id = fix.header.frame_id
    result.status.status = fix.status.status
    result.status.service = fix.status.service
    result.latitude, result.longitude, result.altitude = fix.latitude, fix.longitude, fix.altitude
    result.position_covariance = list(fix.position_covariance)
    result.position_covariance_type = fix.position_covariance_type
    return result


def meters_per_degree(latitude_deg):
    """Số mét ứng với 1° vĩ độ (theo Bắc) và 1° kinh độ (theo Đông) tại vĩ độ đã cho, WGS84."""
    latitude = math.radians(latitude_deg)
    w = math.sqrt(1.0 - WGS84_E2 * math.sin(latitude) ** 2)
    meridian = WGS84_A * (1.0 - WGS84_E2) / w ** 3
    prime_vertical = WGS84_A / w
    return meridian * math.pi / 180.0, prime_vertical * math.cos(latitude) * math.pi / 180.0


def offset_latlon(latitude_deg, longitude_deg, east_m, north_m):
    """Dịch một điểm WGS84 đi (east, north) mét; đủ chính xác với dịch chuyển dưới vài trăm mét."""
    north_per_deg, east_per_deg = meters_per_degree(latitude_deg)
    return latitude_deg + north_m / north_per_deg, longitude_deg + east_m / east_per_deg


class GaussMarkov:
    """Quá trình Gauss-Markov bậc 1, dừng: độ lệch chuẩn sigma, hằng số thời gian tau (s)."""

    def __init__(self, sigma, tau, rng):
        if sigma < 0 or tau <= 0:
            raise ValueError('sigma phải không âm và tau phải dương.')
        self.sigma, self.tau, self.rng = sigma, tau, rng
        self.value = rng.gauss(0.0, sigma)

    def step(self, dt):
        if dt <= 0:
            return self.value
        phi = math.exp(-dt / self.tau)
        self.value = phi * self.value + self.sigma * math.sqrt(1.0 - phi * phi) * self.rng.gauss(0.0, 1.0)
        return self.value


class GpsNoiseModel:
    """Sai số GPS ngang (bias + nhiễu trắng) và dọc; tính cả covariance tương ứng."""

    def __init__(self, white_sigma=1.5, bias_sigma=2.5, bias_tau=120.0, vertical_sigma=3.0, seed=None):
        for name, value in [('white_sigma', white_sigma), ('bias_sigma', bias_sigma),
                            ('vertical_sigma', vertical_sigma)]:
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} phải là số hữu hạn không âm (m).')
        self.rng = random.Random(seed)
        self.white_sigma = white_sigma
        self.vertical_sigma = vertical_sigma
        self.bias_east = GaussMarkov(bias_sigma, bias_tau, self.rng)
        self.bias_north = GaussMarkov(bias_sigma, bias_tau, self.rng)
        self.bias_sigma = bias_sigma
        self.last_time = None

    def horizontal_sigma(self):
        return math.hypot(self.white_sigma, self.bias_sigma)

    def covariance(self):
        """Ma trận 3×3 (Đông, Bắc, Lên) dạng hàng, theo quy ước NavSatFix.

        Có sàn (MIN_HORIZONTAL_SIGMA, MIN_VERTICAL_SIGMA) để khi tắt nhiễu, EKF không nhận covariance
        bằng 0 rồi coi GPS là tuyệt đối chính xác.
        """
        h2 = max(self.horizontal_sigma(), MIN_HORIZONTAL_SIGMA) ** 2
        v2 = max(self.vertical_sigma, MIN_VERTICAL_SIGMA) ** 2
        return [h2, 0.0, 0.0, 0.0, h2, 0.0, 0.0, 0.0, v2]

    def sample_error(self, time_s):
        """Sai số (east, north, up) tại thời điểm time_s; bias tiến theo thời gian đã trôi qua."""
        dt = 0.0 if self.last_time is None else time_s - self.last_time
        self.last_time = time_s
        east = self.bias_east.step(dt) + self.rng.gauss(0.0, self.white_sigma)
        north = self.bias_north.step(dt) + self.rng.gauss(0.0, self.white_sigma)
        up = self.rng.gauss(0.0, self.vertical_sigma)
        return east, north, up

    def corrupt(self, fix, time_s):
        """Trả về NavSatFix mới đã cộng sai số và gắn covariance; không sửa bản tin gốc."""
        east, north, up = self.sample_error(time_s)
        noisy = copy_fix(fix)
        noisy.latitude, noisy.longitude = offset_latlon(fix.latitude, fix.longitude, east, north)
        noisy.altitude = fix.altitude + up
        noisy.position_covariance = self.covariance()
        noisy.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return noisy


class DelayLine:
    """Hàng đợi giữ bản tin tới thời điểm phát; thời gian tính bằng giây."""

    def __init__(self, delay):
        if not math.isfinite(delay) or delay < 0:
            raise ValueError('delay phải là số hữu hạn không âm (s).')
        self.delay = delay
        self.queue = deque()

    def push(self, now_s, item):
        self.queue.append((now_s + self.delay, item))

    def pop_due(self, now_s):
        due = []
        while self.queue and self.queue[0][0] <= now_s + 1e-9:
            due.append(self.queue.popleft()[1])
        return due


def has_fix(fix):
    return (fix.status.status >= NavSatStatus.STATUS_FIX
            and math.isfinite(fix.latitude) and math.isfinite(fix.longitude))


class GpsNoiseInjector(Node):
    def __init__(self):
        super().__init__('gps_noise_injector')
        input_topic = self.declare_parameter('input_topic', '/sim/gps/fix').value
        output_topic = self.declare_parameter('output_topic', '/gps/fix').value
        rate = self.declare_parameter('output_rate', 1.0).value
        delay = self.declare_parameter('delay', 0.5).value
        seed = self.declare_parameter('seed', -1).value
        self.restamp = self.declare_parameter('restamp', True).value
        self.stale_timeout = self.declare_parameter('stale_timeout', 2.0).value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('output_rate phải lớn hơn 0 (Hz).')
        self.model = GpsNoiseModel(
            white_sigma=self.declare_parameter('white_sigma', 1.5).value,
            bias_sigma=self.declare_parameter('bias_sigma', 2.5).value,
            bias_tau=self.declare_parameter('bias_tau', 120.0).value,
            vertical_sigma=self.declare_parameter('vertical_sigma', 3.0).value,
            seed=None if seed < 0 else seed)
        self.delay_line = DelayLine(delay)
        self.latest = None
        self.latest_time = None
        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)
        self.create_subscription(NavSatFix, input_topic, self.on_fix, qos_profile_sensor_data)
        self.create_timer(1.0 / rate, self.sample)
        self.create_timer(0.02, self.release)
        self.get_logger().info(
            f'GPS giả lập: {input_topic} -> {output_topic}, {rate:g} Hz, trễ {delay:g} s, '
            f'sai số ngang {self.model.horizontal_sigma():.1f} m (1 sigma mỗi trục).')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_fix(self, message):
        self.latest = message
        self.latest_time = self.now_s()

    def sample(self):
        now = self.now_s()
        if self.latest is None or now - self.latest_time > self.stale_timeout:
            return
        if has_fix(self.latest):
            output = self.model.corrupt(self.latest, now)
        else:
            output = copy_fix(self.latest)
        self.delay_line.push(now, output)

    def release(self):
        for message in self.delay_line.pop_due(self.now_s()):
            if self.restamp:
                message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = GpsNoiseInjector()
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

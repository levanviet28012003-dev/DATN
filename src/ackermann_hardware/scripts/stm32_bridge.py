#!/usr/bin/env python3
"""Cầu nối ROS 2 với STM32 qua UART.

Nhận /cmd_vel, đổi sang vận tốc và góc lái Ackermann rồi gửi khung CMD tới STM32;
giải mã khung STATE (encoder, điện áp, cờ trạng thái) và phát ra ROS.

Giao thức (khớp firmware/stm32f103c8t6/peripheral/inc/protocol.h), 115200 8N1:
    [0xAA 0x55][Len][Type][Seq][Payload][CRC16]
Len là số byte payload; CRC16-CCITT (0x1021, khởi tạo 0xFFFF) tính trên
Len, Type, Seq, Payload; mọi số nhiều byte theo little-endian.
"""

import binascii
from collections import namedtuple
import math
import signal
import struct
import time

SYNC = b'\xaa\x55'
HEADER_SIZE = 5
CRC_SIZE = 2
MAX_PAYLOAD = 16

TYPE_CMD = 0x01
TYPE_STATE = 0x02
TYPE_ESTOP = 0x03

CMD_FORMAT = '<hhB'      # vận tốc mm/s, góc lái 0,01 độ, enable
STATE_FORMAT = '<iiHB'   # encoder trái, encoder phải, điện áp mV, trạng thái

STATUS_READY = 1 << 0
STATUS_FAULT = 1 << 1
STATUS_ESTOP = 1 << 2
STATUS_CMD_TIMEOUT = 1 << 3
STATUS_DRIVING = 1 << 4
STATUS_ENCODER_VALID = 1 << 5
STATUS_VOLTAGE_VALID = 1 << 6

STATUS_NAMES = (
    (STATUS_READY, 'READY'),
    (STATUS_FAULT, 'FAULT'),
    (STATUS_ESTOP, 'ESTOP'),
    (STATUS_CMD_TIMEOUT, 'CMD_TIMEOUT'),
    (STATUS_DRIVING, 'DRIVING'),
    (STATUS_ENCODER_VALID, 'ENCODER_VALID'),
    (STATUS_VOLTAGE_VALID, 'VOLTAGE_VALID'),
)

INT16_MIN, INT16_MAX = -32768, 32767

Frame = namedtuple('Frame', 'type seq payload')
State = namedtuple('State', 'encoder_left encoder_right voltage_mv status')


def crc16(data):
    """CRC16-CCITT-FALSE, giống Protocol_Crc16() trong firmware."""
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def encode_frame(frame_type, seq, payload=b''):
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f'Payload {len(payload)} byte vượt quá {MAX_PAYLOAD} byte')
    body = bytes((len(payload), frame_type & 0xFF, seq & 0xFF)) + payload
    return SYNC + body + struct.pack('<H', crc16(body))


def _to_int16(value):
    return max(INT16_MIN, min(INT16_MAX, int(round(value))))


def encode_cmd(seq, velocity, steering_angle, enable):
    """velocity theo m/s, steering_angle theo rad (dương = rẽ trái)."""
    payload = struct.pack(CMD_FORMAT,
                          _to_int16(velocity * 1000.0),
                          _to_int16(math.degrees(steering_angle) * 100.0),
                          1 if enable else 0)
    return encode_frame(TYPE_CMD, seq, payload)


def encode_estop(seq, engage):
    return encode_frame(TYPE_ESTOP, seq, bytes((1 if engage else 0,)))


def decode_state(payload):
    if len(payload) != struct.calcsize(STATE_FORMAT):
        raise ValueError(f'STATE cần {struct.calcsize(STATE_FORMAT)} byte, nhận {len(payload)}')
    return State(*struct.unpack(STATE_FORMAT, payload))


def describe_status(status):
    return [name for bit, name in STATUS_NAMES if status & bit]


def twist_to_ackermann(linear, angular, wheelbase, max_steering_angle, max_speed):
    """Đổi (v, ω) của /cmd_vel sang (v, δ) theo mô hình xe đạp tại tâm trục sau.

    ω = v·tan(δ)/L nên δ = atan(L·ω/v). Xe Ackermann không quay tại chỗ được:
    khi v gần 0 thì giữ bánh thẳng.
    """
    if not (math.isfinite(linear) and math.isfinite(angular)):
        return 0.0, 0.0
    velocity = max(-max_speed, min(max_speed, linear))
    if abs(velocity) < 1e-3:
        return velocity, 0.0
    steering = math.atan(wheelbase * angular / velocity)
    return velocity, max(-max_steering_angle, min(max_steering_angle, steering))


class FrameParser:
    """Tách khung từ luồng byte; khung hỏng thì bỏ một byte rồi dò lại đầu khung."""

    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0
        self.length_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            start = self.buffer.find(SYNC)
            if start < 0:
                # Giữ lại 0xAA cuối vì có thể là nửa đầu của cặp đồng bộ.
                keep = 1 if self.buffer[-1:] == SYNC[:1] else 0
                del self.buffer[:len(self.buffer) - keep]
                return frames
            del self.buffer[:start]
            if len(self.buffer) < HEADER_SIZE:
                return frames
            length = self.buffer[2]
            if length > MAX_PAYLOAD:
                self.length_errors += 1
                del self.buffer[:1]
                continue
            size = HEADER_SIZE + length + CRC_SIZE
            if len(self.buffer) < size:
                return frames
            body = bytes(self.buffer[2:HEADER_SIZE + length])
            (received,) = struct.unpack_from('<H', self.buffer, HEADER_SIZE + length)
            if received != crc16(body):
                self.crc_errors += 1
                del self.buffer[:1]
                continue
            frames.append(Frame(body[1], body[2], body[3:]))
            del self.buffer[:size]


def run_node(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import BatteryState, JointState
    from std_msgs.msg import Bool

    class Stm32Bridge(Node):
        def __init__(self):
            super().__init__('stm32_bridge')
            self.port = self.declare_parameter('port', '/dev/serial0').value
            self.baudrate = self.declare_parameter('baudrate', 115200).value
            command_rate = self.declare_parameter('command_rate', 25.0).value
            self.command_timeout = self.declare_parameter('command_timeout', 0.5).value
            self.state_timeout = self.declare_parameter('state_timeout', 0.2).value
            self.wheelbase = self.declare_parameter('wheelbase', 0.170).value
            self.max_steering_angle = self.declare_parameter(
                'max_steering_angle', math.radians(40.0)).value
            self.max_speed = self.declare_parameter('max_speed', 0.20).value
            self.encoder_ppr = self.declare_parameter('encoder_ppr', 3960.0).value
            self.joint_names = [
                self.declare_parameter('left_wheel_joint', 'rear_left_wheel_joint').value,
                self.declare_parameter('right_wheel_joint', 'rear_right_wheel_joint').value,
            ]
            if not 20.0 <= command_rate <= 50.0:
                raise RuntimeError('command_rate phải trong 20–50 Hz theo giao thức')
            if self.command_timeout <= 0.0 or self.wheelbase <= 0.0 or self.encoder_ppr <= 0.0:
                raise RuntimeError('command_timeout, wheelbase, encoder_ppr phải dương')

            self.serial = None
            self.last_open_attempt = 0.0
            self.parser = FrameParser()
            self.tx_seq = 0
            self.velocity = 0.0
            self.steering = 0.0
            self.last_cmd_time = None
            self.estop_requested = False
            self.state = None
            self.state_time = None
            self.state_frames = 0
            self.state_seq_gaps = 0
            self.last_state_seq = None

            self.joint_pub = self.create_publisher(JointState, 'stm32/joint_states', 10)
            self.battery_pub = self.create_publisher(BatteryState, 'stm32/battery', 10)
            self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
            self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
            self.create_subscription(Bool, 'stm32/estop', self.on_estop, 10)
            self.create_timer(0.01, self.read_serial)
            self.create_timer(1.0 / command_rate, self.send_command)
            self.create_timer(1.0, self.publish_diagnostics)
            self.open_serial()

        # ---------------- Cổng serial ----------------
        def open_serial(self):
            try:
                import serial
            except ImportError as error:
                raise RuntimeError('Thiếu pyserial: sudo apt install python3-serial') from error
            self.last_open_attempt = time.monotonic()
            try:
                self.serial = serial.Serial(self.port, self.baudrate, timeout=0,
                                            write_timeout=0.05)
                self.serial.reset_input_buffer()
                self.get_logger().info(f'Đã mở {self.port} ở {self.baudrate} baud')
            except (serial.SerialException, OSError) as error:
                self.serial = None
                self.get_logger().error(f'Không mở được {self.port}: {error}',
                                        throttle_duration_sec=5.0)

        def close_serial(self, reason):
            self.get_logger().error(f'Mất kết nối {self.port}: {reason}')
            try:
                self.serial.close()
            except Exception:  # noqa: BLE001 - đang đóng cổng hỏng
                pass
            self.serial = None

        def write(self, frame):
            if self.serial is None:
                return
            try:
                self.serial.write(frame)
            except Exception as error:  # noqa: BLE001 - SerialException hoặc OSError
                self.close_serial(error)

        # ---------------- Nhận ----------------
        def read_serial(self):
            if self.serial is None:
                if time.monotonic() - self.last_open_attempt >= 1.0:
                    self.open_serial()
                return
            try:
                data = self.serial.read(self.serial.in_waiting or 1)
            except Exception as error:  # noqa: BLE001
                self.close_serial(error)
                return
            for frame in self.parser.feed(data):
                if frame.type == TYPE_STATE:
                    self.on_state(frame)

        def on_state(self, frame):
            try:
                state = decode_state(frame.payload)
            except ValueError as error:
                self.get_logger().warn(str(error), throttle_duration_sec=5.0)
                return
            if self.last_state_seq is not None and frame.seq != (self.last_state_seq + 1) & 0xFF:
                self.state_seq_gaps += 1
            self.last_state_seq = frame.seq
            self.state_frames += 1
            changed = STATUS_FAULT | STATUS_ESTOP | STATUS_READY
            if self.state is None or (state.status ^ self.state.status) & changed:
                flags = ', '.join(describe_status(state.status)) or 'không cờ'
                self.get_logger().info(f'STM32: {flags}')
            self.state = state
            self.state_time = time.monotonic()

            now = self.get_clock().now().to_msg()
            joints = JointState()
            joints.header.stamp = now
            joints.name = list(self.joint_names)
            scale = 2.0 * math.pi / self.encoder_ppr
            joints.position = [state.encoder_left * scale, state.encoder_right * scale]
            self.joint_pub.publish(joints)

            battery = BatteryState()
            battery.header.stamp = now
            battery.voltage = state.voltage_mv / 1000.0
            battery.present = bool(state.status & STATUS_VOLTAGE_VALID)
            for field in ('temperature', 'current', 'charge', 'capacity',
                          'design_capacity', 'percentage'):
                setattr(battery, field, math.nan)
            self.battery_pub.publish(battery)

        # ---------------- Gửi ----------------
        def on_cmd_vel(self, msg):
            self.velocity, self.steering = twist_to_ackermann(
                msg.linear.x, msg.angular.z, self.wheelbase,
                self.max_steering_angle, self.max_speed)
            self.last_cmd_time = time.monotonic()

        def on_estop(self, msg):
            if msg.data != self.estop_requested:
                self.get_logger().warn('Kích hoạt dừng khẩn' if msg.data else 'Nhả dừng khẩn')
            self.estop_requested = msg.data
            if msg.data:
                self.velocity = self.steering = 0.0
                self.last_cmd_time = None
                self.send_frame(encode_estop, True)

        def send_frame(self, encoder, *args):
            self.write(encoder(self.tx_seq, *args))
            self.tx_seq = (self.tx_seq + 1) & 0xFF

        def command_active(self):
            return (not self.estop_requested and self.last_cmd_time is not None
                    and time.monotonic() - self.last_cmd_time <= self.command_timeout)

        def send_command(self):
            if self.estop_requested:
                self.send_frame(encode_estop, True)
                return
            if self.state is not None and self.state.status & STATUS_ESTOP:
                # STM32 còn giữ ESTOP (có thể khung nhả bị mất): gửi lại lệnh nhả.
                self.send_frame(encode_estop, False)
            if self.command_active():
                self.send_frame(encode_cmd, self.velocity, self.steering, True)
            else:
                self.send_frame(encode_cmd, 0.0, 0.0, False)

        def stop(self):
            for _ in range(5):
                self.send_frame(encode_cmd, 0.0, 0.0, False)
                time.sleep(0.02)

        # ---------------- Chẩn đoán ----------------
        def publish_diagnostics(self):
            status = DiagnosticStatus(name='stm32_bridge', hardware_id=self.port)
            fresh = (self.state_time is not None
                     and time.monotonic() - self.state_time <= self.state_timeout)
            flags = self.state.status if self.state else 0
            if self.serial is None:
                status.level, status.message = DiagnosticStatus.ERROR, 'Chưa mở được cổng serial'
            elif not fresh:
                status.level, status.message = DiagnosticStatus.ERROR, 'Không nhận được STATE'
            elif flags & STATUS_FAULT:
                status.level, status.message = DiagnosticStatus.ERROR, 'STM32 báo lỗi phần cứng'
            elif flags & STATUS_ESTOP:
                status.level, status.message = DiagnosticStatus.WARN, 'Đang dừng khẩn'
            elif flags & STATUS_CMD_TIMEOUT:
                status.level, status.message = DiagnosticStatus.WARN, 'STM32 không có lệnh, xe dừng'
            else:
                status.level, status.message = DiagnosticStatus.OK, 'Bình thường'
            values = {
                'flags': ', '.join(describe_status(flags)),
                'state_frames': self.state_frames,
                'state_seq_gaps': self.state_seq_gaps,
                'crc_errors': self.parser.crc_errors,
                'length_errors': self.parser.length_errors,
                'command_velocity': f'{self.velocity:.3f}',
                'command_steering': f'{self.steering:.3f}',
            }
            if self.state:
                values.update(encoder_left=self.state.encoder_left,
                              encoder_right=self.state.encoder_right,
                              voltage_mv=self.state.voltage_mv)
            status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
            array = DiagnosticArray(status=[status])
            array.header.stamp = self.get_clock().now().to_msg()
            self.diag_pub.publish(array)

    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, interrupt)
    node = None
    exit_code = 0
    try:
        node = Stm32Bridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'[stm32_bridge] {error}')
        exit_code = 1
    finally:
        # STM32 tự dừng sau 300 ms mất lệnh; gửi thêm lệnh dừng để dừng ngay.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if node is not None:
            node.stop()
            if node.serial is not None:
                node.serial.close()
            node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_handler)
    return exit_code


if __name__ == '__main__':
    raise SystemExit(run_node())

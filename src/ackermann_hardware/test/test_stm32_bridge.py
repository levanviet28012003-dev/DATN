"""Kiểm tra mã hóa khung UART phía Pi; không cần ROS graph hay cổng serial.

Các khung mẫu trùng với firmware/stm32f103c8t6/tests/test_communication.c,
nên hai phía được kiểm tra khớp nhau từng byte.
"""
import importlib.util
import math
from pathlib import Path
import struct
import unittest

source = Path(__file__).resolve().parents[1] / 'scripts' / 'stm32_bridge.py'
spec = importlib.util.spec_from_file_location('stm32_bridge', source)
bridge = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bridge)

CMD_FRAME = bytes((0xAA, 0x55, 0x05, 0x01, 0x07, 0x96, 0x00, 0x2E, 0xFB, 0x01, 0x72, 0xA0))
STATE_FRAME = bytes((0xAA, 0x55, 0x0B, 0x02, 0x00, 0xC0, 0x1D, 0xFE, 0xFF, 0x06, 0x12, 0x0F,
                     0x00, 0xE8, 0x1C, 0x61, 0xF6, 0x01))
ESTOP_FRAME = bytes((0xAA, 0x55, 0x01, 0x03, 0x09, 0x01, 0x9D, 0x01))


class ProtocolTests(unittest.TestCase):
    def test_crc_matches_ccitt_false_reference(self):
        self.assertEqual(bridge.crc16(b'123456789'), 0x29B1)

    def test_frames_match_firmware_reference_bytes(self):
        self.assertEqual(bridge.encode_cmd(7, 0.150, math.radians(-12.34), True), CMD_FRAME)
        self.assertEqual(bridge.encode_estop(9, True), ESTOP_FRAME)

    def test_decodes_state_from_firmware(self):
        frames = bridge.FrameParser().feed(STATE_FRAME)
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].type, bridge.TYPE_STATE)
        state = bridge.decode_state(frames[0].payload)
        self.assertEqual(state, bridge.State(-123456, 987654, 7400, 0x61))
        self.assertEqual(bridge.describe_status(state.status),
                         ['READY', 'ENCODER_VALID', 'VOLTAGE_VALID'])

    def test_command_fields_are_clamped_to_int16(self):
        frame = bridge.encode_cmd(255, 100.0, -10.0, False)
        velocity, steering, enable = struct.unpack('<hhB', frame[5:10])
        self.assertEqual((velocity, steering, enable), (32767, -32768, 0))
        self.assertEqual(frame[4], 255)
        with self.assertRaises(ValueError):
            bridge.encode_frame(1, 0, bytes(17))

    def test_parser_recovers_from_noise_split_reads_and_corruption(self):
        corrupted = bytearray(STATE_FRAME)
        corrupted[8] ^= 0x10
        # Khung hỏng bị cắt ngắn chứa 0xAA 0x55 giả ở giữa luồng.
        stream = b'\x00\xaa\x13' + bytes(corrupted) + b'\xaa\x55\x40' + STATE_FRAME + STATE_FRAME
        parser = bridge.FrameParser()
        frames = []
        for i in range(0, len(stream), 3):
            frames += parser.feed(stream[i:i + 3])
        self.assertEqual(len(frames), 2)
        self.assertTrue(all(f.payload == STATE_FRAME[5:16] for f in frames))
        self.assertGreaterEqual(parser.crc_errors, 1)
        self.assertGreaterEqual(parser.length_errors, 1)
        self.assertLessEqual(len(parser.buffer), 1)


class TwistConversionTests(unittest.TestCase):
    def test_matches_bicycle_model_forward_and_reverse(self):
        wheelbase = 0.170
        for linear, angular in ((0.2, 0.5), (-0.15, 0.3), (0.1, -0.4)):
            velocity, steering = bridge.twist_to_ackermann(linear, angular, wheelbase, 1.0, 0.3)
            self.assertAlmostEqual(velocity, linear)
            self.assertAlmostEqual(velocity * math.tan(steering) / wheelbase, angular)

    def test_clamps_speed_and_steering(self):
        velocity, steering = bridge.twist_to_ackermann(1.0, 5.0, 0.170, math.radians(40), 0.2)
        self.assertEqual(velocity, 0.2)
        self.assertAlmostEqual(steering, math.radians(40))

    def test_no_turn_in_place_and_rejects_nan(self):
        self.assertEqual(bridge.twist_to_ackermann(0.0, 1.0, 0.17, 0.7, 0.2), (0.0, 0.0))
        self.assertEqual(bridge.twist_to_ackermann(math.nan, 0.1, 0.17, 0.7, 0.2), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()

#!/usr/bin/env python3
"""Gửi đích theo vĩ độ/kinh độ cho Nav2.

Ba cách dùng:
  gps_goal LAT LON        gửi một đích, chờ kết quả rồi thoát (Ctrl+C hủy đích).
  gps_goal --file F.yaml  đi lần lượt các điểm trong file (ghi bằng gps_waypoint_logger); dừng ở điểm
                          đầu tiên không tới được, chạy tiếp bằng --start N.
  gps_goal                chạy nền, nhận đích trên /gps_goal (geometry_msgs/PointStamped):
                          frame_id 'wgs84' thì x = kinh độ, y = vĩ độ (plugin point_click_publisher
                          của Mapviz phát đúng dạng này); frame_id 'map' thì x, y là mét trong map.

TẠM CHƯA CHẠY (2026-11-24): từ khi chuyển sang kiến trúc SLAM, navsat_transform và service /fromLL
đã bỏ. Node này giữ làm điểm xuất phát cho GPS Waypoint Converter: phần đọc file, kiểm chặng 24 m,
gửi NavigateToPose và gửi lại đích dùng lại được; chỉ phần đổi lat/lon sang map phải thay bằng phép
khớp GPS với bản đồ SLAM.

Thiết kế cũ: lat/lon được đổi sang frame map bằng service /fromLL của navsat_transform rồi
gửi Nav2 ở navigation_frame. Mặc định là map: đích cố định, Nav2 tự theo map → odom do ekf_map phát.
Nếu đặt navigation_frame là odom (dùng khi GPS nhảy mạnh, như GPS điện thoại), đích được đổi sang odom
bằng TF map → odom hiện tại và gửi lại khi hiệu chỉnh GPS làm nó lệch quá refresh_distance. Hướng đích
đặt theo hướng từ xe tới đích để xe Ackermann không phải quay đầu tại đích.
"""

import argparse
import math
import sys

from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from robot_localization.srv import FromLL
from tf2_ros import Buffer, TransformException, TransformListener

WGS84_FRAMES = ('wgs84', '/wgs84')
STATUS_SUCCEEDED = 4  # action_msgs/GoalStatus
WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def validate_latlon(latitude, longitude):
    if not (math.isfinite(latitude) and math.isfinite(longitude)
            and -90.0 <= latitude <= 90.0 and -180.0 <= longitude <= 180.0):
        raise ValueError(f'Tọa độ không hợp lệ: vĩ độ {latitude}, kinh độ {longitude}.')
    return latitude, longitude


def latlon_offset_m(latitude0, longitude0, latitude1, longitude1):
    """(Đông, Bắc) tính bằng mét từ điểm 0 tới điểm 1 trên ellipsoid WGS84; sai số không đáng kể
    với quãng vài trăm mét như giữa hai điểm đích."""
    phi = math.radians((latitude0 + latitude1) / 2.0)
    w = math.sqrt(1.0 - WGS84_E2 * math.sin(phi) ** 2)
    north_per_degree = WGS84_A * (1.0 - WGS84_E2) / w ** 3 * math.pi / 180.0
    east_per_degree = WGS84_A / w * math.cos(phi) * math.pi / 180.0
    return ((longitude1 - longitude0) * east_per_degree, (latitude1 - latitude0) * north_per_degree)


def parse_waypoints(data):
    """Danh sách (vĩ độ, kinh độ) từ nội dung YAML dạng {'waypoints': [{latitude, longitude}, ...]}.

    Khóa khác (ví dụ yaw của định dạng demo Nav2) được bỏ qua: đích GPS ngoài trời không xét hướng.
    """
    if not isinstance(data, dict) or not isinstance(data.get('waypoints'), list) or not data['waypoints']:
        raise ValueError("File điểm phải có danh sách 'waypoints' không rỗng.")
    points = []
    for index, item in enumerate(data['waypoints'], start=1):
        try:
            latitude, longitude = float(item['latitude']), float(item['longitude'])
        except (TypeError, KeyError, ValueError):
            raise ValueError(f'Điểm {index} thiếu latitude/longitude dạng số.') from None
        try:
            points.append(validate_latlon(latitude, longitude))
        except ValueError as error:
            raise ValueError(f'Điểm {index}: {error}') from None
    return points


def load_waypoints(path):
    import yaml  # Chỉ cần khi đọc file; phần còn lại của node không phụ thuộc PyYAML.
    try:
        with open(path, encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except OSError as error:
        raise ValueError(f'Không đọc được {path}: {error.strerror}.') from None
    except yaml.YAMLError as error:
        raise ValueError(f'{path} không phải YAML hợp lệ: {error}') from None
    return parse_waypoints(data)


def long_legs(points, limit):
    """[(số thứ tự điểm đích, độ dài chặng)] của các chặng giữa hai điểm liên tiếp dài hơn limit (m)."""
    result = []
    for index in range(1, len(points)):
        length = math.hypot(*latlon_offset_m(*points[index - 1], *points[index]))
        if length > limit:
            result.append((index + 1, length))
    return result


def parse_goal_point(frame_id, x, y):
    """Phân loại điểm nhận được: ('wgs84', vĩ độ, kinh độ) hoặc ('map', x, y)."""
    if frame_id in WGS84_FRAMES:
        latitude, longitude = validate_latlon(y, x)
        return 'wgs84', latitude, longitude
    if frame_id in ('map', '/map'):
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError('Điểm trong frame map phải hữu hạn.')
        return 'map', x, y
    raise ValueError(f"frame_id '{frame_id}' không hỗ trợ; dùng 'wgs84' hoặc 'map'.")


def goal_yaw(robot_xy, goal_xy, default=0.0):
    """Hướng từ xe tới đích; giữ default khi hai điểm trùng nhau."""
    dx, dy = goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1]
    if math.hypot(dx, dy) < 1e-6:
        return default
    return math.atan2(dy, dx)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def transform_point_2d(transform, point):
    """Áp phép biến đổi phẳng (tx, ty, yaw) lên điểm (x, y)."""
    tx, ty, yaw = transform
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * point[0] - s * point[1], ty + s * point[0] + c * point[1]


def needs_refresh(sent_xy, current_xy, robot_xy, refresh_distance, min_robot_distance):
    """Có gửi lại đích hay không: đích trong odom đã dịch đủ xa và xe chưa ở sát đích.

    Không gửi lại khi xe đã gần đích, nếu không mỗi lần GPS dao động xe lại phải lập đường quanh đích.
    """
    shift = math.dist(sent_xy, current_xy)
    return shift > refresh_distance and math.dist(robot_xy, current_xy) > min_robot_distance


class GpsGoal(Node):
    def __init__(self):
        super().__init__('gps_goal')
        self.robot_frame = self.declare_parameter('robot_frame', 'rear_axle').value
        # Phải trùng global_frame của bt_navigator (nav2_outdoor.yaml).
        self.navigation_frame = self.declare_parameter('navigation_frame', 'map').value
        # Global costmap ngoài trời rộng 50 m quanh xe; đích xa hơn nửa cạnh trừ biên sẽ không lập được đường.
        self.max_distance = self.declare_parameter('max_goal_distance', 24.0).value
        # Bằng dung sai đích của Nav2 ngoài trời: lệch ít hơn thì xe vẫn coi là tới đích, không cần lập lại.
        self.refresh_distance = self.declare_parameter('refresh_distance', 1.5).value
        self.refresh_period = self.declare_parameter('refresh_period', 10.0).value
        self.refresh_min_robot_distance = self.declare_parameter('refresh_min_robot_distance', 3.0).value
        self.tf_buffer = Buffer()
        # Luồng riêng nhận TF để chờ được TF ngay trong callback mà không cần executor quay.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.from_ll = self.create_client(FromLL, '/fromLL')
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_publisher = self.create_publisher(PoseStamped, '/gps_goal/pose', 10)
        self.goal_handle = None
        self.goal_map = None        # Đích trong frame map, cố định theo lat/lon
        self.sent_xy = None         # Đích đã gửi Nav2, trong navigation_frame
        self.last_send = None
        self.done = False
        self.succeeded = False
        self.last_feedback_log = None
        self.create_timer(1.0, self.refresh)

    def listen(self):
        self.create_subscription(PointStamped, '/gps_goal', self.on_point, 10)
        self.get_logger().info("Chờ đích trên /gps_goal (frame 'wgs84': x = kinh độ, y = vĩ độ).")

    def on_point(self, message):
        try:
            kind, a, b = parse_goal_point(message.header.frame_id, message.point.x, message.point.y)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        if kind == 'wgs84':
            self.send_latlon(a, b)
        else:
            self.set_map_goal(a, b)

    def send_latlon(self, latitude, longitude):
        if not self.from_ll.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Không có service /fromLL; navsat_transform chưa chạy hoặc chưa có datum.')
            self.finish(False)
            return
        request = FromLL.Request()
        request.ll_point = GeoPoint(latitude=latitude, longitude=longitude, altitude=0.0)
        future = self.from_ll.call_async(request)
        future.add_done_callback(lambda done: self.on_converted(done, latitude, longitude))

    def on_converted(self, future, latitude, longitude):
        try:
            point = future.result().map_point
        except Exception as error:  # noqa: B902 - lỗi service báo qua future
            self.get_logger().error(f'/fromLL lỗi: {error}')
            self.finish(False)
            return
        self.get_logger().info(
            f'Đích GPS {latitude:.7f}, {longitude:.7f} -> map ({point.x:.2f}, {point.y:.2f}) m.')
        self.set_map_goal(point.x, point.y)

    def wait_for_tf(self, timeout=60.0):
        """Chờ có TF map → navigation_frame → robot_frame; tiến trình mới cần thời gian nhận TF khi máy tải nặng."""
        deadline = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < deadline:
            if all(self.tf_buffer.can_transform(self.navigation_frame, source, Time(),
                                                timeout=Duration(seconds=1.0))
                   for source in ('map', self.robot_frame)):
                return True
            self.get_logger().info(f'Chờ TF {self.navigation_frame} -> map, {self.robot_frame}...',
                                   throttle_duration_sec=5.0)
        return False

    def lookup_2d(self, target, source, timeout=10.0):
        """Phép biến đổi phẳng (tx, ty, yaw) đưa điểm từ source sang target, hoặc None.

        Lần đầu chờ lâu: tiến trình vừa khởi động cần vài giây để nhận TF khi máy đang tải nặng.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source, Time(), timeout=Duration(seconds=timeout))
        except TransformException as error:
            self.get_logger().error(f'Chưa có TF {target} -> {source}: {error}')
            return None
        t, q = transform.transform.translation, transform.transform.rotation
        return t.x, t.y, quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def goal_in_navigation_frame(self, timeout=10.0):
        """(đích, vị trí xe) trong navigation_frame, hoặc None nếu thiếu TF."""
        to_navigation = self.lookup_2d(self.navigation_frame, 'map', timeout)
        robot = self.lookup_2d(self.navigation_frame, self.robot_frame, timeout)
        if to_navigation is None or robot is None:
            return None
        return transform_point_2d(to_navigation, self.goal_map), robot[:2]

    def set_map_goal(self, x, y):
        self.goal_map = (x, y)
        map_goal = PoseStamped()
        map_goal.header.frame_id = 'map'
        map_goal.header.stamp = self.get_clock().now().to_msg()
        map_goal.pose.position.x, map_goal.pose.position.y = x, y
        map_goal.pose.orientation.w = 1.0
        self.goal_publisher.publish(map_goal)
        self.send_goal(first=True)

    def send_goal(self, first):
        poses = self.goal_in_navigation_frame()
        if poses is None:
            self.finish(False)
            return
        goal_xy, robot_xy = poses
        distance = math.dist(goal_xy, robot_xy)
        if first and distance > self.max_distance:
            self.get_logger().error(
                f'Đích cách xe {distance:.1f} m, vượt {self.max_distance:.0f} m của global costmap; '
                'chia thành các đích gần hơn.')
            self.goal_map = None
            self.finish(False)
            return
        goal = PoseStamped()
        goal.header.frame_id = self.navigation_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = goal_xy
        (goal.pose.orientation.x, goal.pose.orientation.y,
         goal.pose.orientation.z, goal.pose.orientation.w) = yaw_to_quaternion(goal_yaw(robot_xy, goal_xy))
        if not self.navigator.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Không có action navigate_to_pose; Nav2 chưa sẵn sàng.')
            self.finish(False)
            return
        if first:
            self.get_logger().info(
                f'Gửi đích {self.navigation_frame} ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}), cách xe {distance:.1f} m.')
        # Bỏ handle cũ trước: kết quả của đích bị thay thế sẽ bị bỏ qua trong on_result.
        self.goal_handle = None
        self.sent_xy = goal_xy
        self.last_send = self.get_clock().now()
        future = self.navigator.send_goal_async(
            NavigateToPose.Goal(pose=goal), feedback_callback=self.on_feedback)
        future.add_done_callback(self.on_goal_response)

    def refresh(self):
        """Gửi lại đích khi ekf_map hiệu chỉnh map → odom làm đích trong odom dịch quá xa."""
        if self.navigation_frame == 'map':
            return  # Đích ở map đã cố định theo lat/lon.
        if self.goal_map is None or self.goal_handle is None or self.last_send is None:
            return
        if (self.get_clock().now() - self.last_send).nanoseconds * 1e-9 < self.refresh_period:
            return
        poses = self.goal_in_navigation_frame(timeout=0.5)
        if poses is None:
            return
        goal_xy, robot_xy = poses
        if needs_refresh(self.sent_xy, goal_xy, robot_xy,
                         self.refresh_distance, self.refresh_min_robot_distance):
            self.get_logger().info(
                f'GPS hiệu chỉnh vị trí: đích dịch {math.dist(self.sent_xy, goal_xy):.1f} m, gửi lại.')
            self.send_goal(first=False)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 từ chối đích.')
            self.goal_map = None
            self.finish(False)
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda done: self.on_result(done, handle))

    def on_feedback(self, message):
        now = self.get_clock().now()
        if self.last_feedback_log is None or (now - self.last_feedback_log).nanoseconds > 10e9:
            self.last_feedback_log = now
            self.get_logger().info(
                f'Còn {message.feedback.distance_remaining:.1f} m, '
                f'đã chạy {message.feedback.navigation_time.sec} s.')

    def on_result(self, future, handle):
        if handle is not self.goal_handle:
            return  # Đích cũ đã bị đích mới (hoặc bản gửi lại) thay thế.
        status = future.result().status
        succeeded = status == STATUS_SUCCEEDED
        if succeeded:
            self.get_logger().info('Đã tới đích GPS.')
        else:
            self.get_logger().warn(f'Không tới được đích (trạng thái action {status}).')
        self.goal_handle = None
        self.goal_map = None
        self.finish(succeeded)

    def finish(self, succeeded):
        self.succeeded = succeeded
        self.done = True

    def cancel(self):
        if self.goal_handle is not None:
            self.get_logger().info('Hủy đích đang chạy.')
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)


def run_goal(node, latitude, longitude):
    """Gửi một đích lat/lon và chờ kết quả; True nếu tới đích."""
    node.done = False
    node.succeeded = False
    node.send_latlon(latitude, longitude)
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.succeeded


def follow_waypoints(node, points, start):
    """Đi lần lượt points[start-1:]; trả về mã thoát (0 khi đi hết)."""
    total = len(points)
    for index in range(start, total + 1):
        latitude, longitude = points[index - 1]
        node.get_logger().info(f'Điểm {index}/{total}: {latitude:.7f}, {longitude:.7f}.')
        if not run_goal(node, latitude, longitude):
            node.get_logger().error(
                f'Dừng ở điểm {index}/{total}. Chạy tiếp từ điểm này: gps_goal --file ... --start {index}.')
            return 1
    node.get_logger().info(f'Đã đi hết {total - start + 1} điểm.')
    return 0


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    parser = argparse.ArgumentParser(description='Gửi đích theo vĩ độ/kinh độ cho Nav2.')
    parser.add_argument('latitude', type=float, nargs='?')
    parser.add_argument('longitude', type=float, nargs='?')
    parser.add_argument('--file', help='File YAML danh sách điểm (gps_waypoint_logger ghi ra).')
    parser.add_argument('--start', type=int, default=1, help='Bắt đầu từ điểm thứ N (đếm từ 1).')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(argv)[1:])
    if args.file and args.latitude is not None:
        parser.error('dùng --file hoặc LAT LON, không dùng cả hai.')
    if args.latitude is not None and args.longitude is None:
        parser.error('cần cả vĩ độ và kinh độ.')
    node = GpsGoal()
    exit_code = 0
    try:
        if args.file:
            points = load_waypoints(args.file)
            if not 1 <= args.start <= len(points):
                raise ValueError(f'--start phải từ 1 đến {len(points)}.')
            # Chặng đầu (từ vị trí xe) được send_goal kiểm tra khi gửi; ở đây kiểm các chặng giữa hai điểm.
            legs = long_legs(points[args.start - 1:], node.max_distance)
            if legs:
                details = ', '.join(f'tới điểm {index + args.start - 1}: {length:.1f} m' for index, length in legs)
                raise ValueError(f'Có chặng dài hơn {node.max_distance:.0f} m ({details}); thêm điểm ở giữa.')
            if not node.wait_for_tf():
                node.get_logger().error('Không có TF sau 60 s; định vị (ekf_odom, ekf_map) chưa chạy?')
                return 1
            exit_code = follow_waypoints(node, points, args.start)
        elif args.latitude is None:
            node.listen()
            rclpy.spin(node)
        else:
            validate_latlon(args.latitude, args.longitude)
            if not node.wait_for_tf():
                node.get_logger().error('Không có TF sau 60 s; định vị (ekf_odom, ekf_map) chưa chạy?')
                return 1
            exit_code = 0 if run_goal(node, args.latitude, args.longitude) else 1
    except KeyboardInterrupt:
        node.cancel()
        exit_code = 1
    except ValueError as error:
        node.get_logger().error(str(error))
        exit_code = 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())

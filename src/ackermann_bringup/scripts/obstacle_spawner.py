#!/usr/bin/env python3
"""Thả chướng ngại vật vào Gazebo tại điểm bấm bằng công cụ Publish Point của RViz.

Mỗi điểm nhận trên /clicked_point sinh một trụ tĩnh trong world. Trụ không có trên bản đồ,
nên Nav2 chỉ biết tới nó qua Lidar. Gọi dịch vụ /clear_obstacles để xóa mọi trụ đã thả.
"""

import math
import os
import subprocess

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Empty

# Hệ map trùng hệ tọa độ của world vì bản đồ được sinh từ chính world đó; odom cũng trùng
# vì /odom là vị trí thật của Gazebo.
WORLD_ALIGNED_FRAMES = ('map', 'odom')


def obstacle_sdf(name, radius, height):
    """Sinh SDF của một trụ tĩnh; gốc mô hình nằm ở tâm trụ."""
    geometry = f'<cylinder><radius>{radius:g}</radius><length>{height:g}</length></cylinder>'
    color = '0.85 0.10 0.10 1'
    return (f'<?xml version="1.0"?><sdf version="1.8"><model name="{name}"><static>true</static>'
            f'<link name="link"><collision name="collision"><geometry>{geometry}</geometry></collision>'
            f'<visual name="visual"><geometry>{geometry}</geometry>'
            f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>'
            '</visual></link></model></sdf>')


def placement_error(point, robot, obstacles, radius, min_robot_distance):
    """Trả về lý do không được đặt trụ tại point, hoặc None nếu đặt được.

    point và robot là (x, y); obstacles là danh sách tâm (x, y) của các trụ đã thả.
    """
    if not all(math.isfinite(value) for value in point):
        return 'tọa độ không hợp lệ'
    if robot is None:
        return 'chưa nhận được vị trí xe từ /odom'
    distance = math.dist(point, robot)
    if distance < min_robot_distance:
        return f'cách xe {distance:.2f} m, gần hơn mức tối thiểu {min_robot_distance:.2f} m'
    if any(math.dist(point, other) < 2 * radius for other in obstacles):
        return 'chồng lên một trụ đã thả'
    return None


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.world = self.declare_parameter('world', 'nav_world').value
        self.radius = float(self.declare_parameter('radius', 0.15).value)
        # Phải cao hơn mặt phẳng quét của Lidar (khoảng 0,10 m so với mặt đất).
        self.height = float(self.declare_parameter('height', 0.40).value)
        # Không thả trụ đè lên xe hoặc sát tới mức xe không thể tránh.
        self.min_robot_distance = float(self.declare_parameter('min_robot_distance', 0.60).value)
        if self.radius <= 0 or self.height <= 0 or self.min_robot_distance <= 0:
            raise ValueError('radius, height và min_robot_distance phải lớn hơn 0.')

        self.create_executable = os.path.join(get_package_prefix('ros_gz_sim'), 'lib', 'ros_gz_sim', 'create')
        self.robot = None
        self.obstacles = {}   # tên mô hình -> (x, y)
        self.count = 0
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_point, 10)
        self.create_service(Empty, '/clear_obstacles', self.on_clear)
        self.get_logger().info('Sẵn sàng: bấm Publish Point trong RViz để thả chướng ngại vật.')

    def on_odom(self, message):
        position = message.pose.pose.position
        self.robot = (position.x, position.y)

    def on_point(self, message):
        frame = message.header.frame_id
        if frame not in WORLD_ALIGNED_FRAMES:
            self.get_logger().warn(f'Bỏ qua điểm trong frame "{frame}"; đặt Fixed Frame của RViz là map.')
            return
        point = (message.point.x, message.point.y)
        error = placement_error(point, self.robot, list(self.obstacles.values()),
                                self.radius, self.min_robot_distance)
        if error:
            self.get_logger().warn(f'Không thả chướng ngại vật tại ({point[0]:.2f}, {point[1]:.2f}): {error}.')
            return

        self.count += 1
        name = f'clicked_obstacle_{self.count}'
        command = [self.create_executable, '-world', self.world, '-name', name,
                   '-string', obstacle_sdf(name, self.radius, self.height),
                   f'-x={point[0]:.3f}', f'-y={point[1]:.3f}', f'-z={self.height / 2:.3f}']
        if self.run(command, 'thả chướng ngại vật'):
            self.obstacles[name] = point
            self.get_logger().info(f'Đã thả {name} tại ({point[0]:.2f}, {point[1]:.2f}).')

    def on_clear(self, _request, response):
        for name in list(self.obstacles):
            command = ['ign', 'service', '-s', f'/world/{self.world}/remove',
                       '--reqtype', 'ignition.msgs.Entity', '--reptype', 'ignition.msgs.Boolean',
                       '--timeout', '3000', '--req', f'name: "{name}" type: MODEL']
            if self.run(command, f'xóa {name}'):
                del self.obstacles[name]
        self.get_logger().info(f'Đã xóa chướng ngại vật; còn lại {len(self.obstacles)}.')
        return response

    def run(self, command, action):
        """Chạy lệnh ngoài và báo lỗi bằng log thay vì làm dừng node."""
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=10.0)
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f'Không thể {action}: {error}')
            return False
        if result.returncode != 0:
            self.get_logger().error(f'Không thể {action}: {result.stderr.strip() or result.stdout.strip()}')
            return False
        return True


def main(argv=None):
    rclpy.init(args=argv)
    exit_code = 0
    node = None
    try:
        node = ObstacleSpawner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        rclpy.logging.get_logger('obstacle_spawner').error(f'Cấu hình không hợp lệ: {error}')
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())

# Các package ROS 2 của xe Ackermann

| Package | Nội dung |
| --- | --- |
| ackermann_description | URDF, mesh, vị trí cảm biến và robot_state_publisher |
| ackermann_bringup | Khởi chạy hệ thống, Gazebo, bridge và công cụ thử |
| ackermann_navigation | Nav2, bộ bám quỹ đạo, cấu hình RPP/MPPI và Behavior Tree |
| ackermann_localization | AMCL, map_server và bản đồ |
| ackermann_hardware | Khung cho giao tiếp UART STM32 (chưa triển khai node) |
| ackermann_sensors | Hiển thị tọa độ GPS trong RViz; khung cho driver cảm biến thật |

Từ gốc workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ackermann_bringup
source install/setup.bash
ros2 launch ackermann_bringup simulation.launch.py
```

Điều hướng: `ros2 launch ackermann_bringup navigation.launch.py controller:=rpp`.
Bám quỹ đạo trong mô phỏng: `ros2 launch ackermann_bringup trajectory.launch.py`.
Lệnh tay: `ros2 run ackermann_bringup manual_control forward`.
Chỉ chạy một nguồn lệnh vận tốc tại một thời điểm.

Xem [hướng dẫn chi tiết](ackermann_bringup/README.md).
Firmware STM32 ở `../firmware/stm32f103c8t6`, không phải một package ROS.

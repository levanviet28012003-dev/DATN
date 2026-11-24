# Các package ROS 2 của xe Ackermann

| Package | Nội dung |
| --- | --- |
| ackermann_description | URDF, mesh, vị trí cảm biến và robot_state_publisher |
| ackermann_bringup | Khởi chạy hệ thống, Gazebo, bridge, RViz, Mapviz và công cụ thử |
| ackermann_navigation | Nav2, cấu hình RPP/MPPI, Behavior Tree, ghi điểm GPS |
| ackermann_localization | EKF local (encoder + gyro) và slam_toolbox |
| ackermann_hardware | Cầu nối UART `stm32_bridge` với STM32 (chưa thử với bo thật) |
| ackermann_sensors | Giả lập GPS điện thoại, điền covariance, hiển thị GPS |

Từ gốc workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ackermann_bringup navigation.launch.py      # sân trong phòng: SLAM + Nav2
ros2 launch ackermann_bringup outdoor.launch.py         # đoạn đường Bách khoa
```

Lệnh tay (với `simulation.launch.py`): `ros2 run ackermann_bringup manual_control forward`.
Chỉ chạy một nguồn lệnh vận tốc tại một thời điểm.

Xem [hướng dẫn chi tiết](ackermann_bringup/README.md).
Firmware STM32 ở `../firmware/stm32f103c8t6`, không phải một package ROS.

# Định vị và bản đồ

Kiến trúc (CLAUDE.md, 2026-11-23):

```text
/wheel_odom + /imu/data (chỉ gyro) ─► ekf_odom ─► /odom, TF odom → base_link
/scan + TF odom → base_link ─► slam_toolbox (online async) ─► /map, TF map → odom ─► Nav2
```

```bash
ros2 launch ackermann_localization slam_localization.launch.py use_sim_time:=true
# Mô phỏng đầy đủ: trong phòng hoặc ngoài trời
ros2 launch ackermann_bringup navigation.launch.py
ros2 launch ackermann_bringup outdoor.launch.py
```

- `config/ekf_local.yaml`: EKF local, lấy vx, vy, tốc độ quay từ `/wheel_odom` và tốc độ quay từ gyro; không dùng hướng tuyệt đối (gyro điện thoại không biết hướng Bắc).
- `config/slam_toolbox.yaml`: lấy từ mẫu `mapper_params_online_async.yaml` của slam_toolbox 2.6.10 (nhánh humble), chỉ đổi `base_frame: base_link`, `max_laser_range` 12 m (RPLIDAR A1), `min_laser_range` 0,2 m, `minimum_travel_distance/heading` 0,2 (xe chạy 0,2 m/s), `map_update_interval` 2 s, tắt chế độ tương tác. Code slam_toolbox dùng bản cài apt, không chép vào repo.
- Lưu bản đồ: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'ten_ban_do'}}"`.

GPS không tham gia định vị. Đích GPS sẽ do GPS Waypoint Converter đổi sang frame `map` (chưa có).

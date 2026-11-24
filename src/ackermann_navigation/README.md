# Điều hướng

- `launch/nav2.launch.py`: chỉ khởi chạy các server Nav2; định vị (`slam_toolbox` + `ekf_odom`) do launch khác cung cấp.
- `config/nav2_params.yaml`: planner, costmap, BT navigator, behavior và velocity smoother. Global costmap ở frame `map` của SLAM, lớp tĩnh lấy `/map` của slam_toolbox.
- `config/controller_rpp.yaml`, `controller_mppi.yaml`: chọn qua `controller:=rpp|mppi`.
- `behavior_trees/`: cây hành vi đi một đích và qua nhiều đích.
- `config/nav2_outdoor.yaml`: ghi đè cho ngoài trời (`profile:=outdoor`): rolling costmap 50 m, dung sai 1,5 m, bỏ hướng đích.
- `scripts/gps_goal.py`: gốc của GPS Waypoint Converter (đọc file điểm, kiểm chặng 24 m, gửi `navigate_to_pose`). **Tạm chưa chạy**: phần đổi lat/lon dựa vào `/fromLL` của `navsat_transform`, đã bỏ ngày 2026-11-24; sẽ thay bằng phép khớp GPS với bản đồ SLAM.
- `scripts/gps_waypoint_logger.py`: ghi danh sách điểm GPS ra YAML.
- `config/gps_waypoints_hust.yaml`: bốn điểm mẫu trên đường ở Bách khoa (trùng `OUTDOOR_DEMO_GOALS`).

## Ghi danh sách điểm GPS

Đưa xe tới từng chỗ, dừng hẳn rồi:

```bash
ros2 run ackermann_navigation gps_waypoint_logger --output ~/gps_waypoints.yaml   # --append để ghi tiếp
```

Enter ghi điểm, `u` xóa điểm cuối, `l` liệt kê, `q` lưu và thoát; file được ghi sau mỗi lần thay đổi. Nguồn mặc định là GPS điện thoại `/gps/fix`; mỗi điểm là trung bình 10 s (`--averaging-time`) để giảm nhiễu. Logger từ chối ghi khi mẫu lệch khỏi trung bình hơn 5 m (`--max-spread`, coi như xe đang chạy) và cảnh báo khi chặng dài hơn 24 m.

**Định dạng file**, tương thích file của demo GPS Nav2 (khóa `yaw` nếu có bị bỏ qua, đích ngoài trời không xét hướng):

```yaml
waypoints:
  - latitude: 21.004985868
    longitude: 105.844162606
```

Mỗi chặng (giữa hai điểm liên tiếp, và từ xe tới điểm đầu) phải dưới 24 m vì global costmap ngoài trời chỉ rộng 50 m quanh xe.

Chạy toàn hệ thống: `ros2 launch ackermann_bringup navigation.launch.py` (trong phòng) hoặc `outdoor.launch.py`.
Các launch nhận `use_sim_time` (mặc định true). Không chạy nhiều nguồn phát `/cmd_vel` cùng lúc.
Xem [hướng dẫn và giới hạn mô phỏng](../ackermann_bringup/README.md).

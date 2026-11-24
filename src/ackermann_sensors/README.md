# Cảm biến

Các node trong package này chỉ xử lý và hiển thị dữ liệu; chưa có driver phần cứng (cầu nối iPhone và driver Lidar thật chưa làm).

| Node | Vai trò |
| --- | --- |
| `gps_noise_injector` | Mô phỏng: `/sim/gps/fix` (lý tưởng) → `/gps/fix` giống GPS điện thoại. Mặc định 1 Hz, trễ 0,5 s, bias Gauss-Markov σ 2,5 m τ 120 s + nhiễu trắng σ 1,5 m, covariance theo phương sai lý thuyết; đổi bằng tham số `output_rate`, `delay`, `bias_sigma`, `bias_tau`, `white_sigma`. Giá trị giả định, đo iPhone thật rồi sửa |
| `covariance_setter` | Điền covariance đường chéo cho Imu/Odometry (bridge Fortress để bằng 0). `orientation_variance: [-1, -1, -1]` báo IMU không có hướng tuyệt đối (như gyro điện thoại) |
| `gps_visualizer` | Nhận `NavSatFix`, xuất chữ vĩ độ, kinh độ, độ cao qua `/gps/marker` (RViz) và `/gps/text` (Mapviz) |

`ackermann_bringup/launch/sim_sensors.launch.py` chạy sẵn `gps_noise_injector` và hai `covariance_setter` cho mô phỏng; `navigation.launch.py` và `outdoor.launch.py` chạy sẵn `gps_visualizer`.

Chạy riêng `gps_visualizer` cho một cửa sổ RViz tùy chỉnh:

```bash
ros2 run ackermann_sensors gps_visualizer --ros-args -p use_sim_time:=true
```

Trong RViz chọn **Add → Marker**, topic `/gps/marker`, Fixed Frame `map`. Chữ đi theo TF của `gps_link`, màu vàng khi có tọa độ hợp lệ, đỏ khi chưa có định vị. Tham số: `fix_topic` (`/gps/fix`), `marker_topic` (`/gps/marker`), `text_height` (m, mặc định 0,025), `text_color` (RGB 0–1). Dữ liệu thật thì đặt `use_sim_time:=false`.

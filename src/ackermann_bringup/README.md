# Khởi chạy hệ thống Ackermann

Các package trong workspace:

| Package | Trách nhiệm |
| --- | --- |
| `ackermann_description` | URDF, mesh và robot_state_publisher |
| `ackermann_bringup` | Ghép hệ thống, Gazebo, bridge, RViz, Mapviz và công cụ thử |
| `ackermann_navigation` | Nav2, bộ bám đường RPP/MPPI, ghi điểm GPS |
| `ackermann_localization` | EKF local (encoder + gyro) và slam_toolbox |
| `ackermann_hardware` | Cầu nối UART `stm32_bridge` với STM32 |
| `ackermann_sensors` | Giả lập GPS điện thoại, điền covariance, hiển thị GPS |

Kiến trúc (CLAUDE.md, 2026-11-23):

```text
Encoder + gyro điện thoại ─► ekf_odom ─► TF odom → base_link
Lidar + TF odom → base_link ─► slam_toolbox ─► /map, TF map → odom ─► Nav2
GPS điện thoại ─► GPS Waypoint Converter (chưa có) ─► đích Nav2 trong frame map
```

| Launch | Nội dung |
| --- | --- |
| `simulation.launch.py` | Gazebo (mặc định `nav_world.sdf`), mô hình xe, bridge; chưa có định vị |
| `sim_sensors.launch.py` | Biến cảm biến lý tưởng thành dữ liệu giống xe thật (dùng chung cho hai launch dưới) |
| `navigation.launch.py` | Sân trong phòng: mô phỏng, EKF local, slam_toolbox, Nav2, RViz, thả vật cản |
| `outdoor.launch.py` | Đoạn đường Bách khoa: như trên, thêm ảnh vệ tinh và Mapviz |
| `mapviz.launch.py` | Mapviz trên nền ảnh vệ tinh, gốc tại mốc world |
| `real_robot.launch.py` | Xe thật: robot_state_publisher và `stm32_bridge`; chưa có driver cảm biến |

Bridge duy nhất `config/bridge.yaml` không dùng vị trí thật của Gazebo làm odometry: cảm biến thô ra `/sim/wheel_odom`, `/sim/imu`, `/sim/gps/fix`, vị trí thật ra `/ground_truth/odom` (không phát TF, chỉ để so sai số). `sim_sensors.launch.py` biến chúng thành:

| Topic | Từ | Giống trên xe thật |
| --- | --- | --- |
| `/wheel_odom` | `covariance_setter` | Odometry từ encoder hai bánh sau |
| `/imu/data` | `covariance_setter`, bỏ hướng tuyệt đối | Gyro điện thoại (chỉ tốc độ quay) |
| `/gps/fix` | `gps_noise_injector` | GPS điện thoại: 1 Hz, trễ 0,5 s, bias Gauss-Markov σ 2,5 m τ 120 s + nhiễu trắng σ 1,5 m (giả định, chưa đo iPhone) |

## Điều khiển thủ công bằng lệnh

Terminal thứ nhất (đóng mô phỏng cũ trước khi chạy bản mới):

```bash
cd /home/viet/ackermann_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ackermann_bringup simulation.launch.py
```

Terminal thứ hai (nạp môi trường như trên), chạy lần lượt:

```bash
ros2 run ackermann_bringup manual_control forward          # tiến 3 s rồi tự dừng
ros2 run ackermann_bringup manual_control backward
ros2 run ackermann_bringup manual_control left             # tiến rẽ trái 3 s; right tương tự
ros2 run ackermann_bringup manual_control forward --speed 0.2 --duration 5
ros2 run ackermann_bringup manual_control drive --speed -0.15 --yaw-rate -0.2 --duration 3
ros2 run ackermann_bringup manual_control stop
```

`--speed` tính bằng m/s, tối đa 0,3 m/s; `drive` cho phép tốc độ âm để lùi. `--yaw-rate` là tốc độ đổi hướng thân xe, tối đa ±0,5 rad/s, không phải góc servo; `left`/`right` mặc định 0,3 rad/s. `--duration` là giây thời gian thực, mặc định 3 s. Nhấn **Ctrl+C** để ngắt và gửi vận tốc 0; node phát 20 Hz và gửi lặp lệnh dừng trước khi thoát. Nếu gửi `/cmd_vel` bằng `ros2 topic pub`, sau khi ngắt phải chạy `manual_control stop` vì plugin giữ lệnh cuối cùng. Chỉ chạy một nguồn phát `/cmd_vel` tại một thời điểm.

Chạy không giao diện: `ros2 launch ackermann_bringup simulation.launch.py gz_args:="-r -s $(ros2 pkg prefix ackermann_bringup)/share/ackermann_bringup/worlds/nav_world.sdf"`.

## Cảm biến mô phỏng

- **Lidar:** `lidar_link`, giữa tầng trên, cao hơn `base_link` 0,064 m; 360 tia, 10 Hz, tầm 0,15–12 m, nhiễu 1 cm; ước lượng theo RPLIDAR A1. Ra `/scan`. Cần world có plugin `Sensors` (cả hai world đều có; `empty.sdf` thì không).
- **IMU:** `imu_link`, 50 Hz, nhiễu gyro 0,002 rad/s, gia tốc 0,02 m/s² (ước lượng theo MPU6050). Cảm biến thô biết hướng ENU tuyệt đối, nhưng `sim_sensors` bỏ hướng đi để giống gyro điện thoại. Gyro mô phỏng không trôi bias.
- **GPS:** `gps_link`, NavSat 5 Hz lý tưởng; `gps_noise_injector` hạ xuống 1 Hz và cộng sai số GPS điện thoại. Mốc của `nav_world` là điểm minh họa (10°, 106°); của `outdoor_world` là tim đường Bách khoa (21.0049949, 105.843951). Hướng ENU: +X Đông, +Y Bắc. Bridge không điền covariance nên phải qua `covariance_setter`/`gps_noise_injector` trước khi dùng.

```bash
ros2 topic echo /gps/fix --once
ros2 topic hz /imu/data
```

## Điều hướng trong sân (SLAM + Nav2)

```bash
ros2 launch ackermann_bringup navigation.launch.py                    # RPP
ros2 launch ackermann_bringup navigation.launch.py controller:=mppi
ros2 launch ackermann_bringup navigation.launch.py headless:=true     # không mở cửa sổ Gazebo
```

| Tham số | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `controller` | `rpp` | `rpp` hoặc `mppi` |
| `headless` | `false` | `true` để chạy Gazebo không giao diện |
| `rviz` | `true` | `false` để không mở RViz |

Sân rộng 8 m × 6 m, tâm tại gốc, xe xuất phát tại (0, 0) hướng +X. slam_toolbox lập bản đồ trong lúc chạy: `/map` bắt đầu gần trống và lớn dần khi Lidar quét; bản đồ bắt đầu tại chỗ xe xuất phát nên frame `map` trùng world Gazebo. Chờ dòng `Managed nodes are active`, rồi bấm **2D Goal Pose** trong RViz và kéo để đặt vị trí, hướng đích. Điểm đích là **tâm trục sau** (frame `rear_axle`); dung sai 0,12 m và 0,30 rad. Gửi bằng lệnh:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.5, y: -1.5}, orientation: {w: 1.0}}}"
```

Lưu bản đồ: `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'ban_do_san'}}"`. So vị trí SLAM với vị trí thật: `ros2 run tf2_ros tf2_echo map base_link` và `ros2 topic echo /ground_truth/odom --field pose.pose.position`.

**Kết quả thử (2026-11-23, không giao diện, 6 đích (2,5; −1,5), (2,5; 2,2), (−1; 2,2), (−3; 0), (−2,5; −2,2), (0; 0)):** RPP 6/6, mỗi chặng 19–53 s, trục sau dừng cách đích 0,07–0,13 m, sai số vị trí SLAM so với vị trí thật trung bình 1,9 cm (P95 4,1 cm, lớn nhất 7 cm), sai số hướng trung bình 0,34°; MPPI 6/6, mỗi chặng 14–25 s, dừng cách đích 0,05–0,15 m, sai số SLAM trung bình 2,7 cm (lớn nhất 6 cm). Không có `Failed to make progress`, `collision ahead` hay lỗi lập đường. Odometry bánh xe và gyro mô phỏng gần lý tưởng (không trượt, gyro không trôi), nên xe thật sẽ kém hơn.

| | `controller:=rpp` (mặc định) | `controller:=mppi` |
| --- | --- | --- |
| Kiểu đường (Smac Hybrid-A*, bán kính 0,6 m) | Reeds-Shepp: xe lùi và quay trở được | Dubins: chỉ tiến, quay đầu cần khoảng 1,5 m bề ngang |
| Hợp với | Chỗ chật, nhiều vật cản, đích sát tường | Sân thoáng, vật cản xuất hiện từ xa (trên khoảng 1 m) |
| Né vật cản | Lập lại đường toàn cục, dừng nếu sắp va chạm | Lập lại đường toàn cục và tự tối ưu quỹ đạo cục bộ |
| Điểm yếu | Đường đi hay có đoạn lùi kể cả ở chỗ thoáng | Vật cản sát mũi xe hoặc bị vây kín thì không có đường tiến, báo thất bại |

Dừng bằng Ctrl+C. Nếu chạy `headless:=true` mà tiến trình `ign gazebo` còn sót, dừng Gazebo bằng `ign service -s /server_control --reqtype ignition.msgs.ServerControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'stop: true'` trước khi chạy lại. Không chạy `manual_control` cùng lúc với Nav2 vì cả hai đều phát `/cmd_vel`.

### Thả chướng ngại vật khi xe đang chạy

`obstacle_spawner` (chạy sẵn trong hai launch điều hướng) thả trụ tĩnh màu đỏ, bán kính 0,15 m, cao 0,4 m, vào Gazebo tại mỗi điểm bấm **Publish Point** trong RViz (mỗi lần thả bấm lại Publish Point). Trụ không có sẵn trên bản đồ; Lidar thấy thì hai costmap đánh dấu, đường bị chặn thì Smac lập lại đường (1 lần mỗi giây). Với RPP, đường mới có thể có đoạn lùi nên xe lùi ra rồi vòng qua khi trụ sát mũi; với MPPI, bộ bám đường tự né trong tầm dự đoán 3,5 s nhưng đường toàn cục chỉ có đoạn tiến.

```bash
ros2 service call /clear_obstacles std_srvs/srv/Empty     # xóa mọi trụ đã thả
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.7, y: 0.0}}"   # thả bằng lệnh
```

Node từ chối điểm cách tâm xe (vị trí thật) dưới 0,6 m hoặc chồng lên trụ đã thả. Tham số: `radius` (0,15), `height` (0,40), `min_robot_distance` (0,60), `world`. Phản ứng đã quan sát (2026-11-11, khi còn định vị AMCL): trụ cách xe từ 1,2 m thì lách qua không dừng; khoảng 0,9 m thì RPP vẫn lách qua, MPPI có lần phải lùi hồi phục; khoảng 0,7 m giữa nhiều trụ thì RPP lùi ra vòng lối khác (83 s), MPPI thất bại vì không có đường tiến.

### Giới hạn cần biết

- **Với RPP, đường đi hay có đoạn lùi:** đường Reeds-Shepp ngắn nhất thường có 1–4 lần đổi chiều kể cả ở chỗ thoáng; đổi lại xe quay trở được ở chỗ hẹp.
- **Với MPPI, xe chỉ chạy tiến:** quay đầu 180° cần khoảng 1,5 m; đích sát tường hoặc bị vây có thể không có đường, Nav2 lùi thẳng 0,5 m rồi lập lại, tối đa 8 lần hồi phục.
- **Hướng tới đích chỉ gần đúng:** xe dừng ngay khi vào dung sai, thường còn lệch 0,15–0,3 rad.
- **Khớp lái mô phỏng đáp ứng chậm** (58% sau 1 s, 85% sau 2 s; plugin AckermannSteering của Fortress không chỉnh được hệ số). Vì vậy tốc độ giới hạn 0,20 m/s và bán kính lập đường 0,6 m dù xe quay được tới 0,38 m; RPP hay ra lệnh hết lái để bù trễ (đo 2026-11-23: 42% thời gian lệnh ≥ giới hạn 26°, góc bánh thật trung vị 14°). Servo thật nhanh hơn nhiều, nên tham số phải chỉnh lại trên xe.

### World

`scripts/generate_world.py` sinh `worlds/nav_world.sdf` (sân trong phòng, từ `obstacles()`) và `worlds/outdoor_world.sdf` (đoạn đường Bách khoa, từ `outdoor_obstacles()`). Không sửa tay file world; sửa script rồi chạy lại và build:

```bash
python3 src/ackermann_bringup/scripts/generate_world.py
colcon build --symlink-install --packages-select ackermann_bringup
```

Giữ vùng bán kính 0,5 m quanh gốc trống, các lối đi rộng từ 1,0 m và chỗ cần quay đầu rộng từ 1,8 m. `colcon test` kiểm tra file đã sinh khớp với script.

## Mô phỏng ngoài trời (đoạn đường Bách khoa)

```bash
# Lần đầu (cần mạng): dán ảnh vệ tinh quanh mốc world lên mặt đất Gazebo.
python3 src/ackermann_bringup/scripts/generate_satellite_ground.py
# Gazebo, RViz, Mapviz; chờ "Managed nodes are active" rồi gửi đích bằng 2D Goal Pose (frame map).
ros2 launch ackermann_bringup outdoor.launch.py
```

| Tham số | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `controller` | `rpp` | `rpp` hoặc `mppi` |
| `headless`, `rviz`, `mapviz` | `false`, `true`, `true` | Bật/tắt giao diện |
| `gps_noise` | `true` | `false`: GPS vẫn 1 Hz, trễ 0,5 s nhưng không sai số vị trí |
| `gps_seed` | `-1` | Cố định nhiễu GPS để lặp lại một lần thử |
| `satellite` | `true` | Thả model ảnh vệ tinh nếu đã tạo cho đúng mốc world |

Luồng định vị giống trong phòng (EKF local + slam_toolbox); Nav2 nạp thêm `nav2_outdoor.yaml` (global costmap cửa sổ trượt 50 m vì bản đồ SLAM chỉ phủ vùng Lidar đã quét, dung sai 1,5 m, bỏ hướng đích). Đích phải cách xe dưới 24 m. GPS điện thoại giả lập chỉ để hiển thị (chấm đỏ và chữ tọa độ trên Mapviz); đổi đích lat/lon sang map là việc của GPS Waypoint Converter, chưa có. Lớp "Click: gửi đích GPS" trên Mapviz phát `/gps_goal` nhưng hiện chưa có node nhận.

**Mốc tọa độ.** `OUTDOOR_LATITUDE_DEG`, `OUTDOOR_LONGITUDE_DEG` trong `generate_world.py`: tim đường nội bộ Đông–Tây ĐH Bách khoa Hà Nội, mặt đường y từ −5 đến +5 m, xe xuất phát tại gốc hướng Đông. Launch đọc mốc từ world cho gốc Mapviz và kiểm tra ảnh vệ tinh. Đổi mốc: sửa hai hằng số, chạy `generate_world.py`, build, chạy lại `generate_satellite_ground.py`. Đích mẫu trên mặt đường (x Đông, y Bắc, m): (22, −1), (42, −1), (20, 2), (0, 0). Thân cây theo hàng cây trên ảnh; ô tô đỗ, cọc tiêu, xe máy, rào chắn là vật cản giả định để thử né.

**Ảnh vệ tinh.** `generate_satellite_ground.py` tải ô Esri World Imagery (cache ở `~/.cache/ackermann_ws/tiles`), tự chọn zoom cao nhất còn ảnh thật tại mốc, cắt vùng 160 m (đổi bằng `--size`) có tâm là mốc, ghi model chỉ có visual vào `~/.cache/ackermann_ws/satellite_ground/`. Không đưa vào git: ảnh thuộc Esri, Maxar, Earthstar Geographics; chỉ dùng học tập và phải ghi nguồn. Ở Bách khoa có ảnh zoom 19 (0,28 m/điểm ảnh), khớp `max_zoom` 19 trong `config/mapviz_outdoor.mvc`.

## Dependency khi chạy

ROS 2 Humble, ros_gz_sim/bridge, RViz, Mapviz, robot_localization, slam_toolbox và các package Nav2 khai báo trong `package.xml`. Kiểm tra từ gốc workspace: `rosdep check --from-paths src --ignore-src`.

# Tiến độ dự án xe Ackermann

Cập nhật: **2026-11-24** (bỏ phần thừa theo kiến trúc SLAM + GPS điện thoại).

Tài liệu phân biệt **đã có code**, **đã thử mô phỏng** và **đã thử phần cứng**.

## 1. Tổng quan

- **Kiến trúc (người dùng chốt 2026-11-22/23):** encoder + gyro điện thoại → EKF local → `odom → base_link`; Lidar → slam_toolbox → `/map`, `map → odom`; `/map` → Nav2; GPS điện thoại → GPS Waypoint Converter → đích Nav2 trong frame `map`. Chi tiết ở CLAUDE.md.
- **Mô phỏng:** giai đoạn A (EKF local + SLAM + Nav2) chạy được trong sân trong phòng, 6/6 đích với RPP và MPPI. Mô phỏng ngoài trời đã chuyển sang cùng kiến trúc; GPS điện thoại giả lập chỉ để hiển thị vì chưa có GPS Waypoint Converter.
- **Xe thật:** firmware STM32 (servo, motor, encoder, động học, giao thức UART, watchdog, ESTOP) và `stm32_bridge` trên Pi có code và test trên máy tính, **chưa chạy với nhau trên bo**. Người dùng đã xác nhận servo và hai motor chạy được.
- **Nút thắt là tích hợp phần cứng:** thử UART trên bo, odometry encoder `/wheel_odom`, driver Lidar, cầu nối iPhone, lớp an toàn.
- **2026-11-24:** bỏ toàn bộ phần không thuộc kiến trúc: AMCL và bản đồ có sẵn, hai EKF + `navsat_transform` (thiết kế module GPS), world GPS trống và `gps_view`, bridge dùng vị trí thật làm odom, hai chế độ thử riêng của firmware. Các phần này còn trong commit e73dc03 (AMCL, `gps_world`, `gps_view`, bridge cũ) hoặc chỉ có trong lịch sử làm việc (hai EKF, chưa commit).
- Trước đó: 2026-11-21 xóa chế độ bám quỹ đạo; người dùng xác nhận góc lái tối đa 40°, firmware đổi dải xung servo cho khớp.

| Phần | Có code | Đã thử mô phỏng | Đã thử phần cứng |
| --- | --- | --- | --- |
| Mô hình xe, Gazebo, lái tay | Có | Có | Không áp dụng |
| EKF local + SLAM + Nav2 (giai đoạn A) | Có | Có: trong phòng 6/6 đích RPP và MPPI; ngoài trời xem mục 6 | Chưa |
| GPS Waypoint Converter (đích GPS → map) | Chưa (`gps_goal` là gốc, tạm chưa chạy) | Chưa | Chưa |
| Ghi danh sách điểm GPS | Có (`gps_waypoint_logger`, nguồn `/gps/fix`) | Có với nguồn cũ `/gps/filtered` (2026-11-18) | Chưa |
| Servo (firmware) | Có | Không áp dụng | Có, chế độ thử |
| Motor cầu sau, encoder (firmware) | Có | Không áp dụng | Motor có; encoder chưa đo hệ số |
| Giao thức UART Pi ↔ STM32 | Có, test trên máy tính | Không áp dụng | Chưa |
| Odometry encoder `/wheel_odom` | Chưa | Không áp dụng | Chưa |
| Driver Lidar, cầu nối iPhone (GPS + gyro) | Chưa | Không áp dụng | Chưa |
| `safety_supervisor`, lái tay ưu tiên, nút dừng vật lý | Chưa | Chưa | Chưa |
| Launch xe thật đầy đủ | Khung (robot_state_publisher + bridge) | Không áp dụng | Chưa |

## 2. Cấu trúc ROS 2

| Package | Nội dung hiện có |
| --- | --- |
| ackermann_description | URDF/Xacro, mesh hai tầng, frame `rear_axle`, Lidar/GPS/IMU mô phỏng, robot_state_publisher |
| ackermann_bringup | Launch `simulation`, `sim_sensors`, `navigation`, `outdoor`, `mapviz`, `real_robot`; world `nav_world`, `outdoor_world`; bridge duy nhất `bridge.yaml`; RViz, Mapviz, ảnh vệ tinh; `manual_control`, `obstacle_spawner` |
| ackermann_localization | `slam_localization.launch.py`: `ekf_odom` (`ekf_local.yaml`) + `slam_toolbox` (`slam_toolbox.yaml`) |
| ackermann_navigation | Nav2 (profile trong phòng/ngoài trời), Smac Hybrid-A*, RPP/MPPI, cây hành vi không Spin; `gps_waypoint_logger`; `gps_goal` (gốc của bộ đổi đích) |
| ackermann_sensors | `gps_noise_injector` (GPS điện thoại giả lập), `covariance_setter`, `gps_visualizer` |
| ackermann_hardware | `stm32_bridge` (/cmd_vel → CMD UART, STATE → /stm32/joint_states, /stm32/battery, /diagnostics, /stm32/estop); chưa thử với bo thật |

| Node/script | File | Chức năng |
| --- | --- | --- |
| manual_control | src/ackermann_bringup/scripts/manual_control.py | Phát /cmd_vel thủ công, gửi dừng khi kết thúc |
| obstacle_spawner | src/ackermann_bringup/scripts/obstacle_spawner.py | Nhận /clicked_point, thả trụ; service /clear_obstacles |
| gps_noise_injector | src/ackermann_sensors/scripts/gps_noise_injector.py | /sim/gps/fix → /gps/fix giống GPS điện thoại (1 Hz, trễ 0,5 s, bias σ 2,5 m τ 120 s + nhiễu σ 1,5 m) |
| covariance_setter | src/ackermann_sensors/scripts/covariance_setter.py | Điền covariance Imu/Odometry; IMU bỏ hướng tuyệt đối như gyro điện thoại |
| gps_visualizer | src/ackermann_sensors/scripts/gps_visualizer.py | Chữ tọa độ GPS cho RViz và Mapviz |
| gps_waypoint_logger | src/ackermann_navigation/scripts/gps_waypoint_logger.py | Ghi trung bình 10 s của /gps/fix thành file YAML danh sách điểm |
| gps_goal | src/ackermann_navigation/scripts/gps_goal.py | Gốc của GPS Waypoint Converter; **tạm chưa chạy** vì dựa vào /fromLL của navsat_transform đã bỏ |
| stm32_bridge | src/ackermann_hardware/scripts/stm32_bridge.py | /cmd_vel ↔ khung UART tới STM32; đổi (v, ω) sang (v, δ) tại tâm trục sau |
| generate_world (script) | src/ackermann_bringup/scripts/generate_world.py | Sinh nav_world.sdf và outdoor_world.sdf |
| generate_satellite_ground (script) | src/ackermann_bringup/scripts/generate_satellite_ground.py | Tải ảnh Esri, tạo model mặt đất ảnh vệ tinh cho Gazebo |

## 3. Mô phỏng và cảm biến

| Hạng mục | Trạng thái và giới hạn |
| --- | --- |
| Mô hình xe | Hai tầng; chiều dài cơ sở 0,165 m, vết bánh 0,167 m, bánh 0,075 m, `steering_limit` 0,45 rad. Một số kích thước/khối lượng là ước lượng |
| Chấp hành Ackermann | Plugin AckermannSteering của Fortress; khớp lái trễ khoảng 1 s, không chỉnh được; chưa dùng ros2_control |
| Bridge | `bridge.yaml` duy nhất: cảm biến thô ra /sim/wheel_odom, /sim/imu, /sim/gps/fix, vị trí thật ra /ground_truth/odom (không TF); /scan, /joint_states, /clock, /cmd_vel |
| Odometry | /odom là đầu ra ekf_odom (odometry plugin + gyro). Odometry plugin gần lý tưởng, không trượt |
| Lidar | gpu_lidar 360 tia, 10 Hz, tầm 0,15–12 m, frame lidar_link; theo RPLIDAR A1 |
| IMU | 50 Hz; `sim_sensors` bỏ hướng tuyệt đối, chỉ giữ gyro như điện thoại. Gyro mô phỏng chưa có trôi bias |
| GPS | NavSat 5 Hz lý tưởng → gps_noise_injector → /gps/fix giống GPS điện thoại (giả định, chưa đo iPhone). Chỉ hiển thị |
| World | nav_world.sdf (sân 8 × 6 m có tường, vật cản; mặc định) và outdoor_world.sdf (đường Bách khoa, mốc 21.0049949, 105.843951; cây, ô tô, cọc tiêu, xe máy, rào chắn; ảnh vệ tinh Esri zoom 19) |
| RViz, Mapviz | navigation.rviz, outdoor.rviz; Mapviz: ảnh vệ tinh, GPS điện thoại, vị trí thật, Lidar, /plan, chữ tọa độ, lớp click gửi đích GPS (chưa có node nhận) |

## 4. Điều hướng và định vị

| Hạng mục | Hiện trạng |
| --- | --- |
| Định vị | ekf_odom (/wheel_odom vx, vy, vyaw + tốc độ quay gyro) → odom → base_link; slam_toolbox online async → /map, map → odom. Bản đồ mới mỗi lần chạy |
| Lập đường | Smac Hybrid-A*, bán kính 0,60 m; Reeds-Shepp cho RPP, Dubins cho MPPI |
| Global costmap | Trong phòng: lớp tĩnh từ /map của SLAM. Ngoài trời: cửa sổ trượt 50 m (đích phải cách xe dưới 24 m) |
| Bám đường, tránh vật cản | RPP hoặc MPPI, hai costmap dùng /scan, BackUp, velocity smoother. RPP hay ra lệnh hết lái vì khớp lái mô phỏng trễ (đo 2026-11-23: 42% thời gian lệnh ≥ giới hạn, góc bánh thật trung vị 14°) |
| Vật cản thêm khi chạy | obstacle_spawner; đã thử với AMCL (2026-11-11), chưa thử lại với SLAM |
| Đích GPS | Chưa có GPS Waypoint Converter; đề xuất khớp quỹ đạo GPS với quỹ đạo SLAM (chưa chốt) |

Cấu hình tốc độ: RPP 0,20 m/s, lookahead 0,2–0,4 m; MPPI vx_max 0,20 m/s; smoother ±0,25 m/s.

## 5. Firmware STM32 và phần cứng

| Hạng mục | Hiện trạng |
| --- | --- |
| Nền tảng | C viết thanh ghi, startup, linker 64 KiB Flash/20 KiB RAM, CMSIS, Makefile; không dùng HAL |
| Build/debug | GCC ARM 10.3, OpenOCD, GDB, VS Code/ST-Link; người dùng đã debug trên bo |
| Servo | PA0/TIM2_CH1, PWM 50 Hz, 500–2500 µs cho 0–180°, giới hạn trục 50–130°; người dùng xác nhận quay được |
| Động học, góc lái | (v, δ) → tốc độ hai bánh sau và xung servo. Góc lái tối đa 40° (người dùng xác nhận) ánh xạ 1056/1500/1944 µs = biên ±40° trục servo; giả định cơ cấu lái gần 1:1, chưa kiểm góc bánh |
| Motor cầu sau | Hiwonder I²C 0x34, M1/M2, PB6/PB7; người dùng xác nhận hai motor chạy; dấu -1 cho cả hai bánh |
| Encoder | Đọc qua I²C mỗi 20 ms; 3960 xung/vòng, bánh 0,075 m; chưa đo xác nhận |
| UART, giao thức | USART1 PA9/PA10, 115200 8N1, RX DMA vòng 512 byte, TX ngắt; protocol.c + communication.c (CRC16, watchdog 300 ms, ESTOP); `make test` đạt; chưa đo trên bo |
| Chế độ main | `APP_MANUAL_CONTROL = 1` (đang chọn): điều khiển tay qua Watch `manual_control`, lái theo vận tốc/góc lái hoặc đặt riêng servo/từng motor (11,5 KB Flash, 488 B RAM). `= 0`: nhận lệnh Pi (12,5 KB, 1 KB). Hai chế độ thử riêng servo/motor đã bỏ 2026-11-24 |
| Xử lý lỗi | Lỗi I²C → APP_FAULT, thử gửi dừng mỗi 100 ms; mất CMD 300 ms, enable = 0 hoặc ESTOP thì dừng và trả lái về giữa. Chưa có IWDG |

Thông số cần đồng bộ/hiệu chỉnh:

- ackerman.h: chiều dài cơ sở 0,170 m, vết bánh 0,180 m; URDF 0,165 m và 0,167 m. Lấy số đo xe thật làm chuẩn, gom vào `config/vehicle.yaml` (chưa có).
- Hệ số tốc độ: lệnh 20 được coi là 0,10 m/s. Nếu đơn vị driver là xung/10 ms (chú thích motor.h) thì với PPR 3960, lệnh 20 ≈ 0,119 m/s và lệnh 30 ≈ 0,18 m/s. Cần đo bằng encoder.

## 6. Kết quả kiểm tra đã ghi nhận

Lịch sử kiểm tra; các bài trước 2026-11-23 dùng thiết kế đã bỏ (AMCL, hai EKF), chỉ giữ để tham khảo.

| Thời điểm | Kết quả và phạm vi |
| --- | --- |
| 2026-11-11 (AMCL) | Nav2 trong phòng: RPP và MPPI mỗi bộ 6 đích; khớp lái mô phỏng đạt 58% sau 1 s, 85% sau 2 s |
| 2026-11-17/18 (hai EKF, đã bỏ) | GPS điện thoại với Nav2 ở map: 14 m mất 299 s (xe giật) → bài học: GPS điện thoại không được kéo vị trí mà Nav2 bám. Module GPS: 4/4 đích, EKF sai trung bình 1,5 m |
| 2026-11-21 | Xóa bám quỹ đạo; firmware đổi xung servo, thêm chế độ điều khiển tay; build đạt |
| 2026-11-23, giai đoạn A | SLAM + Nav2 trong phòng, không dùng vị trí thật làm odom. RPP 6/6 (chặng 19–53 s, trục sau cách đích 0,07–0,13 m, sai số SLAM trung bình 1,9 cm, lớn nhất 7 cm, hướng 0,34°); MPPI 6/6 (14–25 s, 0,05–0,15 m, SLAM trung bình 2,7 cm). Không Failed to make progress, collision ahead, lỗi lập đường |
| 2026-11-24, sau khi bỏ phần thừa | Build 4 package ROS đạt; 84 ca test qua (colcon test-result báo 97, gồm 13 mục CTest). Firmware build đạt cả hai chế độ (kích thước không đổi), `make test` đạt |
| Phần cứng, người dùng xác nhận | Servo quay được; hai motor chạy bằng I²C. Chưa đo encoder, chưa thử UART, chưa chạy xe thật |

## 7. Vấn đề và rủi ro đã biết

| Vấn đề | Ảnh hưởng | Hướng xử lý |
| --- | --- | --- |
| Hướng tuyệt đối của map so với GPS | Gyro chỉ cho tốc độ quay; la bàn nhiễu từ; map của SLAM không gắn hướng Bắc | GPS Waypoint Converter khớp quỹ đạo GPS với quỹ đạo SLAM sau 10–20 m |
| SLAM ngoài trời | Lidar 2D cao khoảng 10 cm, tầm 12 m: dọc đường ít vật thể, scan matching có thể trượt theo chiều đường | Encoder làm giá trị đầu; thử trên world ngoài trời và đường thật |
| Mô phỏng quá lý tưởng | Odometry không trượt, gyro không trôi | Thêm trượt và trôi bias vào mô phỏng |
| iPhone qua WiFi | Trễ, mất gói; phải gắn cứng, biết hướng trục | Pi kết nối hotspot iPhone; TF tĩnh `phone_link` |
| Độ chính xác đích GPS | Không tốt hơn GPS điện thoại (vài mét) | Dung sai đích lớn hơn sai số GPS đo được |
| Một lỗi I²C khóa FAULT vĩnh viễn | Nhiễu motor có thể làm xe dừng giữa đường, phải reset; chưa có IWDG | Khóa sau N lỗi liên tiếp, thêm CLEAR_FAULT hoặc GPIO → NRST, bật IWDG |
| STATE không đi cùng lần đọc encoder, không có mốc thời gian | Vận tốc tính trên Pi giật | Gửi STATE ngay sau Encoder_Update hoặc thêm `uint32 stamp_ms` |
| Khởi động lại stm32_bridge tự nhả ESTOP | /stm32/estop QoS volatile | QoS transient_local hoặc khởi động ở trạng thái khóa; nút dừng vật lý |
| Lidar ngoài trời | RPLIDAR A1 kém dưới nắng; đặt thấp không thấy mép vỉa hè | Cân nhắc Lidar chịu nắng |
| Tải Pi4 | Chưa đo SLAM + Nav2 trên Pi4 | Đo CPU; ưu tiên RPP |
| Đáp ứng lái mô phỏng khác xe thật | RPP chỉnh theo trễ lái 1 s, hay ra lệnh hết lái | Đo servo thật rồi chỉnh RPP (khoảng nhìn trước) trên xe |

## 8. Quản lý mã nguồn

- .gitignore ở gốc loại build/install/log, cache Python, .pytest_cache.
- Commit mới nhất là e73dc03. **Mọi thay đổi từ đó tới nay chưa được commit.** Nên commit sớm, chia theo nhóm.
- Lấy lại phần đã xóa có trong e73dc03: `git checkout e73dc03 -- <file>` (bám quỹ đạo, AMCL và bản đồ, `gps_world`, `gps_view`, bridge cũ).

## 9. Việc tiếp theo

### Giai đoạn A trên mô phỏng

- [x] EKF local + slam_toolbox + Nav2 trong phòng (2026-11-23), 6/6 đích với RPP và MPPI.
- [x] Bỏ phần không thuộc kiến trúc (2026-11-24).
- [ ] Mở rộng: gyro có trôi bias, odometry có trượt; thả vật cản khi đang lập bản đồ; lưu và mở lại bản đồ.
- [ ] Thử RPP với khoảng nhìn trước dài hơn để lệnh lái tỉ lệ với độ cong (đang chờ người dùng quyết).

### Giai đoạn B trên xe thật (từ dưới lên)

- [x] Giao thức Pi–STM32, protocol/communication, watchdog, ESTOP, stm32_bridge; đồng bộ góc lái 40°; chế độ điều khiển tay.
- [ ] Commit các thay đổi đang có.
- [ ] B1 giá kê, chế độ điều khiển tay: chiều lái và góc bánh ở ±40°, chiều quay hai bánh, tốc độ theo lệnh.
- [ ] B2 encoder: đo xung/vòng, so quãng đường với thước; gom thông số vào `config/vehicle.yaml`. Sửa STATE gửi cùng lần đọc encoder.
- [ ] B3 UART Pi4 ↔ STM32 (cài python3-serial): /diagnostics, rút dây xe dừng trong 300 ms, ESTOP; lái bằng manual_control.
- [ ] B4 node /wheel_odom từ encoder; đo odometry đi thẳng 5 m và vòng kín.
- [ ] B5 driver Lidar A1 + slam_toolbox, lái tay lập bản đồ.
- [ ] B6 Nav2 trên bản đồ: đích ngắn rồi xa, có vật cản.
- [ ] B7 cầu nối iPhone: gyro vào EKF, GPS ra /gps/fix có covariance; đo GPS đứng yên 10–15 phút rồi sửa mô phỏng.
- [ ] An toàn: ackermann_supervisor (teleop có nút giữ, twist_mux, máy trạng thái), nút dừng vật lý; sửa firmware theo mục 7.

### Sau đó: đích GPS ngoài trời

- [ ] GPS Waypoint Converter (phát triển từ gps_goal), chốt cách khớp GPS với map.
- [ ] Hoàn thiện real_robot.launch.py; đo tải CPU Pi4.
- [ ] Thử tại Bách khoa: lái tay ghi điểm, đi chặng ngắn 5–10 m, rồi danh sách mẫu; ghi bag.

## 10. Lệnh chạy hiện hành

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Sân trong phòng: EKF local + SLAM + Nav2 + RViz (2D Goal Pose, Publish Point).
ros2 launch ackermann_bringup navigation.launch.py
# Đường Bách khoa: như trên, thêm ảnh vệ tinh và Mapviz.
ros2 launch ackermann_bringup outdoor.launch.py
# Chỉ mô phỏng, lái tay.
ros2 launch ackermann_bringup simulation.launch.py
ros2 run ackermann_bringup manual_control forward --speed 0.2 --duration 5
# Xe thật (chưa thử): robot_state_publisher + stm32_bridge.
ros2 launch ackermann_bringup real_robot.launch.py

# Test ROS và firmware.
colcon test --packages-select ackermann_bringup ackermann_navigation ackermann_sensors ackermann_hardware
colcon test-result --verbose
make -C firmware/stm32f103c8t6 test
~~~

Xem hướng dẫn chi tiết ở [src/README.md](src/README.md), [ackermann_bringup/README.md](src/ackermann_bringup/README.md), [ackermann_hardware/README.md](src/ackermann_hardware/README.md) và [firmware/stm32f103c8t6/README.md](firmware/stm32f103c8t6/README.md).

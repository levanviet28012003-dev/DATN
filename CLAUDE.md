# CLAUDE.md

Hướng dẫn làm việc trong repo này. Trạng thái dự án và việc còn lại nằm ở [progress.md](progress.md); hãy cập nhật file đó khi hoàn thành một hạng mục.

## Mục tiêu và kiến trúc đích (chốt 2026-11-17; 2026-11-22/23 đổi sang SLAM + GPS và gyro điện thoại)

Xe chạy **ngoài trời** như xe thật: định vị cục bộ bằng **SLAM Lidar 2D**, đi tới điểm đích GPS lấy từ **điện thoại gắn trên xe**, tránh vật cản bằng **Lidar 2D**. Pi4 chạy ROS 2 Humble, nối STM32F103C8T6 qua UART. Tài liệu Claude Docs "Thiết kế hệ thống xe tự hành Ackermann" viết theo thiết kế module GPS 2026-11-18, chưa cập nhật hướng mới; mục này giữ các quyết định mà code phải tuân theo.

- **Kiến trúc (người dùng vẽ 2026-11-23): SLAM dẫn đường, GPS chỉ cho đích.**
  ```text
  Encoder + gyro điện thoại ─► EKF local (ekf_odom) ─► TF odom → base_link
  Lidar A1 + TF odom → base_link ─► slam_toolbox ─► /map và TF map → odom
  /map ─► Nav2 (lập đường ở map, costmap cục bộ ở odom)
  GPS điện thoại ─► GPS Waypoint Converter ─► đích cho Nav2 trong frame map
  ```
  `map` là frame bản đồ SLAM. Không có khung Đông–Bắc riêng, không `ekf_map`, không `navsat_transform`, không AMCL; GPS không kéo vị trí xe. Đã bỏ hẳn các phần đó ngày 2026-11-24.
- **GPS Waypoint Converter (chưa có):** đổi lat/lon sang `map`. Cần phép quay + tịnh tiến giữa tọa độ GPS và `map`; đề xuất (chưa chốt): khớp quỹ đạo GPS với quỹ đạo SLAM sau khi xe chạy 10–20 m, giá trị đầu từ vị trí GPS lúc khởi động và hướng ban đầu. Phát triển từ `gps_goal` (đọc file điểm, kiểm chặng 24 m, gửi `NavigateToPose`, gửi lại đích khi lệch).
- **Hướng tuyệt đối lấy từ phép khớp hai quỹ đạo**; gyro điện thoại chỉ cho tốc độ quay, la bàn (nhiễu từ motor) chỉ làm giá trị ban đầu.
- **SLAM vừa chạy vừa lập bản đồ** (`slam_toolbox` online async, mỗi lần chạy một bản đồ mới). Code slam_toolbox dùng bản cài apt (2.6.10, nhánh humble); repo chỉ giữ file tham số.
- **Điện thoại là iPhone**; cách gửi GPS + gyro sang Pi (app nào, giao thức nào) chưa chọn. Dự kiến Pi kết nối hotspot của iPhone.
- **Hai bài học GPS điện thoại:** GPS điện thoại (1 Hz, nhảy 3–5 m) không được kéo trực tiếp vị trí mà Nav2 bám (đo 2026-11-18: xe giật, 14 m mất 299 s); dung sai đích phải lớn hơn sai số GPS (vài mét).
- **Pi4 quyết định, STM32 chỉ thực thi.** STM32 không chứa logic điều hướng; mất lệnh quá 300–500 ms thì tự dừng xe (watchdog).
- **Ở lại Humble + Fortress.** Không nâng cấp để dùng `/follow_gps_waypoints` (chỉ có từ Iron).
- **Giao thức UART** 115200 8N1, khung `[0xAA 0x55][Len][Type][Seq][Payload][CRC16]`: CMD 0x01 (Pi→STM: v mm/s int16, δ 0,01° int16, enable; 20–50 Hz), STATE 0x02 (STM→Pi: encoder int32×2, điện áp mV uint16, trạng thái uint8; 50 Hz), ESTOP 0x03.
- **Thêm `ackermann_supervisor/safety_supervisor.py`:** máy trạng thái an toàn, là nơi duy nhất được phát lệnh tới `stm32_bridge`.

Các file dự kiến thêm (chưa tồn tại, đừng giả định): `config/vehicle.yaml` (một nguồn thông số xe), GPS Waypoint Converter, cầu nối iPhone (GPS + gyro → `/gps/fix`, `/imu/data`), node odometry encoder (`/wheel_odom`), driver Lidar thật, package `ackermann_supervisor`.

### Chênh lệch mô phỏng và firmware phải xử lý trước khi chạy xe thật

| Thông số | Mô phỏng (URDF, Nav2) | Firmware |
| --- | --- | --- |
| Chiều dài cơ sở | 0,165 m | 0,170 m (`ackerman.h`) |
| Khoảng cách hai bánh | 0,167 m | 0,180 m |
| Đường kính bánh | 0,075 m | 0,075 m (`ackerman.h`, `encoder.h`) |
| Góc lái tối đa | 0,45 rad (`steering_limit`) | 40°, người dùng xác nhận 2026-11-21 (`ACKERMAN_MAX_STEER_RAD`, `max_steering_angle` của `stm32_bridge.yaml`; xung 1056/1944 µs = biên ±40° trục servo) |
| Tốc độ | 0,20 m/s | `ACKERMAN_MAX_VELOCITY` 0,20; thực tế khoảng 0,15 |

Góc lái tối đa của xe thật là 40°: với chiều dài cơ sở 0,170 m cho bán kính quay khoảng 0,20 m, nhỏ hơn nhiều 0,60 m mà Nav2 dùng để lập đường (0,60 m chỉ cần khoảng 16°, nên Nav2 vẫn đúng cho xe thật). Firmware ánh xạ ±40° lái thành biên ±40° trục servo (`_Static_assert` so với `servo.h`), tức giả định cơ cấu lái gần 1:1; cần kiểm góc bánh và chiều lái trên giá kê. Mô phỏng giữ `steering_limit` 0,45 rad. Dự định gom vào `config/vehicle.yaml`.

Giao thức chi tiết (Len chỉ là số byte payload; CRC16-CCITT 0x1021, khởi tạo 0xFFFF, tính trên Len..Payload; little-endian; bit trạng thái; ESTOP payload 1 byte 1/0) nằm trong `firmware/stm32f103c8t6/peripheral/inc/protocol.h`. `stm32_bridge.py` phải khớp từng byte; test hai phía dùng chung các khung mẫu.

## Tổng quan hiện trạng

Workspace có 6 package ROS 2 Humble (`ament_cmake`, node viết bằng Python):

- `ackermann_description`: URDF/xacro hai tầng, mesh, frame `rear_axle`. Lidar 2D (ước lượng theo RPLIDAR A1), NavSat 5 Hz lý tưởng (`gps_link`), IMU 50 Hz (`imu_link`).
- `ackermann_bringup`: Gazebo Fortress, bridge duy nhất `bridge.yaml` (cảm biến thô ra `/sim/...`, vị trí thật ra `/ground_truth/odom`, không phát TF), world `nav_world.sdf` (sân trong phòng, mặc định) và `outdoor_world.sdf` (đường Bách khoa), launch `simulation`, `sim_sensors`, `navigation`, `outdoor`, `mapviz`, `real_robot`; `manual_control`, `obstacle_spawner`; `generate_world.py`; `generate_satellite_ground.py` (ảnh Esri ở `~/.cache/ackermann_ws/`, không vào git).
- `ackermann_localization`: `slam_localization.launch.py` = `ekf_odom` (`ekf_local.yaml`: encoder + tốc độ quay gyro) + `slam_toolbox` (`slam_toolbox.yaml`, online async, phát `map → odom`).
- `ackermann_navigation`: Nav2 (Smac Hybrid-A*, RPP hoặc MPPI; `profile:=outdoor` nạp thêm `nav2_outdoor.yaml`), cây hành vi, `gps_waypoint_logger` (ghi điểm từ GPS điện thoại `/gps/fix`), `gps_goal` (gốc của GPS Waypoint Converter, **tạm chưa chạy** vì `/fromLL` đã bỏ).
- `ackermann_sensors`: `gps_noise_injector` (giả lập GPS điện thoại: 1 Hz, trễ 0,5 s, nhảy vài mét), `covariance_setter` (điền covariance; IMU bỏ hướng tuyệt đối như gyro điện thoại), `gps_visualizer`.
- `ackermann_hardware`: node `stm32_bridge` (UART ↔ ROS): `/cmd_vel` → CMD, STATE → `/stm32/joint_states`, `/stm32/battery`, `/diagnostics`; `/stm32/estop` (Bool). **Chưa thử với STM32 thật**, chưa có odometry encoder.

Mô phỏng đã chạy được: **giai đoạn A** (`navigation.launch.py`, 2026-11-23): odometry bánh xe + gyro không hướng tuyệt đối → `ekf_odom` → `slam_toolbox` → Nav2 trong sân `nav_world`, 6/6 đích với cả RPP và MPPI, sai số SLAM trung bình 2–3 cm so với vị trí thật. Odometry và gyro mô phỏng gần lý tưởng (không trượt, không trôi bias). Ngoài trời xem progress.md.

Firmware `firmware/stm32f103c8t6/`: servo, motor (I²C, module Hiwonder 0x34), encoder, động học Ackermann, SysTick, driver cấp thanh ghi. `core/src/main.c` có hai chế độ chọn bằng `APP_MANUAL_CONTROL`: 1 (đang chọn) điều khiển tay cả servo và hai motor qua Watch `manual_control` (lái theo vận tốc/góc lái qua động học, hoặc đặt riêng servo và từng motor); 0 nhận lệnh từ Pi qua USART1 (RX bằng DMA1 kênh 5, TX bằng ngắt, watchdog 300 ms, ESTOP, STATE 50 Hz). `make test` chạy test giao thức trên máy tính. UART chưa đo trên bo.

Xem hướng dẫn chạy ở `src/README.md` và `src/ackermann_bringup/README.md`. Sinh world bằng `python3 src/ackermann_bringup/scripts/generate_world.py`.

## Môi trường

Ubuntu 22.04, ROS 2 Humble, Ignition Gazebo Fortress (lệnh `ign`, plugin `ignition-gazebo-*`, kiểu message `ignition.msgs.*`), Python 3.10. Không dùng cú pháp `gz.*` của Gazebo bản mới trong URDF hoặc bridge.

Máy này có `arm-none-eabi-gcc` (10.3), `openocd`, `ros-humble-slam-toolbox` 2.6.10; chưa có `st-flash` và `pyserial` (`sudo apt install python3-serial`, cần cho `stm32_bridge`).

## Build

`build/`, `install/`, `log/`, `__pycache__/` nằm trong `.gitignore`, không được commit. Nếu chuyển repo sang thư mục khác, xóa `build install log` rồi build lại, vì CMake cache giữ đường dẫn tuyệt đối cũ. Xóa file khỏi `src` thì bản cài cũ trong `install/` vẫn còn (colcon không tự gỡ): xóa tay các file/symlink treo (`find install -xtype l`).

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Lệnh thường dùng

```bash
# Chỉ mô phỏng (nav_world), lái tay; thêm gz_args:="-r -s <world>" để chạy không giao diện
ros2 launch ackermann_bringup simulation.launch.py
ros2 run ackermann_bringup manual_control forward --speed 0.2 --duration 5   # forward|backward|left|right|drive|stop

# Sân trong phòng: EKF local + slam_toolbox + Nav2 + RViz; đích bằng 2D Goal Pose, thả vật cản bằng Publish Point
ros2 launch ackermann_bringup navigation.launch.py
ros2 launch ackermann_bringup navigation.launch.py controller:=mppi headless:=true rviz:=false
ros2 service call /clear_obstacles std_srvs/srv/Empty
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'ban_do'}}"

# Đường Bách khoa: như trên, thêm ảnh vệ tinh và Mapviz; đích gửi ở frame map
ros2 launch ackermann_bringup outdoor.launch.py
ros2 launch ackermann_bringup outdoor.launch.py headless:=true rviz:=false mapviz:=false gps_seed:=7
ros2 run ackermann_navigation gps_waypoint_logger --output ~/gps_waypoints.yaml

# Test (world, vật cản, cấu hình GPS/IMU/SLAM/ngoài trời, nhiễu GPS, covariance, ảnh vệ tinh,
# đọc file điểm, khung UART; không cần Gazebo). Hiện 84 ca test; colcon test-result đếm thêm
# một mục CTest cho mỗi file test.
colcon test --packages-select ackermann_bringup ackermann_navigation ackermann_sensors ackermann_hardware
colcon test-result --verbose
make -C firmware/stm32f103c8t6 test

# Sinh lại mesh, hoặc world (chỉ dùng thư viện chuẩn Python)
python3 src/ackermann_description/scripts/generate_meshes.py
python3 src/ackermann_bringup/scripts/generate_world.py
```

Xác minh một thay đổi về Nav2 hoặc SLAM: chạy `navigation.launch.py headless:=true rviz:=false` với `ROS_DOMAIN_ID` và `IGN_PARTITION` riêng (người dùng có thể đang mở mô phỏng), chờ dòng `Managed nodes are active`, gửi 6 đích quanh sân qua action `/navigate_to_pose` (frame `map`): (2,5; −1,5; 0), (2,5; 2,2; π), (−1; 2,2; π), (−3; 0; −π/2), (−2,5; −2,2; 0), (0; 0; 0). Đo sai số SLAM bằng cách so TF `map → base_link` với `/ground_truth/odom`, và so vị trí tâm trục sau thật với đích. Thử cả `controller:=rpp` và `controller:=mppi`, đọc log tìm `Failed to make progress`, `detected collision ahead`, `no valid path found`. Mốc 2026-11-23: 6/6 với cả hai, sai số SLAM trung bình 2–3 cm, trục sau dừng cách đích ≤ 0,15 m.

- Khi chuyển hướng log của `ros2 launch` vào file, log bị đệm nên xuất hiện trễ; đặt `PYTHONUNBUFFERED=1` nếu cần đọc log ngay.
- Ngắt launch chạy `headless` có thể để sót tiến trình `ign gazebo` server. Dừng Gazebo trước bằng `ign service -s /server_control --reqtype ignition.msgs.ServerControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'stop: true'` (chỉ dừng Gazebo cùng `IGN_PARTITION`), rồi mới ngắt launch; khi chạy song song thì dừng theo PID.

## Kiến trúc

```text
Mô phỏng (navigation.launch.py, outdoor.launch.py):
Gazebo ─► bridge.yaml ─► /sim/wheel_odom ─► covariance_setter ─► /wheel_odom ─┐
                      ─► /sim/imu ─► covariance_setter (bỏ hướng) ─► /imu/data ┴─► ekf_odom ─► /odom, TF odom → base_link
                      ─► /scan ─► slam_toolbox ─► /map, TF map → odom
                      ─► /sim/gps/fix ─► gps_noise_injector ─► /gps/fix (chỉ hiển thị; chưa có GPS Waypoint Converter)
                      ─► /joint_states ─► robot_state_publisher ─► TF các khớp
                      ─► /ground_truth/odom (vị trí thật, chỉ để so sai số, không phát TF)

Nav2:
/goal_pose hoặc action navigate_to_pose ─► bt_navigator ─► planner_server (Smac Hybrid-A*; Reeds-Shepp với RPP, Dubins với MPPI)
                                                └─► controller_server (RPP hoặc MPPI) ─► /cmd_vel_nav
/cmd_vel_nav ─► velocity_smoother ─► /cmd_vel ─► bridge ─► AckermannSteering (khớp lái, bánh sau)
Global costmap ở map: trong phòng lớp tĩnh lấy /map của SLAM; ngoài trời cửa sổ trượt 50 m. Local costmap ở odom. Cả hai lấy /scan.
RViz Publish Point ─► /clicked_point ─► obstacle_spawner ─► ros_gz_sim create ─► trụ tĩnh trong Gazebo

Xe thật (real_robot.launch.py, mới có khung): robot_state_publisher + stm32_bridge (/cmd_vel ⇄ UART ⇄ STM32)
```

| File | Vai trò |
| --- | --- |
| `ackermann_description/urdf/ackermann.urdf.xacro` | Hình học, vật lý, khớp, cảm biến và plugin Gazebo |
| `ackermann_bringup/config/bridge.yaml` | Ánh xạ topic ROS 2 ↔ Gazebo (không dùng vị trí thật làm odom) |
| `ackermann_bringup/launch/sim_sensors.launch.py` | `covariance_setter` × 2 và `gps_noise_injector` cho mô phỏng |
| `ackermann_bringup/launch/navigation.launch.py`, `outdoor.launch.py` | Ghép mô phỏng, định vị SLAM, Nav2, RViz/Mapviz |
| `ackermann_bringup/scripts/generate_world.py` | Sinh `nav_world.sdf` (`obstacles()`) và `outdoor_world.sdf` (`outdoor_obstacles()`) |
| `ackermann_bringup/scripts/obstacle_spawner.py` | Thả trụ tĩnh vào Gazebo tại điểm nhận trên `/clicked_point`; `placement_error()`, `obstacle_sdf()` là logic thuần |
| `ackermann_localization/config/ekf_local.yaml`, `slam_toolbox.yaml` | EKF local và tham số SLAM |
| `ackermann_navigation/config/nav2_params.yaml` | Tham số Nav2 chung: costmap, Smac Hybrid-A*, hồi phục, velocity_smoother |
| `ackermann_navigation/config/controller_rpp.yaml`, `controller_mppi.yaml` | `FollowPath` của từng bộ bám đường và kiểu đường (`motion_model_for_search`); launch nạp một trong hai cho cả `controller_server` và `planner_server` |
| `ackermann_navigation/behavior_trees/*.xml` | Cây hành vi Ackermann: không có Spin, chỉ lập lại đường khi đường bị chặn hoặc sau hồi phục |

## Những điều dễ sai

- **Sửa script phải build lại.** `install(PROGRAMS ... RENAME)` sao chép file, kể cả với `--symlink-install`. Tên chạy là tên file bỏ `.py`. Vì bản cài là file sao chép đổi tên, script không import được script khác; logic dùng chung phải chép lại (kèm test so hai bản) hoặc tách thành module. Script mới phải được thêm vào `CMakeLists.txt`.
- **Plugin giữ lệnh vận tốc cuối cùng.** Mọi đường thoát của node phát `/cmd_vel` phải gửi lệnh 0, kể cả khi có lỗi hoặc tín hiệu. `manual_control` và `stm32_bridge` dùng `SignalHandlerOptions.NO` và gửi lặp lệnh dừng trong `finally`; giữ nguyên mẫu này. Chỉ chạy một nguồn phát `/cmd_vel` tại một thời điểm; Nav2 cũng là một nguồn (`velocity_smoother` tự phát 0 khi mất lệnh quá 0,5 s), không chạy cùng `manual_control`.
- **Vị trí thật của Gazebo không phải odometry.** `/odom` là đầu ra `ekf_odom`; vị trí thật chỉ ở `/ground_truth/odom`. Đừng bridge `/tf` hay odometry của plugin vào TF (trùng `odom → base_link`).
- **Frame `map` trùng world Gazebo chỉ vì xe xuất phát tại gốc, hướng +X.** slam_toolbox đặt gốc bản đồ tại chỗ xe khởi động. `obstacle_spawner` và các đích thử dựa vào điều này; đổi vị trí spawn trong `simulation.launch.py` thì map lệch khỏi world. Trên xe thật, `map` không gắn với hướng Bắc.
- **Các giới hạn phải đồng bộ giữa nhiều nơi.** Đổi một giá trị thì kiểm tra các nơi còn lại:
  - Khớp lái ±0,6 rad và `steering_limit` 0,45 rad (URDF); góc bánh trong phải nằm trong giới hạn khớp.
  - Tốc độ ≤ 0,3 m/s trong `manual_control`; plugin cho phép ±0,5 m/s.
  - Nav2: `footprint_padding` local 0,03 nhỏ hơn global 0,05; `minimum_turning_radius` 0,60 m (`nav2_params.yaml`), `min_turning_r` 0,45 m và `vx_max` 0,20 m/s (`controller_mppi.yaml`), `desired_linear_vel` 0,20 m/s (`controller_rpp.yaml`), `footprint` và khoảng lệch `rear_axle` theo `wheelbase` trong URDF.
  - `max_laser_range` của `slam_toolbox.yaml` ≤ tầm Lidar trong URDF (12 m); `base_frame` phải là link có trong URDF.
- **Tham số double khi chạy `ros2 run` phải có dấu chấm:** `-p output_rate:=5.0` (`gps_noise_injector`). Viết `output_rate:=5` sẽ gây `InvalidParameterTypeException`.
- **Mesh:** Gazebo dùng STL chỉ chứa hình học, màu khai báo trong URDF. `base_link` nằm giữa hai trục, ở độ cao trục bánh.
- **Đường dẫn tuyệt đối:** README các package ghi `/home/viet/ackermann_ws`; nếu repo nằm chỗ khác thì đổi theo.
- **Tên file firmware** dùng `ackerman` (một chữ n), khác với `ackermann` bên ROS.
- **Tiến trình sót làm hỏng mô phỏng.** `parameter_bridge`, `covariance_setter`, `gps_noise_injector`, `slam_toolbox` của lần chạy trước còn sống sẽ phát trùng `/clock`, `/imu/data`, TF: TF nhảy lùi, SLAM sai. Kiểm tra `pgrep -af "ign gazebo|parameter_bridge|ackermann_sensors|slam_toolbox"` trống trước khi chạy. Khi tự động hóa, đừng `pkill -f` theo chuỗi có trong chính dòng lệnh đang chạy (tự giết shell). Hai mô phỏng chạy cùng lúc trên một máy cũng hỏng như vậy; muốn chạy thử nền song song với người dùng thì đặt `ROS_DOMAIN_ID` và `IGN_PARTITION` riêng, và dừng theo PID.
- **Mốc ngoài trời nằm ở một chỗ:** `OUTDOOR_*` trong `generate_world.py`, hiện là tim đường nội bộ Đông–Tây ĐH Bách khoa Hà Nội (mặt đường y từ −5 đến +5 m, xe xuất phát hướng Đông). Launch đọc mốc từ `outdoor_world.sdf` cho gốc Mapviz và ảnh vệ tinh; đổi mốc thì sinh lại world, build, chạy lại `generate_satellite_ground.py`.
- **Ngoài trời: đích phải cách xe dưới 24 m** (global costmap cửa sổ trượt 50 m). Bản đồ SLAM chỉ phủ vùng Lidar đã quét, nên global costmap ngoài trời không được chỉ dựa vào lớp tĩnh `/map`.
- **World là file sinh ra.** Không sửa tay `worlds/*.sdf`; sửa `obstacles()`/`outdoor_obstacles()` trong `generate_world.py` rồi chạy lại, nếu không test `worlds` sẽ lỗi. Xe cần khoảng 1,5 m bề ngang để quay đầu, tránh tạo phòng cụt hoặc khe hẹp hơn 1,8 m ở chỗ xe phải quay đầu.
- **Lidar cần plugin Sensors của world** (cả hai world đều có; `empty.sdf` thì không có `/scan`).

### Riêng phần Nav2

Các kết luận dưới đây rút từ chạy thử trên mô phỏng; đọc trước khi đổi tham số.

- **Khớp lái mô phỏng trễ khoảng 1 s.** Plugin AckermannSteering của Fortress điều khiển góc lái bằng khâu tỉ lệ hệ số 1, không chỉnh được: góc lái đạt 58% sau 1 s, 85% sau 2 s. Sai lệch khi vào cua xấp xỉ độ cong × tốc độ² × 1 s². Đây là lý do tốc độ Nav2 giới hạn 0,20 m/s và bán kính lập đường là 0,6 m. Vì trễ này, RPP (khoảng nhìn trước 0,2–0,4 m) hay ra lệnh hết lái: đo 2026-11-23, 42% thời gian lệnh ≥ giới hạn 26°, góc bánh thật trung vị 14°. Servo thật nhanh hơn nhiều, tham số phải chỉnh lại trên xe.
- **Bán kính quay nhỏ nhất thực tế là 0,38 m,** vì plugin tính `wheelbase / sin(steering_limit)`, không phải `tan`. Bộ lập đường phải dùng bán kính lớn hơn rõ rệt (đang là 0,60 m); đặt sát giới hạn thì xe trượt qua đích rồi chạy vòng tròn quanh đích mãi.
- **Nav2 dùng frame `rear_axle`, không phải `base_link`.** Mô hình Ackermann của plugin lái, Smac, RPP và MPPI đều lấy tâm trục sau làm điểm tham chiếu; dùng `base_link` làm xe lệch hướng khoảng 0,2 rad trong cua. `footprint` vì vậy khai báo lệch về phía trước, và điểm đích là vị trí tâm trục sau.
- **Kiểu đường đi theo bộ bám đường: Reeds-Shepp cho RPP, Dubins cho MPPI** (khai báo trong hai file controller). RPP bám được điểm đổi chiều tiến/lùi nên xe lùi ra được khi vật cản xuất hiện sát mũi. MPPI của Humble chững lại ở điểm đổi chiều (thử `enforce_path_inversion: true`: một cú quay đầu mất 137 s), nên giữ Dubins và dựa vào BackUp khi không có đường tiến.
- **RPP cần `max_robot_pose_search_dist: 0.2` khi đường có đoạn lùi.** Với cửa sổ mặc định, ở chỗ quay trở RPP chọn điểm gần nhất trên đoạn sau, bỏ dở thao tác rồi giật tới lui tại chỗ. Hệ quả: không giao lại cho RPP đường mà xe đã đi xa khỏi điểm đầu; cây hành vi luôn lập lại đường sau mỗi lần hồi phục và `FollowPath` không có vòng thử lại riêng.
- **Cây hành vi chỉ lập lại đường khi đường bị chặn hoặc sau khi hồi phục.** Lập lại 1 Hz làm xe đổi đường liên tục khi gần đích và ở đoạn đổi chiều.
- **`progress_checker` dùng bán kính 0,25 m trong 20 s** để bắt xe đứng im hoặc giật tại chỗ; bán kính lớn (thử 1,3 m) coi nhầm thao tác quay trở trong phạm vi hẹp là kẹt.
- **Biên an toàn quanh thân xe:** `footprint_padding` 0,03 ở local costmap (kiểm tra va chạm của bộ bám đường và BackUp) và 0,05 ở global costmap; giá trị global phải lớn hơn local. Xe áp sát tới mức thân nằm trong ô lethal của local costmap thì BackUp bị từ chối và xe kẹt hẳn. Đừng tăng `max_allowed_time_to_collision_up_to_carrot` của RPP quá 0,8 s.
- **MPPI ở tốc độ thấp:** `PathAlignCritic.offset_from_furthest` nhỏ (4) vì tầm dự đoán chỉ vươn 7–8 điểm đường đi; `PathFollowCritic.cost_weight` 10 thay vì 5, nếu không xe chỉ chạy 0,07 m/s.
- **Né vật cản lạ gồm hai tầng.** Lidar đánh dấu vật cản vào cả hai costmap; `IsPathValid` thấy đường bị chặn thì Smac lập lại đường (1 lần mỗi giây); MPPI còn tự tối ưu quỹ đạo cục bộ, RPP chỉ dừng khi sắp va chạm. Vật cản cách xe dưới khoảng 0,8 m thì không có đường tiến: RPP lùi ra theo đường Reeds-Shepp rồi vòng qua; MPPI dừng, BackUp 0,5 m (0,3 m không đủ) rồi lập lại, thất bại nếu bị vây kín. Trụ thả thêm phải cao hơn mặt phẳng quét Lidar (khoảng 0,15 m).

## Quy ước

- Comment, log và thông báo lỗi viết bằng tiếng Việt; tên định danh viết bằng tiếng Anh.
- Đơn vị SI: m, kg, rad, s. Hệ trục: +X tiến, +Y trái, +Z lên.
- Tách logic hình học hoặc thuật toán khỏi `Node` để test được mà không cần Gazebo, như `GpsNoiseModel` hay `FrameParser`. Test nạp script qua `importlib`.
- Khi đổi tham số, launch hoặc topic, cập nhật `README.md` của gói.
- Chỉ báo một tính năng là hoạt động sau khi đã chạy thử trên mô phỏng hoặc phần cứng.
- Mô tả trong mục "Mục tiêu và kiến trúc đích" là thiết kế, chưa phải hiện trạng; khi hoàn thành một phần, chuyển nó sang "Tổng quan hiện trạng" và cập nhật `progress.md`.

# CLAUDE.md

Hướng dẫn làm việc trong repo này. Trạng thái dự án và việc còn lại nằm ở [progress.md](progress.md); hãy cập nhật file đó khi hoàn thành một hạng mục.

## Tổng quan

Xe tự hành khung gầm Ackermann, gồm hai phần:

- `src/ackermann_description/`: gói ROS 2 Humble (`ament_cmake`, node viết bằng Python) mô phỏng xe trên Gazebo Fortress, bám quỹ đạo bằng Pure Pursuit, và tự đi tới điểm đích bằng Nav2 (Smac Hybrid-A*, RPP hoặc MPPI) với Lidar 2D. Phần này đã chạy được trên mô phỏng.
- `firmware/stm32f103c8t6/`: firmware cho xe thật. **Hiện toàn bộ file đều rỗng (0 byte)**, chưa có code, đừng giả định các API trong đó đã tồn tại.

Chưa có cầu nối giữa ROS và firmware. Cảm biến duy nhất trong mô hình là Lidar 2D (thông số ước lượng theo RPLIDAR A1).

## Môi trường

Ubuntu 22.04, ROS 2 Humble, Ignition Gazebo Fortress (lệnh `ign`, plugin `ignition-gazebo-*`, kiểu message `ignition.msgs.*`), Python 3.10. Không dùng cú pháp `gz.*` của Gazebo bản mới trong URDF hoặc bridge.

`arm-none-eabi-gcc` và công cụ nạp (`st-flash`, `openocd`) chưa được cài trên máy này.

## Lần build đầu trên máy mới

`build/`, `install/`, `log/` đang được commit từ máy khác và trỏ tới `/home/viet/ackermann_ws`, nên **`colcon build` sẽ lỗi CMake** và `source install/setup.bash` sẽ không tìm thấy gói. Phải xóa rồi build lại:

```bash
cd /home/long/DATN_KS/DATN
rm -rf build install log    # Các thư mục này đang được track: xem cảnh báo bên dưới
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ackermann_description
source install/setup.bash
```

Cảnh báo: sau mỗi lần build hoặc test, `git status` sẽ có nhiều thay đổi trong `build/`, `install/`, `log/` và `__pycache__/`. Đừng đưa chúng vào commit. Khi stage, chỉ định rõ từng đường dẫn nguồn và không dùng `git add -A`, cho đến khi repo có `.gitignore` và các thư mục này được gỡ khỏi git.

## Lệnh thường dùng

```bash
# Mô phỏng, có giao diện; thêm gz_args:="-r -s empty.sdf" để chạy không giao diện
ros2 launch ackermann_description display.launch.py

# Mô phỏng kèm node bám quỹ đạo
ros2 launch ackermann_description trajectory.launch.py
ros2 launch ackermann_description trajectory.launch.py trajectory:=figure8 radius:=2.5 speed:=0.15

# Điều khiển thủ công: forward | backward | left | right | drive | stop
ros2 run ackermann_description manual_control forward --speed 0.2 --duration 5

# Điều hướng Nav2 trong sân có vật cản; chọn đích bằng 2D Goal Pose trong RViz,
# thả chướng ngại vật bằng Publish Point; xóa bằng: ros2 service call /clear_obstacles std_srvs/srv/Empty
ros2 launch ackermann_description navigation.launch.py
ros2 launch ackermann_description navigation.launch.py controller:=mppi headless:=true rviz:=false

# Test (15 ca: hình học quỹ đạo, world/bản đồ, logic thả chướng ngại vật; không cần Gazebo)
colcon test --packages-select ackermann_description
colcon test-result --verbose

# Sinh lại mesh, hoặc world và bản đồ (chỉ dùng thư viện chuẩn Python)
python3 src/ackermann_description/scripts/generate_meshes.py
python3 src/ackermann_description/scripts/generate_world.py
```

Xác minh một thay đổi về điều khiển: chạy `trajectory.launch.py` không giao diện với `gz_args:="-r -s empty.sdf"` và đọc log `Tiến độ ...%, lệch đường ... m` cho đến `Đã hoàn thành quỹ đạo`. Launch không tự thoát khi node kết thúc, phải tự dừng bằng Ctrl+C.

Xác minh một thay đổi về Nav2: chạy `navigation.launch.py headless:=true rviz:=false`, chờ hai dòng `Managed nodes are active`, gửi đích qua action `/navigate_to_pose` hoặc topic `/goal_pose`, rồi so vị trí cuối trên `/odom` với đích (nhớ đổi sang tâm trục sau). Thử cả `controller:=rpp` và `controller:=mppi`, vài đích liên tiếp, và đọc log tìm `Failed to make progress`, `detected collision ahead`, `no valid path found`. Hai điều cần biết khi chạy tự động:

- Khi chuyển hướng log của `ros2 launch` vào file, log bị đệm nên xuất hiện trễ; đặt `PYTHONUNBUFFERED=1` nếu cần đọc log ngay.
- Ngắt launch chạy `headless` có thể để sót tiến trình `ign gazebo` server. Dừng Gazebo trước bằng `ign service -s /server_control --reqtype ignition.msgs.ServerControl --reptype ignition.msgs.Boolean --timeout 3000 --req 'stop: true'`, rồi mới ngắt launch.

## Kiến trúc

```text
manual_control ─┐
                ├─ /cmd_vel ─► ros_gz_bridge ─► AckermannSteering ─► khớp lái, bánh sau
trajectory_controller                                  │
      ▲                                                └─ /wheel_odom (chỉ để so sánh, không phát TF)
      └── /odom, TF odom→base_link ◄── ros_gz_bridge ◄── OdometryPublisher (vị trí thật của Gazebo)

/joint_states ──► robot_state_publisher ──► TF các khớp
/trajectory/reference, /trajectory/traveled: nav_msgs/Path để xem trong RViz

Nav2 (navigation.launch.py):
/goal_pose hoặc action navigate_to_pose ─► bt_navigator ─► planner_server (Smac Hybrid-A*; Reeds-Shepp với RPP, Dubins với MPPI)
                                                └─► controller_server (RPP hoặc MPPI) ─► /cmd_vel_nav
/cmd_vel_nav ─► velocity_smoother ─► /cmd_vel
Lidar (gpu_lidar, cần plugin Sensors của world) ─► ros_gz_bridge ─► /scan ─► AMCL, hai costmap
map_server (maps/nav_map.yaml) ─► /map;  AMCL ─► TF map→odom
RViz Publish Point ─► /clicked_point ─► obstacle_spawner ─► ros_gz_sim create ─► trụ tĩnh trong Gazebo
                      (Nav2 chỉ thấy trụ qua /scan; dịch vụ /clear_obstacles xóa hết)
```

| File | Vai trò |
| --- | --- |
| `urdf/ackermann.urdf.xacro` | Hình học, vật lý, các khớp và plugin Gazebo |
| `config/bridge.yaml` | Ánh xạ topic ROS 2 và Gazebo |
| `config/trajectory.yaml` | Tham số mặc định của node quỹ đạo |
| `launch/display.launch.py` | Gazebo, bridge, robot_state_publisher, spawn xe |
| `config/nav2_params.yaml` | Tham số Nav2 chung: AMCL, costmap, Smac Hybrid-A*, hồi phục, velocity_smoother |
| `config/controller_rpp.yaml`, `config/controller_mppi.yaml` | Tham số `FollowPath` của từng bộ bám đường và kiểu đường (`motion_model_for_search`) đi kèm; launch nạp một trong hai cho cả `controller_server` và `planner_server` |
| `behavior_trees/*.xml` | Cây hành vi cho xe Ackermann: không có Spin, chỉ lập lại đường khi đường bị chặn hoặc sau mỗi lần hồi phục |
| `launch/navigation.launch.py` | Gọi `display.launch.py` với `worlds/nav_world.sdf`, chạy map_server, AMCL, Nav2 và RViz |
| `scripts/generate_world.py` | Sinh `worlds/nav_world.sdf` và `maps/nav_map.{pgm,yaml}` từ một danh sách vật cản |
| `launch/trajectory.launch.py` | Gọi `display.launch.py` (trừ khi `start_sim:=false`) và chạy node quỹ đạo |
| `scripts/trajectory_controller.py` | `ReferencePath` (hình học, thuần Python) và node `TrajectoryController` (Pure Pursuit, 20 Hz) |
| `scripts/manual_control.py` | Gửi lệnh `/cmd_vel` theo thời gian, 20 Hz |
| `scripts/obstacle_spawner.py` | Thả trụ tĩnh vào Gazebo tại điểm nhận trên `/clicked_point`; `placement_error()` và `obstacle_sdf()` là logic thuần |
| `scripts/generate_meshes.py` | Sinh mesh STL và DAE vào `meshes/` |
| `test/test_trajectory_path.py` | Test `ReferencePath` |
| `test/test_world_map.py` | Test bộ sinh world/bản đồ và kiểm tra file đã sinh khớp với script |
| `test/test_obstacle_spawner.py` | Test SDF của trụ và điều kiện từ chối điểm bấm |

## Những điều dễ sai

- **Sửa script phải build lại.** `install(PROGRAMS ... RENAME)` sao chép file, kể cả với `--symlink-install`. Tên chạy là `manual_control`, `trajectory_controller` và `obstacle_spawner`, không có `.py`. Script mới phải được thêm vào `CMakeLists.txt`.
- **Plugin giữ lệnh vận tốc cuối cùng.** Mọi đường thoát của node phát `/cmd_vel` phải gửi lệnh 0, kể cả khi có lỗi hoặc tín hiệu. Hai node hiện có dùng `SignalHandlerOptions.NO` và gửi lặp lệnh dừng trong `finally`; giữ nguyên mẫu này. Chỉ chạy một nguồn phát `/cmd_vel` tại một thời điểm.
- **`/odom` là vị trí thật của mô phỏng,** không phải ước lượng. Kết quả bám đường trên Gazebo không phản ánh sai số odometry của xe thật. `/wheel_odom` mới là ước lượng từ bánh xe và không được bridge TF của nó để tránh trùng `odom → base_link`.
- **Các giới hạn phải đồng bộ giữa nhiều nơi.** Đổi một giá trị thì kiểm tra các nơi còn lại:
  - Khớp lái ±0,6 rad và `steering_limit` 0,45 rad (URDF); góc bánh trong phải nằm trong giới hạn khớp.
  - `max_curvature` ≤ 2,5 và `max_yaw_rate` ≤ 0,5: mặc định trong `config/trajectory.yaml` và trong code, giới hạn cứng trong `validate_settings()`.
  - Tốc độ ≤ 0,3 m/s trong cả hai script; plugin cho phép ±0,5 m/s.
  - Bán kính tối thiểu trong `validate_settings()`: tròn 0,5 m, số 8 khoảng 2,4 m (hằng số 4,791).
  - Nav2: `footprint_padding` local 0,03 nhỏ hơn global 0,05; `minimum_turning_radius` 0,60 m (`nav2_params.yaml`), `min_turning_r` 0,45 m và `vx_max` 0,20 m/s (`controller_mppi.yaml`), `desired_linear_vel` 0,20 m/s (`controller_rpp.yaml`), `footprint` và khoảng lệch `rear_axle` theo `wheelbase` trong URDF.
- **Hai đồng hồ trong `trajectory_controller`.** Timer điều khiển và `odom_timeout` dùng thời gian thực để phát hiện Gazebo tạm dừng; `max_duration` dùng thời gian mô phỏng. Đừng gộp lại.
- **`project()` chỉ tìm trong cửa sổ quanh tiến độ hiện tại.** Đây là chủ đích để không nhảy nhánh tại giao điểm hình số 8; không đổi thành tìm toàn cục.
- **Tham số double khi chạy `ros2 run` phải có dấu chấm:** `-p radius:=2.0`. Viết `radius:=2` sẽ gây `InvalidParameterTypeException`. Qua `ros2 launch` thì không bị vì launch đã ép kiểu.
- **Mesh:** Gazebo dùng STL chỉ chứa hình học, màu khai báo trong URDF; DAE chỉ là sản phẩm xuất. `base_link` nằm giữa hai trục, ở độ cao trục bánh.
- **Đường dẫn cũ:** `src/ackermann_description/README.md` và `.vscode/settings.json` còn `/home/viet/ackermann_ws`; khi làm theo README phải đổi thành đường dẫn repo này.
- **Tên file firmware** dùng `ackerman` (một chữ n), khác với `ackermann` bên ROS.

### Riêng phần Nav2

Các kết luận dưới đây đều rút từ chạy thử trên mô phỏng; đọc trước khi đổi tham số.

- **Khớp lái mô phỏng trễ khoảng 1 s.** Plugin AckermannSteering của Fortress điều khiển góc lái bằng khâu tỉ lệ hệ số 1, không chỉnh được: góc lái đạt 58% sau 1 s, 85% sau 2 s. Sai lệch khi vào cua xấp xỉ độ cong × tốc độ² × 1 s². Đây là lý do tốc độ Nav2 giới hạn 0,20 m/s và bán kính lập đường là 0,6 m. Muốn chạy nhanh hơn phải sửa cơ cấu lái trong mô hình trước.
- **Bán kính quay nhỏ nhất thực tế là 0,38 m,** vì plugin tính `wheelbase / sin(steering_limit)`, không phải `tan`. Bộ lập đường phải dùng bán kính lớn hơn rõ rệt (đang là 0,60 m); đặt sát giới hạn thì xe trượt qua đích rồi chạy vòng tròn quanh đích mãi.
- **Nav2 dùng frame `rear_axle`, không phải `base_link`.** Mô hình Ackermann của plugin lái, Smac, RPP và MPPI đều lấy tâm trục sau làm điểm tham chiếu; dùng `base_link` (giữa hai trục) làm xe lệch hướng khoảng 0,2 rad trong cua. `footprint` vì vậy khai báo lệch về phía trước, và điểm đích là vị trí tâm trục sau.
- **Kiểu đường đi theo bộ bám đường: Reeds-Shepp cho RPP, Dubins cho MPPI** (khai báo trong hai file controller, không phải `nav2_params.yaml`). RPP bám được điểm đổi chiều tiến/lùi nên xe lùi ra được khi vật cản xuất hiện sát mũi. MPPI của Humble chững lại ở điểm đổi chiều (đã thử với `enforce_path_inversion: true`: một cú quay đầu mất 137 s), nên giữ Dubins và dựa vào BackUp khi không có đường tiến.
- **RPP cần `max_robot_pose_search_dist: 0.2` khi đường có đoạn lùi.** Ở chỗ quay trở, các đoạn tiến và lùi nằm sát nhau; với cửa sổ mặc định RPP chọn điểm gần nhất trên đoạn sau, bỏ dở thao tác, rồi giật tới lui tại chỗ (đổi dấu vận tốc 2 lần mỗi giây). Hệ quả: không được giao lại cho RPP một đường mà xe đã đi xa khỏi điểm đầu; cây hành vi vì vậy luôn lập lại đường sau mỗi lần hồi phục và `FollowPath` không có vòng thử lại riêng.
- **Cây hành vi chỉ lập lại đường khi đường bị chặn hoặc sau khi hồi phục.** Lập lại 1 Hz làm xe đổi đường liên tục khi gần đích và ở các đoạn đổi chiều.
- **`progress_checker` dùng bán kính 0,25 m trong 20 s** để bắt xe đứng im hoặc giật tại chỗ. Bán kính lớn (đã thử 1,3 m) coi nhầm một thao tác quay trở trong phạm vi hẹp là kẹt.
- **AMCL phải bền với vật cản không có trên bản đồ.** Với `likelihood_field` thường, 4 trụ thả quanh xe kéo vị trí ước lượng lệch 16 cm; bộ lập đường tưởng xe đè lên vật cản (`Starting point in lethal space`) và xe đứng yên vĩnh viễn dù chưa chạm gì. Cấu hình hiện tại (`likelihood_field_prob`, beam skip, `laser_likelihood_max_dist` 0,5, `z_rand` 0,4) giữ sai số dưới 9 cm. Khi xe "kẹt vô cớ", đo `map → odom` trước: trong mô phỏng này nó chính là sai số AMCL, vì `/odom` là vị trí thật.
- **Biên an toàn quanh thân xe:** `footprint_padding` 0,03 ở local costmap (kiểm tra va chạm của bộ bám đường và BackUp) và 0,05 ở global costmap (bộ lập đường); giá trị global phải lớn hơn local. Nếu xe áp sát vật cản tới mức thân xe nằm trong ô lethal của local costmap, BackUp bị từ chối và xe kẹt hẳn. Đừng tăng `max_allowed_time_to_collision_up_to_carrot` của RPP quá 0,8 s: ở 1,2 s xe bị chặn liên tục khi luồn qua chỗ chật.
- **MPPI ở tốc độ thấp:** `PathAlignCritic.offset_from_furthest` phải nhỏ (đang là 4) vì tầm dự đoán chỉ vươn 7 đến 8 điểm đường đi; giá trị mặc định làm tiêu chí không bao giờ bật. `PathFollowCritic.cost_weight` 10 thay vì 5, nếu không xe chỉ chạy 0,07 m/s.
- **World và bản đồ là file sinh ra.** Không sửa tay `worlds/nav_world.sdf` hay `maps/`; sửa `obstacles()` trong `generate_world.py` rồi chạy lại, nếu không test `world_map` sẽ lỗi. Xe cần khoảng 1,5 m bề ngang để quay đầu, nên tránh tạo phòng cụt hoặc khe hẹp hơn 1,8 m ở chỗ xe phải quay đầu.
- **Lidar cần plugin Sensors của world.** Với `empty.sdf` sẽ không có `/scan`. AMCL lấy vị trí ban đầu (0, 0, 0) từ `nav2_params.yaml`; nếu đổi vị trí spawn trong `display.launch.py` thì phải đổi theo.
- **Né vật cản không có trên bản đồ gồm hai tầng.** Lidar đánh dấu vật cản vào cả hai costmap; `IsPathValid` trong cây hành vi thấy đường bị chặn thì Smac lập lại đường (1 lần mỗi giây); MPPI còn tự tối ưu quỹ đạo cục bộ, RPP thì chỉ dừng khi sắp va chạm. Vật cản xuất hiện cách xe dưới khoảng 0,8 m thì không có đường tiến để tránh: với RPP, đường Reeds-Shepp mới cho xe lùi ra rồi vòng qua; với MPPI, xe dừng, BackUp 0,5 m rồi lập lại (quãng 0,3 m là không đủ, đã thử), và thất bại nếu bị vây kín.
- **`obstacle_spawner` coi frame `map` trùng với hệ tọa độ world của Gazebo.** Điều này đúng vì bản đồ sinh từ chính world đó; nếu dùng bản đồ khác (ví dụ từ SLAM) thì phải đổi tọa độ trước khi thả. Trụ phải cao hơn mặt phẳng quét của Lidar (khoảng 0,15 m).
- **Nav2 cũng là một nguồn phát `/cmd_vel`.** Không chạy cùng `manual_control` hay `trajectory_controller`. `velocity_smoother` tự phát vận tốc 0 khi mất lệnh quá 0,5 s.

## Quy ước

- Comment, log và thông báo lỗi viết bằng tiếng Việt; tên định danh viết bằng tiếng Anh.
- Đơn vị SI: m, kg, rad, s. Hệ trục: +X tiến, +Y trái, +Z lên.
- Tách logic hình học hoặc thuật toán khỏi `Node` để test được mà không cần Gazebo, như `ReferencePath`. Test nạp script qua `importlib`.
- Khi đổi tham số, launch hoặc topic, cập nhật `README.md` của gói.
- Chỉ báo một tính năng là hoạt động sau khi đã chạy thử trên mô phỏng hoặc phần cứng.

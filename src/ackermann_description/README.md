# Hiwonder-style three-deck Ackermann chassis

Reference: https://www.hiwonder.com/products/ackermann-steering-chassis?variant=40382428348503

The visual model extends the supplied double-deck photograph to a custom three-deck chassis with black perforated aluminium decks, brass spacers, rear motor cases, front servo, black tyres and red wheel rims. It is a procedural approximation, not manufacturer CAD. Machining openings are approximated on a 1 mm grid. Regenerate the committed STL and Collada files with `python3 src/ackermann_description/scripts/generate_meshes.py` (Python standard library only).

| Parameter | Value | Basis |
| --- | --- | --- |
| Deck length | 270 mm | Manufacturer |
| Deck width | 120.5 mm | Manufacturer raised-layer plate |
| Plate thickness | 2 mm | Manufacturer |
| Overall width at straight steering | 197 mm | Manufacturer; track plus tyre width |
| Servo case | 40 × 20 × 40.5 mm | Manufacturer LD-1501MG |
| Wheel diameter / width | 75 / 30 mm | Estimate; retained from original model |
| Wheelbase / centre track | 165 / 167 mm | Estimate; retained from original model |
| Lower / middle / top deck centre above axle | -9 / 43 / 95 mm | Lower two estimated from photograph; third deck is custom |
| Deck centre spacing | 52 mm | Custom design; 50 mm clear gap |
| Total mass | 1.15 kg | Simulation estimate, excludes battery/payload |
| Front wheel steering limit | ±0.6 rad | Simulation assumption, not servo travel |

The third deck adds six brass spacers, six silver fasteners and an estimated 150 g. Its fixed link includes mass and inertia, which Gazebo combines with the base during SDF conversion. The third deck is a custom extension, not a manufacturer configuration.

Manufacturer lists packaged weights, which are not used as measured vehicle mass. Hole locations, tread pattern, motor envelopes, mass distribution and inertia are illustrative. Collision geometry uses simple plates, a servo box and cylindrical wheels for stable contact; decorative hardware is visual only.

Coordinates: +X forward, +Y left, +Z up. `base_link` is centred between axles at wheel-axis height. At rest its Z is approximately 0.0375 m. Existing wheel and steering joint names are preserved. Front wheel rotation is passive and rear wheel joints represent independent motors. The Gazebo AckermannSteering plugin drives the rear wheels with different speeds when turning and controls the two front steering joints. The launch starts ros_gz_bridge for commands, odometry, joint states, TF and simulation time. `/odom` and the `odom → base_link` transform use Gazebo OdometryPublisher ground-truth pose for simulation trajectory feedback. The Ackermann plugin wheel-based estimate is kept separately on `/wheel_odom`; its TF is not bridged to avoid duplicate transforms.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ackermann_description
source install/setup.bash
ros2 launch ackermann_description display.launch.py
```

The launch supplies mesh resource paths for Ignition Fortress and Gazebo Sim and spawns the axle at 40 mm above the ground.

Colours are declared inline in each URDF visual. Gazebo uses geometry-only binary STL meshes with explicit face normals (`chassis_plate.stl` and `wheel_{rubber,red,black,silver}.stl`); colour comes from URDF/SDF, with no embedded Collada materials to override it. DAE files remain as export artifacts. After rebuilding, stop the previous launch and Gazebo window and relaunch to reload the spawned model. The launch prints the absolute Xacro path with `Loading coloured STL model` to identify the package actually loaded.

## Điều khiển thủ công bằng lệnh

Mở terminal thứ nhất để build và chạy Gazebo; đóng mô phỏng cũ trước khi chạy bản mới:

```bash
cd /home/viet/ackermann_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ackermann_description
source install/setup.bash
ros2 launch ackermann_description display.launch.py
```

Mở terminal thứ hai và nạp môi trường:

```bash
cd /home/viet/ackermann_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Chọn từng lệnh sau, chạy lần lượt:

```bash
# Tiến trong 3 giây rồi tự dừng.
ros2 run ackermann_description manual_control forward

# Lùi trong 3 giây rồi tự dừng.
ros2 run ackermann_description manual_control backward

# Tiến rẽ trái hoặc tiến rẽ phải trong 3 giây.
ros2 run ackermann_description manual_control left
ros2 run ackermann_description manual_control right

# Tiến với tốc độ 0,2 m/s trong 5 giây.
ros2 run ackermann_description manual_control forward --speed 0.2 --duration 5

# Chạy tùy chỉnh: vận tốc dọc -0,15 m/s và tốc độ đổi hướng -0,2 rad/s.
ros2 run ackermann_description manual_control drive --speed -0.15 --yaw-rate -0.2 --duration 3

# Gửi lệnh dừng xe.
ros2 run ackermann_description manual_control stop
```

`--speed` tính bằng m/s, tối đa 0,3 m/s; `drive` cho phép tốc độ âm để lùi. `--yaw-rate` là tốc độ đổi hướng thân xe, tối đa ±0,5 rad/s, không phải góc servo. Lệnh `left` và `right` lấy độ lớn của `--yaw-rate` rồi tự đặt dấu; mặc định là 0,3 rad/s. `--duration` là thời gian thực tính bằng giây, mặc định 3 giây. Khi đạt giới hạn rẽ, plugin giới hạn độ cong quỹ đạo theo hình học xe.

Nhấn **Ctrl+C trong terminal đang chạy manual_control** để ngắt lệnh và gửi vận tốc bằng 0. Chương trình phát lệnh ở 20 Hz, chờ bridge kết nối tối đa 5 giây và gửi lặp lệnh dừng trước khi thoát. Chỉ chạy một nguồn phát lệnh điều khiển tại một thời điểm.

Có thể gửi trực tiếp `/cmd_vel` khi cần thử bằng ROS CLI:

```bash
# Tiến với tốc độ 0,15 m/s; lệnh này chạy liên tục tới khi nhấn Ctrl+C.
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'

# Sau khi ngắt ROS CLI, gửi lệnh dừng vì plugin giữ lệnh vận tốc cuối cùng.
ros2 run ackermann_description manual_control stop
```

Xem dữ liệu phản hồi trong một terminal khác:

```bash
ros2 topic echo /odom
ros2 topic echo /joint_states
```

Để chạy Gazebo không có giao diện:

```bash
ros2 launch ackermann_description display.launch.py gz_args:="-r -s empty.sdf"
```

Các tệp điều khiển:

- `urdf/ackermann.urdf.xacro`: plugin lái và plugin xuất trạng thái khớp.
- `config/bridge.yaml`: cấu hình cầu nối ROS 2 với Gazebo.
- `scripts/manual_control.py`: node Python gửi lệnh thủ công.
- `launch/display.launch.py`: khởi chạy Gazebo, mô hình xe và bridge.

Đã kiểm tra trên Gazebo Fortress không có giao diện: tiến, lùi, rẽ trái/phải, góc lái Ackermann, phản hồi `/odom`, `/joint_states`, `/tf`, `/clock` và dừng bằng Ctrl+C. Vị trí thật của mô hình trong Gazebo được dùng để xác nhận xe có chuyển động.

Tham khảo: [AckermannSteering Fortress](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1AckermannSteering.html), [ros_gz_bridge Humble](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/README.md).

## Node Python bám quỹ đạo

`scripts/trajectory_controller.py` dùng phản hồi `/odom` và thuật toán [Pure Pursuit](https://publications.ri.cmu.edu/implementation-of-the-pure-pursuit-path-tracking-algorithm) để tính `/cmd_vel` ở 20 Hz. Quỹ đạo được đặt tại vị trí và hướng xe khi node nhận odometry đầu tiên; mặc định xe chạy một vòng tròn bán kính 1 m với vận tốc 0,12 m/s rồi dừng. Quỹ đạo số 8 dùng đường cong trơn, bắt đầu theo hướng đầu xe. Node chỉ phát lệnh chạy tiến.

Nếu Gazebo đang chạy qua `display.launch.py`, chạy node trong terminal khác:

```bash
source /opt/ros/humble/setup.bash
source /home/viet/ackermann_ws/install/setup.bash
ros2 run ackermann_description trajectory_controller --ros-args -p use_sim_time:=true
```

Chọn hình số 8 với kích thước 2,5 m:

```bash
ros2 run ackermann_description trajectory_controller --ros-args \
  -p use_sim_time:=true -p trajectory:=figure8 -p radius:=2.5
```

`radius` là bán kính đối với đường tròn; với hình số 8, đây là hệ số kích thước, không phải bán kính cong cố định. Hình số 8 cần `radius` ít nhất khoảng 2,4 m để phù hợp giới hạn lái. Đường tròn cần bán kính ít nhất 0,5 m. Nếu đặt quá nhỏ, node báo lỗi cấu hình.

Khi chưa chạy Gazebo, launch sau mở cả mô phỏng và node:

```bash
ros2 launch ackermann_description trajectory.launch.py

# Chạy hình số 8 với vận tốc 0,15 m/s.
ros2 launch ackermann_description trajectory.launch.py trajectory:=figure8 radius:=2.5 speed:=0.15

# Chỉ mở node khi mô phỏng đã chạy; vòng tròn bán kính 1,5 m, hai vòng.
ros2 launch ackermann_description trajectory.launch.py start_sim:=false radius:=1.5 laps:=2
```

Cấu hình nằm trong `config/trajectory.yaml`; build lại sau khi sửa file nguồn trong `src`. Có thể dùng `--ros-args -p tên_tham_số:=giá_trị` khi chạy node trực tiếp. Các tham số được đọc lúc khởi động:

| Tham số | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `trajectory` | `circle` | Chọn `circle` hoặc `figure8` |
| `radius` | `1.0` | Bán kính hoặc hệ số kích thước, m |
| `laps` | `1` | Số vòng, từ 1 đến 20 |
| `speed` | `0.12` | Vận tốc mong muốn, tối đa 0,3 m/s |
| `lookahead` | `0.4` | Khoảng nhìn trước theo chiều dài đường đi, m |
| `goal_tolerance` | `0.06` | Dung sai kết thúc, m |
| `max_duration` | `240.0` | Thời gian chạy tối đa theo thời gian mô phỏng, giây |
| `odom_timeout` | `1.0` | Ngưỡng mất odometry theo thời gian thực, giây |
| `max_tracking_error` | `0.75` | Ngưỡng lệch đường khiến node dừng, m |

Node giới hạn độ cong và tốc độ đổi hướng theo mô hình; tốc độ thực có thể giảm gần điểm cuối hoặc ở khúc cua. Cần tăng `max_duration` nếu chạy nhiều vòng vượt quá 240 giây. Khi đi qua giao điểm của hình số 8, node chỉ tìm điểm gần nhất trong một đoạn phía trước để giữ đúng nhánh và đếm đủ vòng.

Hai topic `nav_msgs/msg/Path` để xem trong RViz khi node đang chạy:

- `/trajectory/reference`: đường tham chiếu, chọn Fixed Frame theo `header.frame_id` (mặc định `odom`).
- `/trajectory/traveled`: vệt xe đã đi theo odometry.

Nhấn **Ctrl+C** để gửi lệnh dừng. Node cũng dừng khi hoàn thành, mất odometry, thay đổi hệ tọa độ, đặt lại thời gian hoặc vượt ngưỡng sai lệch/thời gian. Nếu tạm dừng Gazebo quá `odom_timeout`, node dừng và thoát; nhấn Play rồi chạy lại node để tiếp tục bằng quỹ đạo mới tại vị trí hiện tại. Dừng `manual_control` và các nguồn phát `/cmd_vel` khác trước khi chạy node quỹ đạo.

Đây là bộ bám đường hình học, chưa có tránh vật cản. `/odom` hiện lấy vị trí thật từ plugin OdometryPublisher của Gazebo để bám đường trong mô phỏng; `/wheel_odom` giữ riêng ước lượng từ bánh xe để so sánh. Chỉ nguồn `/odom` phát TF `odom → base_link`. Khi đưa lên xe thật cần thay nguồn phản hồi này bằng odometry/định vị từ cảm biến.

Kiểm tra hình học bằng:

```bash
colcon test --packages-select ackermann_description
colcon test-result --verbose
```

Kết quả thử nghiệm Gazebo không giao diện ở vận tốc 0,2 m/s: hoàn thành một vòng tròn bán kính 1 m và một vòng số 8 với hệ số kích thước 2,5 m; sai lệch vị trí thật lớn nhất lần lượt khoảng 3,4 cm và 4,3 cm. Cả hai đều dừng ở cuối quỹ đạo. Kiểm tra mất odometry và Ctrl+C đều xác nhận lệnh vận tốc cuối bằng 0; bốn kiểm tra hình học trong `colcon test` đều đạt. Các kết quả này áp dụng cho cấu hình mô hình và bộ điều khiển hiện tại.

Nguồn phản hồi mô phỏng: [Gazebo Fortress OdometryPublisher](https://gazebosim.org/api/gazebo/6/classignition_1_1gazebo_1_1systems_1_1OdometryPublisher.html).

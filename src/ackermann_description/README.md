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

## Lidar

Xe có một Lidar 2D quét 360° đặt giữa tầng thứ ba (`lidar_link`, cao hơn `base_link` 0,116 m): 360 tia, 10 Hz, tầm đo 0,15 đến 12 m, nhiễu Gauss 1 cm. Kích thước, khối lượng 170 g và tầm đo là ước lượng theo loại RPLIDAR A1, chưa đối chiếu với cảm biến thật. Dữ liệu ra topic `/scan` (`sensor_msgs/msg/LaserScan`, frame `lidar_link`).

Lidar chỉ phát dữ liệu khi world nạp plugin `Sensors`. `worlds/nav_world.sdf` có plugin này; `empty.sdf` thì không, nên chạy `display.launch.py` mặc định sẽ không có `/scan`.

## Điều hướng tự động với Nav2

`navigation.launch.py` mở Gazebo với sân có vật cản, định vị bằng AMCL trên bản đồ có sẵn, và chạy Nav2 để xe tự đi tới điểm đích:

- Lập đường đi: Smac Hybrid-A*, bán kính quay 0,6 m. Kiểu đường đi theo bộ bám đường: Reeds-Shepp (có đoạn lùi) cho RPP, Dubins (chỉ tiến) cho MPPI.
- Bám đường: Regulated Pure Pursuit (`controller:=rpp`, mặc định) hoặc MPPI với mô hình Ackermann (`controller:=mppi`).

| | `controller:=rpp` (mặc định) | `controller:=mppi` |
| --- | --- | --- |
| Kiểu đường | Reeds-Shepp: xe lùi và quay trở được | Dubins: chỉ tiến, quay đầu cần khoảng 1,5 m bề ngang |
| Hợp với | Chỗ chật, nhiều vật cản, đích sát tường | Sân thoáng, vật cản xuất hiện từ xa (trên khoảng 1 m) |
| Né vật cản | Lập lại đường toàn cục, dừng nếu sắp va chạm | Lập lại đường toàn cục và tự tối ưu quỹ đạo cục bộ |
| Điểm yếu | Đường đi hay có đoạn lùi kể cả ở chỗ thoáng | Vật cản sát mũi xe hoặc bị vây kín thì không có đường tiến, báo thất bại |

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# Gazebo, RViz và Nav2 với bộ điều khiển RPP.
ros2 launch ackermann_description navigation.launch.py

# Dùng MPPI.
ros2 launch ackermann_description navigation.launch.py controller:=mppi

# Không mở cửa sổ Gazebo, chỉ mở RViz.
ros2 launch ackermann_description navigation.launch.py headless:=true
```

| Tham số launch | Mặc định | Ý nghĩa |
| --- | --- | --- |
| `controller` | `rpp` | `rpp` hoặc `mppi` |
| `headless` | `false` | `true` để chạy Gazebo không giao diện |
| `rviz` | `true` | `false` để không mở RViz |

Chọn điểm đích: chờ RViz hiện bản đồ và mô hình xe, bấm **2D Goal Pose** (hoặc **Nav2 Goal**) trên thanh công cụ, rồi bấm và kéo trên bản đồ để đặt vị trí và hướng đích. Không cần đặt vị trí ban đầu vì AMCL được cấu hình sẵn vị trí xuất phát tại gốc bản đồ. Có thể gửi đích bằng lệnh:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.4, y: 2.1}, orientation: {z: 1.0, w: 0.0}}}"
```

Sân rộng 8 m × 6 m, tâm tại gốc tọa độ, xe xuất phát tại (0, 0) hướng +X. Điểm đích là vị trí của **tâm trục sau** (frame `rear_axle`), không phải giữa xe. Dung sai tới đích là 0,12 m và 0,30 rad.

Khi dừng, nhấn Ctrl+C trong terminal chạy launch. Nếu chạy `headless:=true` mà tiến trình `ign gazebo` còn sót lại, dừng nó bằng `pkill -f "ign gazebo"` trước khi chạy lại. Không chạy `manual_control` hoặc `trajectory_controller` cùng lúc với Nav2 vì tất cả đều phát `/cmd_vel`.

### Thả chướng ngại vật khi xe đang chạy

Node `obstacle_spawner` (chạy sẵn trong `navigation.launch.py`) thả một trụ tĩnh màu đỏ, bán kính 0,15 m, cao 0,4 m, vào Gazebo tại mỗi điểm bấm trong RViz:

1. Chọn điểm đích bằng **2D Goal Pose** và chờ xe bắt đầu chạy.
2. Bấm **Publish Point** trên thanh công cụ, rồi bấm lên bản đồ ở vị trí muốn đặt trụ, ví dụ ngay trên đường đi màu xanh phía trước xe. Mỗi lần thả phải bấm lại **Publish Point**.

Trụ không có trên bản đồ, nên xe chỉ biết tới nó khi Lidar quét thấy. Khi đó hai costmap đánh dấu vật cản; nếu đường đi bị chặn thì Smac Hybrid-A* lập lại đường từ vị trí hiện tại (kiểm tra 1 lần mỗi giây). Với RPP, đường mới có thể gồm đoạn lùi, nên khi trụ xuất hiện ngay trước mũi, xe dừng, lùi ra rồi vòng qua. Với MPPI, bộ bám đường còn tự tối ưu quỹ đạo quanh vật cản trong tầm dự đoán 3,5 s, nhưng đường toàn cục chỉ có đoạn tiến.

Xóa mọi trụ đã thả:

```bash
ros2 service call /clear_obstacles std_srvs/srv/Empty
```

Node từ chối điểm bấm cách tâm xe dưới 0,6 m hoặc chồng lên trụ đã thả, và ghi lý do ra log. Tham số của node: `radius` (0,15), `height` (0,40), `min_robot_distance` (0,60), `world` (`nav_world`). Có thể thả trụ bằng lệnh thay vì bấm:

```bash
ros2 topic pub --once /clicked_point geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 1.7, y: 0.0}}"
```

Khoảng cách thả trụ quyết định cách xe phản ứng (khớp lái mô phỏng đáp ứng chậm nên xe cần chỗ để lách):

| Trụ cách xe (theo đường đi) | Phản ứng đã quan sát |
| --- | --- |
| Từ khoảng 1,2 m | Lách qua không dừng, đi cách mép trụ từ 0,2 m |
| Khoảng 0,9 m | RPP lách qua không dừng. MPPI có lần phải lùi hồi phục rồi mới vòng qua |
| Khoảng 0,7 m, giữa nhiều trụ khác | RPP dừng trước trụ, lùi ra và vòng lối khác (cả chặng mất 83 s). MPPI báo thất bại vì không có đường tiến; gửi đích khác thì chạy tiếp được |

### Giới hạn cần biết

- **Với RPP, đường đi hay có đoạn lùi.** Đường Reeds-Shepp ngắn nhất thường chứa 1 đến 4 lần đổi chiều kể cả ở chỗ thoáng, nên xe lùi nhiều hơn một người lái sẽ làm. Đổi lại xe quay trở được ở chỗ hẹp. Riêng thao tác quay đầu tại chỗ có lần chững khoảng 20 s ở điểm đổi chiều rồi mới được cơ chế hồi phục đẩy đi tiếp.
- **Với MPPI, xe chỉ chạy tiến theo đường Dubins.** Quay đầu 180° cần khoảng 1,5 m bề ngang. Nếu đặt đích sát tường, xe đang quay mặt vào tường ở gần, hoặc vật cản vây quanh, có thể không có đường đi; khi đó Nav2 cho xe lùi thẳng 0,5 m rồi lập lại đường, tối đa 8 lần hồi phục trước khi báo thất bại. MPPI của Nav2 Humble chững lại ở các điểm đổi chiều của đường Reeds-Shepp nên không dùng được kiểu đường đó.
- **Vật cản dày đặc vẫn có thể làm xe thất bại.** Xe rộng 0,21 m nhưng cần khe rộng hơn nhiều để luồn qua an toàn; khi các trụ cách nhau dưới khoảng 0,6 m quanh xe, Nav2 có thể báo thất bại sau các lần hồi phục. Xóa bớt trụ bằng `/clear_obstacles` rồi gửi lại đích.
- **Hướng tới đích chỉ gần đúng.** Xe dừng ngay khi vào dung sai, thường còn lệch 0,15 đến 0,3 rad so với hướng đã chọn.
- **Khớp lái trong mô phỏng đáp ứng chậm**, hằng số thời gian đo được khoảng 1 s (góc lái đạt 58% sau 1 s, 85% sau 2 s), do plugin AckermannSteering của Fortress dùng khâu tỉ lệ có hệ số cố định. Vì vậy tốc độ giới hạn ở 0,20 m/s và bán kính lập đường là 0,6 m dù xe quay được tới 0,38 m. Servo thật nhanh hơn nhiều, nên các tham số này cần chỉnh lại khi chạy xe thật.
- **`/odom` vẫn là vị trí thật của Gazebo.** AMCL chỉ hiệu chỉnh `map → odom`; kết quả không phản ánh sai số odometry của xe thật.
- **Trụ thả thêm làm AMCL lệch.** Chúng không có trên bản đồ nên AMCL có xu hướng khớp nhầm chúng với tường. Với cấu hình ban đầu, 4 trụ quanh xe làm vị trí ước lượng lệch 16 cm, Nav2 tưởng xe đè lên vật cản bên cạnh và từ chối lập đường (`Starting point in lethal space`), xe đứng yên dù chưa chạm gì. AMCL hiện dùng mô hình `likelihood_field_prob` có beam skip; sai số đo được trong cùng tình huống còn dưới 9 cm.

### Bản đồ và world

`scripts/generate_world.py` sinh cả world Gazebo và bản đồ Nav2 từ cùng một danh sách vật cản, nên hai thứ luôn khớp nhau. Để đổi bố cục, sửa hàm `obstacles()` rồi chạy lại và build:

```bash
python3 src/ackermann_description/scripts/generate_world.py
colcon build --symlink-install --packages-select ackermann_description
```

Giữ vùng bán kính 0,5 m quanh gốc tọa độ trống (nơi đặt xe), các lối đi rộng từ 1,0 m và chỗ cần quay đầu rộng từ 1,8 m. `colcon test` kiểm tra các file đã sinh có khớp với script hay không.

Các tệp điều hướng:

- `launch/navigation.launch.py`: mô phỏng, AMCL, Nav2 và RViz.
- `config/nav2_params.yaml`: tham số chung (AMCL, costmap, Smac Hybrid-A*, hồi phục, làm mượt vận tốc).
- `config/controller_rpp.yaml`, `config/controller_mppi.yaml`: tham số của từng bộ bám đường.
- `behavior_trees/`: cây hành vi không có thao tác xoay tại chỗ.
- `scripts/obstacle_spawner.py`: thả và xóa chướng ngại vật theo điểm bấm trong RViz.
- `worlds/nav_world.sdf`, `maps/nav_map.pgm`, `maps/nav_map.yaml`: sản phẩm của `generate_world.py`.

### Kết quả thử nghiệm

Chạy Gazebo không giao diện ngày 2026-11-11 với cấu hình hiện tại, đo bằng vị trí thật của Gazebo tại tâm trục sau. Mỗi bài chạy một lượt, nên đây chưa phải số liệu thống kê.

| Bài thử | RPP (Reeds-Shepp) | MPPI (Dubins) |
| --- | --- | --- |
| 6 đích liên tiếp quanh sân | 6/6; 29 đến 45 s mỗi chặng; một chặng cần 2 lần hồi phục | 6/6; 25 đến 39 s mỗi chặng; không hồi phục |
| Lệch vị trí và hướng khi dừng | 0,04 đến 0,13 m; tới 0,30 rad | 0,09 đến 0,12 m; tới 0,24 rad |
| 4 đích cần lùi (đích ngay sau lưng, quay đầu tại chỗ, đích sát tường, rời khỏi tường) | 4/4; 7 đến 65 s; quay đầu tại chỗ cần 2 lần hồi phục | Chưa thử |
| Thả trụ trên đường đi, cách 1,2 đến 1,3 m | Đạt, không hồi phục, cách tâm trụ 0,77 m | Đạt, không hồi phục, cách tâm trụ 0,48 m |
| Thả trụ trên đường đi, cách 0,9 m | Đạt, không hồi phục, cách tâm trụ 0,53 m | Đạt sau 3 lần hồi phục, 74 s |
| Túi cụt: tường cách đuôi xe 0,4 m, trụ cách 0,7 m phía trước | Đạt, 19 s, không hồi phục | Chưa thử |
| Nhiều trụ: 3 trụ có sẵn quanh xe, thả thêm trụ thứ tư cách 0,7 m trên đường đi | Đạt, 83 s, 2 lần hồi phục, không chạm (Lidar cách mặt trụ ít nhất 0,21 m) | Thất bại: không có đường tiến; đích kế tiếp đạt |
| Sai số AMCL lớn nhất trong các bài trên | 0,08 m | 0,07 m |

Khoảng cách tới tâm trụ tính từ giữa xe; trừ bán kính trụ 0,15 m và nửa bề ngang xe 0,105 m để ra khe hở bên. Lệch vị trí đo theo vị trí thật có thể vượt dung sai 0,12 m vài cm vì Nav2 kiểm tra đích theo ước lượng của AMCL.

Trước các thay đổi ngày 2026-11-11 (AMCL bền với vật cản lạ, đường Reeds-Shepp cho RPP, biên an toàn quanh thân xe), bài nhiều trụ làm xe đứng yên vĩnh viễn với MPPI và qua được với RPP sau 9 lần hồi phục.

Chưa kiểm chứng: thao tác bấm trong cửa sổ RViz (các bài thử gửi đích và điểm thả trụ lên đúng các topic mà RViz dùng), chế độ có giao diện Gazebo, vật cản di chuyển, và `NavigateThroughPoses`.

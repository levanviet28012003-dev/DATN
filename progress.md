# Tiến độ dự án

Cập nhật: 2026-11-11, trên nền commit `843ae8e` (phần Lidar và Nav2 chưa commit). Hướng dẫn kỹ thuật nằm ở [CLAUDE.md](CLAUDE.md).

## Cập nhật GPS mô phỏng — 2026-11-14

- Workspace đã tách thành 6 package ROS; mô hình hiện có hai tầng.
- Thêm `gps_link` và cảm biến NavSat trên tầng trên, phát 5 Hz theo thời gian mô phỏng.
- Bridge `ignition.msgs.NavSat` → `/gps/fix` (`sensor_msgs/msg/NavSatFix`), frame `gps_link`.
- `gps_world.sdf` là world trống mặc định cho simulation/trajectory; `nav_world.sdf` cũng có GPS.
  Hai world dùng mốc ENU/WGS84 minh họa (vĩ độ 10°, kinh độ 106°, độ cao 0 m).
- **Đã chạy kiểm chứng Gazebo Fortress không giao diện trên cả hai world:** mỗi bài nhận 28 mẫu GPS,
  timestamp cách nhau 0,2 s, status FIX và frame đúng. Cho xe tiến khoảng 0,70 m:
  GPS đổi về hướng Đông khoảng 0,70 m, khớp odometry. `ros2 topic echo /gps/fix --once` trả dữ liệu.
  World có vật cản vẫn xuất Lidar 360 tia.
- GPS chưa thêm nhiễu; bridge báo covariance UNKNOWN. Đây chưa phải mô phỏng sai số điện thoại.
- Chưa thêm IMU, EKF, nhận GPS điện thoại hoặc điều hướng theo điểm GPS; AMCL hiện tại giữ nguyên.
- Build hai package thay đổi đạt; 16 ca kiểm tra bringup (world/map, vật cản, cấu hình GPS) đã qua.
- Thêm node `ackermann_sensors/gps_visualizer`, cấu hình `gps.rviz` và launch
  `ackermann_bringup gps_view.launch.py`: hiện Lat/Lon/Alt từ `/gps/fix` cạnh xe qua `/gps/marker`.
  Chữ gắn với TF cảm biến, chưa phải định vị GPS/EKF hoặc bản đồ vệ tinh.
- Build phần hiển thị đạt; 4 ca kiểm tra tọa độ hợp lệ, chưa có fix, dữ liệu sai và thiếu độ cao đều qua.
  Đã mở RViz, xác nhận RViz đăng ký topic mô hình và marker; chữ tọa độ cập nhật khi xe tiến.
  Đã xem ảnh RViz: mô hình và bốn dòng GPS/Lat/Lon/Alt hiển thị, Global Status OK.
  Launch riêng GPS loại đường dẫn GTK từ Snap khỏi tiến trình RViz để tránh lỗi GLIBC_PRIVATE.
- Bổ sung `rviz/navigation.rviz` và ghép GPS vào `navigation.launch.py`: cùng cửa sổ
  hiện bản đồ phòng, Lidar, mô hình xe, costmap và tọa độ GPS xanh đậm.
  `gps_view` vẫn là giao diện thử xe/GPS; world trống không có phòng/vật cản.
  Build đạt, 4 ca xử lý marker đạt. Đã chạy bài kiểm tra hiển thị với Gazebo/RViz:
  nhận bản đồ 168 × 128 có ô vật cản, Lidar 360 tia có phản xạ, `/gps/marker`, TF `map → odom`;
  RViz đăng ký đủ bốn topic map/scan/GPS/mô hình và ảnh có tường, vật cản, chữ GPS.
  Bài này không kiểm thử đi tới đích Nav2; lần kiểm tra lifecycle planner trước đó
  hết thời gian chờ 10 s khi planner còn cấu hình, không kết luận Nav2 đã active.

Những số liệu và mô tả kiến trúc trong các mục ngày 2026-11-08/11 bên dưới là bản ghi lịch sử,
trước khi tách package và bổ sung GPS. Hướng dẫn chạy mới nằm ở `src/ackermann_bringup/README.md`.

## Tóm tắt

Phần mô phỏng trên ROS 2 và Gazebo đã chạy được: xe bám được quỹ đạo tròn và số 8, và tự đi tới điểm đích trong sân có vật cản bằng Nav2 (Smac Hybrid-A*, RPP hoặc MPPI) với Lidar 2D và định vị AMCL. Phần xe thật chưa bắt đầu: 14 file firmware đều rỗng và chưa có kết nối giữa ROS với vi điều khiển. Repo đang commit cả sản phẩm build từ máy khác, khiến lệnh build mặc định bị lỗi; nên xử lý việc này trước tiên.

| Hạng mục | Trạng thái |
| --- | --- |
| Mô hình URDF và mesh | Xong (kích thước một phần là ước lượng) |
| Mô phỏng Gazebo và bridge | Xong |
| Điều khiển thủ công | Xong theo README, chưa chạy lại đợt này |
| Bám quỹ đạo Pure Pursuit | Xong trên mô phỏng |
| Dọn repo, sửa đường dẫn | Chưa làm |
| Lidar 2D trong mô hình | Xong (thông số ước lượng theo RPLIDAR A1) |
| GPS NavSat trong Gazebo, topic `/gps/fix` | Đã kiểm chứng ngày 2026-11-14, xem mục cập nhật GPS |
| World có vật cản và bản đồ | Xong (sinh bằng `generate_world.py`) |
| Điều hướng Nav2 tới điểm đích | Xong trên mô phỏng, với các giới hạn nêu ở mục vấn đề đã biết |
| Thả chướng ngại vật khi xe đang chạy và né bằng Lidar | Xong trên mô phỏng (bấm trong RViz chưa thử bằng tay) |
| Xe không kẹt khi áp sát vật cản (AMCL bền, RPP lùi theo đường Reeds-Shepp, biên an toàn) | Xong với RPP; MPPI vẫn thất bại khi bị vây kín nhưng không kẹt vĩnh viễn |
| SLAM, IMU, encoder trong mô hình | Chưa làm |
| Firmware STM32F103C8T6 | Chưa làm (chỉ có file rỗng) |
| Giao tiếp ROS và firmware | Chưa làm |
| Chạy trên xe thật | Chưa làm |

## Đã kiểm chứng ngày 2026-11-08

Chạy trên máy hiện tại, build riêng ngoài repo.

| Kiểm tra | Kết quả |
| --- | --- |
| Build sạch bằng `colcon build` | Đạt |
| `colcon test` | Đạt: 4/4 ca kiểm tra hình học (`colcon test-result` đếm 5 vì tính thêm một mục của CTest) |
| `xacro` và `check_urdf` | Đạt |
| `generate_meshes.py` so với `meshes/` | Cả 11 file giống hệt từng byte |
| Mô phỏng không giao diện, vòng tròn 1 m, 0,2 m/s | Hoàn thành và dừng xe; các mốc log mỗi 5 giây cho thấy lệch đường khoảng 3 cm |
| `colcon build` trên các thư mục đang commit | **Lỗi:** `CMakeCache.txt` trỏ tới `/home/viet/ackermann_ws` |
| `source install/setup.bash` đang commit | **Lỗi:** không tìm thấy gói; 30/32 symlink trong `install/` bị hỏng |

## Đã kiểm chứng ngày 2026-11-11

Chạy Gazebo không giao diện trên máy hiện tại; số đo lấy từ vị trí thật của Gazebo tại tâm trục sau. Mỗi cấu hình chạy một lượt.

| Kiểm tra | Kết quả |
| --- | --- |
| `colcon test` | Đạt: 15/15 ca (4 ca quỹ đạo, 6 ca world/bản đồ, 5 ca thả chướng ngại vật); `colcon test-result` đếm 18 |
| Thả trụ lên đường đi qua `/clicked_point` khi xe đang chạy, RPP và MPPI, ba tình huống mỗi bộ (trụ cách 1,3 m; 0,8 đến 0,9 m; 0,7 m) | 6/6 tới đích, không va chạm; tình huống 0,7 m cần một lần lùi hồi phục 0,5 m |
| `obstacle_spawner`: từ chối điểm sát xe và điểm sai frame; `/clear_obstacles` | Đạt |
| Tái hiện lỗi người dùng báo (xe đứng yên cạnh cụm trụ dù chưa chạm) | Tái hiện được với cấu hình cũ: AMCL lệch 16 cm, `Starting point in lethal space`, mọi đích sau đó đều thất bại |
| Cấu hình mới, RPP: 6 đích chuẩn, 4 đích cần lùi, 2 lần thả trụ trên đường, túi cụt, bài nhiều trụ | Tất cả đạt; bài nhiều trụ mất 83 s với 2 lần hồi phục, không chạm; sai số AMCL tối đa 0,08 m |
| Cấu hình mới, MPPI: 6 đích chuẩn, 2 lần thả trụ trên đường, bài nhiều trụ | 6 đích và 2 lần thả trụ đạt; bài nhiều trụ thất bại (không có đường tiến), đích kế tiếp đạt |
| `xacro` và `check_urdf` sau khi thêm `lidar_link`, `rear_axle` | Đạt |
| `/scan` | 360 tia, frame `lidar_link`, khoảng 7 Hz theo đồng hồ thực |
| Nav2 với RPP, 6 đích liên tiếp | 6/6 đạt, 29 đến 45 s mỗi chặng, lệch vị trí 0,08 đến 0,14 m, lệch hướng tới 0,27 rad, không hồi phục |
| Nav2 với MPPI, 6 đích liên tiếp | 6/6 đạt, 24 đến 37 s mỗi chặng, lệch vị trí 0,07 đến 0,16 m, lệch hướng tới 0,29 rad, không hồi phục |
| Gửi đích qua `/goal_pose` | Đạt |
| RViz với cấu hình mặc định của `nav2_bringup` | Mở được và nạp bản đồ; chưa kiểm tra bằng mắt |
| `navigation.launch.py` có giao diện Gazebo | Chưa chạy thử |
| Đáp ứng bước của khớp lái | 58% sau 1 s, 85% sau 2 s, 98% sau 4 s |
| `trajectory.launch.py`, vòng tròn 0,8 m, 0,2 m/s (kiểm tra hồi quy) | Hoàn thành, lệch đường khoảng 4 cm |

Theo README, chưa chạy lại đợt này:

- Hình số 8, hệ số 2,5 m, 0,2 m/s: lệch lớn nhất khoảng 4,3 cm; vòng tròn 1 m: khoảng 3,4 cm.
- Dừng an toàn khi mất odometry và khi nhấn Ctrl+C.
- Tiến, lùi và rẽ bằng `manual_control`.

## Đã hoàn thành

- **Mô hình xe:** khung ba tầng, tổng khối lượng mô phỏng 1,15 kg, chiều dài cơ sở 165 mm, vết bánh 167 mm, bánh xe đường kính 75 mm. Tầng thứ ba là thiết kế tùy chỉnh. Mesh được sinh bằng code, chỉ gần đúng với xe thật.
- **Mô phỏng:** plugin AckermannSteering, OdometryPublisher và JointStatePublisher; bridge cho `/cmd_vel`, `/odom`, `/wheel_odom`, `/joint_states`, `/tf`, `/clock`.
- **Điều khiển thủ công:** `manual_control`, tự dừng khi hết thời gian hoặc khi ngắt.
- **Bám quỹ đạo:** `trajectory_controller`, kiểm tra cấu hình đầu vào, có các điều kiện dừng an toàn (hết thời gian, mất odometry, lệch đường quá ngưỡng, đổi hệ tọa độ) và xuất đường đi để xem trên RViz.

## Vấn đề đã biết

| Mức | Vấn đề | Vị trí | Hướng xử lý |
| --- | --- | --- | --- |
| Cao | 244 file sản phẩm build và log bị commit; làm hỏng build mặc định, thay đổi rác sau mỗi lần build | `build/`, `install/`, `log/` | Thêm `.gitignore`, `git rm -r --cached` |
| Cao | Toàn bộ firmware rỗng, kể cả `Makefile` và `README.md`; thiếu `main`, startup và linker script | `firmware/stm32f103c8t6/` | Xem giai đoạn 3 |
| Cao | `/odom` lấy vị trí thật của Gazebo, không dùng được trên xe thật | `urdf/ackermann.urdf.xacro`, `config/bridge.yaml` | Đánh giá bộ điều khiển với `/wheel_odom` có nhiễu, sau đó thay bằng định vị từ cảm biến |
| Cao | Khớp lái mô phỏng trễ khoảng 1 s (plugin AckermannSteering dùng khâu tỉ lệ hệ số cố định), chậm hơn nhiều so với servo thật; buộc Nav2 chạy chậm (0,20 m/s) và lập đường với bán kính 0,6 m | `urdf/ackermann.urdf.xacro` | Điều khiển khớp lái bằng `JointPositionController` riêng, rồi chỉnh lại tham số Nav2 |
| Trung bình | Với MPPI, Nav2 chỉ lập đường tiến (Dubins): đích sát tường, xe quay mặt vào tường gần hoặc bị vật cản vây quanh thì thất bại sau vài lần lùi hồi phục. MPPI của Humble chững ở điểm đổi chiều của đường Reeds-Shepp | `config/controller_mppi.yaml` | Thử lại sau khi sửa khớp lái, hoặc dùng Nav2 bản mới hơn |
| Trung bình | Với RPP, đường Reeds-Shepp có 1 đến 4 lần đổi chiều kể cả ở chỗ thoáng, xe lùi nhiều hơn cần thiết; thao tác quay đầu tại chỗ có lần chững khoảng 20 s | `config/controller_rpp.yaml`, `config/nav2_params.yaml` | Thử tăng `reverse_penalty`; cân nhắc Dubins trước, Reeds-Shepp chỉ khi không có đường tiến |
| Trung bình | Vật cản dày đặc (các trụ cách nhau dưới khoảng 0,6 m quanh xe) vẫn có thể làm Nav2 thất bại | | Chạy lặp nhiều bố cục để tìm ngưỡng |
| Trung bình | Hướng tới đích lệch 0,15 đến 0,3 rad vì xe dừng ngay khi vào dung sai | `config/nav2_params.yaml` | Giảm dung sai sau khi sửa khớp lái |
| Trung bình | Vật cản xuất hiện cách xe dưới khoảng 0,8 m không tránh được bằng đường tiến: xe dừng sát trụ (Lidar cách mặt trụ 0,16 m), lùi 0,5 m rồi mới vòng qua | `behavior_trees/`, `config/controller_*.yaml` | Cải thiện sau khi sửa khớp lái; cân nhắc giảm tốc theo chi phí costmap |
| Trung bình | Mỗi cấu hình Nav2 mới thử một lượt; né vật cản mới thử trụ tĩnh, chưa thử vật cản di chuyển, chưa thử `NavigateThroughPoses` | | Chạy lặp nhiều lượt, thêm vật cản động |
| Trung bình | 8 file `.pyc` bị commit | `src/**/__pycache__/` | Gỡ cùng lúc với việc thêm `.gitignore` |
| Trung bình | Đường dẫn `/home/viet/ackermann_ws` | `README.md` của gói, `.vscode/settings.json` | Dùng đường dẫn tương đối |
| Thấp | `-p radius:=2` làm node thoát kèm traceback thay vì báo lỗi cấu hình gọn | `trajectory_controller.py`, `main()` chỉ bắt `ValueError` và `TypeError` | Bắt thêm `ParameterException`, hoặc khai báo tham số cho phép đổi kiểu |
| Thấp | Thông tin người bảo trì còn là giá trị mẫu (`student@example.com`); commit duy nhất có message `a` | `package.xml`, lịch sử git | Điền thông tin thật, viết message rõ nghĩa từ nay |
| Thấp | Thông số bánh xe, khoảng cách trục và khối lượng là ước lượng | README, URDF | Đo trên xe thật rồi cập nhật URDF |

## Việc tiếp theo (đề xuất)

Thứ tự dưới đây là đề xuất dựa trên phụ thuộc kỹ thuật, cần đối chiếu với yêu cầu đồ án và lịch của nhóm.

### Giai đoạn 1: Dọn repo

- [ ] Thêm `.gitignore` cho `build/`, `install/`, `log/`, `__pycache__/`, gỡ các file đó khỏi git
- [ ] Bỏ đường dẫn `/home/viet` trong README và `.vscode/settings.json`
- [ ] Sửa lỗi kiểu tham số của `trajectory_controller`
- [ ] Điền thông tin người bảo trì trong `package.xml`

### Giai đoạn 2: Hoàn thiện mô phỏng

- [x] Thêm Lidar 2D vào URDF (2026-11-11)
- [x] World có vật cản, bản đồ, định vị AMCL và điều hướng Nav2 với Smac Hybrid-A*, RPP, MPPI (2026-11-11)
- [x] Thả chướng ngại vật bằng Publish Point trong RViz và né bằng Lidar (2026-11-11)
- [ ] Sửa khớp lái mô phỏng cho nhanh như servo thật, rồi chỉnh lại Nav2 (tốc độ, bán kính, dung sai)
- [ ] Đối chiếu thông số Lidar với cảm biến thật; chốt các cảm biến còn lại (IMU, encoder)
- [ ] Đánh giá bám quỹ đạo và Nav2 khi dùng `/wheel_odom` thay cho vị trí thật
- [ ] Thử Nav2 nhiều lượt, với vật cản không có trên bản đồ; so sánh RPP và MPPI có thống kê
- [ ] Tạo bản đồ bằng SLAM (`slam_toolbox` đã cài) nếu đồ án yêu cầu
- [ ] Thêm test tự động cho `manual_control` và logic Pure Pursuit (hiện test mới chỉ phủ `ReferencePath`)

### Giai đoạn 3: Firmware

Tên file cho thấy sáu module dự kiến: `motor`, `encoder`, `pid`, `servo`, `ackerman`, `communication`.

- [ ] Chốt phần cứng: driver động cơ, encoder, sơ đồ chân, nguồn và framework (HAL hoặc thanh ghi)
- [ ] Cài toolchain ARM và công cụ nạp; viết `Makefile`, startup, linker script và `main.c`
- [ ] Viết và thử từng module: `motor` và `encoder`, rồi `pid`, `servo`, `ackerman`
- [ ] Định nghĩa giao thức giữa ROS và vi điều khiển, viết `communication`

### Giai đoạn 4: Tích hợp xe thật

- [ ] Viết node ROS nhận `/cmd_vel`, gửi xuống firmware và xuất odometry; tách cấu hình xe thật khỏi cấu hình Gazebo
- [ ] Thêm cơ chế dừng khẩn khi mất kết nối (watchdog ở phía firmware)
- [ ] Chạy thử trên giá kê, rồi trên sàn ở tốc độ thấp
- [ ] Đo sai số quỹ đạo và so sánh với mô phỏng

## Câu hỏi cần nhóm trả lời

1. Phạm vi đồ án gồm những gì: chỉ bám quỹ đạo, hay cả định vị, SLAM, tránh vật cản?
2. Máy tính nào chạy ROS trên xe, và kết nối với STM32 qua giao tiếp gì?
3. Xe thật đã có đủ linh kiện chưa? Tầng thứ ba sẽ chứa những gì?
4. Hạn nộp và mốc báo cáo là khi nào?

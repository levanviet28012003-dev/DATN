# Cảm biến và hiển thị GPS

Node `gps_visualizer` nhận `/gps/fix` (`sensor_msgs/msg/NavSatFix`) và xuất chữ
vĩ độ, kinh độ, độ cao qua `/gps/marker` (`visualization_msgs/msg/Marker`).
Chữ mặc định màu vàng khi có tọa độ hợp lệ, màu đỏ khi bản tin báo chưa có định vị.
Launch Nav2 dùng xanh đậm để đọc trên nền bản đồ trắng.

Muốn thấy cả bản đồ phòng, vật cản và GPS, dùng
`ros2 launch ackermann_bringup navigation.launch.py` sau khi đóng mô phỏng cũ.
Launch này đã chạy sẵn node và display GPS; không cần mở thêm `gps_view`.

Khi Gazebo đang chạy:

```bash
ros2 launch ackermann_bringup gps_view.launch.py
```

Lệnh này mở RViz với cấu hình sẵn. Nếu đã mở RViz và muốn dùng cửa sổ hiện tại:

```bash
ros2 run ackermann_sensors gps_visualizer --ros-args -p use_sim_time:=true
```

Trong RViz, chọn **Add → Marker**, topic `/gps/marker`, Fixed Frame `odom`
(hoặc `map` khi đang chạy Nav2). Chữ đi theo TF của `gps_link`; tọa độ hiển thị lấy
trực tiếp từ bản tin GPS. Node này chưa chuyển GPS thành vị trí xe trong hệ mét,
chưa vẽ bản đồ vệ tinh và chưa thực hiện định vị bằng GPS/EKF.

Tham số `fix_topic` và `marker_topic` cho phép đổi tên topic. Mặc định là
`/gps/fix` và `/gps/marker`. `text_height` là chiều cao chữ tính bằng mét (mặc định
0,025; launch Nav2 đặt 0,18 để xem toàn phòng). `text_color` là ba giá trị RGB từ 0 đến 1.
Nếu dùng dữ liệu thật, đặt `use_sim_time:=false`.

Driver Lidar/IMU/GPS điện thoại trên xe thật chưa triển khai. Cảm biến mô phỏng
nằm trong URDF của `ackermann_description`, được bridge qua
`ackermann_bringup/config/bridge.yaml`.

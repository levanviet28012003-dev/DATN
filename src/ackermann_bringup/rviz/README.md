`gps.rviz` hiển thị mô hình xe và tọa độ GPS bằng chữ qua topic `/gps/marker`.
Fixed Frame là `odom`, camera nhìn theo `base_link`.

Khi Gazebo đang chạy: `ros2 launch ackermann_bringup gps_view.launch.py`.
Để mở cả Gazebo và RViz từ đầu, thêm `start_sim:=true`.

`navigation.rviz` dùng khi chạy `navigation.launch.py`: bản đồ phòng, Lidar,
costmap, mô hình xe, công cụ Nav2 và chữ tọa độ GPS. Cấu hình dựa trên
`nav2_bringup/rviz/nav2_default_view.rviz` của Humble; Fixed Frame là `map`.

Muốn thấy cả phòng và GPS, đóng mô phỏng cũ rồi chạy
`ros2 launch ackermann_bringup navigation.launch.py`.
`gps_world.sdf` là world trống và `gps.rviz` chỉ hiển thị xe/GPS.

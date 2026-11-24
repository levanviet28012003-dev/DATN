`navigation.rviz` dùng khi chạy `navigation.launch.py`: bản đồ SLAM `/map` (lớn dần khi Lidar quét),
Lidar, costmap, mô hình xe, công cụ Nav2 (2D Goal Pose, Publish Point) và chữ tọa độ GPS. Cấu hình
dựa trên `nav2_bringup/rviz/nav2_default_view.rviz` của Humble; Fixed Frame là `map`.

`outdoor.rviz` dùng khi chạy `outdoor.launch.py`: Lidar, costmap, đường đi, vị trí thật
(`/ground_truth/odom`), EKF local (`/odom`), đích (`/gps_goal/pose`); Fixed Frame là `map`, camera
theo `base_link`. Bản đồ vệ tinh xem bằng Mapviz.

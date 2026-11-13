# Định vị và bản đồ

`config/amcl.yaml` giữ tham số AMCL và map_server tách từ cấu hình Nav2 cũ.
`maps/nav_map.yaml` và `nav_map.pgm` khớp world trong `ackermann_bringup`.

```bash
ros2 launch ackermann_localization localization.launch.py use_sim_time:=true
```

Đổi bản đồ bằng `map:=/duong/dan/map.yaml`. Cấu hình hiện dành cho mô phỏng,
có initial pose (0, 0, 0) và dùng `/odom` từ Gazebo. Xe thật cần hiệu chỉnh lại
initial pose, nhiễu chuyển động và dữ liệu cảm biến. Package này chưa triển khai SLAM.
Sinh lại map/world bằng `python3 src/ackermann_bringup/scripts/generate_world.py`.

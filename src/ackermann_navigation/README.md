# Điều hướng và bám quỹ đạo

- `launch/nav2.launch.py`: chỉ khởi chạy các server Nav2, không tự mở Gazebo hoặc AMCL.
- `config/nav2_params.yaml`: planner, costmap, BT navigator, behavior và velocity smoother.
- `config/controller_rpp.yaml`, `controller_mppi.yaml`: chọn qua `controller:=rpp|mppi`.
- `behavior_trees/`: cây hành vi đi một đích và qua nhiều đích.
- `scripts/trajectory_controller.py`: Pure Pursuit đường tròn/số 8.

Chạy toàn hệ thống: `ros2 launch ackermann_bringup navigation.launch.py`.
Chạy mô phỏng bám quỹ đạo: `ros2 launch ackermann_bringup trajectory.launch.py`.
Khi đã có mô phỏng: `ros2 launch ackermann_navigation trajectory.launch.py`.
Các launch nhận `use_sim_time` (mặc định true). Không chạy nhiều nguồn phát `/cmd_vel` cùng lúc.
Xem [hướng dẫn và giới hạn mô phỏng](../ackermann_bringup/README.md).

# Hiwonder-style two-deck Ackermann chassis

Reference: https://www.hiwonder.com/products/ackermann-steering-chassis?variant=40382428348503

The visual model follows the supplied double-deck photograph with black perforated aluminium decks, brass spacers, rear motor cases, front servo, black tyres and red wheel rims. It is a procedural approximation, not manufacturer CAD. Machining openings are approximated on a 1 mm grid. Regenerate the committed STL and Collada files with `python3 src/ackermann_description/scripts/generate_meshes.py` (Python standard library only).

| Parameter | Value | Basis |
| --- | --- | --- |
| Deck length | 270 mm | Manufacturer |
| Deck width | 120.5 mm | Manufacturer raised-layer plate |
| Plate thickness | 2 mm | Manufacturer |
| Overall width at straight steering | 197 mm | Manufacturer; track plus tyre width |
| Servo case | 40 × 20 × 40.5 mm | Manufacturer LD-1501MG |
| Wheel diameter / width | 75 / 30 mm | Estimate; retained from original model |
| Wheelbase / centre track | 165 / 167 mm | Estimate; retained from original model |
| Lower / upper deck centre above axle | -9 / 43 mm | Estimated from photograph |
| Deck centre spacing | 52 mm | Estimated; 50 mm clear gap |
| Total mass including Lidar and GPS | 1.175 kg | Simulation estimate, excludes battery/payload |
| Front wheel steering limit | ±0.6 rad | Simulation assumption, not servo travel |

The two decks retain six brass spacers and silver fasteners. A small illustrative GPS antenna is mounted behind the Lidar, at (-0.070, 0, 0.047) m in `base_link`; its estimated mass is 5 g. The Lidar sits on the upper deck, with its scan plane 64 mm above `base_link` (about 101.5 mm above level ground).

Manufacturer lists packaged weights, which are not used as measured vehicle mass. Hole locations, tread pattern, motor envelopes, mass distribution and inertia are illustrative. Collision geometry uses simple plates, a servo box and cylindrical wheels for stable contact; decorative hardware is visual only.

Coordinates: +X forward, +Y left, +Z up. `base_link` is centred between axles at wheel-axis height. At rest its Z is approximately 0.0375 m. Existing wheel and steering joint names are preserved. Front wheel rotation is passive and rear wheel joints represent independent motors. The Gazebo AckermannSteering plugin drives the rear wheels with different speeds when turning and controls the two front steering joints. The launch starts ros_gz_bridge for commands, odometry, joint states, TF and simulation time. `/odom` and the `odom → base_link` transform use Gazebo OdometryPublisher ground-truth pose for simulation trajectory feedback. The Ackermann plugin wheel-based estimate is kept separately on `/wheel_odom`; its TF is not bridged to avoid duplicate transforms.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ackermann_bringup
source install/setup.bash
ros2 launch ackermann_bringup simulation.launch.py
```

The launch supplies mesh resource paths for Ignition Fortress and Gazebo Sim and spawns the axle at 40 mm above the ground.

Colours are declared inline in each URDF visual. Gazebo uses geometry-only binary STL meshes with explicit face normals (`chassis_plate.stl` and `wheel_{rubber,red,black,silver}.stl`); colour comes from URDF/SDF, with no embedded Collada materials to override it. DAE files remain as export artifacts. After rebuilding, stop the previous launch and Gazebo window and relaunch to reload the spawned model. The launch prints the absolute Xacro path with `Loading coloured STL model` to identify the package actually loaded.

## Sử dụng package sau khi tách

Package này giữ URDF/Xacro, mesh, script sinh mesh và `description.launch.py`.
Launch mô hình: `ros2 launch ackermann_description description.launch.py`.
Mô phỏng: `ros2 launch ackermann_bringup simulation.launch.py`.
Xem [hướng dẫn mô phỏng và điều khiển](../ackermann_bringup/README.md).

## GPS mô phỏng

Mô hình có frame `gps_link` gắn cố định vào `base_link` và cảm biến NavSat 5 Hz.
Gazebo phát `ignition.msgs.NavSat` trên `/model/ackermann_car/gps/fix`; bridge trong
`ackermann_bringup` chuyển sang `/gps/fix` (`sensor_msgs/msg/NavSatFix`).
Cần world có plugin `ignition::gazebo::systems::NavSat` và `spherical_coordinates`.
Hai world `gps_world.sdf` và `nav_world.sdf` của bringup đã được cấu hình.
GPS đang lý tưởng, chưa thêm nhiễu để kiểm chứng tọa độ; không đại diện độ chính xác điện thoại.

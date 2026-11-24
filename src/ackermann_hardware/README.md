# Giao tiếp phần cứng

Node `stm32_bridge` nối ROS 2 trên Pi4 với STM32F103C8T6 qua UART. Pi quyết định, STM32 chỉ thực thi: Pi đổi `/cmd_vel` thành vận tốc và góc lái rồi gửi xuống; STM32 tự dừng xe nếu 300 ms không nhận được lệnh hợp lệ. Firmware nằm ở `firmware/stm32f103c8t6`.

Trạng thái: đã build, có test mã hóa khung và test firmware trên máy tính; **chưa thử trên phần cứng thật**.

## Nối dây

| Pi4 (header 40 chân) | STM32 (Blue Pill) |
| --- | --- |
| GPIO14 / TXD, chân 8 | PA10 (USART1_RX) |
| GPIO15 / RXD, chân 10 | PA9 (USART1_TX) |
| GND, chân 6 | GND |

Cả hai đều dùng mức 3,3 V, không cần chuyển mức. Không nối 5 V giữa hai bo nếu chúng đã có nguồn riêng.

## Chuẩn bị Pi4

UART chính `/dev/ttyAMA0` mặc định dành cho Bluetooth; `/dev/ttyS0` (mini UART) đổi baud theo xung CPU nên không ổn định. Trên Ubuntu 22.04 cho Pi:

1. Thêm `dtoverlay=disable-bt` vào `/boot/firmware/config.txt`.
2. Bỏ `console=serial0,115200` khỏi `/boot/firmware/cmdline.txt` để kernel không ghi log ra UART.
3. `sudo systemctl disable --now hciuart` (nếu có), cho người dùng vào nhóm `dialout`, khởi động lại.
4. `sudo apt install python3-serial`.

Sau đó `/dev/serial0` trỏ tới `ttyAMA0`.

## Chạy

```bash
ros2 launch ackermann_hardware hardware.launch.py            # port:=/dev/ttyUSB0 nếu dùng USB-UART
ros2 topic echo /diagnostics                                  # trạng thái liên kết và cờ STM32
ros2 run ackermann_bringup manual_control forward --speed 0.1 --duration 3
ros2 topic pub --once /stm32/estop std_msgs/msg/Bool "{data: true}"   # dừng khẩn; false để nhả
```

`real_robot.launch.py` của `ackermann_bringup` cũng gọi launch này.

| Topic | Hướng | Kiểu | Ghi chú |
| --- | --- | --- | --- |
| `/cmd_vel` | vào | `geometry_msgs/Twist` | Đổi tên bằng `cmd_vel_topic:=...`. δ = atan(L·ω/v), v ≈ 0 thì bánh thẳng |
| `/stm32/estop` | vào | `std_msgs/Bool` | `true` chốt dừng khẩn; nhả xong phải có CMD mới xe mới chạy |
| `/stm32/joint_states` | ra | `sensor_msgs/JointState` | Góc hai bánh sau (rad) từ encoder, 50 Hz |
| `/stm32/battery` | ra | `sensor_msgs/BatteryState` | Điện áp module motor |
| `/diagnostics` | ra | `diagnostic_msgs/DiagnosticArray` | 1 Hz: cờ trạng thái, số khung, lỗi CRC, Seq bị đứt |

Tham số nằm trong `config/stm32_bridge.yaml`. `wheelbase`, `max_steering_angle`, `max_speed`, `encoder_ppr` phải khớp `ackerman.h` và `encoder.h` của firmware. Không có `/cmd_vel` quá `command_timeout` (0,5 s) thì node gửi `enable = 0`; khi thoát node gửi lặp lệnh dừng.

## Giao thức

115200 8N1, khung `[0xAA 0x55][Len][Type][Seq][Payload][CRC16]`. Len là số byte payload. CRC16-CCITT (đa thức 0x1021, khởi tạo 0xFFFF) tính trên Len, Type, Seq, Payload. Mọi số nhiều byte, kể cả CRC, đều little-endian.

| Type | Hướng | Payload |
| --- | --- | --- |
| 0x01 CMD | Pi → STM | int16 v (mm/s), int16 δ (0,01°, dương = trái), uint8 enable; 25 Hz |
| 0x02 STATE | STM → Pi | int32 encoder trái, int32 encoder phải, uint16 điện áp (mV), uint8 trạng thái; 50 Hz |
| 0x03 ESTOP | Pi → STM | uint8: 1 kích hoạt, 0 nhả |

Bit trạng thái: 0 READY, 1 FAULT, 2 ESTOP, 3 CMD_TIMEOUT, 4 DRIVING, 5 ENCODER_VALID, 6 VOLTAGE_VALID. Định nghĩa gốc nằm trong `firmware/stm32f103c8t6/peripheral/inc/protocol.h`; sửa giao thức thì sửa cả hai phía và các khung mẫu trong hai test.

## Test

```bash
colcon test --packages-select ackermann_hardware && colcon test-result --verbose
cd firmware/stm32f103c8t6 && make test     # test parser, watchdog, ESTOP của firmware trên máy tính
```

# Giao tiếp phần cứng

Đây là khung package cho node ROS 2 giao tiếp UART với STM32.
Chưa có node serial, bộ giải mã phản hồi hoặc nguồn odometry từ encoder thật.
`hardware.launch.py` hiện chỉ thông báo trạng thái, không mở cổng serial.
Firmware STM32 vẫn nằm trong `firmware/stm32f103c8t6`, ngoài các package ROS.

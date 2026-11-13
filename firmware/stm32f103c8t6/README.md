# Firmware STM32F103C8T6 viết thanh ghi trực tiếp

Project C thuần, build bằng `arm-none-eabi-gcc` và Makefile. Các thanh ghi ngoại vi được khai báo trong các header `driver/inc/` như `RCC.h`, `GPIO.h`, `UART.h`; không cần CubeMX, HAL hoặc thư viện Standard Peripheral.

## Cấu trúc

```text
stm32f103c8t6/
├── .vscode/                  # Build, IntelliSense và debug
├── core/
│   ├── inc/                 # CMSIS Core và systick.h
│   └── src/                 # main.c, mốc SysTick và runtime khởi tạo
├── driver/
│   ├── inc/                 # RCC.h, GPIO.h, UART.h, I2C.h, TIM.h, EXTI.h
│   ├── src/                 # Driver ngoại vi viết thanh ghi trực tiếp
│   └── device/
│       ├── inc/             # CMSIS Device, STM32F103xB và system_stm32f1xx.h
│       └── src/             # SystemInit, SystemCoreClockUpdate và các hàm ngắt
├── peripheral/
│   ├── inc/                 # Header ứng dụng motor, servo, communication
│   └── src/                 # servo.c và các phần ứng dụng khác
├── startup/                 # Bảng vector, Reset_Handler
├── linker/                  # Bố trí Flash và RAM
├── tools/                   # Tạo compile_commands.json
├── Makefile
└── build/                   # ELF, HEX, BIN, MAP và compile_commands.json
```

Startup đặt bảng vector tại `0x08000000`, khởi tạo `.data`, xóa `.bss`, gọi `SystemInit()` trong CMSIS Device rồi khởi tạo dữ liệu/runtime và vào `main()`. Muốn dùng ngắt ngoại vi thì định nghĩa hàm C trùng tên trong startup để ghi đè hàm yếu. Linker dành 64 KiB Flash và 20 KiB SRAM, đồng thời kiểm tra còn tối thiểu 1 KiB cho stack. Project hiện dành cho C; `runtime.c` xử lý các bảng preinit/init mà startup gọi qua `__libc_init_array`, chưa cấu hình heap động.

## Chức năng đang có

- Ứng dụng chuyển từ HSI sang HSE 8 MHz nhân PLL ×9: HCLK 72 MHz, PCLK1 36 MHz, PCLK2 72 MHz và TIM2CLK 72 MHz.
- SysTick tạo mốc 1 ms; `SysTick_Millis()` trả số mili giây không dấu 32 bit.
- Servo tại PA0/TIM2_CH1, Alternate Function Push-Pull, PSC = 71, ARR = 19999, PWM Mode 1, tần số 50 Hz.
- Mặc định chỉ thử hai motor cầu sau qua Watch, dùng module I²C tại địa chỉ 7 bit `0x34`; hai bánh dừng cho đến khi bật lệnh chạy, không khởi tạo PWM servo.
- Vòng lặp đặt lịch lệnh motor và encoder mỗi 20 ms, điện áp mỗi 1000 ms; tốc độ thực tế còn phụ thuộc thời gian giao dịch I²C. Phản hồi nằm trong `app_status` để xem bằng debugger.
- Lệnh góc được giới hạn 50–130°, tương ứng ±40° quanh giữa 90°; lệnh đặt xung trực tiếp cũng giữ giới hạn này.

Driver motor có khởi tạo/retry, điều khiển hai bánh, đọc encoder và điện áp. Phần Ackermann và communication chưa được triển khai; chương trình mẫu chưa nhận lệnh ROS và chưa bật UART.

`SysTick_Init()` cập nhật `SystemCoreClock` trước khi cấu hình ngắt 1 ms. Vòng lặp dùng phép trừ không dấu để kiểm tra thời gian, vẫn hoạt động khi bộ đếm mili giây tràn. TIM2 được dành riêng cho PWM servo; khởi tạo lại sẽ xóa cấu hình đếm/ngắt cũ. Khi đặt remap PA0, driver đặt `SWJ_CFG = 000` tường minh để giữ đầy đủ JTAG/SWD.

## Kết nối và hiệu chỉnh servo

Nối dây tín hiệu servo vào PA0 và nối GND nguồn servo chung với GND STM32. Với LD-1501MG trong tài liệu Hiwonder, dùng nguồn servo riêng 6–8,4 V. Cấu hình clock hiện yêu cầu bo có HSE 8 MHz.

Các hàm `Servo_Init()`, `Servo_SetAngle()`, `Servo_SetPulse()` và `Servo_Center()` giữ nguyên giao diện. Gọi cấu hình clock, khởi tạo SysTick rồi khởi tạo servo như trong `core/src/main.c`. Không cần chạy PID vị trí bên ngoài để phát lệnh PWM cho servo này.

Thông số hiệu chỉnh nằm trong `peripheral/inc/servo.h`: `SERVO_MIN_PULSE`, `SERVO_CENTER_PULSE`, `SERVO_MAX_PULSE` mô tả xung theo góc trục servo; `SERVO_STEERING_LIMIT` mặc định 40°. Nội suy hai phía tôn trọng xung giữa đã hiệu chỉnh. Với thông số mặc định, các hàm điều khiển servo chỉ cho phép xung 1056–1944 us. Đây là góc trục servo; góc bánh xe thực tế cần đo và hiệu chỉnh theo cơ cấu lái.

## Module motor cầu sau

Giao tiếp dùng PB6/SCL và PB7/SDA; GND module và STM32 phải nối chung. Kiểm tra dây theo đúng phiên bản module, đặc biệt bản V1.2 có ghi đảo nhãn SDA/SCL trong tài liệu hãng. Nguồn motor chọn theo động cơ thực tế; kiểm tra mức điện áp và điện trở kéo lên của SCL/SDA trên bo.

Driver `driver/src/I2C.c` dùng I2C1 phần cứng, AF Open-Drain, PCLK1 36 MHz, CCR = 180 và TRISE = 37, tương ứng 100 kHz. SCL và SDA cần điện trở kéo lên ngoài; cấu hình xe hiện đã chạy với hai điện trở 4,7 kΩ lên 3,3 V.

`peripheral/inc/motor.h` giữ mapping bánh trái M1, bánh phải M2. Đổi `MOTOR_LEFT_INDEX`/`MOTOR_RIGHT_INDEX` nếu dây nối khác. Đặt `MOTOR_LEFT_DIRECTION` hoặc `MOTOR_RIGHT_DIRECTION` thành `-1` nếu bánh tương ứng quay ngược chiều tiến; driver tự áp dụng cùng phép đổi dấu cho encoder. `MOTOR_ENCODER_POLARITY` là cấu hình phản hồi bên trong module, cần khớp loại motor; nó khác với quy ước chiều tiến của xe.

Đơn vị tốc độ vòng kín là **xung encoder/10 ms**, không phải m/s hoặc phần trăm PWM. `MOTOR_MIN_SPEED`/`MOTOR_MAX_SPEED` chỉ là giới hạn phần mềm; hiệu chỉnh theo động cơ, nguồn và số xung mỗi vòng thực tế. Chương trình thử mặc định dùng mức 10.

Gọi `Motor_Init()` từ main sau khi SysTick đã chạy và ngắt đang bật. Hàm chỉ chặn trong lúc khởi tạo: chờ 200 ms, gửi dừng, ghi loại JGB, chờ 5 ms, ghi cực tính, chờ 5 ms và gửi dừng lại. Có tối đa 3 lần thử, cách nhau 20 ms. Các hàm đặt tốc độ/đọc phản hồi trả về 0 nếu chưa khởi tạo thành công; `Motor_Stop()` vẫn cho phép thử gửi lệnh dừng khi đang lỗi.

Driver I2C1 bảo vệ các đoạn ADDR/ACK/STOP bằng lưu và khôi phục PRIMASK, chờ STOP tự xóa trước khi khôi phục ACK/POS. Vòng chờ phần cứng có giới hạn số lần polling; sau lỗi truyền, bộ điều khiển I²C được reset/cấu hình lại. Việc này không thay thế xử lý bus bị giữ mức thấp do dây hoặc module.

Nếu khởi tạo thấy BUSY khi chưa là master, driver giữ trình tự phục hồi bộ lọc analog theo ES096 mục 2.8.7. Các bước kiểm tra mức chân có timeout, luôn nhả SDA/SCL và cấu hình lại I2C khi kết thúc.

## Chạy chương trình hệ thống

Mặc định `APP_REAR_MOTOR_TEST_ONLY = 1`, `APP_SERVO_TEST_ONLY = 0`. Chế độ thử cầu sau không phát PWM servo. Nối hai motor có encoder vào M1/M2, PB6 tới SCL, PB7 tới SDA, GND chung; cấp nguồn module đúng điện áp định mức motor. Không cấp nguồn motor từ chân 3V3 hoặc đầu nạp ST-Link.

Kê hai bánh khỏi mặt đất để kiểm tra chiều quay, build rồi F5 nạp/debug. Sau khi khởi tạo, kiểm tra `app_status.state = 1` và `app_status.error = 0`. Trong Watch, chỉ thêm tên biến, không nhập biểu thức gán cố định:

| Biến | Giá trị khởi động | Cách dùng |
| --- | --- | --- |
| `motor_test_left_speed` | 0 | Tốc độ M1, giới hạn thử -10 đến 10 xung/10 ms |
| `motor_test_right_speed` | 0 | Tốc độ M2, cùng đơn vị và giới hạn |
| `motor_test_enable` | 0 | 0 gửi dừng; khác 0 cho phép dùng hai tốc độ trên |

Khi debugger đang tạm dừng, đặt tốc độ (ví dụ 5 và 0 để thử riêng M1), bật `motor_test_enable = 1`, rồi F5 chạy tiếp. Thử M2 bằng 0 và 5; thử hai bánh bằng 5 và 5. Dấu dương chỉ tương ứng chiều tiến sau khi đã kiểm tra cách lắp và cấu hình chiều trong `motor.h`. Muốn dừng, Pause, đặt `motor_test_enable = 0`, rồi F5 để STM32 thực sự gửi lệnh dừng; chỉ Pause hoặc dừng debugger không đảm bảo motor dừng vì module có thể giữ lệnh cuối. Có sẵn cách ngắt nguồn motor trong khi thử.

Có thể xem `app_status` trong Watch để theo dõi trạng thái, encoder trái/phải, điện áp mV, thời điểm cập nhật và lỗi giao tiếp. `encoder_valid`/`voltage_valid` cho biết mẫu còn hợp lệ hay đã bị đánh dấu lỗi. Khi lỗi được giữ lại, các lệnh thử cũng bị xóa; cần sửa nguyên nhân rồi reset.

Muốn chỉ thử servo, đặt `APP_REAR_MOTOR_TEST_ONLY = 0` và `APP_SERVO_TEST_ONLY = 1`, rồi dùng biến `servo_test_angle` (50–130°, khởi động ở 90°). Không bật đồng thời hai chế độ thử riêng.

Muốn khởi tạo cả motor và servo, đặt hai macro thử riêng bằng 0. Chế độ này giữ servo ở giữa và tốc độ motor bằng 0; chưa có giao tiếp nhận lệnh Pi/ROS. Chuỗi chạy thử tự động cũ đã được bỏ.

Khi khởi tạo, gửi lệnh, đọc encoder hoặc đọc điện áp thất bại, ứng dụng giữ trạng thái `APP_FAULT` và thử gửi dừng mỗi 100 ms; không tự tiếp tục lệnh chạy cũ. Servo được đưa về giữa trong chế độ hệ thống; chế độ thử cầu sau không gọi servo. `app_status.error` có các giá trị:

| Giá trị | Ý nghĩa |
| --- | --- |
| 0 | Chưa phát hiện lỗi |
| 1 | Không khởi tạo được SysTick |
| 2 | Khởi tạo module motor thất bại |
| 3 | Gửi lệnh tốc độ thất bại |
| 4 | Đọc encoder thất bại |
| 5 | Đọc điện áp thất bại |

Theo dõi lỗi bằng `app_status.error` và số lần gửi lỗi bằng `app_status.i2c_error_count`. Bảng chẩn đoán thanh ghi tạm thời đã được bỏ; xử lý ACK/NACK, timeout, STOP và phục hồi I2C vẫn được giữ.

`stop_command_ok = 1` chỉ xác nhận giao dịch gửi tốc độ 0 thành công, không phải phép đo xe đã dừng. Nếu I²C hỏng thì phần mềm không đảm bảo driver nhận được lệnh dừng; cần khắc phục phần cứng rồi reset để khởi tạo lại. Khi phát triển giao tiếp Pi, thay chuỗi thử bằng bộ nhận lệnh và timeout lệnh tại vòng lặp này.

Nếu không khởi tạo được SysTick, chương trình dừng khởi tạo trước khi bật servo/motor và ghi mã lỗi 1; nhánh này không chạy được bộ lịch thử gửi dừng.

## Build

```bash
cd /home/viet/ackermann_ws/firmware/stm32f103c8t6
make -j4
```

Kết quả là `build/ackermann.elf`, `build/ackermann.hex`, `build/ackermann.bin` và `build/ackermann.map`. Makefile tìm mã C trong `core/src`, `driver/src`, `driver/device/src` và `peripheral/src`. Thư mục include tương ứng là `core/inc`, `driver/inc`, `driver/device/inc`, `peripheral/inc`.

Điểm vào ứng dụng mặc định là `core/src/main.c`. Khi muốn chuyển vị trí main, truyền `make APP_MAIN=duong/dan/main.c` hoặc sửa `APP_MAIN` trong Makefile; chỉ chọn một file main.

Macro `STM32F103xB` chọn đúng CMSIS Device cho STM32F103C8T6. `stm32f103x6.h` dành cho C4/C6, không được chọn khi build C8. Không bật `USE_HAL_DRIVER`; các định nghĩa CMSIS không yêu cầu HAL. `SystemInit` chỉ được định nghĩa một lần trong `driver/device/src/system_stm32f1xx.c`; `Reset_Handler` nằm trong startup ASM, không tạo thêm vòng lặp Reset_Handler trong file C.

Máy hiện có GCC ARM, objcopy, size, OpenOCD và gdb-multiarch. Đã kiểm tra build và mô hình phần mềm: I²C đọc 1/2/3/4/16/255 byte, ghi dữ liệu, lưu/khôi phục ngắt, STOP, NACK, mất quyền master, timeout; khởi tạo/retry motor, encoder có dấu, đảo chiều; main ở chế độ mặc định/chạy thử, tràn bộ đếm và khóa lỗi. Các kiểm tra phần mềm không thay thế việc đo SCL/SDA/PWM hoặc thử motor trên bo thật.

## VS Code

Mở riêng thư mục này bằng **File → Open Folder**, vì các đường dẫn `${workspaceFolder}` trong `.vscode` được cấu hình theo gốc firmware. Khi mở `ackermann_ws`, cấu hình tại `.vscode` ở gốc workspace đã được bổ sung để nhận diện firmware qua `build/compile_commands.json`. File này được Makefile cập nhật mỗi lần build; chứa đúng compiler ARM, đường dẫn header và cờ Cortex-M3. Các file ROS khác không có trong danh sách này tiếp tục dùng compiler của máy tính. Tác vụ `STM32: build` cũng có ở gốc workspace; cấu hình debug ST-Link hiện nằm trong thư mục firmware.

- `Ctrl+Shift+B`: chạy tác vụ `STM32: build`.
- **Terminal → Run Task → STM32: flash**: build và nạp bằng ST-Link.
- `F5`: debug bằng Cortex-Debug, OpenOCD và `/usr/bin/gdb-multiarch`, dừng tại `main`.

Danh sách extension gợi ý nằm trong `.vscode/extensions.json`. Chúng không được tự cài trong lần thiết lập này.

## Nạp và debug qua ST-Link

Cấu hình hiện dùng ST-Link qua SWD: SWDIO nối PA13, SWCLK nối PA14, GND chung và chân tham chiếu nguồn theo đầu nạp/bo thực tế. Sau khi kết nối có thể nạp bằng:

```bash
make flash
```

Lệnh này ghi Flash, kiểm tra rồi reset. Chưa chạy lệnh nạp vì chưa xác nhận kết nối bo và đầu nạp. Nếu dùng bộ nạp UART thay vì ST-Link, cần cấu hình công cụ nạp khác phù hợp.

Chương trình mẫu chưa cấu hình giao tiếp UART hoặc USB CDC trên Blue Pill.

## Tài liệu đối chiếu

- [ST RM0008: thanh ghi và ngoại vi STM32F1](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).
- [ST AN2824: trình tự nhận và kết thúc giao dịch I²C STM32F1](https://www.st.com/resource/en/application_note/an2824-stm32f10xxx-ic-optimized-examples-stmicroelectronics.pdf).
- [ST ES096 mục 2.8.7: bộ lọc analog có thể giữ cờ BUSY dù SCL/SDA cao](https://www.st.com/resource/en/errata_sheet/es096-stm32f101x8b-stm32f102x8b-and-stm32f103x8b-mediumdensity-device-limitations-stmicroelectronics.pdf).
- [Datasheet STM32F103x8/xB](https://www.st.com/resource/en/datasheet/stm32f103c8.pdf).
- [Bảng vector chuẩn STM32F103xB của ST](https://github.com/STMicroelectronics/cmsis-device-f1/blob/master/Source/Templates/gcc/startup_stm32f103xb.s).
- [Hiwonder: servo LD-1501MG và thông số PWM](https://docs.hiwonder.com/projects/Ackermann-Chassis/en/latest/docs/3_Arduino_Version_formatted.html#steering-servo).

Nếu VS Code còn giữ lỗi include cũ, chạy **C/C++: Reset IntelliSense Database** hoặc **Developer: Reload Window** từ Command Palette. Có thể tạo lại riêng danh sách lệnh biên dịch bằng `make compile-commands` trong thư mục firmware.

#ifndef ACKERMANN_SYSTICK_H
#define ACKERMANN_SYSTICK_H

#include <stdint.h>

// Khởi tạo mốc 1 ms sau khi cấu hình clock; trả về 1 nếu thành công.
uint8_t SysTick_Init(void);
uint8_t SysTick_IsReady(void);
uint32_t SysTick_Millis(void);
void SysTick_Handler(void);

#endif

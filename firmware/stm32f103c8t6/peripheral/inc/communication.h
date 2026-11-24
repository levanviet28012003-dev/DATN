#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <stdint.h>
#include "protocol.h"

/*
 * Nhận lệnh từ Pi4 qua USART1 và gửi trạng thái về.
 * STM32 chỉ thực thi: không có lệnh hợp lệ trong COMM_CMD_TIMEOUT_MS thì
 * Comm_GetCommand() trả về 0 và vòng lặp chính phải dừng xe.
 */

#define COMM_CMD_TIMEOUT_MS   300U

typedef struct
{
    float velocity;         /* m/s */
    float steering_angle;   /* rad, dương = rẽ trái */
} Comm_Command_t;

/* Bộ đếm để xem trong Watch hoặc chẩn đoán đường truyền. */
typedef struct
{
    uint32_t rx_frames;       /* Khung CRC đúng */
    uint32_t cmd_frames;
    uint32_t estop_frames;
    uint32_t crc_errors;
    uint32_t length_errors;   /* Len quá lớn hoặc sai độ dài theo loại khung */
    uint32_t unknown_frames;  /* Loại khung không xử lý */
    uint32_t seq_gaps;        /* Số lần Seq của Pi không liên tiếp */
    uint32_t tx_frames;
    uint32_t tx_dropped;      /* Hàng đợi phát đầy */
    uint32_t last_cmd_ms;
} Comm_Stats_t;

/* Khởi tạo USART1 115200 8N1 và bộ giải khung. Gọi sau RCC_Config_72Mhz(). */
void Comm_Init(void);

/* Đọc hết byte đã nhận và xử lý các khung. Gọi thường xuyên trong vòng lặp chính. */
void Comm_Poll(uint32_t now_ms);

/* Trả về 1 và ghi lệnh nếu có CMD enable = 1, chưa quá hạn và không ESTOP. */
uint8_t Comm_GetCommand(uint32_t now_ms, Comm_Command_t *command);

uint8_t Comm_IsEstopActive(void);
uint8_t Comm_IsCommandTimedOut(uint32_t now_ms);

/* Trả về 0 nếu hàng đợi phát không đủ chỗ (khung bị bỏ). */
uint8_t Comm_SendState(const Protocol_State_t *state);

const volatile Comm_Stats_t *Comm_GetStats(void);

#endif

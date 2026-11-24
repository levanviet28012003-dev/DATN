#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

/*
 * Giao thức UART Pi4 <-> STM32, 115200 8N1.
 *
 * Khung: [0xAA 0x55][Len][Type][Seq][Payload: Len byte][CRC16]
 *  - Len: số byte payload (không tính header và CRC).
 *  - Seq: bộ đếm 8 bit của bên gửi, tăng 1 sau mỗi khung.
 *  - CRC16-CCITT (đa thức 0x1021, khởi tạo 0xFFFF, không đảo bit, không XOR cuối)
 *    tính trên Len, Type, Seq, Payload.
 *  - Mọi số nhiều byte, kể cả CRC, gửi theo little-endian.
 *
 * Logic ở đây không chạm phần cứng để test được trên máy tính.
 * Phía Pi: src/ackermann_hardware/scripts/stm32_bridge.py phải khớp từng byte.
 */

#define PROTOCOL_SYNC1                 0xAAU
#define PROTOCOL_SYNC2                 0x55U
#define PROTOCOL_HEADER_SIZE           5U
#define PROTOCOL_CRC_SIZE              2U
#define PROTOCOL_OVERHEAD              (PROTOCOL_HEADER_SIZE + PROTOCOL_CRC_SIZE)
#define PROTOCOL_MAX_PAYLOAD           16U
#define PROTOCOL_MAX_FRAME             (PROTOCOL_OVERHEAD + PROTOCOL_MAX_PAYLOAD)

/* Loại khung */
#define PROTOCOL_TYPE_CMD              0x01U  /* Pi -> STM32 */
#define PROTOCOL_TYPE_STATE            0x02U  /* STM32 -> Pi */
#define PROTOCOL_TYPE_ESTOP            0x03U  /* Pi -> STM32 */

/* CMD: int16 vận tốc mm/s, int16 góc lái 0,01 độ (dương = rẽ trái), uint8 enable */
#define PROTOCOL_CMD_PAYLOAD_SIZE      5U
/* STATE: int32 encoder trái, int32 encoder phải, uint16 điện áp mV, uint8 trạng thái */
#define PROTOCOL_STATE_PAYLOAD_SIZE    11U
/* ESTOP: uint8, 1 = kích hoạt dừng khẩn, 0 = nhả */
#define PROTOCOL_ESTOP_PAYLOAD_SIZE    1U

/* Các bit của byte trạng thái trong STATE */
#define PROTOCOL_STATUS_READY          (1U << 0)  /* Khởi tạo xong, không lỗi */
#define PROTOCOL_STATUS_FAULT          (1U << 1)  /* Lỗi phần cứng đã khóa, cần reset */
#define PROTOCOL_STATUS_ESTOP          (1U << 2)  /* Đang dừng khẩn theo lệnh ESTOP */
#define PROTOCOL_STATUS_CMD_TIMEOUT    (1U << 3)  /* Quá thời gian không nhận CMD hợp lệ */
#define PROTOCOL_STATUS_DRIVING        (1U << 4)  /* Đang thực thi CMD có enable = 1 */
#define PROTOCOL_STATUS_ENCODER_VALID  (1U << 5)
#define PROTOCOL_STATUS_VOLTAGE_VALID  (1U << 6)

typedef struct
{
    int16_t velocity_mm_s;
    int16_t steering_cdeg;
    uint8_t enable;
} Protocol_Cmd_t;

typedef struct
{
    int32_t encoder_left;
    int32_t encoder_right;
    uint16_t voltage_mv;
    uint8_t status;
} Protocol_State_t;

typedef struct
{
    uint8_t type;
    uint8_t seq;
    uint8_t length;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD];
} Protocol_Frame_t;

typedef enum
{
    PROTOCOL_RESULT_NONE = 0,      /* Chưa đủ khung */
    PROTOCOL_RESULT_FRAME,         /* Có khung hợp lệ trong parser->frame */
    PROTOCOL_RESULT_CRC_ERROR,
    PROTOCOL_RESULT_LENGTH_ERROR   /* Len lớn hơn PROTOCOL_MAX_PAYLOAD */
} Protocol_Result_t;

typedef struct
{
    uint8_t state;
    uint8_t index;
    uint16_t crc;
    uint16_t received_crc;
    Protocol_Frame_t frame;
} Protocol_Parser_t;

uint16_t Protocol_Crc16(const uint8_t *data, uint16_t length);

void Protocol_ParserInit(Protocol_Parser_t *parser);

/* Nạp từng byte nhận được. Khi trả về PROTOCOL_RESULT_FRAME, đọc parser->frame
 * trước khi nạp byte tiếp theo. */
Protocol_Result_t Protocol_ParseByte(Protocol_Parser_t *parser, uint8_t byte);

/* Đóng khung vào out; trả về số byte của khung, 0 nếu tham số sai hoặc out quá nhỏ. */
uint16_t Protocol_Encode(uint8_t type, uint8_t seq, const uint8_t *payload,
                         uint8_t length, uint8_t *out, uint16_t out_size);

/* Trả về 0 nếu khung không đúng loại hoặc sai độ dài payload. */
uint8_t Protocol_DecodeCmd(const Protocol_Frame_t *frame, Protocol_Cmd_t *cmd);
uint8_t Protocol_DecodeEstop(const Protocol_Frame_t *frame, uint8_t *engage);

void Protocol_PackState(const Protocol_State_t *state,
                        uint8_t payload[PROTOCOL_STATE_PAYLOAD_SIZE]);

#endif

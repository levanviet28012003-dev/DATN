/*
 * Test giao thức UART và lớp communication trên máy tính (gcc của host).
 * UART1 được thay bằng stub: byte "nhận" lấy từ rx_data, byte "phát" ghi vào tx_data.
 * Các khung mẫu sinh độc lập bằng binascii.crc_hqx của Python, dùng chung với
 * test phía Pi (src/ackermann_hardware/test/test_stm32_bridge.py).
 *
 * Chạy: make test
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "communication.h"
#include "protocol.h"
#include "UART.h"

/* ---------------- Stub UART1 ---------------- */
static uint8_t rx_data[1024];
static uint16_t rx_length;
static uint16_t rx_index;
static uint8_t tx_data[1024];
static uint16_t tx_length;
static uint16_t tx_capacity = sizeof(tx_data);

void UART1_Init(void)
{
    rx_length = rx_index = tx_length = 0U;
}

uint8_t UART1_ReadByte(uint8_t *byte)
{
    if (rx_index >= rx_length)
    {
        return 0U;
    }
    *byte = rx_data[rx_index++];
    return 1U;
}

uint8_t UART1_Write(const uint8_t *data, uint16_t length)
{
    if ((uint32_t)tx_length + length > tx_capacity)
    {
        return 0U;
    }
    memcpy(&tx_data[tx_length], data, length);
    tx_length = (uint16_t)(tx_length + length);
    return 1U;
}

static void feed(const uint8_t *data, uint16_t length)
{
    memcpy(&rx_data[rx_length], data, length);
    rx_length = (uint16_t)(rx_length + length);
}

/* ---------------- Khung mẫu ---------------- */
/* CMD seq 7: v = 150 mm/s, góc = -12,34 độ, enable = 1 */
static const uint8_t CMD_FRAME[] = {
    0xAA, 0x55, 0x05, 0x01, 0x07, 0x96, 0x00, 0x2E, 0xFB, 0x01, 0x72, 0xA0};
/* STATE seq 0: encoder -123456 / 987654, 7400 mV, trạng thái 0x61 */
static const uint8_t STATE_FRAME[] = {
    0xAA, 0x55, 0x0B, 0x02, 0x00, 0xC0, 0x1D, 0xFE, 0xFF, 0x06, 0x12, 0x0F,
    0x00, 0xE8, 0x1C, 0x61, 0xF6, 0x01};
/* ESTOP seq 9: kích hoạt */
static const uint8_t ESTOP_FRAME[] = {0xAA, 0x55, 0x01, 0x03, 0x09, 0x01, 0x9D, 0x01};

/* ---------------- Khung test ---------------- */
static int failures;
#define CHECK(cond)                                                          \
    do                                                                       \
    {                                                                        \
        if (!(cond))                                                         \
        {                                                                    \
            printf("  LOI %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                      \
        }                                                                    \
    } while (0)

static void make_cmd(uint8_t seq, int16_t v, int16_t cdeg, uint8_t enable,
                     uint8_t *out, uint16_t *size)
{
    uint8_t payload[PROTOCOL_CMD_PAYLOAD_SIZE] = {
        (uint8_t)v, (uint8_t)((uint16_t)v >> 8),
        (uint8_t)cdeg, (uint8_t)((uint16_t)cdeg >> 8), enable};
    *size = Protocol_Encode(PROTOCOL_TYPE_CMD, seq, payload, sizeof(payload),
                            out, PROTOCOL_MAX_FRAME);
}

static void make_estop(uint8_t seq, uint8_t engage, uint8_t *out, uint16_t *size)
{
    *size = Protocol_Encode(PROTOCOL_TYPE_ESTOP, seq, &engage, 1U, out, PROTOCOL_MAX_FRAME);
}

static void test_crc_reference(void)
{
    CHECK(Protocol_Crc16((const uint8_t *)"123456789", 9U) == 0x29B1U);
}

static void test_encode_matches_reference(void)
{
    uint8_t out[PROTOCOL_MAX_FRAME];
    uint16_t size;
    Protocol_State_t state = {-123456, 987654, 7400U, 0x61U};
    uint8_t payload[PROTOCOL_STATE_PAYLOAD_SIZE];

    make_cmd(7U, 150, -1234, 1U, out, &size);
    CHECK(size == sizeof(CMD_FRAME) && memcmp(out, CMD_FRAME, size) == 0);

    Protocol_PackState(&state, payload);
    size = Protocol_Encode(PROTOCOL_TYPE_STATE, 0U, payload, sizeof(payload), out, sizeof(out));
    CHECK(size == sizeof(STATE_FRAME) && memcmp(out, STATE_FRAME, size) == 0);

    make_estop(9U, 1U, out, &size);
    CHECK(size == sizeof(ESTOP_FRAME) && memcmp(out, ESTOP_FRAME, size) == 0);

    CHECK(Protocol_Encode(1U, 0U, payload, PROTOCOL_MAX_PAYLOAD + 1U, out, sizeof(out)) == 0U);
    CHECK(Protocol_Encode(1U, 0U, payload, 4U, out, 10U) == 0U);
}

static void test_parser_resyncs_after_noise_and_corruption(void)
{
    Protocol_Parser_t parser;
    Protocol_Cmd_t cmd;
    uint8_t stream[64];
    uint16_t n = 0U;
    int frames = 0, crc_errors = 0, length_errors = 0;

    /* Rác, 0xAA lặp, khung lỗi CRC, Len quá lớn, rồi khung đúng */
    const uint8_t noise[] = {0x00, 0x13, 0xAA, 0xAA};
    memcpy(&stream[n], noise, sizeof(noise));
    n += sizeof(noise);
    stream[n++] = 0x55;                       /* 0xAA 0x55 hợp lệ sau 0xAA lặp */
    memcpy(&stream[n], &CMD_FRAME[2], sizeof(CMD_FRAME) - 2U);
    stream[n + 5] ^= 0x01U;                   /* hỏng một bit payload */
    n += sizeof(CMD_FRAME) - 2U;
    stream[n++] = 0xAA;
    stream[n++] = 0x55;
    stream[n++] = 0x40;                       /* Len 64 > tối đa */
    memcpy(&stream[n], CMD_FRAME, sizeof(CMD_FRAME));
    n += sizeof(CMD_FRAME);

    Protocol_ParserInit(&parser);
    for (uint16_t i = 0U; i < n; ++i)
    {
        Protocol_Result_t result = Protocol_ParseByte(&parser, stream[i]);
        if (result == PROTOCOL_RESULT_FRAME)
        {
            ++frames;
            CHECK(Protocol_DecodeCmd(&parser.frame, &cmd));
            CHECK(parser.frame.seq == 7U);
            CHECK(cmd.velocity_mm_s == 150 && cmd.steering_cdeg == -1234 && cmd.enable == 1U);
        }
        crc_errors += (result == PROTOCOL_RESULT_CRC_ERROR);
        length_errors += (result == PROTOCOL_RESULT_LENGTH_ERROR);
    }
    CHECK(frames == 1 && crc_errors == 1 && length_errors == 1);
}

static void test_command_watchdog_and_conversion(void)
{
    uint8_t frame[PROTOCOL_MAX_FRAME];
    uint16_t size;
    Comm_Command_t command;

    Comm_Init();
    CHECK(!Comm_GetCommand(0U, &command));
    CHECK(Comm_IsCommandTimedOut(0U));

    /* Gửi từng byte qua nhiều lần Poll để giả lập khung đến rời rạc */
    for (uint16_t i = 0U; i < sizeof(CMD_FRAME); ++i)
    {
        feed(&CMD_FRAME[i], 1U);
        Comm_Poll(1000U);
    }
    CHECK(Comm_GetCommand(1000U, &command));
    CHECK(fabsf(command.velocity - 0.150f) < 1e-6f);
    CHECK(fabsf(command.steering_angle - (-12.34f * 3.14159265f / 180.0f)) < 1e-5f);
    CHECK(Comm_GetCommand(1000U + COMM_CMD_TIMEOUT_MS, &command));
    CHECK(!Comm_GetCommand(1000U + COMM_CMD_TIMEOUT_MS + 1U, &command));
    CHECK(Comm_IsCommandTimedOut(1000U + COMM_CMD_TIMEOUT_MS + 1U));

    /* enable = 0 nghĩa là dừng dù lệnh còn mới */
    make_cmd(8U, 100, 0, 0U, frame, &size);
    feed(frame, size);
    Comm_Poll(2000U);
    CHECK(!Comm_GetCommand(2000U, &command));
    CHECK(!Comm_IsCommandTimedOut(2000U));
    CHECK(Comm_GetStats()->cmd_frames == 2U && Comm_GetStats()->seq_gaps == 0U);
}

static void test_watchdog_survives_millis_wraparound(void)
{
    uint8_t frame[PROTOCOL_MAX_FRAME];
    uint16_t size;
    Comm_Command_t command;

    Comm_Init();
    make_cmd(0U, 50, 0, 1U, frame, &size);
    feed(frame, size);
    Comm_Poll(0xFFFFFF00U);
    CHECK(Comm_GetCommand(0x00000010U, &command));        /* 0x110 ms = 272 ms sau */
    CHECK(!Comm_GetCommand(0x00000100U, &command));       /* 512 ms sau */
}

static void test_estop_latches_and_requires_new_command(void)
{
    uint8_t frame[PROTOCOL_MAX_FRAME];
    uint16_t size;
    Comm_Command_t command;

    Comm_Init();
    make_cmd(1U, 100, 500, 1U, frame, &size);
    feed(frame, size);
    Comm_Poll(10U);
    CHECK(Comm_GetCommand(10U, &command));

    feed(ESTOP_FRAME, sizeof(ESTOP_FRAME));
    Comm_Poll(20U);
    CHECK(Comm_IsEstopActive());
    CHECK(!Comm_GetCommand(20U, &command));

    /* CMD trong lúc ESTOP bị bỏ qua */
    make_cmd(10U, 100, 500, 1U, frame, &size);
    feed(frame, size);
    Comm_Poll(30U);
    CHECK(!Comm_GetCommand(30U, &command));

    /* Nhả ESTOP: chưa chạy cho tới khi có CMD mới */
    make_estop(11U, 0U, frame, &size);
    feed(frame, size);
    Comm_Poll(40U);
    CHECK(!Comm_IsEstopActive());
    CHECK(!Comm_GetCommand(40U, &command));

    make_cmd(12U, 100, 500, 1U, frame, &size);
    feed(frame, size);
    Comm_Poll(50U);
    CHECK(Comm_GetCommand(50U, &command));

    /* Seq 1 -> 9 là một lần đứt quãng */
    CHECK(Comm_GetStats()->seq_gaps == 1U);
    CHECK(Comm_GetStats()->estop_frames == 2U);
}

static void test_wrong_length_and_unknown_type_are_counted(void)
{
    uint8_t frame[PROTOCOL_MAX_FRAME];
    uint8_t payload[4] = {0};
    uint16_t size;
    Comm_Command_t command;

    Comm_Init();
    size = Protocol_Encode(PROTOCOL_TYPE_CMD, 0U, payload, 4U, frame, sizeof(frame));
    feed(frame, size);
    size = Protocol_Encode(0x7FU, 1U, payload, 2U, frame, sizeof(frame));
    feed(frame, size);
    Comm_Poll(0U);
    CHECK(!Comm_GetCommand(0U, &command));
    CHECK(Comm_GetStats()->length_errors == 1U);
    CHECK(Comm_GetStats()->unknown_frames == 1U);
    CHECK(Comm_GetStats()->rx_frames == 2U);
}

static void test_send_state_and_tx_full(void)
{
    Protocol_State_t state = {-123456, 987654, 7400U, 0x61U};

    Comm_Init();
    CHECK(Comm_SendState(&state));
    CHECK(tx_length == sizeof(STATE_FRAME) && memcmp(tx_data, STATE_FRAME, tx_length) == 0);
    CHECK(Comm_SendState(&state));
    CHECK(tx_data[sizeof(STATE_FRAME) + 4U] == 1U);   /* Seq tăng */

    tx_capacity = (uint16_t)(tx_length + 5U);          /* Không đủ chỗ cho một khung */
    CHECK(!Comm_SendState(&state));
    CHECK(Comm_GetStats()->tx_dropped == 1U && Comm_GetStats()->tx_frames == 2U);
    tx_capacity = sizeof(tx_data);
}

int main(void)
{
    test_crc_reference();
    test_encode_matches_reference();
    test_parser_resyncs_after_noise_and_corruption();
    test_command_watchdog_and_conversion();
    test_watchdog_survives_millis_wraparound();
    test_estop_latches_and_requires_new_command();
    test_wrong_length_and_unknown_type_are_counted();
    test_send_state_and_tx_full();

    if (failures)
    {
        printf("THAT BAI: %d kiem tra loi\n", failures);
        return 1;
    }
    printf("Tat ca test giao tiep UART deu dat\n");
    return 0;
}

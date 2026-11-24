#include "communication.h"
#include "UART.h"

#define COMM_PI 3.14159265358979323846f

static Protocol_Parser_t parser;
static Protocol_Cmd_t last_command;
static uint8_t has_command;
static uint8_t estop_active;
static uint8_t has_rx_seq;
static uint8_t last_rx_seq;
static uint8_t tx_seq;

// Watch: bộ đếm đường truyền.
volatile Comm_Stats_t comm_status;

static void Comm_TrackSequence(uint8_t seq)
{
    if (has_rx_seq && seq != (uint8_t)(last_rx_seq + 1U))
    {
        ++comm_status.seq_gaps;
    }
    has_rx_seq = 1U;
    last_rx_seq = seq;
}

static void Comm_HandleFrame(const Protocol_Frame_t *frame, uint32_t now_ms)
{
    Protocol_Cmd_t command;
    uint8_t engage;

    ++comm_status.rx_frames;
    Comm_TrackSequence(frame->seq);

    if (frame->type == PROTOCOL_TYPE_CMD)
    {
        if (!Protocol_DecodeCmd(frame, &command))
        {
            ++comm_status.length_errors;
            return;
        }
        ++comm_status.cmd_frames;
        // Trong lúc ESTOP vẫn ghi nhận đường truyền còn sống nhưng không nhận lệnh chạy.
        if (!estop_active)
        {
            last_command = command;
            has_command = 1U;
        }
        comm_status.last_cmd_ms = now_ms;
    }
    else if (frame->type == PROTOCOL_TYPE_ESTOP)
    {
        if (!Protocol_DecodeEstop(frame, &engage))
        {
            ++comm_status.length_errors;
            return;
        }
        ++comm_status.estop_frames;
        if (engage)
        {
            estop_active = 1U;
            has_command = 0U;
        }
        else if (estop_active)
        {
            // Nhả ESTOP không khôi phục lệnh cũ; Pi phải gửi CMD mới.
            estop_active = 0U;
            has_command = 0U;
        }
    }
    else
    {
        ++comm_status.unknown_frames;
    }
}

void Comm_Init(void)
{
    Protocol_ParserInit(&parser);
    last_command = (Protocol_Cmd_t){0};
    has_command = 0U;
    estop_active = 0U;
    has_rx_seq = 0U;
    last_rx_seq = 0U;
    tx_seq = 0U;
    comm_status = (Comm_Stats_t){0};
    UART1_Init();
}

void Comm_Poll(uint32_t now_ms)
{
    uint8_t byte;
    while (UART1_ReadByte(&byte))
    {
        switch (Protocol_ParseByte(&parser, byte))
        {
        case PROTOCOL_RESULT_FRAME:
            Comm_HandleFrame(&parser.frame, now_ms);
            break;
        case PROTOCOL_RESULT_CRC_ERROR:
            ++comm_status.crc_errors;
            break;
        case PROTOCOL_RESULT_LENGTH_ERROR:
            ++comm_status.length_errors;
            break;
        default:
            break;
        }
    }
}

uint8_t Comm_IsEstopActive(void)
{
    return estop_active;
}

uint8_t Comm_IsCommandTimedOut(uint32_t now_ms)
{
    // Phép trừ không dấu vẫn đúng khi bộ đếm mili giây tràn.
    return (uint8_t)(!has_command ||
                     (uint32_t)(now_ms - comm_status.last_cmd_ms) > COMM_CMD_TIMEOUT_MS);
}

uint8_t Comm_GetCommand(uint32_t now_ms, Comm_Command_t *command)
{
    if (estop_active || Comm_IsCommandTimedOut(now_ms) || !last_command.enable)
    {
        return 0U;
    }
    command->velocity = (float)last_command.velocity_mm_s * 0.001f;
    command->steering_angle = (float)last_command.steering_cdeg * (COMM_PI / 18000.0f);
    return 1U;
}

uint8_t Comm_SendState(const Protocol_State_t *state)
{
    uint8_t payload[PROTOCOL_STATE_PAYLOAD_SIZE];
    uint8_t frame[PROTOCOL_OVERHEAD + PROTOCOL_STATE_PAYLOAD_SIZE];
    uint16_t size;

    Protocol_PackState(state, payload);
    size = Protocol_Encode(PROTOCOL_TYPE_STATE, tx_seq, payload,
                           PROTOCOL_STATE_PAYLOAD_SIZE, frame, sizeof(frame));
    if (size == 0U || !UART1_Write(frame, size))
    {
        ++comm_status.tx_dropped;
        return 0U;
    }
    ++tx_seq;
    ++comm_status.tx_frames;
    return 1U;
}

const volatile Comm_Stats_t *Comm_GetStats(void)
{
    return &comm_status;
}

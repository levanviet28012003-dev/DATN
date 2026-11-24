#include "protocol.h"

enum
{
    PARSER_WAIT_SYNC1 = 0,
    PARSER_WAIT_SYNC2,
    PARSER_LENGTH,
    PARSER_TYPE,
    PARSER_SEQ,
    PARSER_PAYLOAD,
    PARSER_CRC_LOW,
    PARSER_CRC_HIGH
};

static uint16_t Protocol_CrcUpdate(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)((uint16_t)byte << 8);
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
        if (crc & 0x8000U)
        {
            crc = (uint16_t)((crc << 1) ^ 0x1021U);
        }
        else
        {
            crc = (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t Protocol_ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void Protocol_WriteU16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

static void Protocol_WriteU32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    data[2] = (uint8_t)((value >> 16) & 0xFFU);
    data[3] = (uint8_t)(value >> 24);
}

uint16_t Protocol_Crc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < length; ++i)
    {
        crc = Protocol_CrcUpdate(crc, data[i]);
    }
    return crc;
}

void Protocol_ParserInit(Protocol_Parser_t *parser)
{
    parser->state = PARSER_WAIT_SYNC1;
    parser->index = 0U;
    parser->crc = 0xFFFFU;
    parser->received_crc = 0U;
    parser->frame.type = 0U;
    parser->frame.seq = 0U;
    parser->frame.length = 0U;
}

Protocol_Result_t Protocol_ParseByte(Protocol_Parser_t *parser, uint8_t byte)
{
    switch (parser->state)
    {
    case PARSER_WAIT_SYNC1:
        if (byte == PROTOCOL_SYNC1)
        {
            parser->state = PARSER_WAIT_SYNC2;
        }
        break;

    case PARSER_WAIT_SYNC2:
        if (byte == PROTOCOL_SYNC2)
        {
            parser->state = PARSER_LENGTH;
        }
        else if (byte != PROTOCOL_SYNC1)
        {
            // Giữ trạng thái khi gặp 0xAA lặp lại, nếu không thì tìm lại đầu khung.
            parser->state = PARSER_WAIT_SYNC1;
        }
        break;

    case PARSER_LENGTH:
        if (byte > PROTOCOL_MAX_PAYLOAD)
        {
            parser->state = (byte == PROTOCOL_SYNC1) ? PARSER_WAIT_SYNC2 : PARSER_WAIT_SYNC1;
            return PROTOCOL_RESULT_LENGTH_ERROR;
        }
        parser->frame.length = byte;
        parser->crc = Protocol_CrcUpdate(0xFFFFU, byte);
        parser->state = PARSER_TYPE;
        break;

    case PARSER_TYPE:
        parser->frame.type = byte;
        parser->crc = Protocol_CrcUpdate(parser->crc, byte);
        parser->state = PARSER_SEQ;
        break;

    case PARSER_SEQ:
        parser->frame.seq = byte;
        parser->crc = Protocol_CrcUpdate(parser->crc, byte);
        parser->index = 0U;
        parser->state = (parser->frame.length == 0U) ? PARSER_CRC_LOW : PARSER_PAYLOAD;
        break;

    case PARSER_PAYLOAD:
        parser->frame.payload[parser->index++] = byte;
        parser->crc = Protocol_CrcUpdate(parser->crc, byte);
        if (parser->index >= parser->frame.length)
        {
            parser->state = PARSER_CRC_LOW;
        }
        break;

    case PARSER_CRC_LOW:
        parser->received_crc = byte;
        parser->state = PARSER_CRC_HIGH;
        break;

    case PARSER_CRC_HIGH:
        parser->received_crc |= (uint16_t)((uint16_t)byte << 8);
        parser->state = PARSER_WAIT_SYNC1;
        if (parser->received_crc != parser->crc)
        {
            return PROTOCOL_RESULT_CRC_ERROR;
        }
        return PROTOCOL_RESULT_FRAME;

    default:
        parser->state = PARSER_WAIT_SYNC1;
        break;
    }
    return PROTOCOL_RESULT_NONE;
}

uint16_t Protocol_Encode(uint8_t type, uint8_t seq, const uint8_t *payload,
                         uint8_t length, uint8_t *out, uint16_t out_size)
{
    uint16_t frame_size = (uint16_t)(PROTOCOL_OVERHEAD + length);
    uint16_t crc;

    if (length > PROTOCOL_MAX_PAYLOAD || out == 0 || out_size < frame_size ||
        (length > 0U && payload == 0))
    {
        return 0U;
    }

    out[0] = PROTOCOL_SYNC1;
    out[1] = PROTOCOL_SYNC2;
    out[2] = length;
    out[3] = type;
    out[4] = seq;
    for (uint8_t i = 0U; i < length; ++i)
    {
        out[PROTOCOL_HEADER_SIZE + i] = payload[i];
    }
    // CRC bắt đầu từ Len, bỏ qua hai byte đồng bộ.
    crc = Protocol_Crc16(&out[2], (uint16_t)(3U + length));
    Protocol_WriteU16(&out[PROTOCOL_HEADER_SIZE + length], crc);
    return frame_size;
}

uint8_t Protocol_DecodeCmd(const Protocol_Frame_t *frame, Protocol_Cmd_t *cmd)
{
    if (frame->type != PROTOCOL_TYPE_CMD || frame->length != PROTOCOL_CMD_PAYLOAD_SIZE)
    {
        return 0U;
    }
    cmd->velocity_mm_s = (int16_t)Protocol_ReadU16(&frame->payload[0]);
    cmd->steering_cdeg = (int16_t)Protocol_ReadU16(&frame->payload[2]);
    cmd->enable = frame->payload[4];
    return 1U;
}

uint8_t Protocol_DecodeEstop(const Protocol_Frame_t *frame, uint8_t *engage)
{
    if (frame->type != PROTOCOL_TYPE_ESTOP || frame->length != PROTOCOL_ESTOP_PAYLOAD_SIZE)
    {
        return 0U;
    }
    *engage = (uint8_t)(frame->payload[0] != 0U);
    return 1U;
}

void Protocol_PackState(const Protocol_State_t *state,
                        uint8_t payload[PROTOCOL_STATE_PAYLOAD_SIZE])
{
    Protocol_WriteU32(&payload[0], (uint32_t)state->encoder_left);
    Protocol_WriteU32(&payload[4], (uint32_t)state->encoder_right);
    Protocol_WriteU16(&payload[8], state->voltage_mv);
    payload[10] = state->status;
}

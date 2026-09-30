#include "can_protocol.h"

static int16_t decode_i16_le(const uint8_t *data)
{
    const uint16_t bits = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return (int16_t)bits;
}

static void encode_i32_le(int32_t value, uint8_t *data)
{
    const uint32_t bits = (uint32_t)value;
    data[0] = (uint8_t)(bits & UINT32_C(0xff));
    data[1] = (uint8_t)((bits >> 8) & UINT32_C(0xff));
    data[2] = (uint8_t)((bits >> 16) & UINT32_C(0xff));
    data[3] = (uint8_t)((bits >> 24) & UINT32_C(0xff));
}

bool can_protocol_is_request_id(uint32_t can_id)
{
    return can_id == CAN_ID_ADD_REQUEST || can_id == CAN_ID_SUB_REQUEST;
}

can_operation_t can_protocol_operation_from_id(uint32_t can_id)
{
    return can_id == CAN_ID_ADD_REQUEST ? CAN_OPERATION_ADD : CAN_OPERATION_SUB;
}

can_status_t can_protocol_decode_request(const uint8_t data[CAN_CALCULATOR_DLC],
                                         can_request_t *request)
{
    request->version = data[0];
    request->sequence = data[1];
    request->operand_a = 0;
    request->operand_b = 0;

    if (data[0] != CAN_PROTOCOL_VERSION) {
        return CAN_STATUS_UNSUPPORTED_VERSION;
    }
    if (data[6] != 0U || data[7] != 0U) {
        return CAN_STATUS_INVALID_FORMAT;
    }

    request->operand_a = decode_i16_le(&data[2]);
    request->operand_b = decode_i16_le(&data[4]);
    return CAN_STATUS_OK;
}

void can_protocol_encode_result(const can_result_t *result,
                                uint8_t data[CAN_CALCULATOR_DLC])
{
    data[0] = result->version;
    data[1] = result->sequence;
    data[2] = (uint8_t)result->operation;
    data[3] = (uint8_t)result->status;
    encode_i32_le(result->status == CAN_STATUS_OK ? result->result : 0,
                  &data[4]);
}

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_CALCULATOR_BITRATE        UINT32_C(500000)
#define CAN_ID_RESULT                 UINT32_C(0x001)
#define CAN_ID_ADD_REQUEST            UINT32_C(0x002)
#define CAN_ID_SUB_REQUEST            UINT32_C(0x003)
#define CAN_CALCULATOR_DLC            UINT8_C(8)
#define CAN_PROTOCOL_VERSION          UINT8_C(0x01)

typedef enum {
    CAN_OPERATION_ADD = 0x02,
    CAN_OPERATION_SUB = 0x03,
} can_operation_t;

typedef enum {
    CAN_STATUS_OK = 0x00,
    CAN_STATUS_INVALID_FORMAT = 0x01,
    CAN_STATUS_UNSUPPORTED_VERSION = 0x02,
    CAN_STATUS_INTERNAL_ERROR = 0x03,
} can_status_t;

typedef struct {
    uint8_t version;
    uint8_t sequence;
    int16_t operand_a;
    int16_t operand_b;
} can_request_t;

typedef struct {
    uint8_t version;
    uint8_t sequence;
    can_operation_t operation;
    can_status_t status;
    int32_t result;
} can_result_t;

bool can_protocol_is_request_id(uint32_t can_id);
can_operation_t can_protocol_operation_from_id(uint32_t can_id);

/* Caller must verify DLC == CAN_CALCULATOR_DLC before calling this function. */
can_status_t can_protocol_decode_request(const uint8_t data[CAN_CALCULATOR_DLC],
                                         can_request_t *request);

void can_protocol_encode_result(const can_result_t *result,
                                uint8_t data[CAN_CALCULATOR_DLC]);

#ifdef __cplusplus
}
#endif

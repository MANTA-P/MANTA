#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "can_protocol.h"

typedef struct {
    uint32_t request_id;
    uint8_t request[CAN_CALCULATOR_DLC];
    uint8_t expected[CAN_CALCULATOR_DLC];
} test_vector_t;

static int32_t calculate(can_operation_t operation, const can_request_t *request)
{
    const int32_t a = (int32_t)request->operand_a;
    const int32_t b = (int32_t)request->operand_b;
    return operation == CAN_OPERATION_ADD ? a + b : a - b;
}

int main(void)
{
    static const test_vector_t vectors[] = {
        {0x002, {0x01,0x01,0x01,0x00,0x02,0x00,0x00,0x00},
                {0x01,0x01,0x02,0x00,0x03,0x00,0x00,0x00}},
        {0x002, {0x01,0x2A,0xE8,0x03,0x06,0xFF,0x00,0x00},
                {0x01,0x2A,0x02,0x00,0xEE,0x02,0x00,0x00}},
        {0x003, {0x01,0x2B,0x0A,0x00,0x19,0x00,0x00,0x00},
                {0x01,0x2B,0x03,0x00,0xF1,0xFF,0xFF,0xFF}},
        {0x002, {0x01,0xFE,0xFF,0x7F,0xFF,0x7F,0x00,0x00},
                {0x01,0xFE,0x02,0x00,0xFE,0xFF,0x00,0x00}},
        {0x003, {0x01,0xFF,0xFF,0x7F,0x00,0x80,0x00,0x00},
                {0x01,0xFF,0x03,0x00,0xFF,0xFF,0x00,0x00}},
        {0x002, {0x01,0x00,0x01,0x00,0x01,0x00,0x00,0x00},
                {0x01,0x00,0x02,0x00,0x02,0x00,0x00,0x00}},
    };

    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); ++i) {
        can_request_t request;
        assert(can_protocol_decode_request(vectors[i].request, &request) ==
               CAN_STATUS_OK);
        const can_operation_t operation =
            can_protocol_operation_from_id(vectors[i].request_id);
        const can_result_t result = {
            .version = CAN_PROTOCOL_VERSION,
            .sequence = request.sequence,
            .operation = operation,
            .status = CAN_STATUS_OK,
            .result = calculate(operation, &request),
        };
        uint8_t encoded[CAN_CALCULATOR_DLC];
        can_protocol_encode_result(&result, encoded);
        assert(memcmp(encoded, vectors[i].expected, sizeof(encoded)) == 0);
    }

    uint8_t bad_version[CAN_CALCULATOR_DLC] = {0x02,0x55,0,0,0,0,0,0};
    can_request_t request;
    assert(can_protocol_decode_request(bad_version, &request) ==
           CAN_STATUS_UNSUPPORTED_VERSION);
    assert(request.sequence == 0x55);

    uint8_t bad_reserved[CAN_CALCULATOR_DLC] = {0x01,0x56,0,0,0,0,1,0};
    assert(can_protocol_decode_request(bad_reserved, &request) ==
           CAN_STATUS_INVALID_FORMAT);

    puts("all CAN protocol tests passed");
    return 0;
}

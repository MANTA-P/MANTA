#include "calculator_node.h"

#include <inttypes.h>

#include "esp_err.h"
#include "esp_log.h"

#include "can_protocol.h"
#include "can_transport.h"

static const char *TAG = "calculator_node";
static const uint32_t RX_WAIT_MS = 50;
static const uint32_t TX_WAIT_MS = 100;

static const char *operation_name(can_operation_t operation)
{
    return operation == CAN_OPERATION_ADD ? "ADD" : "SUB";
}

static void process_frame(const can_transport_frame_t *frame)
{
    const twai_frame_header_t *header = &frame->header;
    if (header->ide) {
        ESP_LOGW(TAG, "discarded extended frame ID=0x%" PRIx32, header->id);
        return;
    }
    if (header->rtr) {
        ESP_LOGW(TAG, "discarded RTR frame ID=0x%03" PRIx32, header->id);
        return;
    }
    if (header->fdf) {
        ESP_LOGW(TAG, "discarded CAN FD frame ID=0x%03" PRIx32, header->id);
        return;
    }
    if (!can_protocol_is_request_id(header->id)) {
        ESP_LOGW(TAG, "discarded unknown ID=0x%03" PRIx32, header->id);
        return;
    }
    if (header->dlc != CAN_CALCULATOR_DLC) {
        ESP_LOGW(TAG, "discarded ID=0x%03" PRIx32 ": DLC=%u, expected=%u",
                 header->id, header->dlc, CAN_CALCULATOR_DLC);
        return;
    }

    can_request_t request;
    const can_status_t status = can_protocol_decode_request(frame->data, &request);
    const can_operation_t operation = can_protocol_operation_from_id(header->id);
    can_result_t response = {
        .version = CAN_PROTOCOL_VERSION,
        .sequence = request.sequence,
        .operation = operation,
        .status = status,
        .result = 0,
    };

    if (status == CAN_STATUS_OK) {
        const int32_t operand_a = (int32_t)request.operand_a;
        const int32_t operand_b = (int32_t)request.operand_b;
        response.result = operation == CAN_OPERATION_ADD
                              ? operand_a + operand_b
                              : operand_a - operand_b;
        ESP_LOGI(TAG,
                 "RX ID=0x%03" PRIx32 " seq=%u op=%s: %" PRId16 " %c %" PRId16,
                 header->id, request.sequence, operation_name(operation),
                 request.operand_a,
                 operation == CAN_OPERATION_ADD ? '+' : '-',
                 request.operand_b);
    } else {
        ESP_LOGW(TAG, "invalid request ID=0x%03" PRIx32
                 " seq=%u op=%s status=0x%02x",
                 header->id, request.sequence, operation_name(operation), status);
    }

    uint8_t payload[CAN_CALCULATOR_DLC];
    can_protocol_encode_result(&response, payload);
    const esp_err_t send_err = can_transport_send_standard(
        CAN_ID_RESULT, payload, CAN_CALCULATOR_DLC, TX_WAIT_MS);
    if (send_err != ESP_OK) {
        ESP_LOGE(TAG, "result TX failed ID=0x%03" PRIx32
                 " seq=%u op=%s: %s",
                 CAN_ID_RESULT, request.sequence, operation_name(operation),
                 esp_err_to_name(send_err));
        return;
    }

    if (status == CAN_STATUS_OK) {
        ESP_LOGI(TAG, "TX complete ID=0x%03" PRIx32
                 " seq=%u op=%s result=%" PRId32,
                 CAN_ID_RESULT, request.sequence, operation_name(operation),
                 response.result);
    } else {
        ESP_LOGI(TAG, "TX complete ID=0x%03" PRIx32
                 " seq=%u op=%s status=0x%02x result=0",
                 CAN_ID_RESULT, request.sequence, operation_name(operation), status);
    }
}

void calculator_node_run(void)
{
    while (true) {
        can_transport_frame_t frame;
        if (can_transport_receive(&frame, RX_WAIT_MS)) {
            process_frame(&frame);
        }
        can_transport_process_events();
    }
}

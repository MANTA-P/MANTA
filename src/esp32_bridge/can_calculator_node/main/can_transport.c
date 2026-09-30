#include "can_transport.h"

#include <inttypes.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "sdkconfig.h"

#include "can_protocol.h"

static const char *TAG = "can_transport";

typedef enum {
    TRANSPORT_EVENT_STATE,
    TRANSPORT_EVENT_ERROR,
} transport_event_kind_t;

typedef struct {
    transport_event_kind_t kind;
    union {
        struct {
            twai_error_state_t old_state;
            twai_error_state_t new_state;
        } state;
        uint32_t error_flags;
    } value;
} transport_event_t;

static twai_node_handle_t s_node;
static QueueHandle_t s_rx_queue;
static QueueHandle_t s_event_queue;
static SemaphoreHandle_t s_tx_mutex;
static SemaphoreHandle_t s_tx_done;
static twai_frame_t s_tx_frame;
static uint8_t s_tx_data[TWAI_FRAME_MAX_LEN];
static volatile bool s_tx_pending;
static volatile bool s_tx_success;

static bool IRAM_ATTR on_rx_done(twai_node_handle_t node,
                                 const twai_rx_done_event_data_t *event,
                                 void *user_ctx)
{
    (void)event;
    (void)user_ctx;

    can_transport_frame_t item = {0};
    twai_frame_t rx_frame = {
        .buffer = item.data,
        .buffer_len = sizeof(item.data),
    };
    if (twai_node_receive_from_isr(node, &rx_frame) != ESP_OK) {
        return false;
    }
    item.header = rx_frame.header;

    BaseType_t task_woken = pdFALSE;
    if (xQueueSendFromISR(s_rx_queue, &item, &task_woken) != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "RX queue full; frame dropped");
    }
    return task_woken == pdTRUE;
}

static bool IRAM_ATTR on_tx_done(twai_node_handle_t node,
                                 const twai_tx_done_event_data_t *event,
                                 void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    s_tx_success = event->is_tx_success;
    s_tx_pending = false;
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_tx_done, &task_woken);
    return task_woken == pdTRUE;
}

static bool IRAM_ATTR on_state_change(twai_node_handle_t node,
                                      const twai_state_change_event_data_t *event,
                                      void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    const transport_event_t item = {
        .kind = TRANSPORT_EVENT_STATE,
        .value.state = {
            .old_state = event->old_sta,
            .new_state = event->new_sta,
        },
    };
    BaseType_t task_woken = pdFALSE;
    xQueueSendFromISR(s_event_queue, &item, &task_woken);
    return task_woken == pdTRUE;
}

static bool IRAM_ATTR on_error(twai_node_handle_t node,
                               const twai_error_event_data_t *event,
                               void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    const transport_event_t item = {
        .kind = TRANSPORT_EVENT_ERROR,
        .value.error_flags = event->err_flags.val,
    };
    BaseType_t task_woken = pdFALSE;
    xQueueSendFromISR(s_event_queue, &item, &task_woken);
    return task_woken == pdTRUE;
}

esp_err_t can_transport_init(void)
{
    s_rx_queue = xQueueCreate(CONFIG_CAN_CALCULATOR_RX_QUEUE_DEPTH,
                              sizeof(can_transport_frame_t));
    s_event_queue = xQueueCreate(16, sizeof(transport_event_t));
    s_tx_mutex = xSemaphoreCreateMutex();
    s_tx_done = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_rx_queue && s_event_queue && s_tx_mutex && s_tx_done,
                        ESP_ERR_NO_MEM, TAG, "failed to allocate RTOS objects");

    const twai_onchip_node_config_t config = {
        .io_cfg = {
            .tx = CONFIG_CAN_CALCULATOR_TX_GPIO,
            .rx = CONFIG_CAN_CALCULATOR_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = CAN_CALCULATOR_BITRATE,
        },
        .fail_retry_cnt = -1,
        .tx_queue_depth = 4,
    };
    ESP_RETURN_ON_ERROR(twai_new_node_onchip(&config, &s_node), TAG,
                        "failed to create TWAI node");

    const twai_event_callbacks_t callbacks = {
        .on_tx_done = on_tx_done,
        .on_rx_done = on_rx_done,
        .on_state_change = on_state_change,
        .on_error = on_error,
    };
    ESP_RETURN_ON_ERROR(twai_node_register_event_callbacks(s_node, &callbacks, NULL),
                        TAG, "failed to register TWAI callbacks");
    ESP_RETURN_ON_ERROR(twai_node_enable(s_node), TAG, "failed to enable TWAI node");
    return ESP_OK;
}

bool can_transport_receive(can_transport_frame_t *frame, uint32_t timeout_ms)
{
    return xQueueReceive(s_rx_queue, frame, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

esp_err_t can_transport_send_standard(uint32_t can_id,
                                      const uint8_t *data,
                                      uint8_t data_len,
                                      uint32_t timeout_ms)
{
    if (data == NULL || data_len > TWAI_FRAME_MAX_LEN ||
        can_id > TWAI_STD_ID_MASK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t result = ESP_OK;
    if (s_tx_pending &&
        xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        result = ESP_ERR_TIMEOUT;
        goto out;
    }
    while (xSemaphoreTake(s_tx_done, 0) == pdTRUE) {
    }

    memcpy(s_tx_data, data, data_len);
    s_tx_frame = (twai_frame_t) {
        .header = {
            .id = can_id,
            .dlc = data_len,
        },
        .buffer = s_tx_data,
        .buffer_len = data_len,
    };
    s_tx_success = false;
    s_tx_pending = true;
    result = twai_node_transmit(s_node, &s_tx_frame, (int)timeout_ms);
    if (result != ESP_OK) {
        s_tx_pending = false;
        goto out;
    }
    if (xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        result = ESP_ERR_TIMEOUT;
        goto out;
    }
    result = s_tx_success ? ESP_OK : ESP_FAIL;

out:
    xSemaphoreGive(s_tx_mutex);
    return result;
}

void can_transport_process_events(void)
{
    transport_event_t event;
    while (xQueueReceive(s_event_queue, &event, 0) == pdTRUE) {
        if (event.kind == TRANSPORT_EVENT_ERROR) {
            ESP_LOGW(TAG, "TWAI error flags=0x%" PRIx32,
                     event.value.error_flags);
            continue;
        }

        ESP_LOGW(TAG, "TWAI state changed: %d -> %d",
                 event.value.state.old_state, event.value.state.new_state);
        if (event.value.state.new_state == TWAI_ERROR_BUS_OFF) {
            twai_node_status_t status;
            const esp_err_t info_err = twai_node_get_info(s_node, &status, NULL);
            if (info_err == ESP_OK) {
                ESP_LOGE(TAG, "bus-off: TEC=%u REC=%u; starting recovery",
                         status.tx_error_count, status.rx_error_count);
            } else {
                ESP_LOGE(TAG, "bus-off; status read failed: %s",
                         esp_err_to_name(info_err));
            }
            const esp_err_t recover_err = twai_node_recover(s_node);
            if (recover_err != ESP_OK) {
                ESP_LOGE(TAG, "bus-off recovery start failed: %s",
                         esp_err_to_name(recover_err));
            } else {
                ESP_LOGI(TAG, "bus-off recovery started");
            }
        } else if (event.value.state.old_state == TWAI_ERROR_BUS_OFF &&
                   event.value.state.new_state == TWAI_ERROR_ACTIVE) {
            ESP_LOGI(TAG, "bus-off recovery completed");
        }
    }
}

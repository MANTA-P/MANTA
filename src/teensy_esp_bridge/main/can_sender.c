#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char *TAG = "can_sender";
static twai_node_handle_t s_twai_node;

static uint32_t can_bitrate(void)
{
#if CONFIG_CAN_BITRATE_125K
    return 125000;
#elif CONFIG_CAN_BITRATE_250K
    return 250000;
#elif CONFIG_CAN_BITRATE_1M
    return 1000000;
#else
    return 500000;
#endif
}

static const char *can_bitrate_name(void)
{
#if CONFIG_CAN_BITRATE_125K
    return "125 kbit/s";
#elif CONFIG_CAN_BITRATE_250K
    return "250 kbit/s";
#elif CONFIG_CAN_BITRATE_1M
    return "1 Mbit/s";
#else
    return "500 kbit/s";
#endif
}

static uint8_t payload_checksum(const uint8_t *data, size_t length)
{
    uint8_t checksum = 0;

    for (size_t i = 0; i < length; ++i) {
        checksum ^= data[i];
    }

    return checksum;
}

static bool can_tx_done_callback(twai_node_handle_t handle,
                                 const twai_tx_done_event_data_t *event,
                                 void *user_context)
{
    (void)handle;
    (void)user_context;

    if (!event->is_tx_success) {
        ESP_EARLY_LOGW(TAG, "CAN frame transmission failed, id=0x%03lX",
                       (unsigned long)event->done_tx_frame->header.id);
    }

    return false;
}

static bool can_error_callback(twai_node_handle_t handle,
                               const twai_error_event_data_t *event,
                               void *user_context)
{
    (void)handle;
    (void)user_context;

    ESP_EARLY_LOGW(TAG, "TWAI bus error flags=0x%02lX",
                   (unsigned long)event->err_flags.val);
    return false;
}

static bool can_state_change_callback(
    twai_node_handle_t handle,
    const twai_state_change_event_data_t *event,
    void *user_context)
{
    (void)handle;
    (void)user_context;

    ESP_EARLY_LOGW(TAG, "TWAI state changed: %d -> %d",
                   (int)event->old_sta, (int)event->new_sta);
    return false;
}

static esp_err_t recover_bus_if_needed(bool *recovery_in_progress)
{
    twai_node_status_t status = {0};
    esp_err_t result = twai_node_get_info(s_twai_node, &status, NULL);
    if (result != ESP_OK) {
        return result;
    }

    if (status.state == TWAI_ERROR_BUS_OFF && !*recovery_in_progress) {
        ESP_LOGE(TAG,
                 "TWAI bus-off: TEC=%u REC=%u; starting recovery",
                 status.tx_error_count, status.rx_error_count);
        result = twai_node_recover(s_twai_node);
        if (result == ESP_OK) {
            *recovery_in_progress = true;
        }
        return result;
    }

    if (*recovery_in_progress && status.state == TWAI_ERROR_ACTIVE) {
        *recovery_in_progress = false;
        ESP_LOGI(TAG, "TWAI bus recovered");
    }

    return ESP_OK;
}

static void fill_payload(uint8_t *data, uint32_t sequence)
{
    data[0] = (uint8_t)sequence;
    data[1] = (uint8_t)(sequence >> 8);
    data[2] = (uint8_t)(sequence >> 16);
    data[3] = (uint8_t)(sequence >> 24);
    data[4] = 0xA5;
    data[5] = 0x5A;
    data[6] = 0x00;
    data[7] = payload_checksum(data, 7);
}

static void can_transmit_task(void *argument)
{
    (void)argument;
    uint32_t sequence = 0;
    bool frame_pending = false;
    bool recovery_in_progress = false;
    TickType_t last_wake_time = xTaskGetTickCount();
    uint8_t payload[8] = {0};
    twai_frame_t frame = {
        .header = {
            .id = CONFIG_CAN_MESSAGE_ID,
            .dlc = 8,
            .ide = 0,
            .rtr = 0,
            .fdf = 0,
            .brs = 0,
        },
        .buffer = payload,
        .buffer_len = sizeof(payload),
    };

    while (true) {
        esp_err_t result = recover_bus_if_needed(&recovery_in_progress);
        if (result != ESP_OK) {
            ESP_LOGE(TAG, "TWAI status/recovery failed: %s",
                     esp_err_to_name(result));
        }

        if (recovery_in_progress) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_CAN_SEND_PERIOD_MS));
            last_wake_time = xTaskGetTickCount();
            continue;
        }

        if (frame_pending) {
            result = twai_node_transmit_wait_all_done(
                s_twai_node, CONFIG_CAN_TX_TIMEOUT_MS);
            if (result == ESP_OK) {
                frame_pending = false;
                ESP_LOGI(TAG, "TX completed id=0x%03" PRIX32
                         " seq=%" PRIu32,
                         frame.header.id, sequence);
                ++sequence;
            } else if (result != ESP_ERR_TIMEOUT &&
                       result != ESP_ERR_INVALID_STATE) {
                ESP_LOGE(TAG, "Waiting for TX completion failed: %s",
                         esp_err_to_name(result));
            }
        }

        if (!frame_pending) {
            fill_payload(payload, sequence);
            result = twai_node_transmit(
                s_twai_node, &frame, CONFIG_CAN_TX_TIMEOUT_MS);
            if (result == ESP_OK) {
                frame_pending = true;
            } else if (result == ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "TX queue timeout; retrying next period");
            } else if (result == ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "TWAI is bus-off; waiting for recovery");
            } else {
                ESP_LOGE(TAG, "twai_node_transmit failed: %s",
                         esp_err_to_name(result));
            }
        }

        xTaskDelayUntil(&last_wake_time,
                        pdMS_TO_TICKS(CONFIG_CAN_SEND_PERIOD_MS));
    }
}

void app_main(void)
{
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CONFIG_CAN_TX_GPIO,
            .rx = CONFIG_CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = can_bitrate(),
        },
        .fail_retry_cnt = 3,
        .tx_queue_depth = 10,
    };

    ESP_LOGI(TAG,
             "Starting ESP32-S3 TWAI: TX=GPIO%d RX=GPIO%d bitrate=%s "
             "id=0x%03X period=%d ms",
             CONFIG_CAN_TX_GPIO,
             CONFIG_CAN_RX_GPIO,
             can_bitrate_name(),
             CONFIG_CAN_MESSAGE_ID,
             CONFIG_CAN_SEND_PERIOD_MS);

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_twai_node));

    const twai_event_callbacks_t callbacks = {
        .on_tx_done = can_tx_done_callback,
        .on_state_change = can_state_change_callback,
        .on_error = can_error_callback,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(
        s_twai_node, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_twai_node));

    BaseType_t task_created = xTaskCreate(
        can_transmit_task, "can_transmit", 4096, NULL, 9, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

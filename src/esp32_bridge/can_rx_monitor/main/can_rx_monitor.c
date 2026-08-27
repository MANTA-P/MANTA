#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "can_rx_config.h"

static const char *TAG = "can_rx_monitor";

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} can_rx_item_t;

static QueueHandle_t s_rx_queue;

static bool IRAM_ATTR on_rx_done(twai_node_handle_t node,
                                 const twai_rx_done_event_data_t *event,
                                 void *user_ctx)
{
    (void)event;
    (void)user_ctx;

    can_rx_item_t item = {0};
    item.frame.buffer = item.data;
    item.frame.buffer_len = sizeof(item.data);

    if (twai_node_receive_from_isr(node, &item.frame) != ESP_OK) {
        return false;
    }

    /* The frame is copied into the queue; do not retain the ISR-stack pointer. */
    item.frame.buffer = NULL;

    BaseType_t higher_priority_task_woken = pdFALSE;
    /* The queue copies the complete frame and payload out of ISR context. */
    const BaseType_t queued = xQueueSendFromISR(s_rx_queue, &item,
                                                 &higher_priority_task_woken);
    if (queued != pdTRUE) {
        ESP_EARLY_LOGW(TAG, "RX queue full; frame dropped");
    }
    return higher_priority_task_woken == pdTRUE;
}

static bool IRAM_ATTR on_state_change(twai_node_handle_t node,
                                      const twai_state_change_event_data_t *event,
                                      void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    ESP_EARLY_LOGI(TAG, "TWAI state: %d -> %d", event->old_sta, event->new_sta);
    return false;
}

static bool IRAM_ATTR on_error(twai_node_handle_t node,
                               const twai_error_event_data_t *event,
                               void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    ESP_EARLY_LOGW(TAG, "TWAI error flags: 0x%" PRIx32, event->err_flags.val);
    return false;
}

static twai_node_handle_t can_init(void)
{
    twai_node_handle_t node = NULL;
    const twai_onchip_node_config_t config = {
        .io_cfg = {
            .tx = CAN_TX_GPIO,
            .rx = CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = CAN_BITRATE,
        },
        /* Required by ESP-IDF even though this application does not send data frames. */
        .tx_queue_depth = 1,
        .flags = {
            /* Normal mode lets this node ACK frames from the transmitter. */
            .no_receive_rtr = true,
        },
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&config, &node));

    const twai_event_callbacks_t callbacks = {
        .on_rx_done = on_rx_done,
        .on_state_change = on_state_change,
        .on_error = on_error,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_enable(node));
    return node;
}

static void log_frame(const can_rx_item_t *item)
{
    const twai_frame_header_t *header = &item->frame.header;
    const size_t data_len = header->dlc <= TWAI_FRAME_MAX_LEN ? header->dlc : 0;
    char hex[3 * TWAI_FRAME_MAX_LEN + 1] = {0};
    char ascii[TWAI_FRAME_MAX_LEN + 1] = {0};
    size_t offset = 0;

    for (size_t i = 0; i < data_len; ++i) {
        offset += (size_t)snprintf(&hex[offset], sizeof(hex) - offset,
                                   "%02" PRIX8 "%s", item->data[i],
                                   i + 1 < data_len ? " " : "");
        ascii[i] = (item->data[i] >= 0x20 && item->data[i] <= 0x7e)
                       ? (char)item->data[i] : '.';
    }

    ESP_LOGI(TAG, "RX %s ID=0x%" PRIX32 " DLC=%u DATA=[%s] ASCII=\"%s\"",
             header->ide ? "EXT" : "STD", header->id, header->dlc, hex, ascii);
}

void app_main(void)
{
    s_rx_queue = xQueueCreate(CAN_RX_QUEUE_DEPTH, sizeof(can_rx_item_t));
    ESP_ERROR_CHECK(s_rx_queue != NULL ? ESP_OK : ESP_ERR_NO_MEM);

    (void)can_init();
    ESP_LOGI(TAG, "ready: CAN RX GPIO=%d, bitrate=%" PRIu32,
             CAN_RX_GPIO, (uint32_t)CAN_BITRATE);
    ESP_LOGI(TAG, "waiting for CAN frames; run `idf.py monitor` to view RX logs");

    while (true) {
        can_rx_item_t item;
        if (xQueueReceive(s_rx_queue, &item, portMAX_DELAY) == pdTRUE) {
            log_frame(&item);
        }
    }
}

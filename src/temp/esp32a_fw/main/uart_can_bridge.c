/*
 * ESP32-S3 A: Torpedo HIL USB Serial/JTAG <-> Classical CAN gateway.
 *
 * IMPORTANT sdkconfig requirement:
 *   USB Serial/JTAG must not be selected as a primary or secondary console.
 *   Binary protocol bytes and ESP_LOG/console text must never share this port.
 *
 * UART wire format:
 *   AA 55 | MESSAGE_ID | LENGTH | PAYLOAD
 *
 * PC -> A:
 *   0x10, 32 bytes: BlueROV odometry -> CAN 0x110, six fragments
 *   0x20, 32 bytes: Torpedo odometry -> CAN 0x120, six fragments
 *   0x30,  8 bytes: Control command   -> CAN 0x010, one frame
 *
 * A -> PC:
 *   CAN 0x180 + 0x181 pair -> UART 0x80, 12-byte actuator payload
 *   CAN 0x1F0             -> UART 0x81,  8-byte status payload
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#if (defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) && CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG) || \
    (defined(CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG) && \
     CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG)
#error "Disable the USB Serial/JTAG console before building this binary gateway"
#endif

#define UART_SYNC_FIRST                 0xAAU
#define UART_SYNC_SECOND                0x55U
#define UART_HEADER_SIZE                4U
#define UART_MAX_PAYLOAD_SIZE           32U
#define UART_FRAME_TIMEOUT_MS           50U

#define UART_ID_BLUEROV_ODOMETRY        0x10U
#define UART_ID_TORPEDO_ODOMETRY        0x20U
#define UART_ID_CONTROL_COMMAND         0x30U
#define UART_ID_TORPEDO_ACTUATOR        0x80U
#define UART_ID_CONTROLLER_STATUS       0x81U

#define CAN_ID_CONTROL_COMMAND          0x010U
#define CAN_ID_BLUEROV_ODOMETRY         0x110U
#define CAN_ID_TORPEDO_ODOMETRY         0x120U
#define CAN_ID_ACTUATOR_A               0x180U
#define CAN_ID_ACTUATOR_B               0x181U
#define CAN_ID_CONTROLLER_STATUS        0x1F0U

#define CAN_TX_GPIO                     GPIO_NUM_17
#define CAN_RX_GPIO                     GPIO_NUM_18
#define CAN_BITRATE                     500000U
#define CAN_FRAME_SIZE                  8U
#define CAN_FRAGMENT_DATA_SIZE          6U
#define ODOMETRY_FRAGMENT_COUNT         6U
#define CAN_TX_TIMEOUT_MS               100U
#define ACTUATOR_PAIR_TIMEOUT_MS        50U

#define USB_READ_CHUNK_SIZE             64U
#define USB_READ_WAIT_MS                10U
#define USB_WRITE_WAIT_MS               20U
#define UART_MESSAGE_QUEUE_DEPTH        16U
#define CAN_RX_QUEUE_DEPTH              32U

static const char *TAG = "esp32a_can";

typedef struct {
    uint8_t message_id;
    uint8_t length;
    uint8_t payload[UART_MAX_PAYLOAD_SIZE];
} uart_message_t;

typedef struct {
    uint32_t id;
    uint8_t length;
    uint8_t data[CAN_FRAME_SIZE];
} can_rx_item_t;

typedef enum {
    PARSER_WAIT_SYNC_FIRST = 0,
    PARSER_WAIT_SYNC_SECOND,
    PARSER_READ_MESSAGE_ID,
    PARSER_READ_LENGTH,
    PARSER_READ_PAYLOAD,
} parser_state_t;

typedef struct {
    parser_state_t state;
    TickType_t frame_start_tick;
    uint8_t message_id;
    uint8_t expected_length;
    uint8_t payload_index;
    uint8_t payload[UART_MAX_PAYLOAD_SIZE];
} uart_parser_t;

typedef struct {
    bool active;
    bool received_a;
    bool received_b;
    uint8_t sequence;
    TickType_t start_tick;
    uint8_t actuator_a[CAN_FRAME_SIZE];
    uint8_t actuator_b[CAN_FRAME_SIZE];
} actuator_pair_t;

typedef struct {
    volatile uint32_t uart_valid_frames;
    volatile uint32_t uart_invalid_frames;
    volatile uint32_t uart_timeout_frames;
    volatile uint32_t uart_tx_failures;
    volatile uint32_t message_queue_drops;
    volatile uint32_t can_tx_frames;
    volatile uint32_t can_tx_failures;
    volatile uint32_t can_rx_frames;
    volatile uint32_t can_load_rx_frames;
    volatile uint32_t can_rx_drops;
    volatile uint32_t can_error_events;
    volatile uint32_t can_bus_off_events;
    volatile uint32_t can_recovery_failures;
    volatile uint32_t actuator_pair_timeouts;
    volatile uint32_t actuator_pair_mismatches;
    volatile uint32_t actuator_pairs;
    volatile uint32_t actuator_uart_frames;
    volatile uint32_t status_uart_frames;
    volatile uint8_t last_pair_sequence;
    volatile uint32_t last_pair_ms;
    volatile uint32_t last_actuator_uart_ms;
} gateway_statistics_t;

static twai_node_handle_t s_twai_node;
static QueueHandle_t s_uart_message_queue;
static QueueHandle_t s_can_rx_queue;
static SemaphoreHandle_t s_can_tx_done;
static twai_frame_t s_can_tx_frame;
static uint8_t s_can_tx_data[CAN_FRAME_SIZE];
static volatile bool s_can_tx_pending;
static volatile bool s_can_tx_success;
static gateway_statistics_t s_statistics;

static uint8_t expected_pc_payload_length(uint8_t message_id)
{
    switch (message_id) {
        case UART_ID_BLUEROV_ODOMETRY:
        case UART_ID_TORPEDO_ODOMETRY:
            return 32U;
        case UART_ID_CONTROL_COMMAND:
            return 8U;
        default:
            return 0U;
    }
}

static void parser_reset(uart_parser_t *parser)
{
    parser->state = PARSER_WAIT_SYNC_FIRST;
    parser->message_id = 0U;
    parser->expected_length = 0U;
    parser->payload_index = 0U;
}

static void parser_seek_sync(uart_parser_t *parser, uint8_t byte, TickType_t now)
{
    parser_reset(parser);
    if (byte == UART_SYNC_FIRST) {
        parser->state = PARSER_WAIT_SYNC_SECOND;
        parser->frame_start_tick = now;
    }
}

static bool parser_has_timed_out(const uart_parser_t *parser, TickType_t now)
{
    return parser->state != PARSER_WAIT_SYNC_FIRST &&
           (now - parser->frame_start_tick) >= pdMS_TO_TICKS(UART_FRAME_TIMEOUT_MS);
}

static void enqueue_uart_message(const uart_message_t *message)
{
    if (xQueueSend(s_uart_message_queue, message, 0) == pdTRUE) {
        return;
    }

    /* Keep latency bounded: discard the oldest complete message, then retry. */
    uart_message_t discarded;
    (void)xQueueReceive(s_uart_message_queue, &discarded, 0);
    if (xQueueSend(s_uart_message_queue, message, 0) != pdTRUE) {
        ++s_statistics.message_queue_drops;
        return;
    }
    ++s_statistics.message_queue_drops;
}

static void parser_consume_byte(uart_parser_t *parser, uint8_t byte, TickType_t now)
{
    if (parser_has_timed_out(parser, now)) {
        ++s_statistics.uart_timeout_frames;
        parser_seek_sync(parser, byte, now);
        return;
    }

    switch (parser->state) {
        case PARSER_WAIT_SYNC_FIRST:
            if (byte == UART_SYNC_FIRST) {
                parser->state = PARSER_WAIT_SYNC_SECOND;
                parser->frame_start_tick = now;
            }
            break;

        case PARSER_WAIT_SYNC_SECOND:
            if (byte == UART_SYNC_SECOND) {
                parser->state = PARSER_READ_MESSAGE_ID;
            } else if (byte == UART_SYNC_FIRST) {
                parser->frame_start_tick = now;
            } else {
                parser_reset(parser);
            }
            break;

        case PARSER_READ_MESSAGE_ID:
            parser->message_id = byte;
            parser->expected_length = expected_pc_payload_length(byte);
            if (parser->expected_length == 0U) {
                ++s_statistics.uart_invalid_frames;
                parser_seek_sync(parser, byte, now);
            } else {
                parser->state = PARSER_READ_LENGTH;
            }
            break;

        case PARSER_READ_LENGTH:
            if (byte != parser->expected_length) {
                ++s_statistics.uart_invalid_frames;
                parser_seek_sync(parser, byte, now);
            } else {
                parser->payload_index = 0U;
                parser->state = PARSER_READ_PAYLOAD;
            }
            break;

        case PARSER_READ_PAYLOAD:
            parser->payload[parser->payload_index++] = byte;
            if (parser->payload_index == parser->expected_length) {
                uart_message_t message = {
                    .message_id = parser->message_id,
                    .length = parser->expected_length,
                };
                memcpy(message.payload, parser->payload, parser->expected_length);
                enqueue_uart_message(&message);
                ++s_statistics.uart_valid_frames;
                parser_reset(parser);
            }
            break;

        default:
            parser_reset(parser);
            break;
    }
}

static bool usb_write_all(const uint8_t *data, size_t length)
{
    size_t offset = 0U;
    while (offset < length) {
        const int written = usb_serial_jtag_write_bytes(
            data + offset, length - offset, pdMS_TO_TICKS(USB_WRITE_WAIT_MS));
        if (written <= 0) {
            ++s_statistics.uart_tx_failures;
            return false;
        }
        offset += (size_t)written;
    }
    return true;
}

static bool send_uart_frame(uint8_t message_id, const uint8_t *payload, uint8_t length)
{
    uint8_t frame[UART_HEADER_SIZE + UART_MAX_PAYLOAD_SIZE] = {0};
    if (payload == NULL || length > UART_MAX_PAYLOAD_SIZE) {
        return false;
    }

    frame[0] = UART_SYNC_FIRST;
    frame[1] = UART_SYNC_SECOND;
    frame[2] = message_id;
    frame[3] = length;
    memcpy(&frame[UART_HEADER_SIZE], payload, length);
    return usb_write_all(frame, UART_HEADER_SIZE + length);
}

static esp_err_t can_send_standard(uint32_t id, const uint8_t data[CAN_FRAME_SIZE])
{
    /* ESP-IDF retains these pointers until the asynchronous transfer ends. */
    if (s_can_tx_pending &&
        xSemaphoreTake(s_can_tx_done, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS)) != pdTRUE) {
        ++s_statistics.can_tx_failures;
        return ESP_ERR_TIMEOUT;
    }
    while (xSemaphoreTake(s_can_tx_done, 0) == pdTRUE) {
    }

    memcpy(s_can_tx_data, data, CAN_FRAME_SIZE);
    s_can_tx_frame = (twai_frame_t) {
        .header = {
            .id = id,
            .dlc = CAN_FRAME_SIZE,
            .ide = false,
            .rtr = false,
        },
        .buffer = s_can_tx_data,
        .buffer_len = CAN_FRAME_SIZE,
    };

    s_can_tx_success = false;
    s_can_tx_pending = true;
    esp_err_t result = twai_node_transmit(
        s_twai_node, &s_can_tx_frame, CAN_TX_TIMEOUT_MS);
    if (result != ESP_OK) {
        s_can_tx_pending = false;
    } else if (xSemaphoreTake(
                   s_can_tx_done, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS)) != pdTRUE) {
        /* Static storage remains valid; a later call waits before reuse. */
        result = ESP_ERR_TIMEOUT;
    } else if (!s_can_tx_success) {
        result = ESP_FAIL;
    }
    if (result == ESP_OK) {
        ++s_statistics.can_tx_frames;
    } else {
        ++s_statistics.can_tx_failures;
    }
    return result;
}

static bool IRAM_ATTR on_can_tx_done(twai_node_handle_t node,
                                     const twai_tx_done_event_data_t *event,
                                     void *user_context)
{
    (void)node;
    (void)user_context;
    s_can_tx_success = event->is_tx_success;
    s_can_tx_pending = false;
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_can_tx_done, &task_woken);
    return task_woken == pdTRUE;
}

static void send_odometry_fragments(const uart_message_t *message,
                                    uint32_t can_id,
                                    uint8_t *next_sequence)
{
    const uint8_t sequence = (*next_sequence)++;
    for (uint8_t fragment = 0U; fragment < ODOMETRY_FRAGMENT_COUNT; ++fragment) {
        uint8_t can_data[CAN_FRAME_SIZE] = {0};
        can_data[0] = sequence;
        can_data[1] = fragment;

        const size_t offset = (size_t)fragment * CAN_FRAGMENT_DATA_SIZE;
        const size_t remaining = message->length - offset;
        const size_t chunk = remaining < CAN_FRAGMENT_DATA_SIZE ?
                             remaining : CAN_FRAGMENT_DATA_SIZE;
        memcpy(&can_data[2], &message->payload[offset], chunk);

        if (can_send_standard(can_id, can_data) != ESP_OK) {
            /* Do not mix the remainder of an incomplete payload with later data. */
            return;
        }
    }
}

static void usb_receive_task(void *argument)
{
    (void)argument;
    uart_parser_t parser = {0};
    parser_reset(&parser);
    uint8_t bytes[USB_READ_CHUNK_SIZE];

    while (true) {
        const int received = usb_serial_jtag_read_bytes(
            bytes, sizeof(bytes), pdMS_TO_TICKS(USB_READ_WAIT_MS));
        const TickType_t now = xTaskGetTickCount();

        if (received > 0) {
            for (int index = 0; index < received; ++index) {
                parser_consume_byte(&parser, bytes[index], now);
            }
        } else if (parser_has_timed_out(&parser, now)) {
            ++s_statistics.uart_timeout_frames;
            parser_reset(&parser);
        }
    }
}

static bool can_bus_ready(bool *recovery_in_progress)
{
    twai_node_status_t status = {0};
    if (twai_node_get_info(s_twai_node, &status, NULL) != ESP_OK) {
        return false;
    }

    if (status.state == TWAI_ERROR_BUS_OFF && !*recovery_in_progress) {
        ++s_statistics.can_bus_off_events;
        if (twai_node_recover(s_twai_node) == ESP_OK) {
            *recovery_in_progress = true;
        } else {
            ++s_statistics.can_recovery_failures;
        }
        return false;
    }
    if (*recovery_in_progress) {
        if (status.state != TWAI_ERROR_ACTIVE) {
            return false;
        }
        *recovery_in_progress = false;
    }
    return status.state != TWAI_ERROR_BUS_OFF;
}

static void can_transmit_task(void *argument)
{
    (void)argument;
    uint8_t bluerov_sequence = 0U;
    uint8_t torpedo_sequence = 0U;
    bool recovery_in_progress = false;
    uart_message_t message;

    while (true) {
        if (!can_bus_ready(&recovery_in_progress)) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (xQueueReceive(s_uart_message_queue, &message, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }

        switch (message.message_id) {
            case UART_ID_BLUEROV_ODOMETRY:
                send_odometry_fragments(
                    &message, CAN_ID_BLUEROV_ODOMETRY, &bluerov_sequence);
                break;
            case UART_ID_TORPEDO_ODOMETRY:
                send_odometry_fragments(
                    &message, CAN_ID_TORPEDO_ODOMETRY, &torpedo_sequence);
                break;
            case UART_ID_CONTROL_COMMAND:
                (void)can_send_standard(CAN_ID_CONTROL_COMMAND, message.payload);
                break;
            default:
                ++s_statistics.uart_invalid_frames;
                break;
        }
    }
}

static bool IRAM_ATTR on_can_rx_done(twai_node_handle_t node,
                                     const twai_rx_done_event_data_t *event,
                                     void *user_context)
{
    (void)event;
    (void)user_context;

    can_rx_item_t item = {0};
    twai_frame_t frame = {
        .buffer = item.data,
        .buffer_len = sizeof(item.data),
    };
    if (twai_node_receive_from_isr(node, &frame) != ESP_OK) {
        return false;
    }

    ++s_statistics.can_rx_frames;
    if (!frame.header.ide && !frame.header.rtr &&
        frame.header.id == 0x001U) {
        ++s_statistics.can_load_rx_frames;
        return false;
    }
    if (frame.header.ide || frame.header.rtr ||
        (frame.header.id != CAN_ID_ACTUATOR_A &&
         frame.header.id != CAN_ID_ACTUATOR_B &&
         frame.header.id != CAN_ID_CONTROLLER_STATUS)) {
        return false;
    }

    item.id = frame.header.id;
    item.length = frame.header.dlc;
    BaseType_t task_woken = pdFALSE;
    if (xQueueSendFromISR(s_can_rx_queue, &item, &task_woken) != pdTRUE) {
        ++s_statistics.can_rx_drops;
    }
    return task_woken == pdTRUE;
}

static bool IRAM_ATTR on_can_error(twai_node_handle_t node,
                                   const twai_error_event_data_t *event,
                                   void *user_context)
{
    (void)node;
    (void)event;
    (void)user_context;
    ++s_statistics.can_error_events;
    return false;
}

static void actuator_pair_reset(actuator_pair_t *pair)
{
    memset(pair, 0, sizeof(*pair));
}

static void process_actuator_frame(actuator_pair_t *pair, const can_rx_item_t *item)
{
    const TickType_t now = xTaskGetTickCount();
    if (pair->active &&
        (now - pair->start_tick) >= pdMS_TO_TICKS(ACTUATOR_PAIR_TIMEOUT_MS)) {
        ++s_statistics.actuator_pair_timeouts;
        actuator_pair_reset(pair);
    }

    const uint8_t sequence = item->data[0];
    if (!pair->active || pair->sequence != sequence) {
        if (pair->active && pair->sequence != sequence) {
            ++s_statistics.actuator_pair_mismatches;
        }
        actuator_pair_reset(pair);
        pair->active = true;
        pair->sequence = sequence;
        pair->start_tick = now;
    }

    if (item->id == CAN_ID_ACTUATOR_A) {
        memcpy(pair->actuator_a, item->data, CAN_FRAME_SIZE);
        pair->received_a = true;
    } else {
        memcpy(pair->actuator_b, item->data, CAN_FRAME_SIZE);
        pair->received_b = true;
    }

    if (!pair->received_a || !pair->received_b) {
        return;
    }

    uint8_t payload[12] = {0};
    payload[0] = pair->sequence;
    payload[1] = pair->actuator_a[1];
    memcpy(&payload[2], &pair->actuator_a[2], 6U);  /* thrust, top, bottom */
    memcpy(&payload[8], &pair->actuator_b[2], 4U);  /* left, right */
    ++s_statistics.actuator_pairs;
    s_statistics.last_pair_sequence = pair->sequence;
    s_statistics.last_pair_ms =
        (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (send_uart_frame(UART_ID_TORPEDO_ACTUATOR, payload,
                        sizeof(payload))) {
        ++s_statistics.actuator_uart_frames;
        s_statistics.last_actuator_uart_ms =
            (uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    actuator_pair_reset(pair);
}

static void can_receive_task(void *argument)
{
    (void)argument;
    actuator_pair_t actuator_pair = {0};
    can_rx_item_t item;

    while (true) {
        if (xQueueReceive(s_can_rx_queue, &item, pdMS_TO_TICKS(10)) != pdTRUE) {
            if (actuator_pair.active &&
                (xTaskGetTickCount() - actuator_pair.start_tick) >=
                    pdMS_TO_TICKS(ACTUATOR_PAIR_TIMEOUT_MS)) {
                ++s_statistics.actuator_pair_timeouts;
                actuator_pair_reset(&actuator_pair);
            }
            continue;
        }
        if (item.length != CAN_FRAME_SIZE) {
            ++s_statistics.can_rx_drops;
            continue;
        }

        if (item.id == CAN_ID_ACTUATOR_A || item.id == CAN_ID_ACTUATOR_B) {
            process_actuator_frame(&actuator_pair, &item);
        } else if (item.id == CAN_ID_CONTROLLER_STATUS) {
            if (send_uart_frame(
                    UART_ID_CONTROLLER_STATUS, item.data, CAN_FRAME_SIZE)) {
                ++s_statistics.status_uart_frames;
            }
        }
    }
}

static void statistics_task(void *argument)
{
    (void)argument;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
        ESP_LOGI(TAG,
                 "UART in=%" PRIu32 " invalid=%" PRIu32
                 " parse_timeout=%" PRIu32 " input_queue_drops=%" PRIu32
                 " CAN tx_ok=%" PRIu32 " tx_fail=%" PRIu32,
                 s_statistics.uart_valid_frames,
                 s_statistics.uart_invalid_frames,
                 s_statistics.uart_timeout_frames,
                 s_statistics.message_queue_drops,
                 s_statistics.can_tx_frames,
                 s_statistics.can_tx_failures);
        ESP_LOGI(TAG,
                 "CAN rx_total=%" PRIu32 " load=%" PRIu32
                 " rx_drops=%" PRIu32 " pair_ok=%" PRIu32
                 " pair_mismatch=%" PRIu32 " pair_timeout=%" PRIu32
                 " UART act_ok=%" PRIu32 " status_ok=%" PRIu32
                 " write_fail=%" PRIu32 " last_pair_seq=%u"
                 " pair_ms=%" PRIu32 " uart_ms=%" PRIu32,
                 s_statistics.can_rx_frames,
                 s_statistics.can_load_rx_frames,
                 s_statistics.can_rx_drops,
                 s_statistics.actuator_pairs,
                 s_statistics.actuator_pair_mismatches,
                 s_statistics.actuator_pair_timeouts,
                 s_statistics.actuator_uart_frames,
                 s_statistics.status_uart_frames,
                 s_statistics.uart_tx_failures,
                 s_statistics.last_pair_sequence,
                 s_statistics.last_pair_ms,
                 s_statistics.last_actuator_uart_ms);
    }
}

static void usb_serial_init(void)
{
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
}

static void can_init(void)
{
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
        .fail_retry_cnt = 3,
        .tx_queue_depth = 10,
        .flags = {
            .no_receive_rtr = true,
        },
    };
    const twai_event_callbacks_t callbacks = {
        .on_tx_done = on_can_tx_done,
        .on_rx_done = on_can_rx_done,
        .on_error = on_can_error,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&config, &s_twai_node));
    ESP_ERROR_CHECK(
        twai_node_register_event_callbacks(s_twai_node, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_twai_node));
}

void app_main(void)
{
    s_uart_message_queue =
        xQueueCreate(UART_MESSAGE_QUEUE_DEPTH, sizeof(uart_message_t));
    s_can_rx_queue = xQueueCreate(CAN_RX_QUEUE_DEPTH, sizeof(can_rx_item_t));
    s_can_tx_done = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(
        (s_uart_message_queue != NULL && s_can_rx_queue != NULL &&
         s_can_tx_done != NULL) ?
        ESP_OK : ESP_ERR_NO_MEM);

    usb_serial_init();
    can_init();

    BaseType_t result = xTaskCreate(
        usb_receive_task, "usb_rx", 4096, NULL, 12, NULL);
    ESP_ERROR_CHECK(result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    result = xTaskCreate(
        can_transmit_task, "can_tx", 4096, NULL, 10, NULL);
    ESP_ERROR_CHECK(result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    result = xTaskCreate(
        can_receive_task, "can_rx", 4096, NULL, 11, NULL);
    ESP_ERROR_CHECK(result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    result = xTaskCreate(
        statistics_task, "can_stats", 4096, NULL, 4, NULL);
    ESP_ERROR_CHECK(result == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

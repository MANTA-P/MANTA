#include "can_protocol.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "controller_state.h"
#include "watchdog.h"

static const char *TAG = "esp32b_can";

#define CAN_TX_GPIO 17
#define CAN_RX_GPIO 18
#define CAN_BITRATE 500000U

#define CAN_ID_CONTROL 0x010U
#define CAN_ID_BLUEROV_ODOMETRY 0x110U
#define CAN_ID_TORPEDO_ODOMETRY 0x120U
#define CAN_ID_ACTUATOR_A 0x180U
#define CAN_ID_ACTUATOR_B 0x181U
#define CAN_ID_STATUS 0x1F0U
#define CAN_ID_TEST_LOAD 0x001U

#define MODE_NONE 0U
#define MODE_SIMPLE 1U
#define MODE_PNG 2U

#define CAN_RX_QUEUE_DEPTH 64U
#define CAN_TX_TIMEOUT_MS 100U
#define CAN_FRAME_SIZE 8U
#define ODOMETRY_SIZE 32U
#define ODOMETRY_FRAGMENT_DATA_SIZE 6U
#define ODOMETRY_FRAGMENT_COUNT 6U
#define ODOMETRY_COMPLETE_MASK ((1U << ODOMETRY_FRAGMENT_COUNT) - 1U)
#define ODOMETRY_TIMEOUT_MS 50U
#define MONITOR_LOG_INTERVAL_MS 1000U
#define TX_CHANNEL_COUNT 3U

typedef struct {
    volatile uint32_t attempts;
    volatile uint32_t success;
    volatile uint32_t failed;
    volatile uint32_t timeouts;
    volatile uint32_t hw_done_success;
    volatile uint32_t hw_done_failed;
    volatile uint32_t last_attempt_ms;
    volatile uint32_t last_done_ms;
    volatile uint32_t max_wait_ms;
    volatile uint8_t last_sequence;
} tx_statistics_t;

typedef struct {
    volatile uint32_t arbitration_lost;
    volatile uint32_t bit;
    volatile uint32_t form;
    volatile uint32_t stuff;
    volatile uint32_t ack;
} error_statistics_t;

typedef struct {
    bool active;
    bool valid;
    uint8_t sequence;
    uint8_t received_mask;
    TickType_t last_fragment_tick;
    TickType_t last_complete_tick;
    TickType_t last_log_tick;
    uint32_t complete_count;
    uint8_t data[ODOMETRY_SIZE];
} odometry_reassembly_t;

static twai_node_handle_t s_twai_node = NULL;
static QueueHandle_t s_rx_queue;
static SemaphoreHandle_t s_tx_mutex;
static SemaphoreHandle_t s_tx_done;
static twai_frame_t s_tx_frame;
static uint8_t s_tx_data[CAN_FRAME_SIZE];
static volatile bool s_tx_pending;
static volatile bool s_tx_success;
static volatile uint32_t s_last_error_flags;
static volatile uint32_t s_error_event_count;
static volatile uint32_t s_test_load_rx_count;
static volatile uint32_t s_rx_queue_drops;
static tx_statistics_t s_tx_statistics[TX_CHANNEL_COUNT];
static error_statistics_t s_error_statistics;
static bool s_recovery_in_progress;
static uint8_t s_odometry_valid_mask;
static TickType_t s_last_control_log_tick;
static uint32_t s_control_rx_count;
static TickType_t s_last_monitor_tick;
static odometry_reassembly_t s_bluerov_odometry;
static odometry_reassembly_t s_torpedo_odometry;

static int tx_channel_for_id(uint32_t id)
{
    switch (id) {
        case CAN_ID_ACTUATOR_A:
            return 0;
        case CAN_ID_ACTUATOR_B:
            return 1;
        case CAN_ID_STATUS:
            return 2;
        default:
            return -1;
    }
}

static bool on_rx_done(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    (void)edata;
    (void)user_ctx;

    can_rx_item_t item = {0};
    twai_frame_t frame = {
        .buffer = item.data,
        .buffer_len = sizeof(item.data),
    };

    if (twai_node_receive_from_isr(handle, &frame) != ESP_OK) {
        return false;
    }
    if (!frame.header.ide && !frame.header.rtr &&
        frame.header.id == CAN_ID_TEST_LOAD) {
        ++s_test_load_rx_count;
        return false;
    }
    if (frame.header.ide || frame.header.rtr ||
        (frame.header.id != CAN_ID_CONTROL &&
         frame.header.id != CAN_ID_BLUEROV_ODOMETRY &&
         frame.header.id != CAN_ID_TORPEDO_ODOMETRY)) {
        return false;
    }

    item.header = frame.header;
    item.length = frame.header.dlc <= TWAI_FRAME_MAX_LEN ?
                  frame.header.dlc : 0U;

    BaseType_t task_woken = pdFALSE;
    if (xQueueSendFromISR(s_rx_queue, &item, &task_woken) != pdTRUE) {
        ++s_rx_queue_drops;
    }
    return task_woken == pdTRUE;
}

static bool on_tx_done(twai_node_handle_t handle,
                       const twai_tx_done_event_data_t *edata,
                       void *user_ctx)
{
    (void)handle;
    (void)user_ctx;
    if (edata->done_tx_frame != NULL) {
        const int channel = tx_channel_for_id(edata->done_tx_frame->header.id);
        if (channel >= 0) {
            tx_statistics_t *statistics = &s_tx_statistics[channel];
            if (edata->is_tx_success) {
                ++statistics->hw_done_success;
            } else {
                ++statistics->hw_done_failed;
            }
            statistics->last_done_ms =
                (uint32_t)xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
        }
    }
    s_tx_success = edata->is_tx_success;
    s_tx_pending = false;
    BaseType_t task_woken = pdFALSE;
    xSemaphoreGiveFromISR(s_tx_done, &task_woken);
    return task_woken == pdTRUE;
}

static bool on_error(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    (void)user_ctx;
    s_last_error_flags = edata->err_flags.val;
    ++s_error_event_count;
    s_error_statistics.arbitration_lost += edata->err_flags.arb_lost;
    s_error_statistics.bit += edata->err_flags.bit_err;
    s_error_statistics.form += edata->err_flags.form_err;
    s_error_statistics.stuff += edata->err_flags.stuff_err;
    s_error_statistics.ack += edata->err_flags.ack_err;
    return false;
}

static const char *can_state_name(twai_error_state_t state)
{
    switch (state) {
        case TWAI_ERROR_ACTIVE:
            return "active";
        case TWAI_ERROR_WARNING:
            return "warning";
        case TWAI_ERROR_PASSIVE:
            return "passive";
        case TWAI_ERROR_BUS_OFF:
            return "bus-off";
        default:
            return "unknown";
    }
}

static void log_tx_failure(uint32_t id, const char *phase, esp_err_t result)
{
    twai_node_status_t status = {0};
    const esp_err_t status_result = s_twai_node != NULL ?
        twai_node_get_info(s_twai_node, &status, NULL) : ESP_ERR_INVALID_STATE;

    if (status_result == ESP_OK) {
        ESP_LOGW(TAG,
                 "CAN TX failed: id=0x%03X phase=%s err=%s(0x%x) "
                 "state=%s TEC=%u REC=%u tx_queue_free=%lu pending=%u "
                 "recent_error_flags=0x%08lx error_events=%lu",
                 (unsigned int)id, phase, esp_err_to_name(result),
                 (unsigned int)result, can_state_name(status.state),
                 status.tx_error_count, status.rx_error_count,
                 (unsigned long)status.tx_queue_remaining,
                 s_tx_pending ? 1U : 0U,
                 (unsigned long)s_last_error_flags,
                 (unsigned long)s_error_event_count);
        return;
    }

    ESP_LOGW(TAG,
             "CAN TX failed: id=0x%03X phase=%s err=%s(0x%x) "
             "status_err=%s(0x%x) pending=%u recent_error_flags=0x%08lx "
             "error_events=%lu",
             (unsigned int)id, phase, esp_err_to_name(result),
             (unsigned int)result, esp_err_to_name(status_result),
             (unsigned int)status_result, s_tx_pending ? 1U : 0U,
             (unsigned long)s_last_error_flags,
             (unsigned long)s_error_event_count);
}

static void set_be_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)((value >> 8) & 0xFFU);
    out[1] = (uint8_t)(value & 0xFFU);
}

static uint16_t get_be_u16(const uint8_t *in)
{
    return (uint16_t)(((uint16_t)in[0] << 8) | (uint16_t)in[1]);
}

static int16_t get_be_i16(const uint8_t *in)
{
    return (int16_t)get_be_u16(in);
}

static int32_t get_be_i32(const uint8_t *in)
{
    const uint32_t value = ((uint32_t)in[0] << 24) |
                           ((uint32_t)in[1] << 16) |
                           ((uint32_t)in[2] << 8) |
                           (uint32_t)in[3];
    return (int32_t)value;
}

static void log_odometry(const char *name, uint32_t id,
                         const odometry_reassembly_t *state)
{
    const uint8_t *data = state->data;
    ESP_LOGI(TAG,
             "RX %s id=0x%03" PRIX32 " seq=%u complete=%" PRIu32 " "
             "pos_mm=(%" PRId32 ",%" PRId32 ",%" PRId32 ") "
             "quat_q14=(%" PRId16 ",%" PRId16 ",%" PRId16 ",%" PRId16 ") "
             "linear_mmps=(%" PRId16 ",%" PRId16 ",%" PRId16 ") "
             "angular_mradps=(%" PRId16 ",%" PRId16 ",%" PRId16 ")",
             name, id, state->sequence, state->complete_count,
             get_be_i32(&data[0]), get_be_i32(&data[4]), get_be_i32(&data[8]),
             get_be_i16(&data[12]), get_be_i16(&data[14]),
             get_be_i16(&data[16]), get_be_i16(&data[18]),
             get_be_i16(&data[20]), get_be_i16(&data[22]),
             get_be_i16(&data[24]), get_be_i16(&data[26]),
             get_be_i16(&data[28]), get_be_i16(&data[30]));
}

static esp_err_t can_send_frame(uint32_t id, const uint8_t *data, size_t len)
{
    const int channel = tx_channel_for_id(id);
    tx_statistics_t *statistics = channel >= 0 ? &s_tx_statistics[channel] : NULL;
    const TickType_t started = xTaskGetTickCount();
    if (statistics != NULL) {
        ++statistics->attempts;
        statistics->last_attempt_ms = (uint32_t)started * portTICK_PERIOD_MS;
        statistics->last_sequence = data != NULL && len > 0U ?
            (id == CAN_ID_STATUS && len > 1U ? data[1] : data[0]) : 0U;
    }
    if (s_twai_node == NULL || data == NULL || len > sizeof(s_tx_data)) {
        if (statistics != NULL) {
            ++statistics->failed;
        }
        log_tx_failure(id, "validate", ESP_ERR_INVALID_ARG);
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS)) != pdTRUE) {
        if (statistics != NULL) {
            ++statistics->failed;
            ++statistics->timeouts;
        }
        log_tx_failure(id, "tx_mutex", ESP_ERR_TIMEOUT);
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t result = ESP_OK;
    const char *phase = "none";
    if (s_tx_pending &&
        xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS)) != pdTRUE) {
        result = ESP_ERR_TIMEOUT;
        phase = "previous_tx_pending";
        goto out;
    }
    while (xSemaphoreTake(s_tx_done, 0) == pdTRUE) {
    }

    memcpy(s_tx_data, data, len);
    s_tx_frame = (twai_frame_t) {
        .header = {
            .id = id,
            .dlc = len,
            .ide = false,
            .rtr = false,
        },
        .buffer = s_tx_data,
        .buffer_len = len,
    };

    s_tx_success = false;
    s_tx_pending = true;
    result = twai_node_transmit(s_twai_node, &s_tx_frame, CAN_TX_TIMEOUT_MS);
    if (result != ESP_OK) {
        s_tx_pending = false;
        phase = "enqueue";
        goto out;
    }
    if (xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS)) != pdTRUE) {
        /* Static storage stays valid; the next call waits before reuse. */
        result = ESP_ERR_TIMEOUT;
        phase = "tx_done_timeout";
        goto out;
    }
    result = s_tx_success ? ESP_OK : ESP_FAIL;
    if (result != ESP_OK) {
        phase = "tx_done_failed";
    }

out:
    xSemaphoreGive(s_tx_mutex);
    if (statistics != NULL) {
        const uint32_t waited_ms =
            (uint32_t)(xTaskGetTickCount() - started) * portTICK_PERIOD_MS;
        if (waited_ms > statistics->max_wait_ms) {
            statistics->max_wait_ms = waited_ms;
        }
        if (result == ESP_OK) {
            ++statistics->success;
        } else {
            ++statistics->failed;
            if (result == ESP_ERR_TIMEOUT) {
                ++statistics->timeouts;
            }
        }
    }
    if (result != ESP_OK) {
        if (statistics == NULL || statistics->failed == 1U ||
            statistics->failed % 50U == 0U) {
            log_tx_failure(id, phase, result);
        }
    }
    return result;
}

twai_node_handle_t can_get_node(void)
{
    return s_twai_node;
}

void can_init(void)
{
    s_rx_queue = xQueueCreate(CAN_RX_QUEUE_DEPTH, sizeof(can_rx_item_t));
    s_tx_mutex = xSemaphoreCreateMutex();
    s_tx_done = xSemaphoreCreateBinary();
    ESP_ERROR_CHECK(
        (s_rx_queue != NULL && s_tx_mutex != NULL && s_tx_done != NULL) ?
        ESP_OK : ESP_ERR_NO_MEM);

    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = GPIO_NUM_17,
            .rx = GPIO_NUM_18,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = CAN_BITRATE,
        },
        .fail_retry_cnt = 15,
        .tx_queue_depth = 10,
    };

    twai_event_callbacks_t callbacks = {
        .on_tx_done = on_tx_done,
        .on_rx_done = on_rx_done,
        .on_error = on_error,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_twai_node));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_twai_node, &callbacks, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_twai_node));

    ESP_LOGI(TAG, "CAN initialized: TX=%d RX=%d bitrate=%lu", CAN_TX_GPIO, CAN_RX_GPIO, (unsigned long)CAN_BITRATE);
}

void can_poll(void)
{
    twai_node_status_t status = {0};
    if (twai_node_get_info(s_twai_node, &status, NULL) == ESP_OK) {
        if (status.state == TWAI_ERROR_BUS_OFF && !s_recovery_in_progress) {
            ESP_LOGE(TAG, "CAN bus-off: TEC=%u REC=%u; starting recovery",
                     status.tx_error_count, status.rx_error_count);
            if (twai_node_recover(s_twai_node) == ESP_OK) {
                s_recovery_in_progress = true;
            } else {
                ESP_LOGE(TAG, "CAN bus-off recovery could not start");
            }
        } else if (s_recovery_in_progress && status.state == TWAI_ERROR_ACTIVE) {
            s_recovery_in_progress = false;
            ESP_LOGI(TAG, "CAN bus-off recovery completed");
        }
    }

    const TickType_t now = xTaskGetTickCount();
    const bool bluerov_valid = s_bluerov_odometry.valid &&
        (now - s_bluerov_odometry.last_complete_tick) <
            pdMS_TO_TICKS(ODOMETRY_TIMEOUT_MS);
    const bool torpedo_valid = s_torpedo_odometry.valid &&
        (now - s_torpedo_odometry.last_complete_tick) <
            pdMS_TO_TICKS(ODOMETRY_TIMEOUT_MS);
    const uint8_t valid_mask = (torpedo_valid ? 1U : 0U) |
                               (bluerov_valid ? 2U : 0U);
    if (valid_mask != s_odometry_valid_mask) {
        s_odometry_valid_mask = valid_mask;
        controller_set_odometry_validities(torpedo_valid ? 1U : 0U,
                                           bluerov_valid ? 1U : 0U);
    }

    if (s_last_monitor_tick == 0U ||
        (now - s_last_monitor_tick) >= pdMS_TO_TICKS(MONITOR_LOG_INTERVAL_MS)) {
        s_last_monitor_tick = now;
        ESP_LOGI(TAG,
                 "CAN RX count: control=%" PRIu32 " bluerov=%" PRIu32
                 " torpedo=%" PRIu32 " load=%" PRIu32 " queue_drops=%" PRIu32,
                 s_control_rx_count, s_bluerov_odometry.complete_count,
                 s_torpedo_odometry.complete_count, s_test_load_rx_count,
                 s_rx_queue_drops);
        for (unsigned int i = 0U; i < TX_CHANNEL_COUNT; ++i) {
            const tx_statistics_t *stats = &s_tx_statistics[i];
            const unsigned int id = i == 0U ? CAN_ID_ACTUATOR_A :
                                    i == 1U ? CAN_ID_ACTUATOR_B : CAN_ID_STATUS;
            ESP_LOGI(TAG,
                     "CAN TX id=0x%03X attempts=%" PRIu32 " ok=%" PRIu32
                     " fail=%" PRIu32 " timeout=%" PRIu32
                     " hw_ok=%" PRIu32 " hw_fail=%" PRIu32
                     " last_seq=%u attempt_ms=%" PRIu32
                     " done_ms=%" PRIu32 " max_wait_ms=%" PRIu32,
                     id, stats->attempts, stats->success, stats->failed,
                     stats->timeouts, stats->hw_done_success,
                     stats->hw_done_failed, stats->last_sequence,
                     stats->last_attempt_ms, stats->last_done_ms,
                     stats->max_wait_ms);
        }
        ESP_LOGI(TAG,
                 "CAN error events=%" PRIu32 " arb=%" PRIu32
                 " bit=%" PRIu32 " form=%" PRIu32 " stuff=%" PRIu32
                 " ack=%" PRIu32 " recent_flags=0x%08" PRIx32,
                 s_error_event_count, s_error_statistics.arbitration_lost,
                 s_error_statistics.bit, s_error_statistics.form,
                 s_error_statistics.stuff, s_error_statistics.ack,
                 s_last_error_flags);
    }
}

bool can_try_receive(can_rx_item_t *out_frame)
{
    return out_frame != NULL &&
           xQueueReceive(s_rx_queue, out_frame, 0) == pdTRUE;
}

static void handle_odometry_fragment(odometry_reassembly_t *state,
                                     const can_rx_item_t *msg,
                                     const char *name)
{
    if (msg->length != CAN_FRAME_SIZE) {
        return;
    }
    const uint8_t sequence = msg->data[0];
    const uint8_t fragment = msg->data[1];
    if (fragment >= ODOMETRY_FRAGMENT_COUNT) {
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    if (!state->active || state->sequence != sequence ||
        (now - state->last_fragment_tick) >= pdMS_TO_TICKS(ODOMETRY_TIMEOUT_MS)) {
        state->active = true;
        state->sequence = sequence;
        state->received_mask = 0U;
        memset(state->data, 0, sizeof(state->data));
    }
    state->last_fragment_tick = now;

    const size_t offset = (size_t)fragment * ODOMETRY_FRAGMENT_DATA_SIZE;
    const size_t remaining = ODOMETRY_SIZE - offset;
    const size_t copy_length = remaining < ODOMETRY_FRAGMENT_DATA_SIZE ?
                               remaining : ODOMETRY_FRAGMENT_DATA_SIZE;
    memcpy(&state->data[offset], &msg->data[2], copy_length);
    state->received_mask |= (uint8_t)(1U << fragment);

    if (state->received_mask == ODOMETRY_COMPLETE_MASK) {
        const uint8_t source = msg->header.id == CAN_ID_TORPEDO_ODOMETRY ? 0U : 1U;
        if (!controller_set_odometry_raw(source, state->data)) {
            state->active = false;
            return;
        }
        state->valid = true;
        state->active = false;
        state->last_complete_tick = now;
        ++state->complete_count;
        if (state->complete_count == 1U ||
            (now - state->last_log_tick) >=
                pdMS_TO_TICKS(MONITOR_LOG_INTERVAL_MS)) {
            state->last_log_tick = now;
            log_odometry(name, msg->header.id, state);
        }
    }
}

void can_handle_message(const can_rx_item_t *msg)
{
    if (msg == NULL) {
        return;
    }

    if (msg->length == 0U || msg->header.ide || msg->header.rtr) {
        return;
    }

    switch (msg->header.id) {
        case CAN_ID_CONTROL: {
            if (msg->length < CAN_FRAME_SIZE) {
                ESP_LOGW(TAG, "Discarded short control frame, DLC=%u", msg->length);
                return;
            }

            const uint8_t version = msg->data[0];
            const uint8_t sequence = msg->data[1];
            const uint8_t armed = msg->data[2];
            const uint8_t mode = msg->data[3];
            const uint16_t target_thrust = get_be_u16(&msg->data[4]);

            (void)sequence;
            if (version != 1U) {
                ESP_LOGW(TAG, "Unsupported control version %u", version);
                return;
            }

            if (mode > MODE_PNG) {
                ESP_LOGW(TAG, "Invalid control mode %u", mode);
                return;
            }

            if (armed > 1U || target_thrust > 1000U ||
                msg->data[6] != 0U || msg->data[7] != 0U) {
                ESP_LOGW(TAG, "Invalid control payload");
                return;
            }

            controller_set_cmd(armed, mode, target_thrust);
            watchdog_note_control(1U);
            ++s_control_rx_count;
            const TickType_t now = xTaskGetTickCount();
            if (s_control_rx_count == 1U ||
                (now - s_last_control_log_tick) >=
                    pdMS_TO_TICKS(MONITOR_LOG_INTERVAL_MS)) {
                s_last_control_log_tick = now;
                ESP_LOGI(TAG,
                         "RX control id=0x010 seq=%u count=%" PRIu32 " "
                         "version=%u armed=%u mode=%u target_thrust=%u",
                         sequence, s_control_rx_count, version, armed, mode,
                         target_thrust);
            }
            break;
        }

        case CAN_ID_BLUEROV_ODOMETRY:
            handle_odometry_fragment(&s_bluerov_odometry, msg, "bluerov_odometry");
            break;

        case CAN_ID_TORPEDO_ODOMETRY:
            handle_odometry_fragment(&s_torpedo_odometry, msg, "torpedo_odometry");
            break;

        default:
            break;
    }
}

esp_err_t can_send_actuator_frame(uint8_t seq, uint8_t state, uint16_t thrust, int16_t fin_top, int16_t fin_bottom)
{
    uint8_t data[8] = {0};
    data[0] = seq;
    data[1] = state;
    set_be_u16(&data[2], thrust);
    set_be_u16(&data[4], (uint16_t)(int16_t)fin_top);
    set_be_u16(&data[6], (uint16_t)(int16_t)fin_bottom);
    return can_send_frame(CAN_ID_ACTUATOR_A, data, sizeof(data));
}

esp_err_t can_send_actuator_b_frame(uint8_t seq, uint8_t mode, int16_t fin_left, int16_t fin_right, uint8_t flags)
{
    uint8_t data[8] = {0};
    data[0] = seq;
    data[1] = mode;
    set_be_u16(&data[2], (uint16_t)(int16_t)fin_left);
    set_be_u16(&data[4], (uint16_t)(int16_t)fin_right);
    data[6] = flags;
    return can_send_frame(CAN_ID_ACTUATOR_B, data, sizeof(data));
}

esp_err_t can_send_status_frame(uint8_t seq, uint8_t state, uint8_t mode, uint8_t flags, uint8_t last_error)
{
    uint8_t data[8] = {0};
    data[0] = 1U;
    data[1] = seq;
    data[2] = state;
    data[3] = mode;
    data[4] = flags;
    data[5] = last_error;
    data[6] = 0U;
    data[7] = 0U;
    return can_send_frame(CAN_ID_STATUS, data, sizeof(data));
}

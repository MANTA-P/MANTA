#include <stdint.h>

#include "esp_log.h"
#include "esp_twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can_protocol.h"
#include "controller_state.h"
#include "watchdog.h"

static const char *TAG = "esp32b_fw";

static void periodic_tx_task(void *arg)
{
    (void)arg;

    uint8_t seq = 0U;
    uint32_t output_cycles = 0U;
    uint32_t max_cycle_ms = 0U;
    TickType_t last_log_tick = 0U;
    while (1) {
        const TickType_t cycle_start = xTaskGetTickCount();
        controller_cycle_snapshot_t snapshot = {0};
        controller_get_cycle_snapshot(&snapshot);

        const esp_err_t actuator_a_result =
            can_send_actuator_frame(seq, snapshot.state, snapshot.thrust,
                                    snapshot.fin_top, snapshot.fin_bottom);
        const esp_err_t actuator_b_result =
            can_send_actuator_b_frame(seq, snapshot.mode, snapshot.fin_left,
                                      snapshot.fin_right, snapshot.flags);
        watchdog_note_heartbeat(
            actuator_a_result == ESP_OK && actuator_b_result == ESP_OK);

        (void)can_send_status_frame(seq, snapshot.state, snapshot.mode,
                                    snapshot.flags, snapshot.last_error);

        ++output_cycles;
        const TickType_t now = xTaskGetTickCount();
        const uint32_t cycle_ms =
            (uint32_t)(now - cycle_start) * portTICK_PERIOD_MS;
        if (cycle_ms > max_cycle_ms) {
            max_cycle_ms = cycle_ms;
        }
        if (last_log_tick == 0U ||
            (now - last_log_tick) >= pdMS_TO_TICKS(1000U)) {
            last_log_tick = now;
            ESP_LOGI(TAG, "Output cycles=%lu last_seq=%u cycle_ms=%lu max_cycle_ms=%lu",
                     (unsigned long)output_cycles, seq,
                     (unsigned long)cycle_ms, (unsigned long)max_cycle_ms);
        }
        seq++;
        const TickType_t period = pdMS_TO_TICKS(50U);
        const TickType_t elapsed = xTaskGetTickCount() - cycle_start;
        if (elapsed < period) {
            vTaskDelay(period - elapsed);
        } else {
            taskYIELD();
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32 B starting");

    controller_init();
    can_init();
    watchdog_init();

    xTaskCreate(periodic_tx_task, "tx_task", 4096, NULL, 5, NULL);

    TickType_t last_control_tick = xTaskGetTickCount();
    while (1) {
        can_rx_item_t frame;
        unsigned int processed = 0U;
        while (processed < 32U && can_try_receive(&frame)) {
            can_handle_message(&frame);
            ++processed;
        }

        can_poll();
        const TickType_t now = xTaskGetTickCount();
        if ((now - last_control_tick) >= pdMS_TO_TICKS(50U)) {
            last_control_tick = now;
            controller_tick_20hz();
        }
        watchdog_tick();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

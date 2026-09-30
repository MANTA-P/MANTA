#include "watchdog.h"

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HEARTBEAT_TIMEOUT_MS 500U
#define CONTROL_TIMEOUT_MS 200U

static const char *TAG = "esp32b_watchdog";

static uint32_t control_last_seen_ms;
static uint32_t heartbeat_last_seen_ms;
static uint32_t control_last_log_ms;
static uint32_t heartbeat_last_log_ms;

void watchdog_init(void)
{
    control_last_seen_ms = 0U;
    heartbeat_last_seen_ms = 0U;
    control_last_log_ms = 0U;
    heartbeat_last_log_ms = 0U;
}

void watchdog_note_control(uint8_t valid)
{
    if (valid) {
        control_last_seen_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
}

void watchdog_note_heartbeat(uint8_t valid)
{
    if (valid) {
        heartbeat_last_seen_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
}

void watchdog_tick(void)
{
    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if ((now_ms - control_last_seen_ms) > CONTROL_TIMEOUT_MS &&
        (control_last_log_ms == 0U ||
         (now_ms - control_last_log_ms) >= 1000U)) {
        control_last_log_ms = now_ms;
        ESP_LOGW(TAG, "Control timeout: no valid command for %lu ms", (unsigned long)(now_ms - control_last_seen_ms));
    }

    if ((now_ms - heartbeat_last_seen_ms) > HEARTBEAT_TIMEOUT_MS &&
        (heartbeat_last_log_ms == 0U ||
         (now_ms - heartbeat_last_log_ms) >= 1000U)) {
        heartbeat_last_log_ms = now_ms;
        ESP_LOGW(TAG, "CAN output timeout: no successful actuator pair for %lu ms",
                 (unsigned long)(now_ms - heartbeat_last_seen_ms));
    }
}

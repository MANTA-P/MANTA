#include "controller_state.h"

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "control_algorithm.h"

#define STATE_DISARMED 0U
#define STATE_RUNNING 1U
#define STATE_TIMEOUT 2U
#define STATE_ESTOP 3U
#define STATE_FAULT 4U

#define MODE_NONE 0U
#define MODE_PNG 2U
#define CONTROL_TIMEOUT_MS 200U

typedef struct {
    uint8_t armed;
    uint8_t mode;
    uint16_t target_thrust;
    uint8_t state;
    uint8_t last_error;
    control_algorithm_output_t output;
    bool torpedo_valid;
    bool target_valid;
    bool estop_active;
    bool command_valid;
    bool fault_latched;
    TickType_t last_command_tick;
} controller_ctx_t;

static controller_ctx_t g_ctx;
static SemaphoreHandle_t g_mutex;

static void safe_output(void)
{
    g_ctx.output = (control_algorithm_output_t){0};
}

static void refresh_state(TickType_t now)
{
    if (g_ctx.command_valid &&
        (now - g_ctx.last_command_tick) > pdMS_TO_TICKS(CONTROL_TIMEOUT_MS)) {
        g_ctx.command_valid = false;
    }
    if (g_ctx.estop_active) {
        g_ctx.state = STATE_ESTOP;
    } else if (g_ctx.fault_latched) {
        if (g_ctx.armed == 0U) {
            g_ctx.fault_latched = false;
            g_ctx.last_error = 0U;
            g_ctx.state = STATE_DISARMED;
        } else {
            g_ctx.state = STATE_FAULT;
        }
    } else if (!g_ctx.torpedo_valid || !g_ctx.target_valid || !g_ctx.command_valid) {
        g_ctx.state = STATE_TIMEOUT;
    } else if (g_ctx.armed == 0U || g_ctx.mode == MODE_NONE || g_ctx.target_thrust == 0U) {
        g_ctx.state = STATE_DISARMED;
    } else {
        g_ctx.state = STATE_RUNNING;
    }
    if (g_ctx.state != STATE_RUNNING) safe_output();
}

void controller_init(void)
{
    g_mutex = xSemaphoreCreateMutex();
    configASSERT(g_mutex != NULL);
    g_ctx.state = STATE_DISARMED;
    safe_output();
}

void controller_set_cmd(uint8_t armed, uint8_t mode, uint16_t target_thrust)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (g_ctx.armed != armed || g_ctx.mode != mode ||
        g_ctx.target_thrust != target_thrust) safe_output();
    g_ctx.armed = armed;
    g_ctx.mode = mode <= MODE_PNG ? mode : MODE_NONE;
    g_ctx.target_thrust = target_thrust <= 1000U ? target_thrust : 1000U;
    g_ctx.command_valid = true;
    g_ctx.last_command_tick = xTaskGetTickCount();
    refresh_state(g_ctx.last_command_tick);
    xSemaphoreGive(g_mutex);
}

bool controller_set_odometry_raw(uint8_t source, const uint8_t data[32])
{
    /* CAN receive and the 20 Hz calculation both run in app_main. */
    return control_algorithm_set_odometry(source, data);
}

void controller_set_odometry_validities(uint8_t torpedo_valid, uint8_t target_valid)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_ctx.torpedo_valid = torpedo_valid != 0U;
    g_ctx.target_valid = target_valid != 0U;
    refresh_state(xTaskGetTickCount());
    xSemaphoreGive(g_mutex);
}

void controller_set_estop(uint8_t active)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    g_ctx.estop_active = active != 0U;
    refresh_state(xTaskGetTickCount());
    xSemaphoreGive(g_mutex);
}

void controller_tick_20hz(void)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    refresh_state(xTaskGetTickCount());
    if (g_ctx.state == STATE_RUNNING &&
        !control_algorithm_compute(g_ctx.mode, g_ctx.target_thrust, &g_ctx.output)) {
        g_ctx.fault_latched = true;
        g_ctx.state = STATE_FAULT;
        g_ctx.last_error = 1U; /* invalid algorithm output */
        safe_output();
    }
    xSemaphoreGive(g_mutex);
}

uint8_t controller_get_state(void)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    const uint8_t value = g_ctx.state;
    xSemaphoreGive(g_mutex);
    return value;
}

uint8_t controller_get_mode(void)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    const uint8_t value = g_ctx.mode;
    xSemaphoreGive(g_mutex);
    return value;
}

uint8_t controller_get_armed(void)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    const uint8_t value = g_ctx.armed;
    xSemaphoreGive(g_mutex);
    return value;
}

uint16_t controller_get_target_thrust(void)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    const uint16_t value = g_ctx.target_thrust;
    xSemaphoreGive(g_mutex);
    return value;
}

void controller_get_outputs(uint16_t *thrust, int16_t *fin_top, int16_t *fin_bottom,
                            int16_t *fin_left, int16_t *fin_right)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (thrust != NULL) *thrust = g_ctx.output.thrust;
    if (fin_top != NULL) *fin_top = g_ctx.output.fin_top;
    if (fin_bottom != NULL) *fin_bottom = g_ctx.output.fin_bottom;
    if (fin_left != NULL) *fin_left = g_ctx.output.fin_left;
    if (fin_right != NULL) *fin_right = g_ctx.output.fin_right;
    xSemaphoreGive(g_mutex);
}

void controller_get_status_snapshot(uint8_t *state, uint8_t *mode,
                                    uint8_t *flags, uint8_t *last_error)
{
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    if (state != NULL) *state = g_ctx.state;
    if (mode != NULL) *mode = g_ctx.mode;
    if (flags != NULL) {
        *flags = (g_ctx.torpedo_valid ? 0x01U : 0U) |
                 (g_ctx.target_valid ? 0x02U : 0U) |
                 (g_ctx.command_valid ? 0x04U : 0U) |
                 (g_ctx.estop_active ? 0x08U : 0U);
    }
    if (last_error != NULL) *last_error = g_ctx.last_error;
    xSemaphoreGive(g_mutex);
}

void controller_get_cycle_snapshot(controller_cycle_snapshot_t *snapshot)
{
    if (snapshot == NULL) return;
    xSemaphoreTake(g_mutex, portMAX_DELAY);
    snapshot->thrust = g_ctx.output.thrust;
    snapshot->fin_top = g_ctx.output.fin_top;
    snapshot->fin_bottom = g_ctx.output.fin_bottom;
    snapshot->fin_left = g_ctx.output.fin_left;
    snapshot->fin_right = g_ctx.output.fin_right;
    snapshot->state = g_ctx.state;
    snapshot->mode = g_ctx.mode;
    snapshot->flags = (g_ctx.torpedo_valid ? 0x01U : 0U) |
                      (g_ctx.target_valid ? 0x02U : 0U) |
                      (g_ctx.command_valid ? 0x04U : 0U) |
                      (g_ctx.estop_active ? 0x08U : 0U);
    snapshot->last_error = g_ctx.last_error;
    xSemaphoreGive(g_mutex);
}

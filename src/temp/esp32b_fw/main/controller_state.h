#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t thrust;
    int16_t fin_top;
    int16_t fin_bottom;
    int16_t fin_left;
    int16_t fin_right;
    uint8_t state;
    uint8_t mode;
    uint8_t flags;
    uint8_t last_error;
} controller_cycle_snapshot_t;

void controller_init(void);
void controller_tick_20hz(void);
void controller_set_cmd(uint8_t armed, uint8_t mode, uint16_t target_thrust);
bool controller_set_odometry_raw(uint8_t source, const uint8_t data[32]);
void controller_set_odometry_validities(uint8_t torpedo_valid, uint8_t target_valid);
void controller_set_estop(uint8_t active);

uint8_t controller_get_state(void);
uint8_t controller_get_mode(void);
uint8_t controller_get_armed(void);
uint16_t controller_get_target_thrust(void);
void controller_get_outputs(uint16_t *thrust, int16_t *fin_top, int16_t *fin_bottom, int16_t *fin_left, int16_t *fin_right);
void controller_get_status_snapshot(uint8_t *state, uint8_t *mode, uint8_t *flags, uint8_t *last_error);
void controller_get_cycle_snapshot(controller_cycle_snapshot_t *snapshot);

#ifdef __cplusplus
}
#endif

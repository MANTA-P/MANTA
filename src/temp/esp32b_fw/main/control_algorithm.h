#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t thrust;
    int16_t fin_top;
    int16_t fin_bottom;
    int16_t fin_left;
    int16_t fin_right;
} control_algorithm_output_t;

/* source: 0 = torpedo, 1 = target (BlueROV). Wire data is 32-byte BE odometry. */
bool control_algorithm_set_odometry(uint8_t source, const uint8_t data[32]);
bool control_algorithm_compute(uint8_t mode, uint16_t target_thrust,
                               control_algorithm_output_t *output);

#ifdef __cplusplus
}
#endif

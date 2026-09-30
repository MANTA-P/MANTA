#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void watchdog_init(void);
void watchdog_tick(void);
void watchdog_note_control(uint8_t valid);
void watchdog_note_heartbeat(uint8_t valid);

#ifdef __cplusplus
}
#endif

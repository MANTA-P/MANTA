#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_twai.h"

#ifdef __cplusplus
extern "C" {
#endif

void uart_can_bridge_init(void);
void uart_can_bridge_task(void *arg);
bool uart_can_bridge_try_receive_can(twai_frame_t *out_frame);
void uart_can_bridge_send_can_to_uart(const twai_frame_t *frame);

#ifdef __cplusplus
}
#endif

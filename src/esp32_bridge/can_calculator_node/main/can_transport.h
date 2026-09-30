#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_twai_types.h"

typedef struct {
    twai_frame_header_t header;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} can_transport_frame_t;

esp_err_t can_transport_init(void);
bool can_transport_receive(can_transport_frame_t *frame, uint32_t timeout_ms);
esp_err_t can_transport_send_standard(uint32_t can_id,
                                      const uint8_t *data,
                                      uint8_t data_len,
                                      uint32_t timeout_ms);
void can_transport_process_events(void);

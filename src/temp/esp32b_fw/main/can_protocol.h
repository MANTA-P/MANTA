#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_twai.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    twai_frame_header_t header;
    uint8_t length;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} can_rx_item_t;

twai_node_handle_t can_get_node(void);
void can_init(void);
void can_poll(void);
bool can_try_receive(can_rx_item_t *out_frame);
void can_handle_message(const can_rx_item_t *msg);
esp_err_t can_send_actuator_frame(uint8_t seq, uint8_t state, uint16_t thrust, int16_t fin_top, int16_t fin_bottom);
esp_err_t can_send_actuator_b_frame(uint8_t seq, uint8_t mode, int16_t fin_left, int16_t fin_right, uint8_t flags);
esp_err_t can_send_status_frame(uint8_t seq, uint8_t state, uint8_t mode, uint8_t flags, uint8_t last_error);

#ifdef __cplusplus
}
#endif

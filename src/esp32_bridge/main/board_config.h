#pragma once

#include "driver/gpio.h"

/* ESP32-S3 -> CAN transceiver connection. */
#define CAN_TX_GPIO             GPIO_NUM_17
#define CAN_RX_GPIO             GPIO_NUM_18
#define CAN_BITRATE             500000

/* One standard CAN data frame is sent for each UART character. */
#define CAN_CHAR_FRAME_ID       0x200
#define CAN_TX_QUEUE_DEPTH      5
#define CAN_TX_TIMEOUT_MS       1000

#pragma once

#include "driver/gpio.h"

/* CAN transceiver pins: ESP32 TX -> transceiver TXD, ESP32 RX <- transceiver RXD. */
#define CAN_TX_GPIO             GPIO_NUM_17
#define CAN_RX_GPIO             GPIO_NUM_18
#define CAN_BITRATE             500000

/* The RX callback copies frames here before waking the logging task. */
#define CAN_RX_QUEUE_DEPTH      16


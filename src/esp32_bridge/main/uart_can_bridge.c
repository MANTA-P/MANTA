#include <ctype.h>
#include <stdint.h>

#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_config.h"

static const char *TAG = "uart_can_bridge";
static twai_node_handle_t s_twai_node;

static void usb_serial_init(void)
{
    /*
     * ESP32-S3 USB Serial/JTAG exposes a USB CDC-ACM port on the host.
     * It is not UART1 and does not use GPIO15/GPIO16.
     *
     * The host baud-rate setting is retained for compatibility with serial
     * tools, but USB Serial/JTAG itself has no UART baud clock.
     */
    usb_serial_jtag_driver_config_t config =
        USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&config));
}

static void can_init(void)
{
    const twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CAN_TX_GPIO,
            .rx = CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing = {
            .bitrate = CAN_BITRATE,
        },
        .tx_queue_depth = CAN_TX_QUEUE_DEPTH,
        .fail_retry_cnt = 3,
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_twai_node));
    ESP_ERROR_CHECK(twai_node_enable(s_twai_node));
}

static esp_err_t send_character_to_can(uint8_t character)
{
    /*
     * The ESP-IDF TWAI driver transmits asynchronously. Both this frame and its
     * payload stay on this function's stack until wait_all_done() completes.
     */
    uint8_t payload = character;
    const twai_frame_t frame = {
        .header = {
            .id = CAN_CHAR_FRAME_ID,
            .ide = false, /* Standard 11-bit CAN identifier. */
            .rtr = false,
        },
        .buffer = &payload,
        .buffer_len = sizeof(payload), /* DLC = 1 */
    };

    esp_err_t err = twai_node_transmit(s_twai_node, &frame, CAN_TX_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    return twai_node_transmit_wait_all_done(s_twai_node, CAN_TX_TIMEOUT_MS);
}

void app_main(void)
{
    usb_serial_init();
    can_init();

    ESP_LOGI(TAG, "ready: USB Serial/JTAG -> CAN ID 0x%03X, %d bit/s",
             CAN_CHAR_FRAME_ID,
             CAN_BITRATE);

    while (true) {
        uint8_t character;
        const int received = usb_serial_jtag_read_bytes(&character,
                                                        sizeof(character),
                                                        portMAX_DELAY);
        if (received != 1) {
            continue;
        }

        const esp_err_t err = send_character_to_can(character);
        const char printable = isprint(character) ? (char)character : '.';
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "USB '%c' (0x%02X) -> CAN ID 0x%03X DLC 1 payload[0]=0x%02X",
                     printable, character, CAN_CHAR_FRAME_ID, character);
        } else {
            ESP_LOGE(TAG, "CAN send failed for UART byte 0x%02X: %s",
                     character, esp_err_to_name(err));
        }
    }
}

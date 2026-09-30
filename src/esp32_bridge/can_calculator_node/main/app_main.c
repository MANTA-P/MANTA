#include <inttypes.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include "calculator_node.h"
#include "can_protocol.h"
#include "can_transport.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "role=CALCULATOR bitrate=%" PRIu32 " TX_GPIO=%d RX_GPIO=%d",
             CAN_CALCULATOR_BITRATE,
             CONFIG_CAN_CALCULATOR_TX_GPIO,
             CONFIG_CAN_CALCULATOR_RX_GPIO);
    ESP_ERROR_CHECK(can_transport_init());
    ESP_LOGI(TAG, "CAN calculator node ready");
    calculator_node_run();
}

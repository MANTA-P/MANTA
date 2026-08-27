idf_component_register(
    SRCS "can_rx_monitor.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_twai
)

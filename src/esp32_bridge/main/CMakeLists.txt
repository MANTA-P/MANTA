idf_component_register(
    SRCS "uart_can_bridge.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_usb_serial_jtag esp_driver_twai
)

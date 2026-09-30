idf_component_register(
    SRCS
        "app_main.c"
        "calculator_node.c"
        "can_protocol.c"
        "can_transport.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_gpio esp_driver_twai
)

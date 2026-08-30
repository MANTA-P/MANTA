idf_component_register(
    SRCS "can_sender.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_twai
)

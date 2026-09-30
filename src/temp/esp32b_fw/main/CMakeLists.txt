idf_component_register(
    SRCS
        "app_main.c"
        "can_protocol.c"
        "controller_state.c"
        "watchdog.c"
        "control_algorithm.cpp"
        "control/actuator_mixer.cpp"
        "control/control_core.cpp"
        "control/default_control_config.cpp"
        "control/png_controller.cpp"
        "control/roll_controller.cpp"
        "control/simple_tracking_controller.cpp"
        "control/vector3.cpp"
    INCLUDE_DIRS "."
)

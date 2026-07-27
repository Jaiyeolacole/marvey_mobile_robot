idf_component_register(
    SRCS "robot_state.c"
    INCLUDE_DIRS "include"
    REQUIRES color_sensor freertos esp_timer
)

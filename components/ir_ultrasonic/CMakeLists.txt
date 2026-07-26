idf_component_register(
    SRCS "ir_ultrasonic.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio esp_timer esp_rom log
)

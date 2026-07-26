idf_component_register(
    SRCS "color_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c freertos log
)

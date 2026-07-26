idf_component_register(
    SRCS "backup.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES motor ir_ultrasonic color_sensor freertos log
)

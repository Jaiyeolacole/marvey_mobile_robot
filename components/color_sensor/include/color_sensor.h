#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    COLOR_NONE,
    COLOR_BLACK,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_YELLOW
} color_id_t;

// Configures I2C and the TCS34725 sensor. Returns ESP_OK on success.
esp_err_t color_sensor_init(void);

// Reads raw R/G/B/Clear channel data.
esp_err_t color_sensor_get_raw(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);

// Classifies raw channel data into a color_id_t.
// NOTE: thresholds are a starting point - calibrate for your
// sensor/lighting (see comment in color_sensor.c).
color_id_t color_classify(uint16_t r, uint16_t g, uint16_t b, uint16_t c);

// Human-readable name for logging.
const char *color_name(color_id_t color);

#ifdef __cplusplus
}
#endif

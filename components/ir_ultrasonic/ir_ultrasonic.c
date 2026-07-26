#include "ir_ultrasonic.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

static const char *TAG = "ir_ultrasonic";

// ---------------- IR Line Sensor Pins ----------------
#define IR_LEFT_PIN       32
#define IR_CENTER_L_PIN   33
#define IR_CENTER_R_PIN   25
#define IR_RIGHT_PIN      26

// Many IR modules read HIGH over black, some read LOW (inverted
// comparator output). Flip this to 0 if line following behaves
// backwards on your modules.
#define LINE_DETECTED_LEVEL 1

// ---------------- HC-SR04 Pins ----------------
#define TRIG_PIN 18
#define ECHO_PIN 19

// NOTE: HC-SR04 ECHO outputs 5V logic. ESP32 GPIOs are only 3.3V
// tolerant. Use a voltage divider (e.g. 1k + 2k resistors) or a
// logic level shifter between ECHO and GPIO19 before powering on.

#define ECHO_TIMEOUT_US 30000  // ~30ms -> ~5m max range

void ir_ultrasonic_init(void)
{
    gpio_config_t ir_conf = {
        .pin_bit_mask = (1ULL << IR_LEFT_PIN) | (1ULL << IR_CENTER_L_PIN) |
                         (1ULL << IR_CENTER_R_PIN) | (1ULL << IR_RIGHT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ir_conf);

    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_conf);

    gpio_set_level(TRIG_PIN, 0);
    ESP_LOGI(TAG, "IR + ultrasonic sensors initialized.");
}

ir_state_t ir_read_all(void)
{
    ir_state_t s;
    s.left         = gpio_get_level(IR_LEFT_PIN)     == LINE_DETECTED_LEVEL;
    s.center_left  = gpio_get_level(IR_CENTER_L_PIN) == LINE_DETECTED_LEVEL;
    s.center_right = gpio_get_level(IR_CENTER_R_PIN) == LINE_DETECTED_LEVEL;
    s.right        = gpio_get_level(IR_RIGHT_PIN)    == LINE_DETECTED_LEVEL;
    return s;
}

float ultrasonic_get_distance_cm(void)
{
    // Trigger a 10us pulse
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    // Wait for ECHO to go high (start of pulse)
    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - wait_start > ECHO_TIMEOUT_US) {
            return -1.0f; // sensor never responded
        }
    }

    // Measure how long ECHO stays high
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > ECHO_TIMEOUT_US) {
            return -1.0f; // out of range
        }
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t duration_us = echo_end - echo_start;
    return (float)duration_us * 0.0343f / 2.0f; // speed of sound -> cm
}

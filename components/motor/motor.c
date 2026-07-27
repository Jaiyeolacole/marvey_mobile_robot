#include "motor.h"
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char *TAG = "motor";

// ---------------- L298N Pin Map ----------------
// NOTE: IN3 uses GPIO12, a boot-strapping pin. If you see random
// boot failures after wiring the motor driver, move IN3 to
// another free pin (e.g. GPIO5 or GPIO17).
#define IN1_PIN 14
#define IN2_PIN 27
#define IN3_PIN 12
#define IN4_PIN 4
#define ENA_PIN 13
#define ENB_PIN 15

#define PWM_FREQ_HZ   5000
#define PWM_RES_BITS  LEDC_TIMER_8_BIT   // 0-255 duty range
#define LEDC_MODE     LEDC_LOW_SPEED_MODE
#define ENA_CHANNEL   LEDC_CHANNEL_0
#define ENB_CHANNEL   LEDC_CHANNEL_1

void motor_init(void)
{
    gpio_config_t dir_conf = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) |
                         (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_conf);

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ena_conf = {
        .gpio_num = ENA_PIN,
        .speed_mode = LEDC_MODE,
        .channel = ENA_CHANNEL,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&ena_conf);

    ledc_channel_config_t enb_conf = ena_conf;
    enb_conf.gpio_num = ENB_PIN;
    enb_conf.channel = ENB_CHANNEL;
    ledc_channel_config(&enb_conf);

    motor_stop();
    ESP_LOGI(TAG, "Motor driver initialized.");
}

static void set_side(int speed, int in_pin_fwd, int in_pin_rev, ledc_channel_t channel)
{
    if (speed > 255) speed = 255;
    if (speed < -255) speed = -255;

    gpio_set_level(in_pin_fwd, speed >= 0 ? 1 : 0);
    gpio_set_level(in_pin_rev, speed >= 0 ? 0 : 1);

    ledc_set_duty(LEDC_MODE, channel, abs(speed));
    ledc_update_duty(LEDC_MODE, channel);
}

void motor_set(int left_speed, int right_speed)
{
    set_side(left_speed, IN1_PIN, IN2_PIN, ENA_CHANNEL);
    set_side(right_speed, IN3_PIN, IN4_PIN, ENB_CHANNEL);
}

void motor_stop(void)
{
    motor_set(0, 0);
}

void motor_brake(void)
{
    // Active braking: hold both inputs on each side HIGH while duty is
    // high. This shorts each motor's terminals through the H-bridge,
    // which resists motor rotation electrically - a noticeably faster
    // stop than motor_stop() (which just cuts power and lets it coast).
    gpio_set_level(IN1_PIN, 1);
    gpio_set_level(IN2_PIN, 1);
    gpio_set_level(IN3_PIN, 1);
    gpio_set_level(IN4_PIN, 1);

    ledc_set_duty(LEDC_MODE, ENA_CHANNEL, 255);
    ledc_update_duty(LEDC_MODE, ENA_CHANNEL);
    ledc_set_duty(LEDC_MODE, ENB_CHANNEL, 255);
    ledc_update_duty(LEDC_MODE, ENB_CHANNEL);
}

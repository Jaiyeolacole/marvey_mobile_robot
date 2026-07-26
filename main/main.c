#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor.h"
#include "ir_ultrasonic.h"
#include "color_sensor.h"

static const char *TAG = "delivery_robot";

// ---------------- Driving ----------------
#define BASE_SPEED     160   // cruising speed on open floor
#define TURN_TRIM      110   // gentle boundary correction
#define SHARP_TRIM      70   // sharp boundary correction

// ---------------- Obstacle avoidance ----------------
#define TURN_SPEED             150
#define OBSTACLE_DISTANCE_CM    15
#define TURN_90_DURATION_MS    500   // calibrate for your chassis

// ---------------- Color / destination logic ----------------
#define COLOR_STABLE_READS       3

#define LOOP_DELAY_MS            10   // single delay for the whole loop - required
                                        // minimum for FreeRTOS/watchdog, not pacing

// Ultrasonic is mounted facing the opposite way from the motors'
// original "forward" wiring.
#define FORWARD_SIGN (-1)

static void turn_90_degrees(void)
{
    motor_set(FORWARD_SIGN * TURN_SPEED, -FORWARD_SIGN * TURN_SPEED);
    vTaskDelay(pdMS_TO_TICKS(TURN_90_DURATION_MS)); // inherent to the maneuver itself
    motor_stop();
}

void app_main(void)
{
    motor_init();
    ir_ultrasonic_init();

    if (color_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Color sensor init failed - halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    bool entry_seen = false;
    bool robot_stopped = false;
    bool waiting_for_clear = false; // true while still sitting on the same color patch
    color_id_t last_color = COLOR_NONE;
    int color_match_count = 0;

    ESP_LOGI(TAG, "Robot ready.");

    while (1) {
        if (robot_stopped) {
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            continue;
        }

        // 1. Obstacle avoidance - highest priority
        float distance = ultrasonic_get_distance_cm();
        if (distance >= 0.0f && distance <= OBSTACLE_DISTANCE_CM) {
            ESP_LOGI(TAG, "Obstacle at ~%.1f cm - turning 90 degrees.", distance);
            turn_90_degrees();
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            continue;
        }

        // 2. Color / destination check
        uint16_t r, g, b, c;
        if (color_sensor_get_raw(&r, &g, &b, &c) == ESP_OK) {
            color_id_t color = color_classify(r, g, b, c);

            if (color == COLOR_NONE || color == COLOR_BLACK) {
                color_match_count = 0;
                last_color = COLOR_NONE;
                waiting_for_clear = false; // floor clear again - ready for next marker
            } else if (!waiting_for_clear) {
                color_match_count = (color == last_color) ? color_match_count + 1 : 1;
                last_color = color;

                if (color_match_count >= COLOR_STABLE_READS) {
                    if (!entry_seen) {
                        entry_seen = true;
                        ESP_LOGI(TAG, "Entry marker detected: %s - continuing.",
                                 color_name(color));
                    } else {
                        motor_stop();
                        robot_stopped = true;
                        ESP_LOGI(TAG, "Destination reached: %s zone. Stopping for delivery.",
                                 color_name(color));
                        continue;
                    }
                    waiting_for_clear = true; // ignore this same patch until we leave it
                    color_match_count = 0;
                }
            }
        }

        ir_state_t ir = ir_read_all();
        int left_speed = BASE_SPEED;
        int right_speed = BASE_SPEED;

        bool left_side_black  = ir.left || ir.center_left;
        bool right_side_black = ir.right || ir.center_right;

        if (left_side_black && right_side_black) {
            left_speed = 0;
            right_speed = 0;
            ESP_LOGW(TAG, "Black on both sides - stopping.");
        } else if (ir.left) {
            right_speed = BASE_SPEED - SHARP_TRIM;
        } else if (ir.center_left) {
            right_speed = BASE_SPEED - TURN_TRIM;
        } else if (ir.right) {
            left_speed = BASE_SPEED - SHARP_TRIM;
        } else if (ir.center_right) {
            left_speed = BASE_SPEED - TURN_TRIM;
        }

        motor_set(left_speed, right_speed);

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}
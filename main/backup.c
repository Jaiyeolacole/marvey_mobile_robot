// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// #include "motor.h"
// #include "ir_ultrasonic.h"
// #include "color_sensor.h"

// static const char *TAG = "delivery_robot";

// // ---------------- Tuning Constants ----------------
// #define BASE_SPEED           160  // 0-255 cruising speed
// #define TURN_TRIM            110  // slow inner wheel on gentle correction
// #define SHARP_TRIM           70   // slow further on sharp correction
// #define OBSTACLE_DISTANCE_CM 15   // stop threshold
// #define COLOR_STABLE_READS   3    // consecutive matching reads before acting (debounce)
// #define LOOP_DELAY_MS        20

// // =========================================================
// // Assumption (see color_sensor.c / color_classify comments
// // for calibration, and the block below for behavior):
// //   - The FIRST distinct color seen is the ENTRY marker -> log only.
// //   - Once BLACK has been confirmed at least once (i.e. we're
// //     truly on the line), the NEXT distinct color seen is the
// //     END marker -> stop for delivery.
// // To customize per-color behavior instead of a full stop, edit
// // the "destination reached" branch below.
// // =========================================================

// void app_main(void)
// {
//     motor_init();
//     ir_ultrasonic_init();

//     if (color_sensor_init() != ESP_OK) {
//         ESP_LOGE(TAG, "Color sensor init failed - halting.");
//         while (1) {
//             vTaskDelay(pdMS_TO_TICKS(1000));
//         }
//     }

//     bool on_line_confirmed = false;
//     bool robot_stopped = false;
//     color_id_t last_color = COLOR_NONE;
//     int color_match_count = 0;

//     ESP_LOGI(TAG, "Robot ready.");

//     while (1) {
//         if (robot_stopped) {
//             // Destination reached - stay halted. Power-cycle or add
//             // your own reset condition here if you want it to run again.
//             vTaskDelay(pdMS_TO_TICKS(500));
//             continue;
//         }

//         // 1. Obstacle check - highest priority, always runs
//         float distance = ultrasonic_get_distance_cm();
//         if (distance >= 0.0f && distance <= OBSTACLE_DISTANCE_CM) {
//             motor_stop();
//             ESP_LOGI(TAG, "Obstacle at ~%.1f cm - holding position.", distance);
//             vTaskDelay(pdMS_TO_TICKS(150));
//             continue; // re-check next loop instead of moving blindly
//         }

//         // 2. Color check
//         uint16_t r, g, b, c;
//         if (color_sensor_get_raw(&r, &g, &b, &c) == ESP_OK) {
//             color_id_t color = color_classify(r, g, b, c);

//             if (color == COLOR_BLACK) {
//                 on_line_confirmed = true;
//                 color_match_count = 0;
//                 last_color = COLOR_NONE;
//             } else if (color == COLOR_NONE) {
//                 color_match_count = 0;
//                 last_color = COLOR_NONE;
//             } else {
//                 // Debounce: require the same color for a few consecutive reads
//                 color_match_count = (color == last_color) ? color_match_count + 1 : 1;
//                 last_color = color;

//                 if (color_match_count >= COLOR_STABLE_READS) {
//                     if (!on_line_confirmed) {
//                         ESP_LOGI(TAG, "Entry marker detected: %s - continuing.",
//                                  color_name(color));
//                         // Not stopping here - treated as the start marker.
//                     } else {
//                         motor_stop();
//                         robot_stopped = true;
//                         ESP_LOGI(TAG, "Destination reached: %s zone. Stopping for delivery.",
//                                  color_name(color));
//                         continue;
//                     }
//                 }
//             }
//         }

//         // 3. Line following
//         ir_state_t ir = ir_read_all();
//         int left_speed = BASE_SPEED;
//         int right_speed = BASE_SPEED;

//         if (ir.center_left && ir.center_right) {
//             // centered - go straight
//         } else if (ir.left && !ir.center_left && !ir.center_right && !ir.right) {
//             // drifted right, line far left -> hard left turn
//             left_speed = BASE_SPEED - SHARP_TRIM;
//         } else if (ir.right && !ir.center_left && !ir.center_right && !ir.left) {
//             // drifted left, line far right -> hard right turn
//             right_speed = BASE_SPEED - SHARP_TRIM;
//         } else if (ir.center_left && !ir.center_right) {
//             // slightly left of center -> gentle left correction
//             left_speed = BASE_SPEED - TURN_TRIM;
//         } else if (ir.center_right && !ir.center_left) {
//             // slightly right of center -> gentle right correction
//             right_speed = BASE_SPEED - TURN_TRIM;
//         } else if (!ir.left && !ir.center_left && !ir.center_right && !ir.right) {
//             // line lost entirely - stop rather than drive blind.
//             // (You could add a "search" routine here: pivot toward the
//             // last known direction until a sensor re-detects the line.)
//             left_speed = 0;
//             right_speed = 0;
//         }

//         motor_set(left_speed, right_speed);

//         vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
//     }
// }

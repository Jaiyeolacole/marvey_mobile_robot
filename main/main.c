#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "motor.h"
#include "ir_ultrasonic.h"
#include "color_sensor.h"
#include "robot_state.h"
#include "web_server.h"

static const char *TAG = "delivery_robot";

// ---------------- Boundary avoidance (autonomous driving) ----------------
#define BASE_SPEED     160
#define TURN_TRIM      110
#define SHARP_TRIM      70

// ---------------- Obstacle avoidance ----------------
#define TURN_SPEED             150
#define OBSTACLE_DISTANCE_CM    15
#define TURN_90_DURATION_MS    500   // calibrate for your chassis

// ---------------- Color / mission logic ----------------
#define COLOR_STABLE_READS       3

// ---------------- Manual mode safety ----------------
#define MANUAL_CMD_TIMEOUT_US 500000  // auto-stop if no fresh command in 500ms

#define LOOP_DELAY_MS 10  // minimum yield for FreeRTOS/watchdog - not used for pacing

// Ultrasonic is mounted facing the opposite way from the motors'
// original "forward" wiring.
#define FORWARD_SIGN (-1)

static void turn_90_degrees(void)
{
    motor_set(FORWARD_SIGN * TURN_SPEED, -FORWARD_SIGN * TURN_SPEED);
    vTaskDelay(pdMS_TO_TICKS(TURN_90_DURATION_MS)); // inherent to the maneuver itself
    motor_stop();
}

static void apply_manual_command(manual_direction_t dir, int pwm)
{
    int fwd = FORWARD_SIGN * pwm;
    switch (dir) {
        case MANUAL_FORWARD:  motor_set(fwd, fwd); break;
        case MANUAL_BACKWARD: motor_set(-fwd, -fwd); break;
        case MANUAL_LEFT:     motor_set(fwd / 2, fwd); break;   // arc turn
        case MANUAL_RIGHT:    motor_set(fwd, fwd / 2); break;   // arc turn
        case MANUAL_SPIN_CW:  motor_set(fwd, -fwd); break;      // pivot in place
        case MANUAL_SPIN_CCW: motor_set(-fwd, fwd); break;      // pivot in place
        default:              motor_stop(); break;
    }
}

static void run_manual_mode(void)
{
    manual_direction_t dir;
    int pwm;
    int64_t age_us;
    robot_state_get_manual_command(&dir, &pwm, &age_us);

    if (dir == MANUAL_STOP || age_us > MANUAL_CMD_TIMEOUT_US) {
        motor_stop();
    } else {
        apply_manual_command(dir, pwm);
    }

    robot_state_set_telemetry("manual", -1.0f, COLOR_NONE);
}

static void run_autonomous_mode(void)
{
    if (!robot_state_is_mission_running()) {
        motor_stop();
        robot_state_set_telemetry("idle", -1.0f, COLOR_NONE);
        return;
    }

    color_id_t target = robot_state_get_target_color();

    // 1. Obstacle avoidance - highest priority
    float distance = ultrasonic_get_distance_cm();
    if (distance >= 0.0f && distance <= OBSTACLE_DISTANCE_CM) {
        robot_state_set_telemetry("obstacle", distance, COLOR_NONE);
        ESP_LOGI(TAG, "Obstacle at ~%.1f cm - turning 90 degrees.", distance);
        turn_90_degrees();
        return;
    }

    // 2. Target color check
    static color_id_t last_seen = COLOR_NONE;
    static int match_count = 0;

    uint16_t r, g, b, c;
    if (color_sensor_get_raw(&r, &g, &b, &c) == ESP_OK) {
        color_id_t detected = color_classify(r, g, b, c);

        if (detected == target) {
            match_count = (detected == last_seen) ? match_count + 1 : 1;
            last_seen = detected;

            if (match_count >= COLOR_STABLE_READS) {
                motor_stop();
                robot_state_stop_mission();
                robot_state_set_telemetry("arrived", distance, target);
                ESP_LOGI(TAG, "Target color %s reached - mission complete.",
                         color_to_lower_string(target));
                match_count = 0;
                return;
            }
        } else {
            match_count = 0;
            last_seen = COLOR_NONE;
        }
    }

    // 3. Boundary avoidance (stay on light floor between the black guide lines)
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
    robot_state_set_telemetry("searching", distance, COLOR_NONE);
}

void app_main(void)
{
    motor_init();
    ir_ultrasonic_init();
    robot_state_init();

    if (color_sensor_init() != ESP_OK) {
        ESP_LOGE(TAG, "Color sensor init failed - halting.");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    if (web_server_start() != ESP_OK) {
        ESP_LOGE(TAG, "Web server failed to start - check WiFi credentials "
                       "(idf.py menuconfig) and signal. Continuing without WiFi control.");
    }

    ESP_LOGI(TAG, "Robot ready.");

    while (1) {
        if (robot_state_get_mode() == ROBOT_MODE_MANUAL) {
            run_manual_mode();
        } else {
            run_autonomous_mode();
        }
        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

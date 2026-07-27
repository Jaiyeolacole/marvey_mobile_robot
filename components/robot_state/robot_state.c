#include "robot_state.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

static SemaphoreHandle_t s_mutex;

static robot_mode_t s_mode = ROBOT_MODE_AUTONOMOUS;
static bool s_mission_running = false;
static color_id_t s_target_color = COLOR_NONE;

static manual_direction_t s_manual_dir = MANUAL_STOP;
static int s_manual_pwm = 0;
static int64_t s_manual_cmd_time_us = 0;

static char s_behavior_state[32] = "idle";
static float s_distance_cm = -1.0f;
static color_id_t s_last_color = COLOR_NONE;

void robot_state_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    s_manual_cmd_time_us = esp_timer_get_time();
}

void robot_state_set_mode(robot_mode_t mode)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_mode = mode;
    if (mode == ROBOT_MODE_MANUAL) {
        s_mission_running = false; // switching to manual cancels any running mission
    }
    xSemaphoreGive(s_mutex);
}

robot_mode_t robot_state_get_mode(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    robot_mode_t m = s_mode;
    xSemaphoreGive(s_mutex);
    return m;
}

void robot_state_start_mission(color_id_t target_color)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_target_color = target_color;
    s_mission_running = true;
    s_mode = ROBOT_MODE_AUTONOMOUS;
    xSemaphoreGive(s_mutex);
}

void robot_state_stop_mission(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_mission_running = false;
    xSemaphoreGive(s_mutex);
}

bool robot_state_is_mission_running(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool r = s_mission_running;
    xSemaphoreGive(s_mutex);
    return r;
}

color_id_t robot_state_get_target_color(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    color_id_t c = s_target_color;
    xSemaphoreGive(s_mutex);
    return c;
}

void robot_state_set_manual_command(manual_direction_t dir, int pwm)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_manual_dir = dir;
    s_manual_pwm = pwm;
    s_manual_cmd_time_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
}

void robot_state_get_manual_command(manual_direction_t *dir, int *pwm, int64_t *age_us)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (dir) *dir = s_manual_dir;
    if (pwm) *pwm = s_manual_pwm;
    if (age_us) *age_us = esp_timer_get_time() - s_manual_cmd_time_us;
    xSemaphoreGive(s_mutex);
}

void robot_state_set_telemetry(const char *behavior_state, float distance_cm, color_id_t last_color)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    strncpy(s_behavior_state, behavior_state, sizeof(s_behavior_state) - 1);
    s_behavior_state[sizeof(s_behavior_state) - 1] = '\0';
    s_distance_cm = distance_cm;
    s_last_color = last_color;
    xSemaphoreGive(s_mutex);
}

void robot_state_get_telemetry(char *behavior_state_out, size_t len, float *distance_cm, color_id_t *last_color)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (behavior_state_out && len > 0) {
        strncpy(behavior_state_out, s_behavior_state, len - 1);
        behavior_state_out[len - 1] = '\0';
    }
    if (distance_cm) *distance_cm = s_distance_cm;
    if (last_color) *last_color = s_last_color;
    xSemaphoreGive(s_mutex);
}

void robot_state_estop(void)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_mission_running = false;
    s_manual_dir = MANUAL_STOP;
    s_manual_pwm = 0;
    s_manual_cmd_time_us = esp_timer_get_time();
    xSemaphoreGive(s_mutex);
}

manual_direction_t manual_direction_from_string(const char *s)
{
    if (!s) return MANUAL_STOP;
    if (strcmp(s, "forward") == 0)   return MANUAL_FORWARD;
    if (strcmp(s, "backward") == 0)  return MANUAL_BACKWARD;
    if (strcmp(s, "left") == 0)      return MANUAL_LEFT;
    if (strcmp(s, "right") == 0)     return MANUAL_RIGHT;
    if (strcmp(s, "spin_cw") == 0)   return MANUAL_SPIN_CW;
    if (strcmp(s, "spin_ccw") == 0)  return MANUAL_SPIN_CCW;
    return MANUAL_STOP;
}

const char *manual_direction_to_string(manual_direction_t d)
{
    switch (d) {
        case MANUAL_FORWARD:  return "forward";
        case MANUAL_BACKWARD: return "backward";
        case MANUAL_LEFT:     return "left";
        case MANUAL_RIGHT:    return "right";
        case MANUAL_SPIN_CW:  return "spin_cw";
        case MANUAL_SPIN_CCW: return "spin_ccw";
        default:              return "stop";
    }
}

color_id_t color_from_string(const char *s)
{
    if (!s) return COLOR_NONE;
    if (strcmp(s, "red") == 0)    return COLOR_RED;
    if (strcmp(s, "green") == 0)  return COLOR_GREEN;
    if (strcmp(s, "blue") == 0)   return COLOR_BLUE;
    if (strcmp(s, "yellow") == 0) return COLOR_YELLOW;
    if (strcmp(s, "black") == 0)  return COLOR_BLACK;
    return COLOR_NONE;
}

const char *color_to_lower_string(color_id_t c)
{
    switch (c) {
        case COLOR_RED:    return "red";
        case COLOR_GREEN:  return "green";
        case COLOR_BLUE:   return "blue";
        case COLOR_YELLOW: return "yellow";
        case COLOR_BLACK:  return "black";
        default:           return "none";
    }
}

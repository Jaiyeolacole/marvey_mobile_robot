#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "color_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROBOT_MODE_AUTONOMOUS,
    ROBOT_MODE_MANUAL
} robot_mode_t;

typedef enum {
    MANUAL_STOP,
    MANUAL_FORWARD,
    MANUAL_BACKWARD,
    MANUAL_LEFT,
    MANUAL_RIGHT,
    MANUAL_SPIN_CW,
    MANUAL_SPIN_CCW
} manual_direction_t;

// Must be called once before any other robot_state_* function.
void robot_state_init(void);

// ---------------- Mode ----------------
void robot_state_set_mode(robot_mode_t mode);
robot_mode_t robot_state_get_mode(void);

// ---------------- Autonomous mission ----------------
void robot_state_start_mission(color_id_t target_color);
void robot_state_stop_mission(void);
bool robot_state_is_mission_running(void);
color_id_t robot_state_get_target_color(void);

// ---------------- Manual driving ----------------
// Called by the HTTP handler whenever a move command arrives.
void robot_state_set_manual_command(manual_direction_t dir, int pwm);
// Called by the control loop. age_us tells the caller how long ago this
// command was received - used to implement the auto-stop safety timeout.
void robot_state_get_manual_command(manual_direction_t *dir, int *pwm, int64_t *age_us);

// ---------------- Telemetry (written by control loop, read by /api/status) ----------------
void robot_state_set_telemetry(const char *behavior_state, float distance_cm, color_id_t last_color);
void robot_state_get_telemetry(char *behavior_state_out, size_t len, float *distance_cm, color_id_t *last_color);

// ---------------- Emergency stop ----------------
// Cancels any running mission and zeroes the manual command. The caller
// (the HTTP handler) is still responsible for calling motor_stop()
// immediately for the fastest possible reaction.
void robot_state_estop(void);

// ---------------- String <-> enum helpers (for JSON parsing/serializing) ----------------
manual_direction_t manual_direction_from_string(const char *s);
const char *manual_direction_to_string(manual_direction_t d);
color_id_t color_from_string(const char *s);       // "red"/"green"/"blue"/"yellow"/"black"
const char *color_to_lower_string(color_id_t c);   // -> "red"/"green"/.../"none"

#ifdef __cplusplus
}
#endif

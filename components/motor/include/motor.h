#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Initializes GPIOs and LEDC PWM channels for the L298N driver.
void motor_init(void);

// Sets wheel speeds. Range: -255 (full reverse) to 255 (full forward).
void motor_set(int left_speed, int right_speed);

// Convenience wrapper for motor_set(0, 0). Cuts power - the robot
// coasts to a stop (slower, uses momentum).
void motor_stop(void);

// Active braking - shorts each motor's terminals through the H-bridge
// for a much faster stop than motor_stop(). Use this when you need to
// halt quickly (e.g. obstacle very close), not for routine stopping.
void motor_brake(void);

#ifdef __cplusplus
}
#endif
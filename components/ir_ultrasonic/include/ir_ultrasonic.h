#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool left;
    bool center_left;
    bool center_right;
    bool right;
} ir_state_t;

// Configures GPIOs for the 4 IR sensors and the HC-SR04.
void ir_ultrasonic_init(void);

// Reads all 4 IR line sensors. true = line detected (see
// LINE_DETECTED_LEVEL in ir_ultrasonic.c if readings look inverted).
ir_state_t ir_read_all(void);

// Triggers the HC-SR04 and returns distance in cm.
// Returns -1.0f on timeout / out-of-range.
float ultrasonic_get_distance_cm(void);

#ifdef __cplusplus
}
#endif

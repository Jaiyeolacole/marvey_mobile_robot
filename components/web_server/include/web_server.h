#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Connects to WiFi (blocking until connected or all retries exhausted)
// and starts the HTTP server exposing the control panel UI at "/" and
// the robot control REST API under "/api/...". Returns ESP_FAIL if
// WiFi connection or server startup fails.
esp_err_t web_server_start(void);

#ifdef __cplusplus
}
#endif

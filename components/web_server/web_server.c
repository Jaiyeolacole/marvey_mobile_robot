#include "web_server.h"
#include <string.h>
#include <stdio.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "robot_state.h"
#include "motor.h"

static const char *TAG = "web_server";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_MAX_RETRY     10

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

// The control panel HTML, embedded directly into the firmware binary
// from components/web_server/www/index.html (see CMakeLists.txt).
extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

// =========================================================
//                    WIFI (station mode)
// =========================================================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "Retrying WiFi connection (%d/%d)...", s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "Connected. Robot IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Open http://" IPSTR "/ in a browser, or enter that IP in the control panel.",
                 IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                          &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                          &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "Marv",
            .password = "12345678",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to WiFi SSID: %s", CONFIG_WIFI_SSID);

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                            pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Failed to connect to WiFi after %d retries.", WIFI_MAX_RETRY);
    return ESP_FAIL;
}

// =========================================================
//                   HTTP helpers
// =========================================================
static cJSON *parse_body(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 511) return NULL;
    char buf[512];
    int received = httpd_req_recv(req, buf, req->content_len);
    if (received <= 0) return NULL;
    buf[received] = '\0';
    return cJSON_Parse(buf);
}

static esp_err_t send_json_ok(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t send_json_error(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_status(req, "400 Bad Request");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"%s\"}", msg);
    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// =========================================================
//                   Route handlers
// =========================================================
static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_end - index_html_start);
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req)
{
    char behavior_state[32];
    float distance_cm;
    color_id_t last_color;
    robot_state_get_telemetry(behavior_state, sizeof(behavior_state), &distance_cm, &last_color);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddBoolToObject(root, "connected", true);
    cJSON_AddStringToObject(root, "mode",
        robot_state_get_mode() == ROBOT_MODE_AUTONOMOUS ? "autonomous" : "manual");
    cJSON_AddStringToObject(root, "state", behavior_state);
    cJSON_AddNumberToObject(root, "distance_cm", distance_cm);
    cJSON_AddStringToObject(root, "last_color", color_to_lower_string(last_color));

    char *json_str = cJSON_PrintUnformatted(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, json_str, HTTPD_RESP_USE_STRLEN);
    free(json_str);
    cJSON_Delete(root);
    return ESP_OK;
}

static esp_err_t mode_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_body(req);
    if (!body) return send_json_error(req, "invalid_body");

    cJSON *mode_item = cJSON_GetObjectItem(body, "mode");
    if (!cJSON_IsString(mode_item)) {
        cJSON_Delete(body);
        return send_json_error(req, "missing_mode");
    }

    robot_state_set_mode(strcmp(mode_item->valuestring, "manual") == 0
                              ? ROBOT_MODE_MANUAL : ROBOT_MODE_AUTONOMOUS);
    cJSON_Delete(body);
    return send_json_ok(req);
}

static esp_err_t autonomous_start_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_body(req);
    if (!body) return send_json_error(req, "invalid_body");

    cJSON *color_item = cJSON_GetObjectItem(body, "target_color");
    if (!cJSON_IsString(color_item)) {
        cJSON_Delete(body);
        return send_json_error(req, "missing_target_color");
    }

    color_id_t target = color_from_string(color_item->valuestring);
    cJSON_Delete(body);
    if (target == COLOR_NONE) return send_json_error(req, "invalid_color");

    robot_state_start_mission(target);
    ESP_LOGI(TAG, "Mission started via API - target color: %s", color_to_lower_string(target));
    return send_json_ok(req);
}

static esp_err_t autonomous_stop_post_handler(httpd_req_t *req)
{
    robot_state_stop_mission();
    motor_stop();
    ESP_LOGI(TAG, "Mission stopped via API.");
    return send_json_ok(req);
}

static esp_err_t manual_move_post_handler(httpd_req_t *req)
{
    cJSON *body = parse_body(req);
    if (!body) return send_json_error(req, "invalid_body");

    cJSON *dir_item = cJSON_GetObjectItem(body, "direction");
    cJSON *pwm_item = cJSON_GetObjectItem(body, "pwm");
    if (!cJSON_IsString(dir_item)) {
        cJSON_Delete(body);
        return send_json_error(req, "missing_direction");
    }

    int pwm = cJSON_IsNumber(pwm_item) ? pwm_item->valueint : 0;
    if (pwm < 0) pwm = 0;
    if (pwm > 255) pwm = 255;

    manual_direction_t dir = manual_direction_from_string(dir_item->valuestring);
    cJSON_Delete(body);

    if (robot_state_get_mode() != ROBOT_MODE_MANUAL) {
        return send_json_error(req, "not_in_manual_mode");
    }

    robot_state_set_manual_command(dir, pwm);
    return send_json_ok(req);
}

static esp_err_t estop_post_handler(httpd_req_t *req)
{
    robot_state_estop();
    motor_stop();
    ESP_LOGW(TAG, "EMERGENCY STOP triggered via API.");
    return send_json_ok(req);
}

// =========================================================
//                   Server startup
// =========================================================
static httpd_handle_t start_httpd(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server.");
        return NULL;
    }

    httpd_uri_t routes[] = {
        { .uri = "/",                      .method = HTTP_GET,  .handler = root_get_handler },
        { .uri = "/api/status",            .method = HTTP_GET,  .handler = status_get_handler },
        { .uri = "/api/mode",              .method = HTTP_POST, .handler = mode_post_handler },
        { .uri = "/api/autonomous/start",  .method = HTTP_POST, .handler = autonomous_start_post_handler },
        { .uri = "/api/autonomous/stop",   .method = HTTP_POST, .handler = autonomous_stop_post_handler },
        { .uri = "/api/manual/move",       .method = HTTP_POST, .handler = manual_move_post_handler },
        { .uri = "/api/estop",             .method = HTTP_POST, .handler = estop_post_handler },
    };

    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    return server;
}

esp_err_t web_server_start(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (wifi_init_sta() != ESP_OK) {
        return ESP_FAIL;
    }

    if (start_httpd() == NULL) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Web server ready.");
    return ESP_OK;
}

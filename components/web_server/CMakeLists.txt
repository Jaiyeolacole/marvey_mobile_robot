idf_component_register(
    SRCS "web_server.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_http_server robot_state motor log
    EMBED_TXTFILES "www/index.html"
)

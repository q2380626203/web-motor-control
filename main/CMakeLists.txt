idf_component_register(SRCS "main.c" "motor_control.c" "wifi_http_server.c" "web_interface.c"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_driver_uart esp_driver_gpio esp_http_server
                    INCLUDE_DIRS ".")

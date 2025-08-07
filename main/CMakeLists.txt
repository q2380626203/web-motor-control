idf_component_register(SRCS "motor_status_scheduler.c" "can_monitor.c" "gcode_unified_control.c" "uart_monitor.c" "main.c" "motor_control.c" "wifi_http_server.c" "web_interface.c"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_driver_uart esp_driver_gpio esp_http_server driver esp_timer
                    INCLUDE_DIRS ".")

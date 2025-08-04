idf_component_register(SRCS "motor_control.cpp" "softap_example_main.c"
                    PRIV_REQUIRES esp_wifi nvs_flash esp_driver_uart esp_driver_gpio esp_http_server
                    INCLUDE_DIRS ".")

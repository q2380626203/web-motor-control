#include "wifi_http_server.h"
#include "web_interface.h"
#include <string.h>
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"

// 外部函数声明
extern float angle_to_position(float angle_degrees);
extern float external_velocity_to_internal(float external_velocity);
extern float external_torque_to_internal(float external_torque);

/* WiFi配置 */
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define EXAMPLE_GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define EXAMPLE_GTK_REKEY_INTERVAL 0
#endif

static const char *TAG = "WiFi_HTTP";

// 全局电机控制器指针
static motor_controller_t* g_motor_controller = NULL;

// WiFi事件处理
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "设备 "MACSTR" 连接，AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "设备 "MACSTR" 断开，AID=%d, 原因=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
#ifdef CONFIG_ESP_WIFI_BSS_MAX_IDLE_SUPPORT
            .bss_max_idle_cfg = {
                .period = WIFI_AP_DEFAULT_MAX_IDLE_PERIOD,
                .protected_keep_alive = 1,
            },
#endif
            .gtk_rekey_interval = EXAMPLE_GTK_REKEY_INTERVAL,
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi热点初始化完成。SSID:%s 密码:%s 频道:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

// HTTP处理函数
static esp_err_t web_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, get_web_page_html(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_angle_handler(httpd_req_t *req) {
    char query[200];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char angle_str[32];
        if (httpd_query_key_value(query, "value", angle_str, sizeof(angle_str)) == ESP_OK) {
            float angle = atof(angle_str);
            // 使用角度映射函数进行转换
            float position = angle_to_position(angle);
            
            if (g_motor_controller) {
                motor_control_set_position(g_motor_controller, position);
                char response[100];
                snprintf(response, sizeof(response), "角度: %.1f° -> 位置值: %.3f", angle, position);
                httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
    }
    httpd_resp_send(req, "设置失败", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_position_handler(httpd_req_t *req) {
    char query[200];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char pos_str[32];
        if (httpd_query_key_value(query, "value", pos_str, sizeof(pos_str)) == ESP_OK) {
            float position = atof(pos_str);
            
            if (g_motor_controller) {
                motor_control_set_position(g_motor_controller, position);
                float angle = position;
                char response[100];
                snprintf(response, sizeof(response), "角度: %.3f°", angle);
                httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
    }
    httpd_resp_send(req, "设置失败", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t enable_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        motor_control_enable(g_motor_controller, true);
        httpd_resp_send(req, "成功", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t disable_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        motor_control_enable(g_motor_controller, false);
        httpd_resp_send(req, "成功", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t clear_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        motor_control_clear_errors(g_motor_controller);
        httpd_resp_send(req, "成功", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t set_mode_handler(httpd_req_t *req) {
    char query[200];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char mode_str[32];
        if (httpd_query_key_value(query, "mode", mode_str, sizeof(mode_str)) == ESP_OK) {
            if (g_motor_controller) {
                if (strcmp(mode_str, "velocity") == 0) {
                    motor_control_set_velocity_mode(g_motor_controller);
                    httpd_resp_send(req, "已切换到速度模式", HTTPD_RESP_USE_STRLEN);
                } else if (strcmp(mode_str, "position") == 0) {
                    motor_control_set_position_mode(g_motor_controller);
                    httpd_resp_send(req, "已切换到位置模式", HTTPD_RESP_USE_STRLEN);
                } else if (strcmp(mode_str, "torque") == 0) {
                    motor_control_set_torque_mode(g_motor_controller);
                    httpd_resp_send(req, "已切换到力矩模式", HTTPD_RESP_USE_STRLEN);
                } else {
                    httpd_resp_send(req, "未知模式", HTTPD_RESP_USE_STRLEN);
                }
                return ESP_OK;
            }
        }
    }
    httpd_resp_send(req, "模式切换失败", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_velocity_handler(httpd_req_t *req) {
    char query[200];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char vel_str[32];
        if (httpd_query_key_value(query, "value", vel_str, sizeof(vel_str)) == ESP_OK) {
            float external_velocity = atof(vel_str);
            // 转换为内部电机速度
            float internal_velocity = external_velocity_to_internal(external_velocity);
            
            if (g_motor_controller) {
                motor_control_set_velocity(g_motor_controller, internal_velocity);
                char response[120];
                snprintf(response, sizeof(response), "外部速度: %.2f r/s -> 内部速度: %.2f r/s", 
                        external_velocity, internal_velocity);
                httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
    }
    httpd_resp_send(req, "速度设置失败", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_torque_handler(httpd_req_t *req) {
    char query[200];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char torque_str[32];
        if (httpd_query_key_value(query, "value", torque_str, sizeof(torque_str)) == ESP_OK) {
            float external_torque = atof(torque_str);
            // 转换为内部电机力矩
            float internal_torque = external_torque_to_internal(external_torque);
            
            if (g_motor_controller) {
                motor_control_set_torque(g_motor_controller, internal_torque);
                char response[120];
                snprintf(response, sizeof(response), "外部力矩: %.3f Nm -> 内部力矩: %.3f Nm", 
                        external_torque, internal_torque);
                httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
                return ESP_OK;
            }
        }
    }
    httpd_resp_send(req, "力矩设置失败", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Debug页面处理器
static esp_err_t debug_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, get_debug_page_html(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Debug功能处理器
static esp_err_t debug_restart_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        restart_motor(g_motor_controller->driver_config.uart_port);
        httpd_resp_send(req, "重启电机指令已发送", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机控制器未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t debug_query_torque_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        query_motor_torque(g_motor_controller->driver_config.uart_port);
        httpd_resp_send(req, "查询力矩指令已发送", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机控制器未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t debug_query_power_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        query_motor_power(g_motor_controller->driver_config.uart_port);
        httpd_resp_send(req, "查询功率指令已发送", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机控制器未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t debug_query_encoder_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        query_encoder_count(g_motor_controller->driver_config.uart_port);
        httpd_resp_send(req, "查询编码器指令已发送", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机控制器未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t debug_query_pos_speed_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        query_motor_position_speed(g_motor_controller->driver_config.uart_port);
        httpd_resp_send(req, "查询位置转速指令已发送", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机控制器未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t debug_query_exception_handler(httpd_req_t *req) {
    if (g_motor_controller) {
        char query[200];
        int exception_type = 0; // 默认值
        
        if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
            char type_str[16];
            if (httpd_query_key_value(query, "type", type_str, sizeof(type_str)) == ESP_OK) {
                exception_type = atoi(type_str);
            }
        }
        
        query_motor_exceptions(g_motor_controller->driver_config.uart_port, exception_type);
        char response[100];
        snprintf(response, sizeof(response), "查询异常指令已发送(类型: %d)", exception_type);
        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机控制器未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

httpd_handle_t start_webserver(motor_controller_t* motor_controller) {
    g_motor_controller = motor_controller;  // 保存电机控制器句柄
    
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 20;  // 增加最大URI处理程序数量以支持调试功能
    
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        // 注册处理程序
        httpd_uri_t root = { .uri = "/", .method = HTTP_GET, .handler = web_page_handler };
        httpd_register_uri_handler(server, &root);
        
        httpd_uri_t set_angle = { .uri = "/set_angle", .method = HTTP_GET, .handler = set_angle_handler };
        httpd_register_uri_handler(server, &set_angle);
        
        httpd_uri_t set_position = { .uri = "/set_position", .method = HTTP_GET, .handler = set_position_handler };
        httpd_register_uri_handler(server, &set_position);
        
        httpd_uri_t enable = { .uri = "/enable", .method = HTTP_GET, .handler = enable_handler };
        httpd_register_uri_handler(server, &enable);
        
        httpd_uri_t disable = { .uri = "/disable", .method = HTTP_GET, .handler = disable_handler };
        httpd_register_uri_handler(server, &disable);
        
        httpd_uri_t clear = { .uri = "/clear", .method = HTTP_GET, .handler = clear_handler };
        httpd_register_uri_handler(server, &clear);
        
        httpd_uri_t set_mode = { .uri = "/set_mode", .method = HTTP_GET, .handler = set_mode_handler };
        httpd_register_uri_handler(server, &set_mode);
        
        httpd_uri_t set_velocity = { .uri = "/set_velocity", .method = HTTP_GET, .handler = set_velocity_handler };
        httpd_register_uri_handler(server, &set_velocity);
        
        httpd_uri_t set_torque = { .uri = "/set_torque", .method = HTTP_GET, .handler = set_torque_handler };
        esp_err_t torque_reg_result = httpd_register_uri_handler(server, &set_torque);
        ESP_LOGI(TAG, "注册 /set_torque 处理程序: %s", (torque_reg_result == ESP_OK) ? "成功" : "失败");
        
        // 注册调试页面处理程序
        httpd_uri_t debug_page = { .uri = "/debug", .method = HTTP_GET, .handler = debug_page_handler };
        httpd_register_uri_handler(server, &debug_page);
        
        httpd_uri_t debug_restart = { .uri = "/debug/restart", .method = HTTP_GET, .handler = debug_restart_handler };
        httpd_register_uri_handler(server, &debug_restart);
        
        httpd_uri_t debug_query_torque = { .uri = "/debug/query_torque", .method = HTTP_GET, .handler = debug_query_torque_handler };
        httpd_register_uri_handler(server, &debug_query_torque);
        
        httpd_uri_t debug_query_power = { .uri = "/debug/query_power", .method = HTTP_GET, .handler = debug_query_power_handler };
        httpd_register_uri_handler(server, &debug_query_power);
        
        httpd_uri_t debug_query_encoder = { .uri = "/debug/query_encoder", .method = HTTP_GET, .handler = debug_query_encoder_handler };
        httpd_register_uri_handler(server, &debug_query_encoder);
        
        httpd_uri_t debug_query_pos_speed = { .uri = "/debug/query_pos_speed", .method = HTTP_GET, .handler = debug_query_pos_speed_handler };
        httpd_register_uri_handler(server, &debug_query_pos_speed);
        
        httpd_uri_t debug_query_exception = { .uri = "/debug/query_exception", .method = HTTP_GET, .handler = debug_query_exception_handler };
        httpd_register_uri_handler(server, &debug_query_exception);
        
        ESP_LOGI(TAG, "Web服务器启动成功，端口: %d", config.server_port);
        ESP_LOGI(TAG, "剩余堆内存: %lu bytes", esp_get_free_heap_size());
    }
    return server;
}

void stop_webserver(httpd_handle_t server) {
    if (server) {
        httpd_stop(server);
        ESP_LOGI(TAG, "Web服务器已停止");
    }
}
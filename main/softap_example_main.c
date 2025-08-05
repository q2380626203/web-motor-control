/*  WiFi softAP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "motor_control.h"
#include "esp_http_server.h"

/* The examples use WiFi configuration that you can set via project configuration menu.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define EXAMPLE_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define EXAMPLE_GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define EXAMPLE_GTK_REKEY_INTERVAL 0
#endif

static const char *TAG = "wifi softAP";

// 电机控制器全局变量
static motor_controller_t* motor_controller = NULL;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
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

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}

// Web界面HTML
static const char* web_page = 
"<!DOCTYPE html>"
"<html><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>电机位置控制</title>"
"<style>"
"body{font-family:Arial,sans-serif;max-width:600px;margin:50px auto;padding:20px;background:#f5f5f5}"
".container{background:white;padding:30px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1)}"
"h1{color:#333;text-align:center;margin-bottom:30px}"
".form-group{margin-bottom:20px}"
"label{display:block;margin-bottom:5px;font-weight:bold;color:#555}"
"input[type='number']{width:100%;padding:10px;border:2px solid #ddd;border-radius:5px;font-size:16px}"
"input[type='number']:focus{border-color:#4CAF50;outline:none}"
".btn{background:#4CAF50;color:white;padding:12px 30px;border:none;border-radius:5px;cursor:pointer;font-size:16px;margin:5px}"
".btn:hover{background:#45a049}"
".btn-secondary{background:#2196F3}"
".btn-secondary:hover{background:#1976D2}"
".status{margin-top:20px;padding:15px;border-radius:5px;background:#e8f5e8;border:1px solid #c8e6c8}"
".info{font-size:14px;color:#666;margin-top:10px;padding:10px;background:#f0f8ff;border-radius:5px}"
"</style>"
"</head><body>"
"<div class='container'>"
"<h1>🔧 电机精度测试控制台</h1>"
"<div class='form-group'>"
"<label for='angle'>角度输入 (度):</label>"
"<input type='number' id='angle' step='0.1' min='-180' max='180' placeholder='请输入角度 (例如: 10.5)'>"
"<button class='btn' onclick='setByAngle()'>按角度设置</button>"
"</div>"
"<div class='form-group'>"
"<label for='position'>位置值输入:</label>"
"<input type='number' id='position' step='0.1' min='-100' max='100' placeholder='请输入位置值 (例如: 4.28)'>"
"<button class='btn' onclick='setByPosition()'>按位置值设置</button>"
"</div>"
"<div class='form-group'>"
"<button class='btn btn-secondary' onclick='enableMotor()'>使能电机</button>"
"<button class='btn btn-secondary' onclick='disableMotor()'>失能电机</button>"
"<button class='btn btn-secondary' onclick='clearErrors()'>清除错误</button>"
"</div>"
"<div class='info'>"
"<strong>换算关系:</strong> 8.0位置值 = 18.71度<br>"
"<strong>当前状态:</strong> <span id='status'>待命中...</span>"
"</div>"
"</div>"
"<script>"
"function setByAngle(){"
"let angle=document.getElementById('angle').value;"
"if(angle===''){alert('请输入角度值');return;}"
"fetch('/set_angle?value='+angle).then(r=>r.text()).then(d=>{"
"document.getElementById('status')='角度设置: '+angle+'° | '+d;"
"}).catch(e=>alert('设置失败: '+e));"
"}"
"function setByPosition(){"
"let pos=document.getElementById('position').value;"
"if(pos===''){alert('请输入位置值');return;}"
"fetch('/set_position?value='+pos).then(r=>r.text()).then(d=>{"
"document.getElementById('status')='位置设置: '+pos+' | '+d;"
"}).catch(e=>alert('设置失败: '+e));"
"}"
"function enableMotor(){"
"fetch('/enable').then(r=>r.text()).then(d=>{"
"document.getElementById('status')='电机已使能 | '+d;"
"}).catch(e=>alert('操作失败: '+e));"
"}"
"function disableMotor(){"
"fetch('/disable').then(r=>r.text()).then(d=>{"
"document.getElementById('status')='电机已失能 | '+d;"
"}).catch(e=>alert('操作失败: '+e));"
"}"
"function clearErrors(){"
"fetch('/clear').then(r=>r.text()).then(d=>{"
"document.getElementById('status')='错误已清除 | '+d;"
"}).catch(e=>alert('操作失败: '+e));"
"}"
"</script>"
"</body></html>";

// HTTP处理函数
static esp_err_t web_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, web_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t set_angle_handler(httpd_req_t *req) {
    char query[200];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char angle_str[32];
        if (httpd_query_key_value(query, "value", angle_str, sizeof(angle_str)) == ESP_OK) {
            float angle = atof(angle_str);
            float position = angle;
            
            if (motor_controller) {
                motor_control_set_position(motor_controller, position);
                char response[100];
                snprintf(response, sizeof(response), "位置值: %.3f", position);
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
            
            if (motor_controller) {
                motor_control_set_position(motor_controller, position);
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
    if (motor_controller) {
        motor_control_enable(motor_controller, true);
        httpd_resp_send(req, "成功", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t disable_handler(httpd_req_t *req) {
    if (motor_controller) {
        motor_control_enable(motor_controller, false);
        httpd_resp_send(req, "成功", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t clear_handler(httpd_req_t *req) {
    if (motor_controller) {
        motor_control_clear_errors(motor_controller);
        httpd_resp_send(req, "成功", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, "电机未初始化", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
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
        
        ESP_LOGI(TAG, "Web服务器启动成功，端口: %d", config.server_port);
    }
    return server;
}

void motor_init_task(void *pvParameters) {
    // 电机配置
    motor_driver_config_t motor_config = {
        .uart_port = UART_NUM_1,    // 使用UART1
        .txd_pin = GPIO_NUM_13,     // TXD引脚
        .rxd_pin = GPIO_NUM_12,     // RXD引脚
        .baud_rate = 115200,        // 波特率
        .buf_size = 1024            // 缓冲区大小
    };
    
    // 初始化电机控制器
    motor_controller = motor_control_init(&motor_config, 10.0f); // 速度限制10 r/s
    if (!motor_controller) {
        ESP_LOGE(TAG, "电机控制器初始化失败");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "电机控制器初始化成功");
    
    // 等待2秒让电机稳定
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 设置为位置模式
    motor_control_set_position_mode(motor_controller);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ESP_LOGI(TAG, "电机初始化完成，等待Web界面控制");
    
    // 任务完成，删除自己
    vTaskDelete(NULL);
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
    
    // 启动Web服务器
    start_webserver();
    
    // 创建电机初始化任务
    xTaskCreate(motor_init_task, "motor_init", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "系统启动完成");
    ESP_LOGI(TAG, "请连接WiFi热点，然后访问: http://192.168.4.1");
}

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "motor_control.h"
#include "wifi_http_server.h"

static const char *TAG = "MAIN";

// 全局变量
static motor_controller_t* motor_controller = NULL;
static httpd_handle_t web_server = NULL;

// 电机初始化任务
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
    motor_controller = motor_control_init(&motor_config); 
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
    
    // 启动Web服务器
    web_server = start_webserver(motor_controller);
    if (!web_server) {
        ESP_LOGE(TAG, "Web服务器启动失败");
    }
    
    ESP_LOGI(TAG, "电机初始化完成，Web服务器已启动");
    ESP_LOGI(TAG, "请连接WiFi热点，然后访问: http://192.168.4.1");
    
    // 任务完成，删除自己
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32电机WEB控制系统启动中...");
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化WiFi热点
    ESP_LOGI(TAG, "初始化WiFi热点模式");
    wifi_init_softap();
    
    // 创建电机初始化任务
    xTaskCreate(motor_init_task, "motor_init", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "系统启动完成");
}
#include "uart_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "UART_MONITOR";

// UART监听任务句柄
static TaskHandle_t uart_monitor_task_handle = NULL;


/**
 * @brief UART数据接收和打印任务
 */
static void uart_monitor_task(void *pvParameters) {
    uart_monitor_t* monitor = (uart_monitor_t*)pvParameters;
    uint8_t* data = (uint8_t*)malloc(monitor->config.buf_size);
    
    if (!data) {
        ESP_LOGE(TAG, "内存分配失败");
        monitor->is_running = false;
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(monitor->config.tag, "UART监听任务已启动 - 端口:%d", 
             monitor->config.uart_port);
    
    while (monitor->is_running) {
        // 从UART读取数据
        int length = uart_read_bytes(monitor->config.uart_port, data, 
                                   monitor->config.buf_size - 1, 100 / portTICK_PERIOD_MS);
        
        if (length > 0) {
            // 专门处理电机响应数据
            // 确保字符串以null结尾
            data[length] = '\0';
            
            // 打印接收到的数据
            ESP_LOGI(monitor->config.tag, "接收到电机响应 [长度:%d]: %.*s", length, length, data);
            
            // 如果包含不可打印字符，同时打印十六进制格式
            bool has_non_printable = false;
            for (int i = 0; i < length; i++) {
                if (data[i] < 32 && data[i] != '\r' && data[i] != '\n' && data[i] != '\t') {
                    has_non_printable = true;
                    break;
                }
            }
            
            if (has_non_printable) {
                char hex_str[length * 3 + 1]; // 每字节需要3个字符（XX空格）
                hex_str[0] = '\0';
                for (int i = 0; i < length; i++) {
                    char hex_byte[4];
                    snprintf(hex_byte, sizeof(hex_byte), "%02X ", data[i]);
                    strcat(hex_str, hex_byte);
                }
                ESP_LOGI(monitor->config.tag, "十六进制格式: %s", hex_str);
            }
        }
        
        // 短暂延时避免CPU占用过高
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
    ESP_LOGI(monitor->config.tag, "UART监听任务已停止");
    uart_monitor_task_handle = NULL;
    vTaskDelete(NULL);
}

uart_monitor_t* uart_monitor_init(const uart_monitor_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "配置参数为空");
        return NULL;
    }
    
    uart_monitor_t* monitor = malloc(sizeof(uart_monitor_t));
    if (!monitor) {
        ESP_LOGE(TAG, "内存分配失败");
        return NULL;
    }
    
    // 复制配置
    memcpy(&monitor->config, config, sizeof(uart_monitor_config_t));
    monitor->is_running = false;
    
    if (config->init_uart) {
        // 如果需要初始化UART（独立使用场景）
        ESP_LOGE(TAG, "当前版本不支持独立初始化UART，请设置init_uart为false");
        free(monitor);
        return NULL;
    } else {
        // 复用已有的UART端口（推荐方式）
        ESP_LOGI(TAG, "UART监听器初始化成功 - 复用UART端口:%d", config->uart_port);
    }
    
    return monitor;
}

bool uart_monitor_start(uart_monitor_t* monitor) {
    if (!monitor) {
        ESP_LOGE(TAG, "监听器句柄为空");
        return false;
    }
    
    if (monitor->is_running) {
        ESP_LOGW(TAG, "UART监听器已经在运行");
        return true;
    }
    
    monitor->is_running = true;
    
    // 创建UART监听任务
    BaseType_t ret = xTaskCreate(uart_monitor_task, "uart_monitor", 4096, 
                                monitor, 5, &uart_monitor_task_handle);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建UART监听任务失败");
        monitor->is_running = false;
        return false;
    }
    
    ESP_LOGI(TAG, "UART监听器启动成功");
    return true;
}

void uart_monitor_stop(uart_monitor_t* monitor) {
    if (!monitor || !monitor->is_running) {
        return;
    }
    
    monitor->is_running = false;
    
    // 等待任务结束
    if (uart_monitor_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(200)); // 给任务时间清理资源
    }
    
    ESP_LOGI(TAG, "UART监听器已停止");
}

void uart_monitor_deinit(uart_monitor_t* monitor) {
    if (!monitor) {
        return;
    }
    
    // 停止监听
    uart_monitor_stop(monitor);
    
    // 注意：不删除UART驱动，因为可能还有其他模块在使用
    // 只释放监听器内存
    free(monitor);
    
    ESP_LOGI(TAG, "UART监听器已销毁");
}

bool uart_monitor_is_running(uart_monitor_t* monitor) {
    if (!monitor) {
        return false;
    }
    return monitor->is_running;
}
#include "can_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "CAN_MONITOR";

// CAN监听任务句柄
static TaskHandle_t can_monitor_task_handle = NULL;

/**
 * @brief CAN数据接收和处理任务
 */
static void can_monitor_task(void *pvParameters) {
    can_monitor_t* monitor = (can_monitor_t*)pvParameters;
    
    ESP_LOGI(monitor->config.tag, "CAN监听任务已启动 - TX:%d, RX:%d", 
             monitor->config.tx_gpio, monitor->config.rx_gpio);
    
    uint32_t msg_count = 0;
    
    while (monitor->is_running) {
        twai_message_t rx_msg;
        esp_err_t result = twai_receive(&rx_msg, 100 / portTICK_PERIOD_MS);
        
        if (result == ESP_OK) {
            msg_count++;
            
            // 获取当前时间戳
            uint32_t timestamp = esp_log_timestamp();
            
            ESP_LOGI(monitor->config.tag, "[%lu ms] 消息#%lu: ID=0x%03lX, DLC=%d", 
                     timestamp, msg_count, rx_msg.identifier, rx_msg.data_length_code);
            
            // 显示消息类型和数据
            if (rx_msg.rtr) {
                ESP_LOGI(monitor->config.tag, "类型=远程帧");
            } else {
                // 显示数据帧内容
                char hex_str[64];
                hex_str[0] = '\0';
                for (int i = 0; i < rx_msg.data_length_code; i++) {
                    char hex_byte[4];
                    snprintf(hex_byte, sizeof(hex_byte), "%02X ", rx_msg.data[i]);
                    strcat(hex_str, hex_byte);
                }
                ESP_LOGI(monitor->config.tag, "类型=数据帧, 数据=[%s]", hex_str);
                
                // 如果有G代码控制器，尝试处理G代码
                if (monitor->config.gcode_controller) {
                    // 构建包含CAN ID的数据包，模拟原来UART接收的格式
                    uint8_t can_frame_data[16];
                    size_t frame_length = 0;
                    
                    // 添加CAN ID (模拟原来的格式：00 01 表示G代码帧)
                    if (rx_msg.identifier == 0x001) {
                        can_frame_data[frame_length++] = 0x00;
                        can_frame_data[frame_length++] = 0x01;
                        
                        // 添加数据载荷
                        for (int i = 0; i < rx_msg.data_length_code && frame_length < sizeof(can_frame_data); i++) {
                            can_frame_data[frame_length++] = rx_msg.data[i];
                        }
                        
                        // 处理G代码帧
                        ESP_LOGI(monitor->config.tag, "检测到G代码CAN帧，开始处理");
                        gcode_result_t gcode_result = gcode_process_can_frame(
                            monitor->config.gcode_controller, 
                            can_frame_data, 
                            frame_length
                        );
                        
                        const char* response = gcode_get_response(monitor->config.gcode_controller);
                        ESP_LOGI(monitor->config.tag, "G代码执行结果: %d - %s", gcode_result, response);
                    }
                }
            }
            
            // 显示帧格式
            ESP_LOGI(monitor->config.tag, "格式=%s", rx_msg.extd ? "扩展帧" : "标准帧");
            
        } else if (result != ESP_ERR_TIMEOUT) {
            ESP_LOGW(monitor->config.tag, "接收失败: %s", esp_err_to_name(result));
        }
        
        // 短暂延时避免CPU占用过高
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(monitor->config.tag, "CAN监听任务已停止");
    can_monitor_task_handle = NULL;
    vTaskDelete(NULL);
}

can_monitor_t* can_monitor_init(const can_monitor_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "配置参数为空");
        return NULL;
    }
    
    can_monitor_t* monitor = malloc(sizeof(can_monitor_t));
    if (!monitor) {
        ESP_LOGE(TAG, "内存分配失败");
        return NULL;
    }
    
    // 复制配置
    memcpy(&monitor->config, config, sizeof(can_monitor_config_t));
    monitor->is_running = false;
    
    ESP_LOGI(TAG, "CAN监听器初始化成功 - TX:%d, RX:%d", 
             config->tx_gpio, config->rx_gpio);
    
    return monitor;
}

bool can_monitor_start(can_monitor_t* monitor) {
    if (!monitor) {
        ESP_LOGE(TAG, "监听器句柄为空");
        return false;
    }
    
    if (monitor->is_running) {
        ESP_LOGW(TAG, "CAN监听器已经在运行");
        return true;
    }
    
    // 配置TWAI的通用配置
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(
        monitor->config.tx_gpio, 
        monitor->config.rx_gpio, 
        TWAI_MODE_NORMAL
    );
    
    // 安装TWAI驱动
    esp_err_t result = twai_driver_install(&general_config, 
                                          &monitor->config.timing_config, 
                                          &monitor->config.filter_config);
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "TWAI驱动安装失败: %s", esp_err_to_name(result));
        return false;
    }
    ESP_LOGI(TAG, "TWAI驱动安装成功");
    
    // 启动TWAI驱动
    result = twai_start();
    if (result != ESP_OK) {
        ESP_LOGE(TAG, "TWAI启动失败: %s", esp_err_to_name(result));
        twai_driver_uninstall();
        return false;
    }
    ESP_LOGI(TAG, "TWAI启动成功");
    
    monitor->is_running = true;
    
    // 创建CAN监听任务
    BaseType_t ret = xTaskCreate(can_monitor_task, "can_monitor", 4096, 
                                monitor, 5, &can_monitor_task_handle);
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建CAN监听任务失败");
        monitor->is_running = false;
        twai_stop();
        twai_driver_uninstall();
        return false;
    }
    
    ESP_LOGI(TAG, "CAN监听器启动成功，等待CAN消息...");
    return true;
}

void can_monitor_stop(can_monitor_t* monitor) {
    if (!monitor || !monitor->is_running) {
        return;
    }
    
    monitor->is_running = false;
    
    // 等待任务结束
    if (can_monitor_task_handle) {
        vTaskDelay(pdMS_TO_TICKS(200)); // 给任务时间清理资源
    }
    
    // 停止并卸载TWAI驱动
    twai_stop();
    twai_driver_uninstall();
    
    ESP_LOGI(TAG, "CAN监听器已停止");
}

void can_monitor_deinit(can_monitor_t* monitor) {
    if (!monitor) {
        return;
    }
    
    // 停止监听
    can_monitor_stop(monitor);
    
    // 释放监听器内存
    free(monitor);
    
    ESP_LOGI(TAG, "CAN监听器已销毁");
}

bool can_monitor_is_running(can_monitor_t* monitor) {
    if (!monitor) {
        return false;
    }
    return monitor->is_running;
}
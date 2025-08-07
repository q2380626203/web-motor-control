#include "uart_monitor.h"
#include "motor_control.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "UART_MONITOR";

// UART监听任务句柄
static TaskHandle_t uart_monitor_task_handle = NULL;

// CAN ID定义 (与motor_control.c中保持一致)
#define QUERY_TORQUE_ID     0x003C
#define QUERY_POWER_ID      0x003D  
#define QUERY_ENCODER_ID    0x002A
#define QUERY_EXCEPTION_ID  0x0023
#define QUERY_POS_SPEED_ID  0x0029

// 数据解析辅助函数
static void parse_motor_can_data(const uint8_t *data, int length) {
    motor_status_t *status = get_motor_status();
    if (!status || length < 10) {
        ESP_LOGW(TAG, "数据长度不足，需要至少10字节，当前: %d", length);
        return;
    }
    
    // 提取CAN ID (大端序)
    uint16_t can_id = (data[0] << 8) | data[1];
    
    ESP_LOGI(TAG, "解析电机CAN响应 - ID: 0x%04X, 数据: %02X %02X %02X %02X %02X %02X %02X %02X", 
             can_id, data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9]);
    
    // 根据CAN ID调用对应的解析函数
    switch (can_id) {
        case QUERY_TORQUE_ID:      // 0x003C 力矩查询响应
            parse_torque_data(&data[2], status);  // 跳过CAN ID，从第3字节开始
            ESP_LOGI(TAG, "力矩数据 - 目标: %.3f Nm, 当前: %.3f Nm", 
                     status->target_torque, status->current_torque);
            break;
            
        case QUERY_POWER_ID:       // 0x003D 功率查询响应  
            parse_power_data(&data[2], status);
            ESP_LOGI(TAG, "功率数据 - 电功率: %.3f W, 机械功率: %.3f W", 
                     status->electrical_power, status->mechanical_power);
            break;
            
        case QUERY_ENCODER_ID:     // 0x002A 编码器查询响应
            parse_encoder_data(&data[2], status);
            ESP_LOGI(TAG, "编码器数据 - Shadow: %d, CPR内计数: %d", 
                     status->shadow_count, status->count_in_cpr);
            break;
            
        case QUERY_POS_SPEED_ID:   // 0x0029 位置速度查询响应
            parse_position_speed_data(&data[2], status);
            ESP_LOGI(TAG, "位置速度数据 - 位置: %.3f, 速度: %.3f", 
                     status->position, status->velocity);
            break;
            
        case QUERY_EXCEPTION_ID:   // 0x0023 异常查询响应
            {
                int current_exception_type = get_last_exception_query_type();
                ESP_LOGI(TAG, "收到异常响应 - 当前记录的查询类型: %d", current_exception_type);
                parse_error_data(&data[2], current_exception_type, status);
                ESP_LOGI(TAG, "异常数据 - 查询类型: %d, 电机错误: 0x%08X, 编码器错误: 0x%08X, 控制器错误: 0x%08X, 系统错误: 0x%08X", 
                         current_exception_type, status->motor_error, status->encoder_error, 
                         status->controller_error, status->system_error);
            }
            break;
            
        default:
            ESP_LOGW(TAG, "未知的CAN ID: 0x%04X", can_id);
            break;
    }
}


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
                
                // 处理粘包：按10字节边界分割数据包
                int offset = 0;
                while (offset + 10 <= length) {
                    // 检查是否是有效的CAN响应包（前2字节是ID）
                    uint16_t can_id = (data[offset] << 8) | data[offset + 1];
                    if (can_id >= 0x0023 && can_id <= 0x003D) {
                        parse_motor_can_data(&data[offset], 10);
                        offset += 10;
                    } else {
                        // 如果不是有效包，跳过1字节继续寻找
                        offset++;
                    }
                }
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
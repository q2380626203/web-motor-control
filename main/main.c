#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "motor_control.h"
#include "wifi_http_server.h"
#include "uart_monitor.h"
#include "can_monitor.h"
#include "gcode_unified_control.h"
#include "motor_status_scheduler.h"

// 函数声明
float angle_to_position(float angle_degrees);
float external_velocity_to_internal(float external_velocity);
float external_torque_to_internal(float external_torque);

static const char *TAG = "MAIN";

// 全局变量
static motor_controller_t* motor_controller = NULL;  // 主控制器（用于Web界面）
static motor_controller_t* motor_controller_1 = NULL; // 电机1控制器
static motor_controller_t* motor_controller_4 = NULL; // 电机4控制器
static httpd_handle_t web_server = NULL;
static uart_monitor_t* uart_monitor = NULL;
static can_monitor_t* can_monitor = NULL;
static motor_status_scheduler_t* status_scheduler = NULL;
gcode_controller_t* g_gcode_controller = NULL; // G代码控制器（供CAN监听使用）
static char gcode_response_buffer[512]; // G代码响应缓冲区

// 角度映射参数
#define GEAR_RATIO 19.2158f          // 外部减速比
#define ANGLE_TO_POSITION_SCALE 8.0f // 0-8对应0-360度

/**
 * @brief 角度转换为电机位置值
 * @param angle_degrees 输入角度(度) - 外部输出轴角度
 * @return 电机位置值 - 内部电机需要的位置值
 */
float angle_to_position(float angle_degrees) {
    // 角度归一化到0-360度范围
    while (angle_degrees < 0) angle_degrees += 360.0f;
    while (angle_degrees >= 360.0f) angle_degrees -= 360.0f;
    
    // 外部角度转换为内部电机需要转的圈数
    // 外部转angle_degrees度，内部需要转 angle_degrees * (减速比/360度)
    float internal_rotations = (angle_degrees / 360.0f) * GEAR_RATIO;
    
    // 内部转换为位置值：每转1圈对应位置值8
    float motor_position = internal_rotations * ANGLE_TO_POSITION_SCALE;
    
    return motor_position;
}

/**
 * @brief 外部速度转换为内部电机速度
 * @param external_velocity 外部期望速度 (r/s) - 输出轴转速
 * @return 内部电机需要的速度 (r/s)
 */
float external_velocity_to_internal(float external_velocity) {
    // 外部转1 r/s，内部需要转 减速比 r/s
    return external_velocity * GEAR_RATIO;
}

/**
 * @brief 外部力矩转换为内部电机力矩  
 * @param external_torque 外部期望力矩 (Nm) - 输出轴力矩
 * @return 内部电机需要的力矩 (Nm)
 */
float external_torque_to_internal(float external_torque) {
    // 力矩转换系数：30Nm外部 -> 11Nm内部
    // 转换系数 = 11/30 = 0.3667
    return external_torque * 0.3667f;
}

// 电机初始化任务
void motor_init_task(void *pvParameters) {
    // ========== 初始化电机1控制器 ==========
    motor_driver_config_t motor1_config = {
        .tx_pin = GPIO_NUM_13,   // CAN TX引脚（与CAN监听器使用相同引脚）
        .rx_pin = GPIO_NUM_12,   // CAN RX引脚
        .motor_id = 1            // 电机ID 1
    };

    motor_controller_1 = motor_control_init(&motor1_config);
    if (!motor_controller_1) {
        ESP_LOGE(TAG, "电机1控制器初始化失败");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "电机1控制器初始化成功");

    // ========== 初始化电机4控制器 ==========
    motor_driver_config_t motor4_config = {
        .tx_pin = GPIO_NUM_13,   // CAN TX引脚（共享CAN总线）
        .rx_pin = GPIO_NUM_12,   // CAN RX引脚
        .motor_id = 4            // 电机ID 4
    };

    motor_controller_4 = motor_control_init(&motor4_config);
    if (!motor_controller_4) {
        ESP_LOGE(TAG, "电机4控制器初始化失败");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "电机4控制器初始化成功");

    // 设置主控制器为电机1（用于Web界面）
    motor_controller = motor_controller_1;
    
    // 等待2秒让电机稳定
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 暂时不设置位置模式，后面会设置速度模式
    // motor_control_set_position_mode(motor_controller);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 启动Web服务器
    web_server = start_webserver(motor_controller);
    if (!web_server) {
        ESP_LOGE(TAG, "Web服务器启动失败");
    }
    
    // 初始化状态查询调度器（使用电机1）
    scheduler_config_t scheduler_config = {
        .frequency = 1.0f,               // 默认1Hz查询频率
        // .uart_port = UART_NUM_1,      // 注释掉：不再需要UART，使用TWAI
        .motor_id = motor1_config.motor_id,  // 使用电机1的ID
        .enable_all_queries = false      // 默认不启动自动查询，等待用户手动启动
    };
    
    status_scheduler = motor_status_scheduler_init(&scheduler_config);
    if (status_scheduler) {
        ESP_LOGI(TAG, "状态查询调度器初始化成功");
        // 将调度器传递给Web服务器
        set_status_scheduler(status_scheduler);
    } else {
        ESP_LOGE(TAG, "状态查询调度器初始化失败");
    }
    
    // 初始化G代码控制器
    gcode_controller_config_t gcode_config = {
        .motor_controller = motor_controller,
        .response_buffer = gcode_response_buffer,
        .response_buffer_size = sizeof(gcode_response_buffer)
    };
    
    g_gcode_controller = gcode_controller_init(&gcode_config);
    if (g_gcode_controller) {
        ESP_LOGI(TAG, "G代码控制器初始化成功");
    } else {
        ESP_LOGE(TAG, "G代码控制器初始化失败");
    }

    // 初始化并启动TWAI电机数据监听器（专门监听电机响应数据）
    // 该监听器复用motor_control已初始化的TWAI驱动，不会重复初始化
    uart_monitor_config_t uart_config = {
        .tx_gpio = GPIO_NUM_13,         // CAN TX引脚（与motor_control相同）
        .rx_gpio = GPIO_NUM_12,         // CAN RX引脚
        .tag = "电机响应监听"             // 日志标签
    };

    uart_monitor = uart_monitor_init(&uart_config);
    if (uart_monitor) {
        if (uart_monitor_start(uart_monitor)) {
            ESP_LOGI(TAG, "TWAI电机数据监听器启动成功");
        } else {
            ESP_LOGE(TAG, "TWAI电机数据监听器启动失败");
        }
    } else {
        ESP_LOGE(TAG, "TWAI电机数据监听器初始化失败");
    }

    // 注意：电机响应现在通过CAN总线接收，由uart_monitor（实际使用TWAI）处理

    // /* 注释掉CAN监听器 - TWAI已经被motor_control初始化，避免重复初始化
    // 如果需要监听CAN数据，应该修改can_monitor使其不重新初始化TWAI驱动
    // 初始化并启动CAN监听器（专门监听G代码CAN数据）
    // can_monitor_config_t can_config = {
    //     .tx_gpio = GPIO_NUM_13,               // CAN TX引脚
    //     .rx_gpio = GPIO_NUM_12,               // CAN RX引脚
    //     .timing_config = TWAI_TIMING_CONFIG_500KBITS(), // 500K波特率
    //     .filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(), // 接收所有消息
    //     .tag = "CAN监听",                    // 日志标签
    //     .gcode_controller = g_gcode_controller // G代码控制器
    // };
    //
    // can_monitor = can_monitor_init(&can_config);
    // if (can_monitor) {
    //     if (can_monitor_start(can_monitor)) {
    //         ESP_LOGI(TAG, "CAN数据监听器启动成功");
    //     } else {
    //         ESP_LOGE(TAG, "CAN数据监听器启动失败");
    //     }
    // } else {
    //     ESP_LOGE(TAG, "CAN数据监听器初始化失败");
    // }
    // */

    ESP_LOGI(TAG, "注意：TWAI已由motor_control模块初始化并管理");
    
    ESP_LOGI(TAG, "电机初始化完成，Web服务器已启动，G代码控制器已就绪");
    ESP_LOGI(TAG, "当前控制电机ID: 1 和 4");
    ESP_LOGI(TAG, "电机1 CAN基础ID + 0x00, 电机4 CAN基础ID + 0x60");
    ESP_LOGI(TAG, "请连接WiFi热点，然后访问: http://192.168.4.1");
    ESP_LOGI(TAG, "CAN总线配置: TX=GPIO12, RX=GPIO13, 500kbps标准帧");
    ESP_LOGI(TAG, "电机控制通过TWAI直接发送CAN指令");
    ESP_LOGI(TAG, "Web界面支持: 位置/速度/力矩模式控制");

    // ========== 所有初始化完成后，设置电机1和电机4 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== 开始配置电机运行参数 ==========");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 1. 设置电机1为速度模式
    // ESP_LOGI(TAG, "[电机1] 设置为速度模式...");
    // motor_control_set_velocity_mode(motor_controller_1);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 2. 设置电机4为速度模式
    // ESP_LOGI(TAG, "[电机4] 设置为速度模式...");
    // motor_control_set_velocity_mode(motor_controller_4);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 3. 使能电机1
    // ESP_LOGI(TAG, "[电机1] 使能电机...");
    // motor_control_enable(motor_controller_1, true);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 4. 使能电机4
    // ESP_LOGI(TAG, "[电机4] 使能电机...");
    // motor_control_enable(motor_controller_4, true);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 5. 设置电机1速度为 20 r/s (直接使用外部值，不转换)
    // float motor1_speed = 20.0f;
    // ESP_LOGI(TAG, "[电机1] 设置速度: %.2f r/s", motor1_speed);
    // motor_control_set_velocity(motor_controller_1, motor1_speed);
    // vTaskDelay(pdMS_TO_TICKS(5));  // 指令间隔5ms

    // // 6. 设置电机4速度为 -20 r/s (直接使用外部值，不转换)
    // float motor4_speed = -20.0f;
    // ESP_LOGI(TAG, "[电机4] 设置速度: %.2f r/s", motor4_speed);
    // motor_control_set_velocity(motor_controller_4, motor4_speed);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // ESP_LOGI(TAG, "========== 电机配置完成！电机1和电机4已开始运行 ==========");
    // ESP_LOGI(TAG, "");

    // 任务完成，删除自己 (使用vTaskDelete确保正确清理)
    ESP_LOGI(TAG, "初始化任务完成，释放任务资源");
    vTaskDelay(pdMS_TO_TICKS(100)); // 确保日志输出完成
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
    
    // 创建电机初始化任务 (增加栈大小以避免栈溢出)
    xTaskCreate(motor_init_task, "motor_init", 8192, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "系统启动完成");
}
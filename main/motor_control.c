#include "motor_control.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
// #include "driver/uart.h"  // 注释掉UART，改用TWAI
#include "driver/twai.h"

// TWAI全局状态
static bool g_twai_initialized = false;

// ====================================================================================
// --- 常量定义 ---
// ====================================================================================

// 记录最后查询的异常类型
static int g_last_exception_query_type = -1;

// CAN 基础指令 ID (电机ID 1)
#define BASE_ENABLE_ID           0x0027
#define BASE_VEL_MODE_ID         0x002B      // 设置速度模式的 CAN ID
#define BASE_TARGET_VEL_ID       0x002D      // 发送目标速度的 CAN ID
#define BASE_POS_MODE_ID         0x002B      // 设置位置模式的 CAN ID
#define BASE_TARGET_POS_ID       0x002C      // 发送目标位置的 CAN ID
#define BASE_POS_VEL_LIMIT_ID    0x0031      // 设置位置模式速度限制的 CAN ID
#define BASE_POS_ACC_LIMIT_ID    0x0032      // 设置位置模式加速度/减速度限制的 CAN ID
#define BASE_TORQUE_MODE_ID      0x002B      // 设置力矩模式的 CAN ID
#define BASE_TARGET_TORQUE_ID    0x002E      // 发送目标力矩的 CAN ID
#define BASE_CLEAR_ERROR_ID      0x0038      // 清除错误和异常的 CAN ID
#define BASE_RESTART_MOTOR_ID    0x0036      // 重启电机的 CAN ID
#define BASE_QUERY_TORQUE_ID     0x003C      // 查询目标力矩和当前力矩的 CAN ID
#define BASE_QUERY_POWER_ID      0x003D      // 查询电功率和机械功率的 CAN ID
#define BASE_QUERY_ENCODER_ID    0x002A      // 查询编码器多圈计数和单圈计数的 CAN ID
#define BASE_QUERY_EXCEPTION_ID  0x0023      // 查询电机异常的 CAN ID
#define BASE_QUERY_POS_SPEED_ID  0x0029      // 查询电机转子位置和转速的 CAN ID

// 电机ID偏移量 (每个电机ID之间间隔0x20)
#define MOTOR_ID_OFFSET 0x20

/**
 * @brief 根据电机ID计算实际CAN ID
 * @param base_id 基础CAN ID
 * @param motor_id 电机ID (1-4)
 * @return 实际CAN ID
 */
static inline uint32_t get_can_id(uint32_t base_id, uint8_t motor_id) {
    if (motor_id < 1 || motor_id > 4) {
        printf("[警告] 电机ID超出范围(1-4): %d，默认使用ID 1\n", motor_id);
        motor_id = 1;
    }
    return base_id + (motor_id - 1) * MOTOR_ID_OFFSET;
}

// CAN 指令数据
static const uint8_t ENABLE_DATA[]      = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 致能马达
static const uint8_t DISABLE_DATA[]     = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 失能马达
static const uint8_t VEL_DIRECT_MODE_DATA[] = {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}; // 速度直接模式数据
static const uint8_t POS_DATA[]         = {0x03, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00}; // 进入位置梯形模式
static const uint8_t TORQUE_DIRECT_MODE_DATA[] = {0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}; // 力矩直接模式数据
static const uint8_t CLEAR_ERROR_DATA[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 清除错误和异常数据
static const uint8_t RESTART_MOTOR_DATA[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 重启电机数据
static const uint8_t QUERY_DATA[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 查询指令通用数据

// 内部函数声明
static void send_can_frame(const char* cmd_name, uint32_t id, const uint8_t *data, uint8_t len, uint8_t motor_id);

// TWAI初始化函数（只初始化一次）
static esp_err_t ensure_twai_initialized(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    if (g_twai_initialized) {
        return ESP_OK;  // 已经初始化过
    }

    // 配置TWAI的通用配置
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);

    // 配置500kbps标准帧
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();

    // 接受所有消息
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 安装TWAI驱动
    esp_err_t result = twai_driver_install(&general_config, &timing_config, &filter_config);
    if (result != ESP_OK) {
        printf("[错误] TWAI驱动安装失败: %s\n", esp_err_to_name(result));
        return result;
    }

    // 启动TWAI驱动
    result = twai_start();
    if (result != ESP_OK) {
        printf("[错误] TWAI启动失败: %s\n", esp_err_to_name(result));
        twai_driver_uninstall();
        return result;
    }

    g_twai_initialized = true;
    printf("[信息] TWAI驱动初始化成功 - 500kbps标准帧模式\n");
    return ESP_OK;
}

// ====================================================================================
// --- 电机控制器主要接口实现 ---
// ====================================================================================

motor_controller_t* motor_control_init(const motor_driver_config_t* driver_config) {
    if (!driver_config) {
        printf("[错误] 电机控制器配置参数为空！\n");
        return NULL;
    }

    // 分配控制器内存
    motor_controller_t* controller = (motor_controller_t*)malloc(sizeof(motor_controller_t));
    if (!controller) {
        printf("[错误] 电机控制器内存分配失败！\n");
        return NULL;
    }

    // 复制配置
    memcpy(&controller->driver_config, driver_config, sizeof(motor_driver_config_t));

    // 初始化电机状态
    controller->motor_enabled = false;

    // 初始化TWAI (CAN总线) - 只初始化一次
    esp_err_t result = ensure_twai_initialized(driver_config->tx_pin, driver_config->rx_pin);
    if (result != ESP_OK) {
        printf("[错误] TWAI初始化失败！\n");
        free(controller);
        return NULL;
    }

    // /* 注释掉UART初始化代码
    // uart_config_t uart_config = {
    //     .baud_rate = driver_config->baud_rate,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //     .source_clk = UART_SCLK_DEFAULT
    // };
    //
    // uart_driver_install(driver_config->uart_port, driver_config->buf_size * 2, 0, 0, NULL, 0);
    // uart_param_config(driver_config->uart_port, &uart_config);
    // uart_set_pin(driver_config->uart_port, driver_config->txd_pin, driver_config->rxd_pin,
    //              UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // */

    printf("[信息] 电机控制器初始化完成 - 电机ID:%d, CAN TX:%d, RX:%d\n",
           driver_config->motor_id, driver_config->tx_pin, driver_config->rx_pin);
    return controller;
}

void motor_control_deinit(motor_controller_t* controller) {
    if (!controller) return;

    // 失能电机
    motor_control_enable(controller, false);

    // /* 注释掉UART驱动卸载
    // uart_driver_delete(controller->driver_config.uart_port);
    // */

    // 注意：TWAI驱动可能被多个控制器共享，不在这里卸载
    // 如果需要完全关闭TWAI，需要单独的全局清理函数

    // 释放内存
    free(controller);

    printf("[信息] 电机控制器已销毁\n");
}

void motor_control_enable(motor_controller_t* controller, bool enable) {
    if (!controller) return;

    if (enable) {
        enable_motor(controller->driver_config.motor_id);
        controller->motor_enabled = true;
        printf("[信息] 电机%d已使能\n", controller->driver_config.motor_id);
    } else {
        disable_motor(controller->driver_config.motor_id);
        controller->motor_enabled = false;
        printf("[信息] 电机%d已失能\n", controller->driver_config.motor_id);
    }
}


void motor_control_set_velocity_mode(motor_controller_t* controller) {
    if (!controller) return;

    set_motor_velocity_mode(controller->driver_config.motor_id);
    printf("[信息] 电机%d已设置为速度模式\n", controller->driver_config.motor_id);
}

void motor_control_set_velocity(motor_controller_t* controller, float velocity) {
    if (!controller) return;

    send_target_velocity(controller->driver_config.motor_id, velocity);
    printf("[信息] 电机%d目标速度设置为: %.2f r/s\n", controller->driver_config.motor_id, velocity);
}

void motor_control_set_position_mode(motor_controller_t* controller) {
    if (!controller) return;

    set_motor_position_mode(controller->driver_config.motor_id);
    printf("[信息] 电机%d已设置为位置模式\n", controller->driver_config.motor_id);
}

void motor_control_set_position(motor_controller_t* controller, float position) {
    if (!controller) return;

    send_target_position(controller->driver_config.motor_id, position);
    printf("[信息] 电机%d目标位置设置为: %.2f\n", controller->driver_config.motor_id, position);
}

void motor_control_set_torque_mode(motor_controller_t* controller) {
    if (!controller) return;

    set_motor_torque_mode(controller->driver_config.motor_id);
    printf("[信息] 电机%d已设置为力矩模式\n", controller->driver_config.motor_id);
}

void motor_control_set_torque(motor_controller_t* controller, float torque) {
    if (!controller) return;

    send_target_torque(controller->driver_config.motor_id, torque);
    printf("[信息] 电机%d目标力矩设置为: %.2f Nm\n", controller->driver_config.motor_id, torque);
}


void motor_control_clear_errors(motor_controller_t* controller) {
    if (!controller) return;

    clear_motor_errors(controller->driver_config.motor_id);
    printf("[信息] 电机%d错误和异常已清除\n", controller->driver_config.motor_id);
}

bool motor_control_is_enabled(motor_controller_t* controller) {
    if (!controller) return false;
    return controller->motor_enabled;
}

// ====================================================================================
// --- 低级别电机驱动函数实现 ---
// ====================================================================================

/**
 * @brief 使用TWAI发送CAN帧到电机
 * @param cmd_name 命令名称（用于日志）
 * @param base_id 基础CAN ID
 * @param data CAN数据（8字节）
 * @param len 数据长度
 * @param motor_id 电机ID (1-4)
 */
static void send_can_frame(const char* cmd_name, uint32_t base_id, const uint8_t *data, uint8_t len, uint8_t motor_id) {
    // 根据电机ID计算实际CAN ID
    uint32_t actual_id = get_can_id(base_id, motor_id);

    // 构建TWAI消息
    twai_message_t tx_msg = {
        .identifier = actual_id,      // CAN ID
        .extd = 0,                     // 标准帧（不是扩展帧）
        .rtr = 0,                      // 数据帧（不是远程帧）
        .data_length_code = len,       // 数据长度
    };

    // 复制数据到消息
    memcpy(tx_msg.data, data, len);

    // 发送CAN消息
    esp_err_t result = twai_transmit(&tx_msg, pdMS_TO_TICKS(1000));  // 1秒超时
    if (result == ESP_OK) {
        ESP_LOGD("CAN", "[CAN] 发送成功: %s [电机%d], ID:0x%04lX, 数据长度:%d",
               cmd_name, motor_id, (unsigned long)actual_id, len);
    } else {
        printf("[CAN错误] 发送失败: %s [电机%d], ID:0x%04lX, 错误:%s\n",
               cmd_name, motor_id, (unsigned long)actual_id, esp_err_to_name(result));
    }
}

/* 注释掉原来的UART版本
static void send_serial_can_frame(uart_port_t uart_port, const char* cmd_name,
                                 uint32_t base_id, const uint8_t *data, uint8_t len, uint8_t motor_id) {
    // 根据电机ID计算实际CAN ID
    uint32_t actual_id = get_can_id(base_id, motor_id);

    uint8_t tx_buffer[10];
    tx_buffer[0] = (actual_id >> 8) & 0xFF; // CAN ID high byte
    tx_buffer[1] = actual_id & 0xFF;        // CAN ID low byte
    memcpy(&tx_buffer[2], data, len); // Copy data

    // 发送完整的10字节数据包：2字节ID + 8字节数据
    uart_write_bytes(uart_port, tx_buffer, sizeof(tx_buffer));

    printf("[UART] 发送: %s [电机%d], ID:0x%04lX, 10字节\n", cmd_name, motor_id, (unsigned long)actual_id);
}
*/

void set_motor_velocity_mode(uint8_t motor_id) {
    send_can_frame("设置速度模式", BASE_VEL_MODE_ID, VEL_DIRECT_MODE_DATA, sizeof(VEL_DIRECT_MODE_DATA), motor_id);
}

void set_motor_position_mode(uint8_t motor_id) {
    send_can_frame("设置位置模式", BASE_POS_MODE_ID, POS_DATA, sizeof(POS_DATA), motor_id);
}

void send_target_position(uint8_t motor_id, float position) {
    uint8_t can_data[8] = {0};
    memcpy(can_data, &position, sizeof(position)); // Copy float position to CAN data
    send_can_frame("设置目标位置", BASE_TARGET_POS_ID, can_data, sizeof(can_data), motor_id);
}

void set_position_velocity_limit(uint8_t motor_id, float velocity_limit) {
    uint8_t can_data[8] = {0};
    memcpy(can_data, &velocity_limit, sizeof(velocity_limit)); // 前4字节为速度限制，后4字节为0
    send_can_frame("设置位置模式速度限制", BASE_POS_VEL_LIMIT_ID, can_data, sizeof(can_data), motor_id);
}

void set_position_acceleration_limits(uint8_t motor_id, float acceleration_limit, float deceleration_limit) {
    uint8_t can_data[8] = {0};
    memcpy(can_data, &acceleration_limit, sizeof(acceleration_limit));     // 前4字节为加速度限制
    memcpy(can_data + 4, &deceleration_limit, sizeof(deceleration_limit)); // 后4字节为减速度限制
    send_can_frame("设置位置模式加速度限制", BASE_POS_ACC_LIMIT_ID, can_data, sizeof(can_data), motor_id);
}

void send_target_velocity(uint8_t motor_id, float velocity) {
    uint8_t can_data[8] = {0};
    memcpy(can_data, &velocity, sizeof(velocity)); // Copy float velocity to CAN data
    send_can_frame("设置目标速度", BASE_TARGET_VEL_ID, can_data, sizeof(can_data), motor_id);
}

void set_motor_torque_mode(uint8_t motor_id) {
    send_can_frame("设置力矩模式", BASE_TORQUE_MODE_ID, TORQUE_DIRECT_MODE_DATA, sizeof(TORQUE_DIRECT_MODE_DATA), motor_id);
}

void send_target_torque(uint8_t motor_id, float torque) {
    uint8_t can_data[8] = {0};
    memcpy(can_data, &torque, sizeof(torque)); // Copy float torque to CAN data
    send_can_frame("设置目标力矩", BASE_TARGET_TORQUE_ID, can_data, sizeof(can_data), motor_id);
}

void enable_motor(uint8_t motor_id) {
    send_can_frame("致能马达", BASE_ENABLE_ID, ENABLE_DATA, sizeof(ENABLE_DATA), motor_id);
}

void disable_motor(uint8_t motor_id) {
    send_can_frame("失能马达", BASE_ENABLE_ID, DISABLE_DATA, sizeof(DISABLE_DATA), motor_id);
}


void clear_motor_errors(uint8_t motor_id) {
    send_can_frame("清除错误和异常", BASE_CLEAR_ERROR_ID, CLEAR_ERROR_DATA, sizeof(CLEAR_ERROR_DATA), motor_id);
}

void restart_motor(uint8_t motor_id) {
    send_can_frame("重启电机", BASE_RESTART_MOTOR_ID, RESTART_MOTOR_DATA, sizeof(RESTART_MOTOR_DATA), motor_id);
}

void query_motor_torque(uint8_t motor_id) {
    send_can_frame("查询电机力矩", BASE_QUERY_TORQUE_ID, QUERY_DATA, sizeof(QUERY_DATA), motor_id);
}

void query_motor_power(uint8_t motor_id) {
    send_can_frame("查询电机功率", BASE_QUERY_POWER_ID, QUERY_DATA, sizeof(QUERY_DATA), motor_id);
}

void query_encoder_count(uint8_t motor_id) {
    send_can_frame("查询编码器计数", BASE_QUERY_ENCODER_ID, QUERY_DATA, sizeof(QUERY_DATA), motor_id);
}

void query_motor_exceptions(uint8_t motor_id, int exception_type) {
    uint8_t exception_data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    if (exception_type >= 0 && exception_type <= 4) {
        exception_data[0] = exception_type;
        // 记录查询的异常类型，用于后续解析
        g_last_exception_query_type = exception_type;
        ESP_LOGI("MOTOR_CONTROL", "设置异常查询类型为: %d", exception_type);
    }
    send_can_frame("查询电机异常", BASE_QUERY_EXCEPTION_ID, exception_data, sizeof(exception_data), motor_id);
}

void query_motor_position_speed(uint8_t motor_id) {
    send_can_frame("查询位置和转速", BASE_QUERY_POS_SPEED_ID, QUERY_DATA, sizeof(QUERY_DATA), motor_id);
}

int get_last_exception_query_type(void) {
    return g_last_exception_query_type;
}

// ====================================================================================
// --- 数据解析函数实现 ---
// ====================================================================================

// 全局电机状态变量
static motor_status_t g_motor_status = {0};

// 异常码描述表
typedef struct {
    uint32_t code;
    const char* description;
} error_desc_t;

// 电机异常码表
static const error_desc_t motor_errors[] = {
    {0x00000001, "相间电阻超出正常范围"},
    {0x00000002, "相间电感超出正常范围"}, 
    {0x00000010, "FOC频率太高"},
    {0x00000080, "SVM调制异常"},
    {0x00000400, "相间电流饱和"},
    {0x00001000, "电机电流过大"},
    {0x00020000, "电机温度过高"},
    {0x00040000, "驱动器温度过高"},
    {0x00080000, "FOC处理不及时"},
    {0x00100000, "相间电流采样失效"},
    {0x00200000, "控制器异常"},
    {0x00400000, "母线电压超限"},
    {0x00800000, "刹车电阻驱动异常"},
    {0x01000000, "系统级异常"},
    {0x02000000, "相间电流采样不及时"},
    {0x04000000, "电机位置未知"},
    {0x08000000, "电机速度未知"},
    {0x10000000, "力矩未知"},
    {0x20000000, "力矩控制未知"},
    {0x40000000, "电流采样值未知"},
    {0, NULL}
};

// 编码器异常码表
static const error_desc_t encoder_errors[] = {
    {0x00000001, "编码器带宽过高"},
    {0x00000002, "CPR和极对数不匹配"},
    {0x00000004, "编码器无响应"},
    {0x00000400, "第二编码器通信错误"},
    {0, NULL}
};

// 控制器异常码表  
static const error_desc_t controller_errors[] = {
    {0x00000001, "速度过高"},
    {0x00000002, "控制输入模式不正确"},
    {0x00000004, "锁相环增益不稳"},
    {0x00000020, "位置/速度不稳定"},
    {0x00000080, "机械功率和电气功率不匹配(编码器校准不正确,或磁钢不稳)"},
    {0, NULL}
};

// 系统异常码表
static const error_desc_t system_errors[] = {
    {0x00000002, "电源电压过低"},
    {0x00000004, "电源电压过高"},
    {0x00000008, "电源反向（充电）电流过高"},
    {0x00000010, "电源正向（放电）电流过高"},
    {0, NULL}
};

// 异常类型信息结构
typedef struct {
    const error_desc_t* error_table;
    size_t field_offset;  // 在motor_status_t中的字段偏移量
} error_type_info_t;

// 统一的异常类型信息获取函数
static const error_type_info_t* get_error_type_info(uint8_t error_type) {
    static const error_type_info_t error_types[] = {
        [0] = {motor_errors,      offsetof(motor_status_t, motor_error)},      // 电机异常
        [1] = {encoder_errors,    offsetof(motor_status_t, encoder_error)},    // 编码器异常
        [3] = {controller_errors, offsetof(motor_status_t, controller_error)}, // 控制器异常
        [4] = {system_errors,     offsetof(motor_status_t, system_error)},     // 系统异常
    };
    
    if (error_type < sizeof(error_types)/sizeof(error_types[0]) && 
        error_types[error_type].error_table != NULL) {
        return &error_types[error_type];
    }
    return NULL;
}

float ieee754_bytes_to_float(const uint8_t *bytes) {
    union {
        float f;
        uint32_t u;
    } converter;
    
    // 小端序转换
    converter.u = (uint32_t)bytes[0] | 
                  ((uint32_t)bytes[1] << 8) | 
                  ((uint32_t)bytes[2] << 16) | 
                  ((uint32_t)bytes[3] << 24);
    
    return converter.f;
}

int32_t bytes_to_int32(const uint8_t *bytes) {
    return (int32_t)bytes[0] | 
           ((int32_t)bytes[1] << 8) | 
           ((int32_t)bytes[2] << 16) | 
           ((int32_t)bytes[3] << 24);
}

void parse_torque_data(const uint8_t *data, motor_status_t *status) {
    if (!data || !status) return;
    
    status->target_torque = ieee754_bytes_to_float(&data[0]);
    status->current_torque = ieee754_bytes_to_float(&data[4]);
    status->data_valid = true;
    status->last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void parse_power_data(const uint8_t *data, motor_status_t *status) {
    if (!data || !status) return;
    
    status->electrical_power = ieee754_bytes_to_float(&data[0]);
    status->mechanical_power = ieee754_bytes_to_float(&data[4]);
    status->data_valid = true;
    status->last_update_time = (uint32_t)(esp_timer_get_time() / 1000); // 毫秒时间戳
}

void parse_encoder_data(const uint8_t *data, motor_status_t *status) {
    if (!data || !status) return;
    
    status->shadow_count = bytes_to_int32(&data[0]);
    status->count_in_cpr = bytes_to_int32(&data[4]);
    status->data_valid = true;
    status->last_update_time = (uint32_t)(esp_timer_get_time() / 1000); // 毫秒时间戳
}

void parse_position_speed_data(const uint8_t *data, motor_status_t *status) {
    if (!data || !status) return;
    
    status->position = ieee754_bytes_to_float(&data[0]);
    status->velocity = ieee754_bytes_to_float(&data[4]);
    status->data_valid = true;
    status->last_update_time = (uint32_t)(esp_timer_get_time() / 1000); // 毫秒时间戳
    
}

void parse_error_data(const uint8_t *data, uint8_t error_type, motor_status_t *status) {
    if (!data || !status) return;
    
    // 取前4字节作为异常码（小端序）
    uint32_t error_code = (uint32_t)data[0] | 
                          ((uint32_t)data[1] << 8) | 
                          ((uint32_t)data[2] << 16) | 
                          ((uint32_t)data[3] << 24);
    
    // 使用统一接口获取异常类型信息
    const error_type_info_t *type_info = get_error_type_info(error_type);
    if (type_info) {
        // 通过偏移量计算字段地址并存储异常码（包括0x00000000的正常状态）
        uint32_t *error_field = (uint32_t*)((char*)status + type_info->field_offset);
        *error_field = error_code;  // 无论是异常还是正常都要更新字段
    }
    
    status->data_valid = true;
    status->last_update_time = (uint32_t)(esp_timer_get_time() / 1000); // 毫秒时间戳
}

const char* get_error_description(uint32_t error_code, uint8_t error_type) {
    if (error_code == 0) {
        return "正常";
    }
    
    const error_type_info_t *type_info = get_error_type_info(error_type);
    if (!type_info) {
        return "未知异常类型";
    }
    
    for (int i = 0; type_info->error_table[i].description != NULL; i++) {
        if (type_info->error_table[i].code == error_code) {
            return type_info->error_table[i].description;
        }
    }
    
    return "未知异常码";
}

motor_status_t* get_motor_status(void) {
    return &g_motor_status;
}
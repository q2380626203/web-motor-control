#include "motor_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ====================================================================================
// --- 常量定义 ---
// ====================================================================================

// 减速机换算关系: 8.0 位置值 = 18.711019 度
#define POSITION_TO_DEGREE_RATIO    (18.711019f / 8.0f)  // 2.3388774度/位置值
#define DEGREE_TO_POSITION_RATIO    (8.0f / 18.711019f)  // 0.4276058位置值/度

// CAN 指令 ID
#define ENABLE_ID           0x0027
#define VEL_MODE_ID         0x002B      // 设置速度模式的 CAN ID
#define TARGET_VEL_ID       0x002D      // 发送目标速度的 CAN ID
#define POS_MODE_ID         0x002B      // 设置位置模式的 CAN ID
#define TARGET_POS_ID       0x002C      // 发送目标位置的 CAN ID
#define CLEAR_ERROR_ID      0x0038      // 清除错误和异常的 CAN ID

// CAN 指令数据
static const uint8_t ENABLE_DATA[]      = {0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 致能马达
static const uint8_t DISABLE_DATA[]     = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 失能马达
static const uint8_t VEL_DIRECT_MODE_DATA[] = {0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00}; // 速度直接模式数据
static const uint8_t POS_DATA[]         = {0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00}; // 进入位置斜坡模式
static const uint8_t CLEAR_ERROR_DATA[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 清除错误和异常数据

// 内部函数声明
static void send_serial_can_frame(uart_port_t uart_port, const char* cmd_name, 
                                 uint32_t id, const uint8_t *data, uint8_t len);

// ====================================================================================
// --- 电机控制器主要接口实现 ---
// ====================================================================================

motor_controller_t* motor_control_init(const motor_driver_config_t* driver_config, 
                                      float velocity_limit) {
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
    controller->velocity_limit = velocity_limit;

    // 初始化电机状态
    controller->motor_enabled = false;

    // 初始化UART
    uart_config_t uart_config = {
        .baud_rate = driver_config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    
    uart_driver_install(driver_config->uart_port, driver_config->buf_size * 2, 0, 0, NULL, 0);
    uart_param_config(driver_config->uart_port, &uart_config);
    uart_set_pin(driver_config->uart_port, driver_config->txd_pin, driver_config->rxd_pin, 
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // 初始化电机（不设置模式，等待后续配置）
    printf("[信息] 电机UART已配置，等待模式设置\n");

    printf("[信息] 电机控制器在 UART%d 上初始化完成，速度限制: %.2f r/s\n", 
           driver_config->uart_port, velocity_limit);
    return controller;
}

void motor_control_deinit(motor_controller_t* controller) {
    if (!controller) return;

    // 失能电机
    motor_control_enable(controller, false);

    // 删除UART驱动
    uart_driver_delete(controller->driver_config.uart_port);

    // 释放内存
    free(controller);
    
    printf("[信息] 电机控制器已销毁\n");
}

void motor_control_enable(motor_controller_t* controller, bool enable) {
    if (!controller) return;

    if (enable && !controller->motor_enabled) {
        enable_motor(controller->driver_config.uart_port);
        controller->motor_enabled = true;
        printf("[信息] 电机已使能\n");
    } else if (!enable && controller->motor_enabled) {
        disable_motor(controller->driver_config.uart_port);
        controller->motor_enabled = false;
        printf("[信息] 电机已失能\n");
    }
}

void motor_control_set_velocity(motor_controller_t* controller, float velocity) {
    if (!controller) return;

    move_motor_to_velocity(controller->driver_config.uart_port, 
                          velocity, controller->velocity_limit);
}

void motor_control_set_position_mode(motor_controller_t* controller) {
    if (!controller) return;

    set_motor_position_mode(controller->driver_config.uart_port);
    printf("[信息] 电机已设置为位置模式\n");
}

void motor_control_set_position(motor_controller_t* controller, float position) {
    if (!controller) return;

    send_target_position(controller->driver_config.uart_port, position);
    float angle = motor_position_to_angle(position);
    printf("[信息] 电机目标位置设置为: %.2f (%.2f度)\n", position, angle);
}

float motor_angle_to_position(float angle_degrees) {
    return angle_degrees * DEGREE_TO_POSITION_RATIO;
}

float motor_position_to_angle(float position) {
    return position * POSITION_TO_DEGREE_RATIO;
}

void motor_control_clear_errors(motor_controller_t* controller) {
    if (!controller) return;

    clear_motor_errors(controller->driver_config.uart_port);
    printf("[信息] 电机错误和异常已清除\n");
}

bool motor_control_is_enabled(motor_controller_t* controller) {
    if (!controller) return false;
    return controller->motor_enabled;
}

// ====================================================================================
// --- 低级别电机驱动函数实现 ---
// ====================================================================================

static void send_serial_can_frame(uart_port_t uart_port, const char* cmd_name, 
                                 uint32_t id, const uint8_t *data, uint8_t len) {
    uint8_t tx_buffer[10];
    tx_buffer[0] = (id >> 8) & 0xFF; // CAN ID high byte
    tx_buffer[1] = id & 0xFF;        // CAN ID low byte
    memcpy(&tx_buffer[2], data, len); // Copy data
    
    // 发送完整的10字节数据包：2字节ID + 8字节数据
    uart_write_bytes(uart_port, tx_buffer, sizeof(tx_buffer));
    
    printf("[UART] 发送: %s, ID:0x%04lX, 10字节\n", cmd_name, (unsigned long)id);
}

void set_motor_velocity_mode(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "设置速度模式", VEL_MODE_ID, VEL_DIRECT_MODE_DATA, sizeof(VEL_DIRECT_MODE_DATA));
}

void set_motor_position_mode(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "设置位置模式", POS_MODE_ID, POS_DATA, sizeof(POS_DATA));
}

void send_target_position(uart_port_t uart_port, float position) {
    uint8_t can_data[8] = {0};
    memcpy(can_data, &position, sizeof(position)); // Copy float position to CAN data
    send_serial_can_frame(uart_port, "设置目标位置", TARGET_POS_ID, can_data, sizeof(can_data));
}

void enable_motor(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "致能马达", ENABLE_ID, ENABLE_DATA, sizeof(ENABLE_DATA));
}

void disable_motor(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "失能马达", ENABLE_ID, DISABLE_DATA, sizeof(DISABLE_DATA));
}

void move_motor_to_velocity(uart_port_t uart_port, float velocity, float velocity_limit) {
    // 限制电机速度在设定范围内
    if (velocity > velocity_limit) velocity = velocity_limit;
    if (velocity < -velocity_limit) velocity = -velocity_limit;

    uint8_t can_data[8] = {0};
    memcpy(can_data, &velocity, sizeof(velocity)); // Copy float velocity to CAN data
    send_serial_can_frame(uart_port, "MoveToVelocity", TARGET_VEL_ID, can_data, sizeof(can_data));
}

void clear_motor_errors(uart_port_t uart_port) {
    send_serial_can_frame(uart_port, "清除错误和异常", CLEAR_ERROR_ID, CLEAR_ERROR_DATA, sizeof(CLEAR_ERROR_DATA));
}
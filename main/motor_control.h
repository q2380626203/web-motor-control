#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ====================================================================================
// --- 电机控制模块数据结构定义 ---
// ====================================================================================

// 电机驱动配置结构
typedef struct {
    uart_port_t uart_port;          // UART端口号
    gpio_num_t txd_pin;             // TXD引脚
    gpio_num_t rxd_pin;             // RXD引脚
    int baud_rate;                  // 波特率
    int buf_size;                   // 缓冲区大小
} motor_driver_config_t;

// 电机控制器主结构
typedef struct {
    motor_driver_config_t driver_config;   // 驱动配置
    bool motor_enabled;                    // 电机使能状态
} motor_controller_t;

// ====================================================================================
// --- 电机控制模块接口函数 ---
// ====================================================================================

/**
 * @brief 初始化电机控制器
 * @param driver_config 驱动配置
 * @return 电机控制器句柄，失败返回NULL
 */
motor_controller_t* motor_control_init(const motor_driver_config_t* driver_config);

/**
 * @brief 销毁电机控制器
 * @param controller 电机控制器句柄
 */
void motor_control_deinit(motor_controller_t* controller);

/**
 * @brief 使能/失能电机
 * @param controller 电机控制器句柄
 * @param enable true为使能，false为失能
 */
void motor_control_enable(motor_controller_t* controller, bool enable);


/**
 * @brief 设置电机速度模式
 * @param controller 电机控制器句柄
 */
void motor_control_set_velocity_mode(motor_controller_t* controller);

/**
 * @brief 设置电机目标速度
 * @param controller 电机控制器句柄
 * @param velocity 目标速度 (r/s)
 */
void motor_control_set_velocity(motor_controller_t* controller, float velocity);

/**
 * @brief 设置电机位置模式
 * @param controller 电机控制器句柄
 */
void motor_control_set_position_mode(motor_controller_t* controller);

/**
 * @brief 设置电机目标位置
 * @param controller 电机控制器句柄
 * @param position 目标位置 
 */
void motor_control_set_position(motor_controller_t* controller, float position);

/**
 * @brief 设置电机力矩模式
 * @param controller 电机控制器句柄
 */
void motor_control_set_torque_mode(motor_controller_t* controller);

/**
 * @brief 设置电机目标力矩
 * @param controller 电机控制器句柄
 * @param torque 目标力矩 (Nm)
 */
void motor_control_set_torque(motor_controller_t* controller, float torque);


/**
 * @brief 清除电机错误和异常
 * @param controller 电机控制器句柄
 */
void motor_control_clear_errors(motor_controller_t* controller);

/**
 * @brief 获取电机使能状态
 * @param controller 电机控制器句柄
 * @return 使能状态
 */
bool motor_control_is_enabled(motor_controller_t* controller);

// ====================================================================================
// --- 低级别电机驱动函数 ---
// ====================================================================================

/**
 * @brief 设置电机为速度直接模式
 * @param uart_port UART端口
 */
void set_motor_velocity_mode(uart_port_t uart_port);

/**
 * @brief 设置电机为位置模式
 * @param uart_port UART端口
 */
void set_motor_position_mode(uart_port_t uart_port);

/**
 * @brief 发送目标位置
 * @param uart_port UART端口
 * @param position 目标位置 
 */
void send_target_position(uart_port_t uart_port, float position);

/**
 * @brief 发送目标速度
 * @param uart_port UART端口
 * @param velocity 目标速度 (r/s)
 */
void send_target_velocity(uart_port_t uart_port, float velocity);

/**
 * @brief 设置电机为力矩模式
 * @param uart_port UART端口
 */
void set_motor_torque_mode(uart_port_t uart_port);

/**
 * @brief 发送目标力矩
 * @param uart_port UART端口
 * @param torque 目标力矩 (Nm)
 */
void send_target_torque(uart_port_t uart_port, float torque);

/**
 * @brief 使能电机
 * @param uart_port UART端口
 */
void enable_motor(uart_port_t uart_port);

/**
 * @brief 失能电机
 * @param uart_port UART端口
 */
void disable_motor(uart_port_t uart_port);


/**
 * @brief 清除电机错误和异常
 * @param uart_port UART端口
 */
void clear_motor_errors(uart_port_t uart_port);

/**
 * @brief 重启电机
 * @param uart_port UART端口
 */
void restart_motor(uart_port_t uart_port);

/**
 * @brief 查询电机目标力矩和当前力矩
 * @param uart_port UART端口
 */
void query_motor_torque(uart_port_t uart_port);

/**
 * @brief 查询电机电功率和机械功率
 * @param uart_port UART端口
 */
void query_motor_power(uart_port_t uart_port);

/**
 * @brief 查询编码器多圈计数和单圈计数
 * @param uart_port UART端口
 */
void query_encoder_count(uart_port_t uart_port);

/**
 * @brief 查询电机异常信息
 * @param uart_port UART端口
 * @param exception_type 异常类型 (0-4: 电机异常/编码器异常/控制异常/系统异常)
 */
void query_motor_exceptions(uart_port_t uart_port, int exception_type);

/**
 * @brief 查询电机转子位置和转速
 * @param uart_port UART端口
 */
void query_motor_position_speed(uart_port_t uart_port);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
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
    float velocity_limit;                  // 电机最大速度限制
} motor_controller_t;

// ====================================================================================
// --- 电机控制模块接口函数 ---
// ====================================================================================

/**
 * @brief 初始化电机控制器
 * @param driver_config 驱动配置
 * @param velocity_limit 速度限制
 * @return 电机控制器句柄，失败返回NULL
 */
motor_controller_t* motor_control_init(const motor_driver_config_t* driver_config, 
                                      float velocity_limit);

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
 * @brief 直接设置电机速度
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
 * @param position 目标位置 (8.0代表减速器输出轴转动18.711019度)
 */
void motor_control_set_position(motor_controller_t* controller, float position);

/**
 * @brief 角度转换为位置值
 * @param angle_degrees 角度 (度)
 * @return 位置值 (8.0 = 18.711019度)
 */
float motor_angle_to_position(float angle_degrees);

/**
 * @brief 位置值转换为角度
 * @param position 位置值
 * @return 角度 (度)
 */
float motor_position_to_angle(float position);

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
 * @param position 目标位置 (8.0代表减速器输出轴转动18.711019度)
 */
void send_target_position(uart_port_t uart_port, float position);

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
 * @brief 移动电机到指定速度
 * @param uart_port UART端口
 * @param velocity 目标速度 (r/s)
 * @param velocity_limit 速度限制
 */
void move_motor_to_velocity(uart_port_t uart_port, float velocity, float velocity_limit);

/**
 * @brief 清除电机错误和异常
 * @param uart_port UART端口
 */
void clear_motor_errors(uart_port_t uart_port);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H
#ifndef UART_MONITOR_H
#define UART_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
// #include "driver/uart.h"  // 注释掉UART驱动，改用TWAI
#include "driver/twai.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// TWAI电机数据监听器配置结构
// 注意：该模块专门用于监听电机响应数据，不管理TWAI驱动生命周期
// TWAI驱动应由motor_control模块初始化，此监听器仅复用已初始化的TWAI驱动
typedef struct {
    // uart_port_t uart_port;       // [已废弃] UART端口号
    gpio_num_t tx_gpio;             // CAN TX引脚（仅用于日志，不实际配置）
    gpio_num_t rx_gpio;             // CAN RX引脚（仅用于日志，不实际配置）
    // int buf_size;                // [已废弃] 缓冲区大小（TWAI有自己的队列）
    char* tag;                      // 日志标签
    // bool init_uart;              // [已废弃] 是否初始化（现在总是复用已有TWAI驱动）
} uart_monitor_config_t;

// TWAI电机数据监听器句柄
typedef struct {
    uart_monitor_config_t config;   // 配置信息
    bool is_running;                // 运行状态
} uart_monitor_t;

/**
 * @brief 初始化TWAI电机数据监听器
 * @param config 配置参数
 * @return 监听器句柄，失败返回NULL
 * @note 不会初始化TWAI驱动，仅创建监听器实例，假设TWAI已由motor_control初始化
 */
uart_monitor_t* uart_monitor_init(const uart_monitor_config_t* config);

/**
 * @brief 启动TWAI电机数据监听任务
 * @param monitor 监听器句柄
 * @return 是否启动成功
 * @note 监听任务会通过twai_receive接收电机响应帧并解析
 */
bool uart_monitor_start(uart_monitor_t* monitor);

/**
 * @brief 停止TWAI电机数据监听任务
 * @param monitor 监听器句柄
 * @note 仅停止监听任务，不会卸载TWAI驱动
 */
void uart_monitor_stop(uart_monitor_t* monitor);

/**
 * @brief 销毁TWAI电机数据监听器
 * @param monitor 监听器句柄
 * @note 仅释放监听器内存，不会卸载TWAI驱动
 */
void uart_monitor_deinit(uart_monitor_t* monitor);

/**
 * @brief 获取监听器运行状态
 * @param monitor 监听器句柄
 * @return 运行状态
 */
bool uart_monitor_is_running(uart_monitor_t* monitor);

#ifdef __cplusplus
}
#endif

#endif // UART_MONITOR_H
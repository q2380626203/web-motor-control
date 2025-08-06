#ifndef UART_MONITOR_H
#define UART_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// UART监听器配置结构
typedef struct {
    uart_port_t uart_port;          // UART端口号
    int buf_size;                   // 缓冲区大小
    char* tag;                      // 日志标签
    bool init_uart;                 // 是否需要初始化UART（false表示复用已有的）
} uart_monitor_config_t;

// UART监听器句柄
typedef struct {
    uart_monitor_config_t config;   // 配置信息
    bool is_running;                // 运行状态
} uart_monitor_t;

/**
 * @brief 初始化UART监听器
 * @param config 配置参数
 * @return UART监听器句柄，失败返回NULL
 */
uart_monitor_t* uart_monitor_init(const uart_monitor_config_t* config);

/**
 * @brief 启动UART数据监听任务
 * @param monitor UART监听器句柄
 * @return 是否启动成功
 */
bool uart_monitor_start(uart_monitor_t* monitor);

/**
 * @brief 停止UART数据监听任务
 * @param monitor UART监听器句柄
 */
void uart_monitor_stop(uart_monitor_t* monitor);

/**
 * @brief 销毁UART监听器
 * @param monitor UART监听器句柄
 */
void uart_monitor_deinit(uart_monitor_t* monitor);

/**
 * @brief 获取UART监听器运行状态
 * @param monitor UART监听器句柄
 * @return 运行状态
 */
bool uart_monitor_is_running(uart_monitor_t* monitor);

#ifdef __cplusplus
}
#endif

#endif // UART_MONITOR_H
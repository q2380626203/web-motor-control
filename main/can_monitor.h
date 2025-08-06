#ifndef CAN_MONITOR_H
#define CAN_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/twai.h"
#include "gcode_unified_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// CAN监听器配置结构
typedef struct {
    int tx_gpio;                        // CAN TX引脚
    int rx_gpio;                        // CAN RX引脚
    twai_timing_config_t timing_config; // CAN时序配置
    twai_filter_config_t filter_config; // CAN滤波配置
    char* tag;                          // 日志标签
    gcode_controller_t* gcode_controller; // G代码控制器
} can_monitor_config_t;

// CAN监听器句柄
typedef struct {
    can_monitor_config_t config;        // 配置信息
    bool is_running;                    // 运行状态
} can_monitor_t;

/**
 * @brief 初始化CAN监听器
 * @param config 配置参数
 * @return CAN监听器句柄，失败返回NULL
 */
can_monitor_t* can_monitor_init(const can_monitor_config_t* config);

/**
 * @brief 启动CAN数据监听任务
 * @param monitor CAN监听器句柄
 * @return 是否启动成功
 */
bool can_monitor_start(can_monitor_t* monitor);

/**
 * @brief 停止CAN数据监听任务
 * @param monitor CAN监听器句柄
 */
void can_monitor_stop(can_monitor_t* monitor);

/**
 * @brief 销毁CAN监听器
 * @param monitor CAN监听器句柄
 */
void can_monitor_deinit(can_monitor_t* monitor);

/**
 * @brief 获取CAN监听器运行状态
 * @param monitor CAN监听器句柄
 * @return 运行状态
 */
bool can_monitor_is_running(can_monitor_t* monitor);

#ifdef __cplusplus
}
#endif

#endif // CAN_MONITOR_H
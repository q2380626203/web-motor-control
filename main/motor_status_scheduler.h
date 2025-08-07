#ifndef MOTOR_STATUS_SCHEDULER_H
#define MOTOR_STATUS_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/uart.h"

// 查询事件类型
typedef enum {
    QUERY_EVENT_TORQUE = 0,
    QUERY_EVENT_POWER,
    QUERY_EVENT_ENCODER,  
    QUERY_EVENT_POSITION_SPEED,
    QUERY_EVENT_EXCEPTIONS,
    QUERY_EVENT_MAX
} query_event_type_t;

// 查询事件结构
typedef struct {
    query_event_type_t type;
    uart_port_t uart_port;
    int exception_type;  // 异常查询类型(0-4)，其他查询忽略
} query_event_t;

typedef struct {
    float query_frequency;          // 查询频率 (Hz)
    bool auto_query_enabled;        // 是否启用自动查询
    uart_port_t uart_port;          // UART端口
    TimerHandle_t query_timer;      // FreeRTOS定时器句柄
    uint8_t current_query_index;    // 当前查询索引(轮询不同状态)
    uint8_t current_exception_type; // 当前异常查询类型(0-4循环)
    bool is_running;                // 调度器运行状态
    
    // 新增：事件队列和任务句柄
    QueueHandle_t query_queue;      // 查询事件队列
    TaskHandle_t query_task_handle; // 查询任务句柄
} motor_status_scheduler_t;

typedef struct {
    float frequency;                // 查询频率 (Hz, 范围: 1.0 - 100.0)
    uart_port_t uart_port;         // UART端口
    bool enable_all_queries;       // 是否启用全部查询类型
} scheduler_config_t;

motor_status_scheduler_t* motor_status_scheduler_init(const scheduler_config_t* config);

bool motor_status_scheduler_start(motor_status_scheduler_t* scheduler);

void motor_status_scheduler_stop(motor_status_scheduler_t* scheduler);

void motor_status_scheduler_deinit(motor_status_scheduler_t* scheduler);

bool motor_status_scheduler_set_frequency(motor_status_scheduler_t* scheduler, float frequency);

float motor_status_scheduler_get_frequency(motor_status_scheduler_t* scheduler);

bool motor_status_scheduler_is_running(motor_status_scheduler_t* scheduler);

#endif // MOTOR_STATUS_SCHEDULER_H
#include "motor_status_scheduler.h"
#include "motor_control.h"
#include "esp_log.h"
#include <stdlib.h>
#include <math.h>

static const char *TAG = "MOTOR_SCHEDULER";

#define MIN_FREQUENCY 0.5f
#define MAX_FREQUENCY 5.0f
#define DEFAULT_FREQUENCY 2.0f
#define QUERY_TYPES_COUNT 5
#define QUERY_QUEUE_SIZE 10

static void query_timer_callback(TimerHandle_t xTimer);
static void query_task(void *pvParameters);

// UART查询任务 - 处理实际的UART操作
static void query_task(void *pvParameters) {
    motor_status_scheduler_t* scheduler = (motor_status_scheduler_t*)pvParameters;
    query_event_t event;
    
    ESP_LOGI(TAG, "查询任务启动成功");
    
    while (true) {
        // 等待定时器发送的查询事件
        if (xQueueReceive(scheduler->query_queue, &event, portMAX_DELAY) == pdTRUE) {
            if (!scheduler->auto_query_enabled) {
                continue; // 如果自动查询被禁用，跳过处理
            }
            
            // 根据事件类型执行相应的查询操作
            switch (event.type) {
                case QUERY_EVENT_TORQUE:
                    query_motor_torque(event.uart_port);
                    ESP_LOGI(TAG, "自动查询力矩");
                    break;
                case QUERY_EVENT_POWER:
                    query_motor_power(event.uart_port);
                    ESP_LOGI(TAG, "自动查询功率");
                    break;
                case QUERY_EVENT_ENCODER:
                    query_encoder_count(event.uart_port);
                    ESP_LOGI(TAG, "自动查询编码器");
                    break;
                case QUERY_EVENT_POSITION_SPEED:
                    query_motor_position_speed(event.uart_port);
                    ESP_LOGI(TAG, "自动查询位置速度");
                    break;
                case QUERY_EVENT_EXCEPTIONS:
                    query_motor_exceptions(event.uart_port, event.exception_type);
                    ESP_LOGI(TAG, "自动查询异常状态(类型:%d)", event.exception_type);
                    break;
                default:
                    ESP_LOGW(TAG, "未知查询事件类型: %d", event.type);
                    break;
            }
        }
    }
}

motor_status_scheduler_t* motor_status_scheduler_init(const scheduler_config_t* config) {
    if (!config) {
        ESP_LOGE(TAG, "配置参数为空");
        return NULL;
    }
    
    if (config->frequency < MIN_FREQUENCY || config->frequency > MAX_FREQUENCY) {
        ESP_LOGE(TAG, "查询频率超出范围 [%.1f, %.1f]", MIN_FREQUENCY, MAX_FREQUENCY);
        return NULL;
    }
    
    motor_status_scheduler_t* scheduler = malloc(sizeof(motor_status_scheduler_t));
    if (!scheduler) {
        ESP_LOGE(TAG, "内存分配失败");
        return NULL;
    }
    
    scheduler->query_frequency = config->frequency;
    scheduler->auto_query_enabled = config->enable_all_queries;
    scheduler->uart_port = config->uart_port;
    scheduler->current_query_index = 0;
    scheduler->current_exception_type = 0;
    scheduler->is_running = false;
    scheduler->query_timer = NULL;
    scheduler->query_queue = NULL;
    scheduler->query_task_handle = NULL;
    
    // 创建查询事件队列
    scheduler->query_queue = xQueueCreate(QUERY_QUEUE_SIZE, sizeof(query_event_t));
    if (!scheduler->query_queue) {
        ESP_LOGE(TAG, "创建查询队列失败");
        free(scheduler);
        return NULL;
    }
    
    // 创建查询任务
    BaseType_t ret = xTaskCreate(query_task, "motor_query", 4096, 
                                scheduler, 5, &scheduler->query_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建查询任务失败");
        vQueueDelete(scheduler->query_queue);
        free(scheduler);
        return NULL;
    }
    
    TickType_t timer_period = pdMS_TO_TICKS((uint32_t)(1000.0f / config->frequency));
    
    scheduler->query_timer = xTimerCreate(
        "motor_query_timer",
        timer_period,
        pdTRUE,  // 自动重载
        (void*)scheduler,  // 定时器ID传递scheduler指针
        query_timer_callback
    );
    
    if (!scheduler->query_timer) {
        ESP_LOGE(TAG, "创建定时器失败");
        vTaskDelete(scheduler->query_task_handle);
        vQueueDelete(scheduler->query_queue);
        free(scheduler);
        return NULL;
    }
    
    ESP_LOGI(TAG, "电机状态调度器初始化成功 - 频率: %.1f Hz, 定时器周期: %lu ms", 
             config->frequency, (unsigned long)(1000.0f / config->frequency));
    
    return scheduler;
}

// 轻量级定时器回调 - 只发送事件到队列
static void query_timer_callback(TimerHandle_t xTimer) {
    motor_status_scheduler_t* scheduler = (motor_status_scheduler_t*)pvTimerGetTimerID(xTimer);
    
    if (!scheduler || !scheduler->query_queue) {
        return;
    }
    
    // 创建查询事件
    query_event_t event;
    event.uart_port = scheduler->uart_port;
    event.type = (query_event_type_t)scheduler->current_query_index;
    event.exception_type = 0; // 默认值
    
    // 如果是异常查询，设置异常类型并更新计数器
    if (event.type == QUERY_EVENT_EXCEPTIONS) {
        event.exception_type = scheduler->current_exception_type;
        scheduler->current_exception_type = (scheduler->current_exception_type + 1) % 5; // 0-4循环
    }
    
    // 发送事件到队列（非阻塞）
    BaseType_t ret = xQueueSendFromISR(scheduler->query_queue, &event, NULL);
    if (ret != pdPASS) {
        // 队列满了，跳过这次查询（避免阻塞）
        return;
    }
    
    // 更新查询索引
    scheduler->current_query_index = (scheduler->current_query_index + 1) % QUERY_TYPES_COUNT;
}

bool motor_status_scheduler_start(motor_status_scheduler_t* scheduler) {
    if (!scheduler) {
        ESP_LOGE(TAG, "调度器句柄为空");
        return false;
    }
    
    if (scheduler->is_running) {
        ESP_LOGW(TAG, "调度器已经在运行");
        return true;
    }
    
    if (!scheduler->query_timer) {
        ESP_LOGE(TAG, "定时器未初始化");
        return false;
    }
    
    scheduler->auto_query_enabled = true;
    scheduler->current_query_index = 0;
    scheduler->current_exception_type = 0;
    
    if (xTimerStart(scheduler->query_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "启动定时器失败");
        return false;
    }
    
    scheduler->is_running = true;
    ESP_LOGI(TAG, "电机状态调度器已启动 - 频率: %.1f Hz", scheduler->query_frequency);
    
    return true;
}

void motor_status_scheduler_stop(motor_status_scheduler_t* scheduler) {
    if (!scheduler || !scheduler->is_running) {
        return;
    }
    
    scheduler->auto_query_enabled = false;
    
    if (scheduler->query_timer) {
        xTimerStop(scheduler->query_timer, 0);
    }
    
    scheduler->is_running = false;
    ESP_LOGI(TAG, "电机状态调度器已停止");
}

void motor_status_scheduler_deinit(motor_status_scheduler_t* scheduler) {
    if (!scheduler) {
        return;
    }
    
    motor_status_scheduler_stop(scheduler);
    
    // 删除定时器
    if (scheduler->query_timer) {
        xTimerDelete(scheduler->query_timer, 0);
    }
    
    // 删除查询任务
    if (scheduler->query_task_handle) {
        vTaskDelete(scheduler->query_task_handle);
    }
    
    // 删除队列
    if (scheduler->query_queue) {
        vQueueDelete(scheduler->query_queue);
    }
    
    free(scheduler);
    ESP_LOGI(TAG, "电机状态调度器已销毁");
}

bool motor_status_scheduler_set_frequency(motor_status_scheduler_t* scheduler, float frequency) {
    if (!scheduler) {
        ESP_LOGE(TAG, "调度器句柄为空");
        return false;
    }
    
    if (frequency < MIN_FREQUENCY || frequency > MAX_FREQUENCY) {
        ESP_LOGE(TAG, "频率超出范围 [%.1f, %.1f]: %.1f", MIN_FREQUENCY, MAX_FREQUENCY, frequency);
        return false;
    }
    
    bool was_running = scheduler->is_running;
    
    if (was_running) {
        motor_status_scheduler_stop(scheduler);
    }
    
    scheduler->query_frequency = frequency;
    
    // 重新创建定时器
    if (scheduler->query_timer) {
        xTimerDelete(scheduler->query_timer, 0);
    }
    
    TickType_t timer_period = pdMS_TO_TICKS((uint32_t)(1000.0f / frequency));
    
    scheduler->query_timer = xTimerCreate(
        "motor_query_timer",
        timer_period,
        pdTRUE,
        (void*)scheduler,
        query_timer_callback
    );
    
    if (!scheduler->query_timer) {
        ESP_LOGE(TAG, "重新创建定时器失败");
        return false;
    }
    
    if (was_running) {
        motor_status_scheduler_start(scheduler);
    }
    
    ESP_LOGI(TAG, "查询频率已更新为: %.1f Hz, 定时器周期: %lu ms", 
             frequency, (unsigned long)(1000.0f / frequency));
    
    return true;
}

float motor_status_scheduler_get_frequency(motor_status_scheduler_t* scheduler) {
    if (!scheduler) {
        return 0.0f;
    }
    return scheduler->query_frequency;
}

bool motor_status_scheduler_is_running(motor_status_scheduler_t* scheduler) {
    if (!scheduler) {
        return false;
    }
    return scheduler->is_running;
}
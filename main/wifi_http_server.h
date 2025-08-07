#ifndef WIFI_HTTP_SERVER_H
#define WIFI_HTTP_SERVER_H

#include "esp_http_server.h"
#include "motor_control.h"
#include "motor_status_scheduler.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化WiFi热点
 */
void wifi_init_softap(void);

/**
 * @brief 启动Web服务器
 * @param motor_controller 电机控制器句柄
 * @return HTTP服务器句柄，失败返回NULL
 */
httpd_handle_t start_webserver(motor_controller_t* motor_controller);

/**
 * @brief 停止Web服务器
 * @param server HTTP服务器句柄
 */
void stop_webserver(httpd_handle_t server);

/**
 * @brief 设置状态查询调度器
 * @param scheduler 调度器句柄
 */
void set_status_scheduler(motor_status_scheduler_t* scheduler);

#ifdef __cplusplus
}
#endif

#endif // WIFI_HTTP_SERVER_H
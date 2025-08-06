#ifndef GCODE_UNIFIED_CONTROL_H
#define GCODE_UNIFIED_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

// G代码控制器配置结构
typedef struct {
    motor_controller_t* motor_controller;  // 电机控制器句柄
    char* response_buffer;                 // 响应缓冲区
    size_t response_buffer_size;          // 响应缓冲区大小
} gcode_controller_config_t;

// G代码控制器句柄
typedef struct {
    gcode_controller_config_t config;     // 配置信息
    char command_buffer[256];             // 命令重组缓冲区
    size_t command_length;                // 当前命令长度
    bool is_initialized;                  // 初始化状态
} gcode_controller_t;

// G代码解析结果
typedef enum {
    GCODE_RESULT_OK = 0,                  // 执行成功
    GCODE_RESULT_ERROR = 1,               // 执行错误
    GCODE_RESULT_INVALID_COMMAND = 2,     // 无效命令
    GCODE_RESULT_INVALID_PARAMETER = 3,   // 无效参数
    GCODE_RESULT_MOTOR_ERROR = 4,         // 电机错误
    GCODE_RESULT_BUFFER_FULL = 5          // 缓冲区满
} gcode_result_t;

// 电机控制模式
typedef enum {
    MOTOR_MODE_POSITION = 0,              // 位置模式
    MOTOR_MODE_VELOCITY = 1,              // 速度模式
    MOTOR_MODE_TORQUE = 2                 // 力矩模式
} motor_control_mode_t;

/**
 * @brief 初始化G代码控制器
 * @param config 控制器配置
 * @return G代码控制器句柄，失败返回NULL
 */
gcode_controller_t* gcode_controller_init(const gcode_controller_config_t* config);

/**
 * @brief 销毁G代码控制器
 * @param controller G代码控制器句柄
 */
void gcode_controller_deinit(gcode_controller_t* controller);

/**
 * @brief 处理CAN帧数据
 * @param controller G代码控制器句柄
 * @param data CAN帧数据
 * @param length 数据长度
 * @return 处理结果
 */
gcode_result_t gcode_process_can_frame(gcode_controller_t* controller, 
                                     const uint8_t* data, size_t length);

/**
 * @brief 检查是否为G代码CAN帧
 * @param data CAN帧数据
 * @param length 数据长度
 * @return true为G代码帧，false为其他帧
 */
bool gcode_is_gcode_can_frame(const uint8_t* data, size_t length);

/**
 * @brief ASCII十六进制解码
 * @param hex_data ASCII十六进制数据
 * @param hex_length 十六进制数据长度
 * @param output 输出缓冲区
 * @param output_size 输出缓冲区大小
 * @return 解码后的字符串长度，-1表示错误
 */
int gcode_hex_decode(const uint8_t* hex_data, size_t hex_length, 
                    char* output, size_t output_size);

/**
 * @brief 解析并执行G代码命令
 * @param controller G代码控制器句柄
 * @param command G代码命令字符串
 * @return 执行结果
 */
gcode_result_t gcode_execute_command(gcode_controller_t* controller, 
                                   const char* command);

/**
 * @brief 获取最后一次操作的响应消息
 * @param controller G代码控制器句柄
 * @return 响应消息字符串
 */
const char* gcode_get_response(gcode_controller_t* controller);

// 内部函数声明（用于测试）
gcode_result_t gcode_parse_g1_command(const char* command, char* param_type, float* value);
gcode_result_t gcode_parse_m_command(const char* command, int* m_code);
gcode_result_t gcode_execute_g1(gcode_controller_t* controller, char param_type, float value);
gcode_result_t gcode_execute_m(gcode_controller_t* controller, int m_code);

#ifdef __cplusplus
}
#endif

#endif // GCODE_UNIFIED_CONTROL_H
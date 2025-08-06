#include "gcode_unified_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"

// 引用main.c中的转换函数
extern float angle_to_position(float angle_degrees);
extern float external_velocity_to_internal(float external_velocity);
extern float external_torque_to_internal(float external_torque);

static const char *TAG = "GCODE_CTRL";

/**
 * @brief 初始化G代码控制器
 */
gcode_controller_t* gcode_controller_init(const gcode_controller_config_t* config)
{
    if (!config || !config->motor_controller) {
        ESP_LOGE(TAG, "配置参数无效");
        return NULL;
    }

    gcode_controller_t* controller = malloc(sizeof(gcode_controller_t));
    if (!controller) {
        ESP_LOGE(TAG, "内存分配失败");
        return NULL;
    }

    memset(controller, 0, sizeof(gcode_controller_t));
    controller->config = *config;
    controller->is_initialized = true;

    ESP_LOGI(TAG, "G代码控制器初始化成功");
    return controller;
}

/**
 * @brief 销毁G代码控制器
 */
void gcode_controller_deinit(gcode_controller_t* controller)
{
    if (controller) {
        controller->is_initialized = false;
        free(controller);
        ESP_LOGI(TAG, "G代码控制器已销毁");
    }
}

/**
 * @brief 检查是否为G代码CAN帧
 */
bool gcode_is_gcode_can_frame(const uint8_t* data, size_t length)
{
    // 检查CAN ID: 0x0100 (小端格式: 00 01)
    // 数据格式: [ID_LOW, ID_HIGH, DATA...]
    return (length >= 2 && data[0] == 0x00 && data[1] == 0x01);
}

/**
 * @brief ASCII十六进制解码
 */
int gcode_hex_decode(const uint8_t* hex_data, size_t hex_length, 
                    char* output, size_t output_size)
{
    if (!hex_data || !output || hex_length % 2 != 0) {
        return -1;
    }

    size_t output_length = hex_length / 2;
    if (output_length >= output_size) {
        return -1; // 输出缓冲区不够
    }

    for (size_t i = 0; i < output_length; i++) {
        char high_nibble = hex_data[i * 2];
        char low_nibble = hex_data[i * 2 + 1];

        // 转换高位
        uint8_t high_value;
        if (high_nibble >= '0' && high_nibble <= '9') {
            high_value = high_nibble - '0';
        } else if (high_nibble >= 'A' && high_nibble <= 'F') {
            high_value = high_nibble - 'A' + 10;
        } else if (high_nibble >= 'a' && high_nibble <= 'f') {
            high_value = high_nibble - 'a' + 10;
        } else {
            return -1; // 无效字符
        }

        // 转换低位
        uint8_t low_value;
        if (low_nibble >= '0' && low_nibble <= '9') {
            low_value = low_nibble - '0';
        } else if (low_nibble >= 'A' && low_nibble <= 'F') {
            low_value = low_nibble - 'A' + 10;
        } else if (low_nibble >= 'a' && low_nibble <= 'f') {
            low_value = low_nibble - 'a' + 10;
        } else {
            return -1; // 无效字符
        }

        output[i] = (high_value << 4) | low_value;
    }

    output[output_length] = '\0'; // 添加字符串结束符
    return output_length;
}

/**
 * @brief 处理CAN帧数据
 */
gcode_result_t gcode_process_can_frame(gcode_controller_t* controller, 
                                     const uint8_t* data, size_t length)
{
    if (!controller || !controller->is_initialized || !data) {
        return GCODE_RESULT_ERROR;
    }

    // 检查是否为G代码帧
    if (!gcode_is_gcode_can_frame(data, length)) {
        return GCODE_RESULT_INVALID_COMMAND; // 不是G代码帧
    }

    // 跳过CAN ID (前2字节)
    const uint8_t* payload = data + 2;
    size_t payload_length = length - 2;

    if (payload_length == 0) {
        return GCODE_RESULT_INVALID_PARAMETER;
    }

    // CAN帧数据就是直接的ASCII字符，不需要十六进制解码
    // 检查payload中是否包含额外的CAN ID (00 01)，这可能是粘包问题
    const uint8_t* clean_payload = payload;
    size_t clean_length = payload_length;
    
    // 查找payload中的00 01模式，移除它们
    uint8_t clean_buffer[128];
    size_t clean_pos = 0;
    
    for (size_t i = 0; i < payload_length; i++) {
        // 检查是否是CAN ID开始 (00 01)
        if (i + 1 < payload_length && payload[i] == 0x00 && payload[i + 1] == 0x01) {
            ESP_LOGW(TAG, "发现粘包中的CAN ID，跳过: %02X %02X", payload[i], payload[i + 1]);
            i += 1; // 跳过01，外层循环会跳过00
            continue;
        }
        
        // 只保留可打印的ASCII字符和常见控制字符
        if ((payload[i] >= 32 && payload[i] <= 126) || 
            payload[i] == '\r' || payload[i] == '\n' || payload[i] == '\t') {
            if (clean_pos < sizeof(clean_buffer) - 1) {
                clean_buffer[clean_pos++] = payload[i];
            }
        }
    }
    
    clean_payload = clean_buffer;
    clean_length = clean_pos;
    
    if (clean_length == 0) {
        ESP_LOGW(TAG, "清理后无有效数据");
        return GCODE_RESULT_OK;
    }

    // 添加到命令缓冲区
    if (controller->command_length + clean_length >= sizeof(controller->command_buffer) - 1) {
        ESP_LOGW(TAG, "命令缓冲区溢出，清空重新开始");
        controller->command_length = 0; // 重置缓冲区
    }

    memcpy(controller->command_buffer + controller->command_length, 
           clean_payload, clean_length);
    controller->command_length += clean_length;
    controller->command_buffer[controller->command_length] = '\0';
    
    ESP_LOGI(TAG, "当前命令缓冲区: [%s] (长度:%d)", controller->command_buffer, controller->command_length);

    // 检查是否有完整命令（以回车或换行结束，或者是已知的完整G代码命令）
    char* newline_pos = strchr(controller->command_buffer, '\n');
    char* return_pos = strchr(controller->command_buffer, '\r');
    char* end_pos = newline_pos;
    if (!end_pos || (return_pos && return_pos < end_pos)) {
        end_pos = return_pos;
    }

    bool has_complete_command = false;
    size_t command_end_pos = controller->command_length;

    if (end_pos) {
        // 找到换行符，肯定是完整命令
        has_complete_command = true;
        command_end_pos = end_pos - controller->command_buffer;
    } else {
        // 没有换行符，检查是否是完整的G代码命令
        // 更严格的判断：确保G1后有有效的参数（X/F/T+数字），M后有数字
        if (controller->command_length >= 4 && strncmp(controller->command_buffer, "G1", 2) == 0) {
            // 检查G1后面是否有有效参数
            const char* params = controller->command_buffer + 2;
            while (*params == ' ') params++; // 跳过空格
            
            if ((*params == 'X' || *params == 'F' || *params == 'T') && 
                (params[1] != '\0' && (isdigit((unsigned char)params[1]) || params[1] == '-' || params[1] == '.'))) {
                // 检查是否有完整的数字（简单检查：至少有一个数字字符）
                bool has_digit = false;
                for (const char* p = params + 1; *p != '\0'; p++) {
                    if (isdigit((unsigned char)*p)) {
                        has_digit = true;
                        break;
                    }
                }
                if (has_digit) {
                    has_complete_command = true;
                    command_end_pos = controller->command_length;
                }
            }
        } else if (controller->command_length >= 2 && controller->command_buffer[0] == 'M') {
            // 检查M后面是否有数字
            if (isdigit((unsigned char)controller->command_buffer[1])) {
                has_complete_command = true;
                command_end_pos = controller->command_length;
            }
        }
    }

    if (has_complete_command) {
        // 有完整命令，执行它
        char temp_buffer[256];
        memcpy(temp_buffer, controller->command_buffer, command_end_pos);
        temp_buffer[command_end_pos] = '\0';
        
        gcode_result_t result = gcode_execute_command(controller, temp_buffer);
        
        // 移除已执行的命令
        size_t executed_length = (end_pos) ? (end_pos - controller->command_buffer + 1) : command_end_pos;
        size_t remaining_length = controller->command_length - executed_length;
        if (remaining_length > 0) {
            memmove(controller->command_buffer, 
                   controller->command_buffer + executed_length, 
                   remaining_length);
        }
        controller->command_length = remaining_length;
        controller->command_buffer[controller->command_length] = '\0';

        // 如果命令执行失败，清理可能的垃圾数据
        if (result != GCODE_RESULT_OK && controller->command_length > 0) {
            ESP_LOGW(TAG, "命令执行失败，清理缓冲区垃圾数据");
            controller->command_length = 0;
            controller->command_buffer[0] = '\0';
        }

        return result;
    }

    return GCODE_RESULT_OK; // 等待更多数据
}

/**
 * @brief 解析G1命令
 */
gcode_result_t gcode_parse_g1_command(const char* command, char* param_type, float* value)
{
    if (!command || !param_type || !value) {
        return GCODE_RESULT_INVALID_PARAMETER;
    }

    // 跳过G1部分
    const char* params = command + 2; // "G1"后面的参数
    while (*params && isspace((unsigned char)*params)) {
        params++; // 跳过空格
    }

    if (*params == '\0') {
        return GCODE_RESULT_INVALID_PARAMETER;
    }

    // 提取参数类型（X、F、T）
    *param_type = toupper(*params);
    if (*param_type != 'X' && *param_type != 'F' && *param_type != 'T') {
        return GCODE_RESULT_INVALID_PARAMETER;
    }

    // 提取数值
    params++; // 跳过参数字母
    char* endptr;
    *value = strtof(params, &endptr);
    
    if (endptr == params) {
        return GCODE_RESULT_INVALID_PARAMETER; // 没有数值
    }

    return GCODE_RESULT_OK;
}

/**
 * @brief 解析M命令
 */
gcode_result_t gcode_parse_m_command(const char* command, int* m_code)
{
    if (!command || !m_code) {
        return GCODE_RESULT_INVALID_PARAMETER;
    }

    // 解析M后面的数字
    const char* number_start = command + 1; // "M"后面的数字
    char* endptr;
    *m_code = (int)strtol(number_start, &endptr, 10);
    
    if (endptr == number_start) {
        return GCODE_RESULT_INVALID_PARAMETER; // 没有数字
    }

    return GCODE_RESULT_OK;
}

/**
 * @brief 执行G1命令
 */
gcode_result_t gcode_execute_g1(gcode_controller_t* controller, char param_type, float value)
{
    if (!controller || !controller->config.motor_controller) {
        return GCODE_RESULT_ERROR;
    }

    ESP_LOGI(TAG, "执行G1命令: %c%.2f", param_type, value);

    switch (param_type) {
        case 'X': {
            // 位置模式
            motor_control_set_position_mode(controller->config.motor_controller);
            float position = angle_to_position(value);
            motor_control_set_position(controller->config.motor_controller, position);
            if (controller->config.response_buffer) {
                snprintf(controller->config.response_buffer, 
                        controller->config.response_buffer_size,
                        "OK - 位置模式: %.2f度 -> %.2f", value, position);
            }
            break;
        }
        case 'F': {
            // 速度模式
            motor_control_set_velocity_mode(controller->config.motor_controller);
            float velocity = external_velocity_to_internal(value);
            motor_control_set_velocity(controller->config.motor_controller, velocity);
            if (controller->config.response_buffer) {
                snprintf(controller->config.response_buffer, 
                        controller->config.response_buffer_size,
                        "OK - 速度模式: %.2f r/s -> %.2f r/s", value, velocity);
            }
            break;
        }
        case 'T': {
            // 力矩模式
            motor_control_set_torque_mode(controller->config.motor_controller);
            float torque = external_torque_to_internal(value);
            motor_control_set_torque(controller->config.motor_controller, torque);
            if (controller->config.response_buffer) {
                snprintf(controller->config.response_buffer, 
                        controller->config.response_buffer_size,
                        "OK - 力矩模式: %.2f Nm -> %.2f Nm", value, torque);
            }
            break;
        }
        default:
            return GCODE_RESULT_INVALID_PARAMETER;
    }

    return GCODE_RESULT_OK;
}

/**
 * @brief 执行M命令
 */
gcode_result_t gcode_execute_m(gcode_controller_t* controller, int m_code)
{
    if (!controller || !controller->config.motor_controller) {
        return GCODE_RESULT_ERROR;
    }

    ESP_LOGI(TAG, "执行M命令: M%d", m_code);

    switch (m_code) {
        case 0:
            // M0 - 失能电机
            motor_control_enable(controller->config.motor_controller, false);
            if (controller->config.response_buffer) {
                snprintf(controller->config.response_buffer, 
                        controller->config.response_buffer_size,
                        "OK - 电机已失能");
            }
            break;
        case 1:
            // M1 - 使能电机
            motor_control_enable(controller->config.motor_controller, true);
            if (controller->config.response_buffer) {
                snprintf(controller->config.response_buffer, 
                        controller->config.response_buffer_size,
                        "OK - 电机已使能");
            }
            break;
        default:
            return GCODE_RESULT_INVALID_COMMAND;
    }

    return GCODE_RESULT_OK;
}

/**
 * @brief 解析并执行G代码命令
 */
gcode_result_t gcode_execute_command(gcode_controller_t* controller, const char* command)
{
    if (!controller || !command) {
        return GCODE_RESULT_INVALID_PARAMETER;
    }

    // 去除前后空格
    while (*command && isspace((unsigned char)*command)) {
        command++;
    }

    if (*command == '\0') {
        return GCODE_RESULT_INVALID_COMMAND; // 空命令
    }

    ESP_LOGI(TAG, "收到G代码命令: %s", command);

    // 解析命令类型
    if (strncmp(command, "G1", 2) == 0) {
        // G1命令
        char param_type;
        float value;
        gcode_result_t parse_result = gcode_parse_g1_command(command, &param_type, &value);
        if (parse_result != GCODE_RESULT_OK) {
            if (controller->config.response_buffer) {
                snprintf(controller->config.response_buffer, 
                        controller->config.response_buffer_size,
                        "ERROR - G1命令参数无效");
            }
            return parse_result;
        }
        return gcode_execute_g1(controller, param_type, value);
    }
    else if (*command == 'M') {
        // M命令
        int m_code;
        gcode_result_t parse_result = gcode_parse_m_command(command, &m_code);
        if (parse_result != GCODE_RESULT_OK) {
            if (controller->config.response_buffer) {
                snprintf(controller->config.response_buffer, 
                        controller->config.response_buffer_size,
                        "ERROR - M命令格式无效");
            }
            return parse_result;
        }
        return gcode_execute_m(controller, m_code);
    }
    else {
        // 未知命令
        if (controller->config.response_buffer) {
            snprintf(controller->config.response_buffer, 
                    controller->config.response_buffer_size,
                    "ERROR - 未知命令: %s", command);
        }
        return GCODE_RESULT_INVALID_COMMAND;
    }
}

/**
 * @brief 获取最后一次操作的响应消息
 */
const char* gcode_get_response(gcode_controller_t* controller)
{
    if (controller && controller->config.response_buffer) {
        return controller->config.response_buffer;
    }
    return "ERROR - 无响应缓冲区";
}
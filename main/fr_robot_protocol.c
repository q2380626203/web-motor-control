/**
 * @file fr_robot_protocol.c
 * @brief 法奥机器人UDP扩展轴通信协议实现
 */

#include "fr_robot_protocol.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "FR_PROTO";

// 轴状态
static fr_axis_state_t axis_states[FR_AXIS_COUNT];

// 帧计数
static uint16_t frame_counter = 0;

// ==================== CRC16计算 (Modbus) ====================
uint16_t fr_calc_crc16(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// ==================== 初始化 ====================
void fr_protocol_init(void)
{
    memset(axis_states, 0, sizeof(axis_states));
    frame_counter = 0;
    ESP_LOGI(TAG, "法奥协议模块初始化完成");
}

// ==================== 解析机器人命令包 ====================
bool fr_parse_robot_packet(const uint8_t *data, int len, fr_robot_packet_t *packet)
{
    if (len < FR_ROBOT_PACKET_SIZE) {
        ESP_LOGW(TAG, "数据长度不足: %d < %d", len, FR_ROBOT_PACKET_SIZE);
        return false;
    }

    memcpy(packet, data, sizeof(fr_robot_packet_t));

    // 验证帧头
    if (packet->frame_header != FR_FRAME_HEADER) {
        ESP_LOGW(TAG, "帧头错误: 0x%04X", packet->frame_header);
        return false;
    }

    return true;
}

// ==================== 处理单轴命令 ====================
void fr_process_axis_command(const fr_axis_cmd_t *cmd, int axis_id)
{
    if (axis_id < 0 || axis_id >= FR_AXIS_COUNT) {
        return;
    }

    fr_axis_state_t *state = &axis_states[axis_id];

    // 处理使能命令
    bool enable_cmd = (cmd->control_word & FR_CTRL_ENABLE) != 0;
    if (enable_cmd && !state->enabled) {
        state->enabled = true;
        ESP_LOGI(TAG, "轴%d 使能", axis_id + 1);
    } else if (!enable_cmd && state->enabled) {
        state->enabled = false;
        ESP_LOGI(TAG, "轴%d 去使能", axis_id + 1);
    }

    // 处理回零命令
    if (cmd->home_command == FR_HOME_CMD_START) {
        if (!state->homing && !state->home_done) {
            state->homing = true;
            ESP_LOGI(TAG, "轴%d 开始回零 (高速:%ld, 低速:%ld)",
                     axis_id + 1, cmd->home_high_speed, cmd->home_low_speed);
        }
    }

    // 更新目标位置
    if (state->enabled && !state->homing) {
        if (state->target_position != cmd->target_position) {
            ESP_LOGD(TAG, "轴%d 目标位置: %ld", axis_id + 1, cmd->target_position);
        }
        state->target_position = cmd->target_position;
    }
}

// ==================== 获取轴状态 ====================
fr_axis_state_t* fr_get_axis_state(int axis_id)
{
    if (axis_id < 0 || axis_id >= FR_AXIS_COUNT) {
        return NULL;
    }
    return &axis_states[axis_id];
}

// ==================== 构建PLC响应包 ====================
void fr_build_plc_response(fr_plc_packet_t *packet, uint16_t frame_count)
{
    memset(packet, 0, sizeof(fr_plc_packet_t));

    // 帧头
    packet->frame_header = FR_FRAME_HEADER;

    // 填充各轴状态
    for (int i = 0; i < FR_AXIS_COUNT; i++) {
        fr_axis_status_t *status = &packet->axis_status[i];
        fr_axis_state_t *state = &axis_states[i];

        // 状态字
        if (state->enabled) {
            if (state->homing) {
                status->status_word = FR_STATUS_READY | FR_STATUS_ENABLED;  // 0x0003
            } else {
                status->status_word = FR_STATUS_READY | FR_STATUS_ENABLED;  // 0x0003
            }
        } else {
            status->status_word = FR_STATUS_ENABLED;  // 0x0002 (默认状态)
        }

        // 当前位置
        status->current_position = state->current_position;

        // 回零状态
        if (state->homing) {
            status->home_status = FR_HOME_STATUS_BUSY;  // 0x0001
        } else if (state->home_done) {
            status->home_status = FR_HOME_STATUS_DONE;  // 0x0002
        } else {
            status->home_status = FR_HOME_STATUS_NONE;  // 0x0000
        }

        // 故障码
        status->fault_code = state->fault_code;

        // 速度反馈
        status->speed_feedback = state->speed;
    }

    // 帧计数
    packet->frame_count = frame_count;

    // 计算CRC16 (前158字节)
    packet->crc16 = fr_calc_crc16((const uint8_t*)packet, 158);
}

// ==================== 更新运动（模拟电机运动） ====================
void fr_update_motion(int dt_ms)
{
    float dt = dt_ms / 1000.0f;

    for (int i = 0; i < FR_AXIS_COUNT; i++) {
        fr_axis_state_t *state = &axis_states[i];

        // 回零处理
        if (state->homing) {
            // 模拟回零过程：位置逐渐归零
            if (state->current_position > 0) {
                state->speed = -50000;  // 反向运动
                state->current_position += (int32_t)(state->speed * dt);
                if (state->current_position < 0) {
                    state->current_position = 0;
                }
            } else if (state->current_position < 0) {
                state->speed = 50000;  // 正向运动
                state->current_position += (int32_t)(state->speed * dt);
                if (state->current_position > 0) {
                    state->current_position = 0;
                }
            } else {
                // 回零完成
                state->homing = false;
                state->home_done = true;
                state->speed = 0;
                ESP_LOGI(TAG, "轴%d 回零完成", i + 1);
            }
        }
        // 正常位置控制
        else if (state->enabled && !state->home_done) {
            // 未回零时不能运动
            state->speed = 0;
        }
        else if (state->enabled && state->home_done) {
            // 跟随目标位置
            int32_t diff = state->target_position - state->current_position;
            int32_t max_speed = 100000;  // 最大速度 100000 脉冲/s
            int32_t move_speed;

            if (diff > 0) {
                move_speed = (diff > max_speed * dt) ? max_speed : (int32_t)(diff / dt);
                state->speed = move_speed;
            } else if (diff < 0) {
                move_speed = (-diff > max_speed * dt) ? -max_speed : (int32_t)(diff / dt);
                state->speed = move_speed;
            } else {
                state->speed = 0;
            }

            // 更新位置
            state->current_position += (int32_t)(state->speed * dt);
        }
        else {
            // 未使能，停止
            state->speed = 0;
        }
    }
}
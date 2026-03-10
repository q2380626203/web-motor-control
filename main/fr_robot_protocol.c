/**
 * @file fr_robot_protocol.c
 * @brief 法奥机器人UDP扩展轴通信协议实现
 */

#include "fr_robot_protocol.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "FR_PROTO";

// 轴状态
static fr_axis_state_t axis_states[FR_AXIS_COUNT];

// 帧计数
static uint16_t frame_counter = 0;

// 绑定的电机控制器（轴1对应motor_controller_1）
static motor_controller_t *g_motor = NULL;

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
    g_motor = NULL;
    ESP_LOGI(TAG, "法奥协议模块初始化完成");
}

void fr_protocol_set_motor(motor_controller_t *ctrl)
{
    if (ctrl == NULL) {
        ESP_LOGE(TAG, "fr_protocol_set_motor: 电机控制器为NULL，忽略");
        return;
    }
    g_motor = ctrl;
    ESP_LOGI(TAG, "轴1已绑定电机控制器 (motor_id=%d)", ctrl->driver_config.motor_id);
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
        // 轴1：设置位置模式并使能，同时将目标同步到当前位置防止突然运动
        if (axis_id == 0 && g_motor != NULL) {
            motor_control_set_position_mode(g_motor);
            motor_control_enable(g_motor, true);
            // 同步目标位置到当前电机位置，避免使能瞬间冲向旧目标
            motor_status_t *ms = get_motor_status();
            state->target_position = (int32_t)(ms->position / FR_PULSES_TO_REV);
            state->current_position = state->target_position;
            ESP_LOGI(TAG, "轴1 → 位置模式+使能，当前位置同步: %ld 脉冲", state->target_position);
        }
    } else if (!enable_cmd && state->enabled) {
        state->enabled = false;
        state->homing = false;   // 清除中途回零状态
        state->speed = 0;
        ESP_LOGI(TAG, "轴%d 去使能", axis_id + 1);
        // 轴1：去使能真实电机
        if (axis_id == 0 && g_motor != NULL) {
            motor_control_enable(g_motor, false);
        }
    }

    // 处理回零命令
    if (cmd->home_command == FR_HOME_CMD_START) {
        if (!state->homing) {
            state->homing = true;
            state->home_done = false;  // 允许重复回零
            ESP_LOGI(TAG, "轴%d 开始回零 (高速:%ld, 低速:%ld)",
                     axis_id + 1, cmd->home_high_speed, cmd->home_low_speed);
        }
    }

    // 更新目标位置（脉冲）
    if (state->enabled && !state->homing) {
        if (state->target_position != cmd->target_position) {
            float target_mm = cmd->target_position * FR_PULSES_TO_REV * MOTOR_MM_PER_REV;
            ESP_LOGD(TAG, "轴%d 目标位置: %ld 脉冲 (%.2f mm)",
                     axis_id + 1, cmd->target_position, target_mm);
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

        // 状态字：只看使能状态
        //   0x0003 = 就绪+已使能（enabled=true，包括回零完成后继续使能）
        //   0x0002 = 待机（enabled=false）
        if (state->enabled) {
            status->status_word = FR_STATUS_READY | FR_STATUS_ENABLED;  // 0x0003
        } else {
            status->status_word = FR_STATUS_ENABLED;  // 0x0002
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

// ==================== 更新运动 ====================
void fr_update_motion(int dt_ms)
{
    float dt = dt_ms / 1000.0f;

    for (int i = 0; i < FR_AXIS_COUNT; i++) {
        fr_axis_state_t *state = &axis_states[i];

        // ---- 轴1：驱动真实 TWAI 电机 ----
        if (i == 0 && g_motor != NULL) {
            uint8_t motor_id = g_motor->driver_config.motor_id;

            // 每个周期主动查询一次电机转子位置，确保位置反馈实时（不依赖低频调度器）
            query_motor_position_speed(motor_id);

            if (!state->enabled) {
                // 未使能时仍然更新位置反馈，让FR端看到真实位置
                motor_status_t *ms = get_motor_status();
                state->current_position = (int32_t)(ms->position / FR_PULSES_TO_REV);
                state->speed = 0;
                continue;
            }

            motor_status_t *ms = get_motor_status();
            float current_rev = ms->position;  // 单位：转

            // 回零：发送目标 0 转，到达后完成
            if (state->homing) {
                float diff = 0.0f - current_rev;
                if (fabsf(diff) < 0.05f) {
                    state->homing = false;
                    state->home_done = true;
                    state->speed = 0;
                    state->current_position = 0;
                    ESP_LOGI(TAG, "轴1 回零完成");
                } else {
                    float step = (fabsf(diff) > MAX_STEP_REV) ?
                                 current_rev + (diff > 0 ? MAX_STEP_REV : -MAX_STEP_REV) : 0.0f;
                    send_target_position(motor_id, step);
                    state->speed = (diff > 0) ? 10000 : -10000;
                }
                // 更新位置反馈（转 → 脉冲）
                state->current_position = (int32_t)(current_rev / FR_PULSES_TO_REV);
                continue;
            }

            // 正常位置控制
            // FR目标脉冲 → 目标转数
            float target_rev = (float)state->target_position * FR_PULSES_TO_REV;
            float diff_rev = target_rev - current_rev;

            // 更新位置反馈（转 → 脉冲）
            state->current_position = (int32_t)(current_rev / FR_PULSES_TO_REV);

            if (fabsf(diff_rev) < 0.05f) {
                // 已到位
                state->speed = 0;
                continue;
            }

            // 计算本步目标（单步最多 40 转）
            float next_rev = (fabsf(diff_rev) > MAX_STEP_REV) ?
                             current_rev + (diff_rev > 0 ? MAX_STEP_REV : -MAX_STEP_REV) :
                             target_rev;

            send_target_position(motor_id, next_rev);
            state->speed = (diff_rev > 0) ? 10000 : -10000;

            ESP_LOGD(TAG, "轴1 实际%.2f转(%.2fmm) 目标%.2f转(%.2fmm) 本步%.2f转",
                     current_rev, current_rev * MOTOR_MM_PER_REV,
                     target_rev,  target_rev  * MOTOR_MM_PER_REV,
                     next_rev);
            continue;
        }

        // ---- 轴2-4：模拟运动 ----
        if (state->homing) {
            if (state->current_position > 0) {
                state->speed = -50000;
                state->current_position += (int32_t)(state->speed * dt);
                if (state->current_position < 0) state->current_position = 0;
            } else if (state->current_position < 0) {
                state->speed = 50000;
                state->current_position += (int32_t)(state->speed * dt);
                if (state->current_position > 0) state->current_position = 0;
            } else {
                state->homing = false;
                state->home_done = true;
                state->speed = 0;
                ESP_LOGI(TAG, "轴%d 回零完成", i + 1);
            }
        } else if (state->enabled) {
            int32_t diff = state->target_position - state->current_position;
            int32_t max_speed = 100000;
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
            state->current_position += (int32_t)(state->speed * dt);
        } else {
            state->speed = 0;
        }
    }
}
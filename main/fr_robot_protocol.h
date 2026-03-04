/**
 * @file fr_robot_protocol.h
 * @brief 法奥机器人UDP扩展轴通信协议
 *
 * 协议说明：
 * - 机器人发送: 154字节控制命令包
 * - PLC响应: 160字节状态反馈包
 * - 端口: 8211
 * - 帧头: 0x5A5A
 */

#ifndef FR_ROBOT_PROTOCOL_H
#define FR_ROBOT_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// ==================== 协议常量 ====================
#define FR_FRAME_HEADER         0x5A5A
#define FR_UDP_PORT             8211
#define FR_AXIS_COUNT           4

// 数据包长度
#define FR_ROBOT_PACKET_SIZE    154     // 机器人发送包长度
#define FR_PLC_PACKET_SIZE      160     // PLC响应包长度

// 单轴数据长度
#define FR_AXIS_CMD_SIZE        28      // 单轴控制数据长度
#define FR_AXIS_STATUS_SIZE     30      // 单轴状态数据长度

// ==================== 控制命令字定义 (D200) ====================
#define FR_CTRL_ENABLE          0x0001  // bit0: 使能

// ==================== 回零命令字定义 (D203) ====================
#define FR_HOME_START           0x0001  // bit0: 回零启动
#define FR_HOME_MODE            0x1000  // bit12: 回零模式
#define FR_HOME_CMD_START       0x1001  // 启动回零

// ==================== 状态字定义 (D100) ====================
#define FR_STATUS_READY         0x0001  // bit0: 就绪
#define FR_STATUS_ENABLED       0x0002  // bit1: 已使能

// ==================== 回零状态字定义 (D103) ====================
#define FR_HOME_STATUS_NONE     0x0000  // 未回零
#define FR_HOME_STATUS_BUSY     0x0001  // 回零中
#define FR_HOME_STATUS_DONE     0x0002  // 回零完成
#define FR_HOME_STATUS_ERROR    0x0004  // 回零错误

// ==================== 数据结构 ====================

/**
 * @brief 单轴控制命令数据 (28字节)
 */
typedef struct __attribute__((packed)) {
    int16_t control_word;       // D200: 控制命令字
    int32_t target_position;    // D201: 目标位置 (脉冲)
    uint16_t home_command;      // D203: 回零命令字
    int32_t home_high_speed;    // D204: 回零高速度 (脉冲/s)
    int32_t home_low_speed;     // D206: 回零低速度 (脉冲/s)
    int32_t position_offset;    // D208: 位置偏置
    int32_t speed_offset;       // D210: 速度偏置
    int32_t torque_offset;      // D212: 转矩偏置
} fr_axis_cmd_t;

/**
 * @brief 单轴状态数据 (30字节)
 */
typedef struct __attribute__((packed)) {
    int16_t status_word;        // D100: 状态字
    int32_t current_position;   // D101: 当前位置 (脉冲)
    int16_t home_status;        // D103: 回零状态字
    int32_t home_high_speed_fb; // D104: 寻零速度反馈
    int32_t home_low_speed_fb;  // D106: 爬行速度反馈
    int16_t fault_code;         // D108: 故障码
    int32_t follow_error;       // D109: 随动偏差量
    int32_t speed_feedback;     // D111: 速度反馈 (脉冲/s)
    int32_t torque_feedback;    // D113: 实时转矩
} fr_axis_status_t;

/**
 * @brief 机器人控制命令包 (154字节)
 */
typedef struct __attribute__((packed)) {
    uint16_t frame_header;              // 帧头 0x5A5A
    fr_axis_cmd_t axis_cmd[FR_AXIS_COUNT];  // 4轴控制数据 (112字节)
    uint16_t do_output[8];              // DO输出 (16字节)
    int16_t ao_output[4];               // AO输出 (8字节)
    uint8_t reserved1[6];               // 保留区域
    uint16_t frame_count;               // 帧计数
    uint8_t reserved2[8];               // 保留/校验
} fr_robot_packet_t;

/**
 * @brief PLC状态反馈包 (160字节)
 */
typedef struct __attribute__((packed)) {
    uint16_t frame_header;                  // 帧头 0x5A5A
    fr_axis_status_t axis_status[FR_AXIS_COUNT];  // 4轴状态数据 (120字节)
    uint16_t di_input[8];                   // DI输入 (16字节)
    int16_t ai_input[4];                    // AI输入 (8字节)
    uint8_t reserved[10];                   // 保留区域
    uint16_t frame_count;                   // 帧计数
    uint16_t crc16;                         // CRC16校验
} fr_plc_packet_t;

// ==================== 轴状态结构 ====================
typedef struct {
    bool enabled;               // 使能状态
    bool homing;                // 回零中
    bool home_done;             // 回零完成
    int32_t current_position;   // 当前位置
    int32_t target_position;    // 目标位置
    int32_t speed;              // 当前速度
    int16_t fault_code;         // 故障码
} fr_axis_state_t;

// ==================== 函数声明 ====================

/**
 * @brief 解析机器人控制命令包
 * @param data 接收到的原始数据
 * @param len 数据长度
 * @param packet 输出解析后的命令包
 * @return true 解析成功
 */
bool fr_parse_robot_packet(const uint8_t *data, int len, fr_robot_packet_t *packet);

/**
 * @brief 构建PLC状态响应包
 * @param packet 输出状态包
 * @param frame_count 帧计数
 */
void fr_build_plc_response(fr_plc_packet_t *packet, uint16_t frame_count);

/**
 * @brief 处理机器人命令并更新轴状态
 * @param cmd 控制命令
 * @param axis_id 轴ID (0-3)
 */
void fr_process_axis_command(const fr_axis_cmd_t *cmd, int axis_id);

/**
 * @brief 获取轴状态
 * @param axis_id 轴ID (0-3)
 * @return 轴状态指针
 */
fr_axis_state_t* fr_get_axis_state(int axis_id);

/**
 * @brief 计算CRC16校验 (Modbus)
 * @param data 数据指针
 * @param len 数据长度
 * @return CRC16值
 */
uint16_t fr_calc_crc16(const uint8_t *data, int len);

/**
 * @brief 初始化协议模块
 */
void fr_protocol_init(void);

/**
 * @brief 更新轴运动（周期调用）
 * @param dt_ms 时间间隔（毫秒）
 */
void fr_update_motion(int dt_ms);

#endif // FR_ROBOT_PROTOCOL_H
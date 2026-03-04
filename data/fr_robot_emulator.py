#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
法奥机器人 UDP 扩展轴模拟器

功能：
1. 模拟法奥机器人发送 UDP 控制命令 (154字节)
2. 接收 ESP32 作为 PLC 模拟器的响应 (160字节)
3. 提供控制台交互界面进行测试

协议说明：
- 机器人发送: 154字节控制命令包
- PLC响应: 160字节状态反馈包
- 端口: 8211
- 帧头: 0x5A5A
"""

import socket
import struct
import time
import threading
import json
from datetime import datetime
import sys


class FRAxisState:
    """单轴状态类"""
    def __init__(self):
        # 控制命令
        self.control_word = 0      # D200: 控制命令字
        self.target_position = 0   # D201: 目标位置
        self.home_command = 0      # D203: 回零命令字
        self.home_high_speed = 0   # D204: 回零高速度
        self.home_low_speed = 0    # D206: 回零低速度
        self.position_offset = 0   # D208: 位置偏置
        self.speed_offset = 0      # D210: 速度偏置
        self.torque_offset = 0     # D212: 转矩偏置

        # 状态反馈
        self.status_word = 0x0002  # D100: 状态字 (默认已使能)
        self.current_position = 0  # D101: 当前位置
        self.home_status = 0       # D103: 回零状态字
        self.home_high_spd_fb = 0  # D104: 寻零速度反馈
        self.home_low_spd_fb = 0   # D106: 爬行速度反馈
        self.fault_code = 0        # D108: 故障码
        self.follow_error = 0      # D109: 随动偏差量
        self.speed_feedback = 0    # D111: 速度反馈
        self.torque_feedback = 0   # D113: 实时转矩

        # 内部状态
        self.enabled = False
        self.homing = False
        self.home_done = False
        self.moving = False


class FRRobotEmulator:
    """法奥机器人模拟器"""

    def __init__(self, esp32_ip="192.168.1.100", port=8211):
        self.esp32_ip = esp32_ip
        self.port = port
        self.sock = None
        self.running = False
        self.frame_count = 0

        # 轴状态
        self.axes = [FRAxisState() for _ in range(4)]

        # 通信统计
        self.stats = {
            'sent_packets': 0,
            'received_packets': 0,
            'send_errors': 0,
            'recv_errors': 0
        }

        # 日志文件
        self.log_file = None

    def connect(self):
        """连接到ESP32"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.settimeout(0.1)  # 100ms超时
            self.running = True
            print(f"已连接到 ESP32: {self.esp32_ip}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        self.running = False
        if self.sock:
            self.sock.close()
        print("连接已断开")

    def pack_axis_cmd(self, axis_state):
        """打包单轴控制数据 (28字节)"""
        return struct.pack('<HiHiiiii',
                          axis_state.control_word,      # D200: 控制命令字
                          axis_state.target_position,   # D201: 目标位置
                          axis_state.home_command,      # D203: 回零命令字
                          axis_state.home_high_speed,   # D204: 回零高速度
                          axis_state.home_low_speed,    # D206: 回零低速度
                          axis_state.position_offset,   # D208: 位置偏置
                          axis_state.speed_offset,      # D210: 速度偏置
                          axis_state.torque_offset      # D212: 转矩偏置
                         )

    def pack_robot_packet(self):
        """打包机器人命令包 (154字节)"""
        # 帧头
        packet = struct.pack('<H', 0x5A5A)  # 帧头 0x5A5A

        # 各轴控制数据 (4轴 × 28字节 = 112字节)
        for axis in self.axes:
            packet += self.pack_axis_cmd(axis)

        # DO输出 (8 × 2字节 = 16字节)
        do_output = [0] * 8
        packet += struct.pack('<' + 'H' * 8, *do_output)

        # AO输出 (4 × 2字节 = 8字节)
        ao_output = [0] * 4
        packet += struct.pack('<' + 'h' * 4, *ao_output)

        # 保留区域 (6字节)
        packet += b'\x00' * 6

        # 帧计数 (2字节)
        self.frame_count += 1
        packet += struct.pack('<H', self.frame_count)

        # 保留/校验 (8字节)
        packet += b'\x00' * 8

        return packet

    def unpack_axis_status(self, data, offset):
        """解包单轴状态数据 (30字节)"""
        try:
            (
                status_word,        # D100: 状态字
                current_position,   # D101: 当前位置
                home_status,        # D103: 回零状态字
                home_high_spd_fb,   # D104: 寻零速度反馈
                home_low_spd_fb,    # D106: 爬行速度反馈
                fault_code,         # D108: 故障码
                follow_error,       # D109: 随动偏差量
                speed_feedback,     # D111: 速度反馈
                torque_feedback     # D113: 实时转矩
            ) = struct.unpack_from('<hihiihiii', data, offset)

            return {
                'status_word': status_word,
                'current_position': current_position,
                'home_status': home_status,
                'home_high_spd_fb': home_high_spd_fb,
                'home_low_spd_fb': home_low_spd_fb,
                'fault_code': fault_code,
                'follow_error': follow_error,
                'speed_feedback': speed_feedback,
                'torque_feedback': torque_feedback
            }
        except struct.error as e:
            print(f"解析轴状态数据失败: {e}")
            return None

    def parse_plc_packet(self, data):
        """解析PLC状态包 (160字节)"""
        if len(data) != 160:
            print(f"数据包长度错误: {len(data)}, 期望 160")
            return False

        # 验证帧头
        header = struct.unpack('<H', data[0:2])[0]
        if header != 0x5A5A:
            print(f"帧头错误: 0x{header:04X}, 期望 0x5A5A")
            return False

        # 解析各轴状态
        for i in range(4):
            offset = 2 + i * 30  # 跳过帧头和之前轴的数据
            status_data = self.unpack_axis_status(data, offset)
            if status_data:
                axis = self.axes[i]
                axis.status_word = status_data['status_word']
                axis.current_position = status_data['current_position']
                axis.home_status = status_data['home_status']
                axis.home_high_spd_fb = status_data['home_high_spd_fb']
                axis.home_low_spd_fb = status_data['home_low_spd_fb']
                axis.fault_code = status_data['fault_code']
                axis.follow_error = status_data['follow_error']
                axis.speed_feedback = status_data['speed_feedback']
                axis.torque_feedback = status_data['torque_feedback']

                # 更新内部状态标志
                axis.enabled = (status_data['status_word'] & 0x0002) != 0
                axis.homing = status_data['home_status'] == 1
                axis.home_done = status_data['home_status'] == 2
                axis.moving = abs(status_data['speed_feedback']) > 0

        # 解析帧计数 (偏移156)
        frame_count = struct.unpack_from('<H', data, 156)[0]
        # print(f"接收到响应包，帧计数: {frame_count}")

        return True

    def send_command(self):
        """发送控制命令"""
        try:
            packet = self.pack_robot_packet()
            if len(packet) != 154:
                print(f"错误: 数据包长度 {len(packet)}，期望 154")
                return False

            self.sock.sendto(packet, (self.esp32_ip, self.port))
            self.stats['sent_packets'] += 1
            return True
        except Exception as e:
            print(f"发送失败: {e}")
            self.stats['send_errors'] += 1
            return False

    def receive_response(self):
        """接收PLC响应"""
        try:
            data, addr = self.sock.recvfrom(1024)
            if addr[0] != self.esp32_ip:
                print(f"警告: 响应来自非预期IP: {addr[0]}")

            if self.parse_plc_packet(data):
                self.stats['received_packets'] += 1
                return True
            else:
                return False
        except socket.timeout:
            return False
        except Exception as e:
            print(f"接收失败: {e}")
            self.stats['recv_errors'] += 1
            return False

    def enable_axis(self, axis_id, enable=True):
        """使能/去使能轴"""
        if 0 <= axis_id < 4:
            old_state = self.axes[axis_id].enabled
            self.axes[axis_id].control_word = 0x0001 if enable else 0x0000
            self.axes[axis_id].enabled = enable
            if enable != old_state:
                print(f"轴 {axis_id+1} {'使能' if enable else '去使能'}")
        else:
            print(f"错误: 轴ID {axis_id} 超出范围")

    def set_target_position(self, axis_id, position):
        """设置目标位置"""
        if 0 <= axis_id < 4:
            self.axes[axis_id].target_position = position
            print(f"轴 {axis_id+1} 目标位置: {position}")
        else:
            print(f"错误: 轴ID {axis_id} 超出范围")

    def start_homing(self, axis_id, high_speed=100000, low_speed=10000):
        """启动回零"""
        if 0 <= axis_id < 4:
            self.axes[axis_id].home_command = 0x1001  # 回零启动命令
            self.axes[axis_id].home_high_speed = high_speed
            self.axes[axis_id].home_low_speed = low_speed
            self.axes[axis_id].homing = True
            print(f"轴 {axis_id+1} 开始回零 (高速: {high_speed}, 低速: {low_speed})")
        else:
            print(f"错误: 轴ID {axis_id} 超出范围")

    def print_status(self):
        """打印各轴状态"""
        print("\n=== 法奥机器人状态 ===")
        for i, axis in enumerate(self.axes):
            print(f"轴 {i+1}:")
            print(f"  使能: {axis.enabled}, 回零中: {axis.homing}, 回零完成: {axis.home_done}")
            print(f"  目标位置: {axis.target_position}, 当前位置: {axis.current_position}")
            print(f"  状态字: 0x{axis.status_word:04X}, 回零状态: {axis.home_status}")
            print(f"  速度反馈: {axis.speed_feedback}, 故障码: {axis.fault_code}")
        print(f"通信统计: 发送={self.stats['sent_packets']}, "
              f"接收={self.stats['received_packets']}, "
              f"错误={self.stats['send_errors'] + self.stats['recv_errors']}")
        print("==================\n")

    def run_test_cycle(self):
        """运行测试循环"""
        cycle_time = 0.002  # 2ms 周期

        while self.running:
            start_time = time.time()

            # 发送命令
            if not self.send_command():
                print("发送命令失败")

            # 接收响应
            self.receive_response()

            # 维持通信周期
            elapsed = time.time() - start_time
            if elapsed < cycle_time:
                time.sleep(cycle_time - elapsed)

    def start_communication(self):
        """启动通信线程"""
        comm_thread = threading.Thread(target=self.run_test_cycle, daemon=True)
        comm_thread.start()
        print("通信线程已启动 (2ms 周期)")

    def interactive_mode(self):
        """交互模式"""
        print("\n=== 法奥机器人模拟器 - 交互模式 ===")
        print("命令:")
        print("  e <轴ID> [0|1] - 使能/去使能轴 (1=使能, 0=去使能)")
        print("  p <轴ID> <位置> - 设置目标位置")
        print("  h <轴ID> [高速] [低速] - 启动回零")
        print("  s - 显示状态")
        print("  q - 退出")
        print("示例: 'e 0 1' 使能轴1, 'p 0 1000' 设置轴1目标位置为1000")
        print("==================================\n")

        while True:
            try:
                cmd = input(">>> ").strip().split()
                if not cmd:
                    continue

                if cmd[0].lower() == 'q':
                    break
                elif cmd[0].lower() == 's':
                    self.print_status()
                elif cmd[0].lower() == 'e':
                    if len(cmd) >= 2:
                        axis_id = int(cmd[1])
                        enable = bool(int(cmd[2])) if len(cmd) > 2 else True
                        self.enable_axis(axis_id, enable)
                    else:
                        print("语法: e <轴ID> [0|1]")
                elif cmd[0].lower() == 'p':
                    if len(cmd) >= 3:
                        axis_id = int(cmd[1])
                        position = int(cmd[2])
                        self.set_target_position(axis_id, position)
                    else:
                        print("语法: p <轴ID> <位置>")
                elif cmd[0].lower() == 'h':
                    if len(cmd) >= 2:
                        axis_id = int(cmd[1])
                        high_spd = int(cmd[2]) if len(cmd) > 2 else 100000
                        low_spd = int(cmd[3]) if len(cmd) > 3 else 10000
                        self.start_homing(axis_id, high_spd, low_spd)
                    else:
                        print("语法: h <轴ID> [高速] [低速]")
                else:
                    print(f"未知命令: {cmd[0]}")
            except (ValueError, IndexError):
                print("参数错误，请检查命令格式")
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"执行命令错误: {e}")


def main():
    """主函数"""
    print("法奥机器人 UDP 扩展轴模拟器")
    print("=" * 50)

    # 默认IP
    esp32_ip = "192.168.1.100"

    # 从命令行参数获取IP
    if len(sys.argv) > 1:
        esp32_ip = sys.argv[1]

    print(f"ESP32 IP: {esp32_ip}")

    # 创建模拟器
    robot = FRRobotEmulator(esp32_ip)

    # 连接ESP32
    if not robot.connect():
        return

    try:
        # 启动通信
        robot.start_communication()

        # 进入交互模式
        robot.interactive_mode()

    except KeyboardInterrupt:
        print("\n正在退出...")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
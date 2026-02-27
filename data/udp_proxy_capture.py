#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UDP 中间件/代理程序 - 法奥机器人与PLC通信抓包分析

功能：
1. 接收法奥机器人发送的UDP命令（端口8211）
2. 转发命令到汇川PLC
3. 接收PLC响应并转发回机器人
4. 记录所有通信数据到TXT文件（带时间戳和十六进制）

使用方法：
1. 修改PLC_IP和PLC_PORT为实际PLC地址
2. 运行: python udp_proxy_capture.py
3. 配置机器人连接本机IP:8211
4. 查看生成的日志文件 udp_capture_YYYYMMDD_HHMMSS.txt

作者：自动生成
日期：2024
"""

import socket
import threading
import time
import os
from datetime import datetime
from collections import deque
import struct
import sys

# ==================== 配置参数 ====================

# 本机监听配置（接收机器人数据）
LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 8211

# PLC目标配置（转发目的地）- 需要根据实际情况修改
PLC_IP = "192.168.58.88"      # TODO: 修改为实际PLC IP地址
PLC_PORT = 2021               # TODO: 修改为实际PLC端口

# 日志配置
LOG_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_MAX_SIZE = 50 * 1024 * 1024  # 50MB 单文件最大大小

# ==================== 数据包解析辅助函数 ====================

def parse_control_word(ctrl: int) -> str:
    """解析控制字"""
    bits = []
    if ctrl & 0x0001: bits.append("使能")
    if ctrl & 0x0002: bits.append("定位")
    if ctrl & 0x0004: bits.append("回零")
    if ctrl & 0x0008: bits.append("急停")
    if ctrl & 0x0010: bits.append("位置模式")
    if ctrl & 0x0020: bits.append("速度模式")
    if ctrl & 0x0040: bits.append("点动+")
    if ctrl & 0x0080: bits.append("点动-")
    return "+".join(bits) if bits else "无"

def parse_status_word(status: int) -> str:
    """解析状态字"""
    bits = []
    if status & 0x0001: bits.append("就绪/使能")
    if status & 0x0002: bits.append("已使能")
    if status & 0x0004: bits.append("运动中")
    if status & 0x0008: bits.append("到位")
    if status & 0x0010: bits.append("已回零")
    if status & 0x0020: bits.append("故障")
    if status & 0x0040: bits.append("正限位")
    if status & 0x0080: bits.append("负限位")
    return "+".join(bits) if bits else "无"

def parse_robot_packet(data: bytes) -> str:
    """解析机器人发送的数据包"""
    result = []
    result.append(f"  数据长度: {len(data)} 字节")

    # 检测协议头
    header_offset = 0
    if len(data) >= 2 and data[0:2] == b'\x5a\x5a':
        result.append(f"  协议头: 5A 5A (检测到)")
        header_offset = 2

    # 解析4个电机的控制数据
    for i in range(4):
        offset = header_offset + i * 28
        if offset + 28 > len(data):
            break

        try:
            # 每轴28字节: h(2)+i(4)+H(2)+i(4)+i(4)+i(4)+i(4)+i(4) = 28
            motor_data = struct.unpack_from('<hiHiiiiii', data, offset)
            ctrl = motor_data[0]
            target_pos = motor_data[1]
            home_ctrl = motor_data[2]
            home_high_spd = motor_data[3]
            home_low_spd = motor_data[4]
            pos_offset = motor_data[5]
            spd_offset = motor_data[6]
            torque_offset = motor_data[7]

            result.append(f"  --- 轴{i+1} ---")
            result.append(f"    控制字: 0x{ctrl:04X} [{parse_control_word(ctrl)}]")
            result.append(f"    目标位置: {target_pos}")
            result.append(f"    回零控制: 0x{home_ctrl:04X}")
            result.append(f"    回零高速: {home_high_spd}")
            result.append(f"    回零低速: {home_low_spd}")
            result.append(f"    位置偏置: {pos_offset}")
            result.append(f"    速度偏置: {spd_offset}")
            result.append(f"    转矩偏置: {torque_offset}")
        except Exception as e:
            result.append(f"  --- 轴{i+1} 解析失败: {e} ---")

    return "\n".join(result)

def parse_plc_packet(data: bytes) -> str:
    """解析PLC发送的数据包"""
    result = []
    result.append(f"  数据长度: {len(data)} 字节")

    # 检测是否有协议头
    header_offset = 0
    if len(data) >= 2 and data[0:2] == b'\x5a\x5a':
        result.append(f"  协议头: 5A 5A (检测到)")
        header_offset = 2

    # 解析4个电机的状态数据
    for i in range(4):
        offset = header_offset + i * 30
        if offset + 30 > len(data):
            break

        try:
            # 每轴30字节: h(2)+i(4)+h(2)+i(4)+i(4)+h(2)+i(4)+i(4)+i(4) = 30
            motor_data = struct.unpack_from('<hihiihiii', data, offset)
            status = motor_data[0]
            position = motor_data[1]
            home_status = motor_data[2]
            home_high_spd = motor_data[3]
            home_low_spd = motor_data[4]
            fault = motor_data[5]
            follow_err = motor_data[6]
            speed = motor_data[7]
            torque = motor_data[8]

            result.append(f"  --- 轴{i+1} ---")
            result.append(f"    状态字: 0x{status:04X} [{parse_status_word(status)}]")
            result.append(f"    当前位置: {position}")
            result.append(f"    回零状态: 0x{home_status:04X}")
            result.append(f"    回零高速反馈: {home_high_spd}")
            result.append(f"    回零低速反馈: {home_low_spd}")
            result.append(f"    故障码: {fault}")
            result.append(f"    随动偏差: {follow_err}")
            result.append(f"    速度反馈: {speed}")
            result.append(f"    转矩反馈: {torque}")
        except Exception as e:
            result.append(f"  --- 轴{i+1} 解析失败: {e} ---")

    return "\n".join(result)


# ==================== UDP代理类 ====================

class UDPProxyCapture:
    """UDP代理捕获器"""

    def __init__(self, listen_host: str, listen_port: int,
                 plc_ip: str, plc_port: int):
        self.listen_host = listen_host
        self.listen_port = listen_port
        self.plc_ip = plc_ip
        self.plc_port = plc_port

        # 套接字
        self.listen_sock = None
        self.plc_sock = None

        # 运行状态
        self.running = False

        # 机器人地址（动态获取）
        self.robot_addr = None

        # 统计信息
        self.rx_from_robot = 0
        self.tx_to_plc = 0
        self.rx_from_plc = 0
        self.tx_to_robot = 0
        self.start_time = 0
        self.last_rx_time = 0

        # 日志文件
        self.log_file = None
        self.log_filename = ""
        self.log_lock = threading.Lock()

        # 数据缓存（用于显示）
        self.last_robot_data = None
        self.last_plc_data = None
        self.data_history = deque(maxlen=100)

    def _create_log_file(self):
        """创建日志文件"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(LOG_DIR, f"udp_capture_{timestamp}.txt")
        self.log_file = open(self.log_filename, 'w', encoding='utf-8')

        # 写入头部信息
        header = f"""================================================================================
法奥机器人 <-> PLC UDP通信抓包日志
================================================================================
开始时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
监听地址: {self.listen_host}:{self.listen_port}
PLC地址:  {self.plc_ip}:{self.plc_port}
================================================================================

"""
        self.log_file.write(header)
        self.log_file.flush()
        print(f"日志文件: {self.log_filename}")

    def _log_data(self, direction: str, data: bytes, parsed: str = ""):
        """记录数据到日志"""
        with self.log_lock:
            if self.log_file is None:
                return

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

            # 格式化十六进制数据（每16字节一行）
            hex_lines = []
            for i in range(0, len(data), 16):
                chunk = data[i:i+16]
                hex_str = ' '.join(f'{b:02X}' for b in chunk)
                ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
                hex_lines.append(f"  {i:04X}: {hex_str:<48} {ascii_str}")

            log_entry = f"""
--------------------------------------------------------------------------------
[{timestamp}] {direction}
--------------------------------------------------------------------------------
长度: {len(data)} 字节
原始数据(HEX):
{chr(10).join(hex_lines)}

解析结果:
{parsed}
"""
            self.log_file.write(log_entry)
            self.log_file.flush()

            # 检查文件大小，如果太大则创建新文件
            if self.log_file.tell() > LOG_MAX_SIZE:
                self.log_file.close()
                self._create_log_file()

    def start(self):
        """启动代理服务器"""
        # 创建监听套接字（接收机器人数据）
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listen_sock.bind((self.listen_host, self.listen_port))
        self.listen_sock.settimeout(0.1)

        # 创建PLC通信套接字 - 绑定固定端口以便接收PLC响应
        self.plc_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.plc_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.plc_sock.bind(("0.0.0.0", 0))  # 绑定随机端口
        self.plc_sock.settimeout(0.1)  # 非阻塞接收

        # 创建日志文件
        self._create_log_file()

        self.running = True
        self.start_time = time.time()

        # 启动接收机器人数据线程
        self.rx_thread = threading.Thread(target=self._receive_robot_loop, daemon=True)
        self.rx_thread.start()

        # 启动接收PLC数据线程（独立线程，无超时限制）
        self.plc_rx_thread = threading.Thread(target=self._receive_plc_loop, daemon=True)
        self.plc_rx_thread.start()

        print("=" * 70)
        print("UDP 中间件/代理服务器已启动 (异步模式)")
        print("=" * 70)
        print(f"监听地址: {self.listen_host}:{self.listen_port} (接收机器人命令)")
        print(f"转发目标: {self.plc_ip}:{self.plc_port} (发送到PLC)")
        print(f"日志文件: {self.log_filename}")
        print("=" * 70)
        print("等待机器人连接...")
        print("-" * 70)

    def stop(self):
        """停止代理服务器"""
        self.running = False

        if self.listen_sock:
            self.listen_sock.close()
        if self.plc_sock:
            self.plc_sock.close()
        if self.log_file:
            self.log_file.write(f"\n\n日志结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.close()

    def _receive_robot_loop(self):
        """接收机器人数据循环 - 只负责接收机器人数据并转发到PLC"""
        while self.running:
            try:
                # 接收机器人数据
                data, addr = self.listen_sock.recvfrom(1024)
                self.robot_addr = addr
                self.rx_from_robot += 1
                self.last_rx_time = time.time()
                self.last_robot_data = data

                # 解析机器人数据
                parsed = parse_robot_packet(data)

                # 记录日志
                self._log_data(f"机器人 -> 代理 (来自 {addr[0]}:{addr[1]})", data, parsed)

                # 打印简要信息（每50个包打印一次）
                if self.rx_from_robot % 50 == 1:
                    print(f"\n[收] 机器人 -> 代理: {len(data)} 字节")
                    # 打印控制字
                    if len(data) >= 4:
                        offset = 2 if data[0:2] == b'\x5a\x5a' else 0
                        ctrl = struct.unpack_from('<h', data, offset)[0]
                        print(f"     控制字: 0x{ctrl:04X} [{parse_control_word(ctrl)}]")

                # 转发到PLC
                try:
                    self.plc_sock.sendto(data, (self.plc_ip, self.plc_port))
                    self.tx_to_plc += 1
                except Exception as e:
                    print(f"[!] 转发到PLC失败: {e}")

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[!] 接收机器人数据错误: {e}")

    def _receive_plc_loop(self):
        """接收PLC数据循环 - 独立线程，持续监听PLC响应并转发到机器人"""
        while self.running:
            try:
                # 接收PLC数据（无超时限制，持续监听）
                plc_data, plc_addr = self.plc_sock.recvfrom(1024)
                self.rx_from_plc += 1
                self.last_plc_data = plc_data

                # 解析PLC数据
                plc_parsed = parse_plc_packet(plc_data)

                # 记录日志
                self._log_data(f"PLC -> 代理 (来自 {plc_addr[0]}:{plc_addr[1]})",
                              plc_data, plc_parsed)

                # 打印简要信息
                if self.rx_from_plc % 50 == 1:
                    print(f"\n[收] PLC -> 代理: {len(plc_data)} 字节")
                    # 打印状态字
                    if len(plc_data) >= 4:
                        offset = 2 if plc_data[0:2] == b'\x5a\x5a' else 0
                        status = struct.unpack_from('<h', plc_data, offset)[0]
                        print(f"     状态字: 0x{status:04X} [{parse_status_word(status)}]")

                # 转发回机器人
                if self.robot_addr:
                    try:
                        self.listen_sock.sendto(plc_data, self.robot_addr)
                        self.tx_to_robot += 1
                        if self.rx_from_plc % 50 == 1:
                            print(f"[发] 代理 -> 机器人: {len(plc_data)} 字节 -> {self.robot_addr}")
                    except Exception as e:
                        print(f"[!] 转发到机器人失败: {e}")
                else:
                    if self.rx_from_plc % 50 == 1:
                        print(f"[!] 收到PLC数据但机器人未连接，无法转发")

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"[!] 接收PLC数据错误: {e}")

    def print_status(self):
        """打印状态"""
        uptime = time.time() - self.start_time if self.start_time > 0 else 0
        uptime_str = f"{int(uptime//3600):02d}:{int((uptime%3600)//60):02d}:{int(uptime%60):02d}"

        print("\n" + "=" * 70)
        print(f"UDP代理状态 - 运行时间: {uptime_str}")
        print("=" * 70)

        # 连接状态
        if self.robot_addr:
            elapsed = time.time() - self.last_rx_time
            status = "在线" if elapsed < 1.0 else f"离线({elapsed:.1f}s)"
            print(f"机器人: {self.robot_addr[0]}:{self.robot_addr[1]} [{status}]")
        else:
            print("机器人: 未连接")

        print(f"PLC目标: {self.plc_ip}:{self.plc_port}")

        print("-" * 70)
        print(f"统计: 机器人->代理={self.rx_from_robot}  代理->PLC={self.tx_to_plc}")
        print(f"      PLC->代理={self.rx_from_plc}  代理->机器人={self.tx_to_robot}")
        print("-" * 70)

        # 最近数据摘要
        if self.last_robot_data:
            print(f"最近机器人数据: {len(self.last_robot_data)} 字节")
            if len(self.last_robot_data) >= 4:
                offset = 2 if self.last_robot_data[0:2] == b'\x5a\x5a' else 0
                ctrl = struct.unpack_from('<h', self.last_robot_data, offset)[0]
                print(f"  控制字: 0x{ctrl:04X}")

        if self.last_plc_data:
            print(f"最近PLC数据: {len(self.last_plc_data)} 字节")
            if len(self.last_plc_data) >= 2:
                status = struct.unpack_from('<h', self.last_plc_data, 0)[0]
                print(f"  状态字: 0x{status:04X}")

        print("=" * 70)


def main():
    """主函数"""
    print("=" * 70)
    print("法奥机器人 <-> PLC UDP通信抓包工具")
    print("=" * 70)

    # 检查命令行参数
    plc_ip = PLC_IP
    plc_port = PLC_PORT

    if len(sys.argv) >= 2:
        plc_ip = sys.argv[1]
    if len(sys.argv) >= 3:
        try:
            plc_port = int(sys.argv[2])
        except ValueError:
            print(f"无效的端口号: {sys.argv[2]}")
            sys.exit(1)

    print(f"\n使用方法: python {sys.argv[0]} [PLC_IP] [PLC_PORT]")
    print(f"当前配置: PLC_IP={plc_ip}, PLC_PORT={plc_port}")
    print(f"\n请确保PLC地址配置正确！")
    print("-" * 70)

    # 确认配置
    print(f"\n即将启动代理服务器:")
    print(f"  监听端口: {LISTEN_PORT} (接收机器人命令)")
    print(f"  转发目标: {plc_ip}:{plc_port} (发送到PLC)")
    print(f"\n按 Enter 继续，或 Ctrl+C 取消...")

    try:
        input()
    except KeyboardInterrupt:
        print("\n已取消")
        sys.exit(0)

    # 创建代理
    proxy = UDPProxyCapture(LISTEN_HOST, LISTEN_PORT, plc_ip, plc_port)

    # 非阻塞键盘输入
    def check_keyboard():
        try:
            import msvcrt
            if msvcrt.kbhit():
                key = msvcrt.getch().decode('utf-8', errors='ignore')
                return key
        except ImportError:
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        return None

    try:
        proxy.start()

        print("\n按 'q' 退出, 's' 显示状态")
        print("-" * 70)

        while True:
            key = check_keyboard()
            if key:
                if key.lower() == 'q':
                    break
                elif key.lower() == 's':
                    proxy.print_status()

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n正在关闭...")
    finally:
        proxy.stop()
        print(f"\n日志已保存到: {proxy.log_filename}")
        print("代理服务器已关闭")


if __name__ == "__main__":
    main()

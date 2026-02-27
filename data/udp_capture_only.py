#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UDP 纯抓包程序 - 仅记录机器人发送的数据（不需要PLC在线）

功能：
1. 监听UDP端口8211，接收机器人发送的数据
2. 返回模拟的状态响应（保持通信）
3. 记录所有收发数据到TXT文件

适用场景：
- PLC不在线时，仅分析机器人发送的协议格式
- 对比模拟器响应与机器人期望

使用方法：
1. 运行: python udp_capture_only.py
2. 配置机器人连接本机IP:8211
3. 查看生成的日志文件
"""

import socket
import struct
import threading
import time
import os
from datetime import datetime
import sys

# ==================== 配置 ====================
LISTEN_HOST = "0.0.0.0"
LISTEN_PORT = 8211
LOG_DIR = os.path.dirname(os.path.abspath(__file__))

# ==================== 解析函数 ====================

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

def format_hex_dump(data: bytes) -> str:
    """格式化十六进制输出"""
    lines = []
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_str = ' '.join(f'{b:02X}' for b in chunk)
        ascii_str = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
        lines.append(f"  {i:04X}: {hex_str:<48} {ascii_str}")
    return "\n".join(lines)

def parse_robot_data(data: bytes) -> str:
    """详细解析机器人数据"""
    result = []
    result.append(f"数据长度: {len(data)} 字节")

    # 协议头检测
    header_offset = 0
    if len(data) >= 2 and data[0:2] == b'\x5a\x5a':
        result.append("协议头: 5A 5A")
        header_offset = 2

    # 解析4个电机
    for i in range(4):
        offset = header_offset + i * 28
        if offset + 28 > len(data):
            break

        try:
            motor_data = struct.unpack_from('<hiHiiiiii', data, offset)
            result.append(f"\n--- 轴{i+1} (偏移{offset}) ---")
            result.append(f"  控制字:   0x{motor_data[0]:04X} [{parse_control_word(motor_data[0])}]")
            result.append(f"  目标位置: {motor_data[1]}")
            result.append(f"  回零控制: 0x{motor_data[2]:04X}")
            result.append(f"  回零高速: {motor_data[3]}")
            result.append(f"  回零低速: {motor_data[4]}")
            result.append(f"  位置偏置: {motor_data[5]}")
            result.append(f"  速度偏置: {motor_data[6]}")
            result.append(f"  转矩偏置: {motor_data[7]}")
        except Exception as e:
            result.append(f"\n--- 轴{i+1} 解析失败: {e} ---")

    # DO解析
    do_offset = header_offset + 4 * 28  # 112字节后
    if do_offset + 16 <= len(data):
        result.append(f"\n--- DO输出 (偏移{do_offset}) ---")
        for i in range(8):
            do_val = struct.unpack_from('<H', data, do_offset + i * 2)[0]
            if do_val != 0:
                result.append(f"  DO[{i}]: 0x{do_val:04X}")

    # AO解析
    ao_offset = do_offset + 16
    if ao_offset + 8 <= len(data):
        result.append(f"\n--- AO输出 (偏移{ao_offset}) ---")
        for i in range(4):
            ao_val = struct.unpack_from('<h', data, ao_offset + i * 2)[0]
            if ao_val != 0:
                result.append(f"  AO[{i}]: {ao_val}")

    # 尾部数据
    tail_offset = ao_offset + 8
    if tail_offset < len(data):
        result.append(f"\n--- 尾部数据 (偏移{tail_offset}, {len(data)-tail_offset}字节) ---")
        result.append(f"  {data[tail_offset:].hex()}")

    return "\n".join(result)


class UDPCaptureOnly:
    """仅抓包模式"""

    def __init__(self):
        self.sock = None
        self.running = False
        self.robot_addr = None

        # 模拟状态
        self.motor_status = [0x0000] * 4  # 状态字
        self.motor_position = [0] * 4     # 位置

        # 统计
        self.rx_count = 0
        self.tx_count = 0
        self.start_time = 0

        # 日志
        self.log_file = None
        self.log_filename = ""
        self.log_lock = threading.Lock()

        # 上一次控制字（检测变化）
        self.prev_ctrl = [0] * 4

    def _create_log(self):
        """创建日志"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(LOG_DIR, f"robot_capture_{timestamp}.txt")
        self.log_file = open(self.log_filename, 'w', encoding='utf-8')

        header = f"""================================================================================
法奥机器人UDP命令抓包日志（模拟响应模式）
================================================================================
开始时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
监听端口: {LISTEN_PORT}
================================================================================

说明：
- 本程序仅用于分析机器人发送的命令格式
- 程序会返回简单的模拟响应以保持通信
- 所有数据都会记录到本文件

================================================================================

"""
        self.log_file.write(header)
        self.log_file.flush()

    def _log(self, direction: str, data: bytes, parsed: str = ""):
        """记录日志"""
        with self.log_lock:
            if not self.log_file:
                return

            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
            entry = f"""
--------------------------------------------------------------------------------
[{timestamp}] {direction} ({len(data)} 字节)
--------------------------------------------------------------------------------
原始数据(HEX):
{format_hex_dump(data)}

解析:
{parsed}
"""
            self.log_file.write(entry)
            self.log_file.flush()

    def _build_response(self) -> bytes:
        """构建模拟响应（144字节）"""
        data = bytearray()

        for i in range(4):
            # 30字节每轴: h+i+h+i+i+h+i+i+i
            motor_data = struct.pack('<hihiihiii',
                self.motor_status[i],   # 状态字
                self.motor_position[i], # 位置
                0,                      # 回零状态
                0,                      # 回零高速反馈
                0,                      # 回零低速反馈
                0,                      # 故障码
                0,                      # 随动误差
                0,                      # 速度
                0                       # 转矩
            )
            data.extend(motor_data)

        # DI (16字节)
        for _ in range(8):
            data.extend(struct.pack('<H', 0))

        # AI (8字节)
        for _ in range(4):
            data.extend(struct.pack('<h', 0))

        return bytes(data)

    def _receive_loop(self):
        """接收循环"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                self.robot_addr = addr
                self.rx_count += 1

                # 解析数据
                parsed = parse_robot_data(data)

                # 检测控制字变化
                header_offset = 2 if data[0:2] == b'\x5a\x5a' else 0
                for i in range(4):
                    offset = header_offset + i * 28
                    if offset + 2 <= len(data):
                        ctrl = struct.unpack_from('<h', data, offset)[0]

                        # 使能检测
                        if (ctrl & 0x0001) and not (self.prev_ctrl[i] & 0x0001):
                            self.motor_status[i] = 0x0001  # 使能响应
                            print(f"\n[!] 轴{i+1} 使能请求 -> 响应状态字 0x0001")
                        elif not (ctrl & 0x0001) and (self.prev_ctrl[i] & 0x0001):
                            self.motor_status[i] = 0x0000  # 去使能
                            print(f"\n[!] 轴{i+1} 去使能请求 -> 响应状态字 0x0000")

                        # 检测其他变化
                        if ctrl != self.prev_ctrl[i]:
                            print(f"\n[CMD] 轴{i+1} 控制字变化: 0x{self.prev_ctrl[i]:04X} -> 0x{ctrl:04X}")
                            print(f"      [{parse_control_word(ctrl)}]")

                        self.prev_ctrl[i] = ctrl

                # 记录日志
                self._log("机器人 -> 本机", data, parsed)

                # 发送响应
                response = self._build_response()
                self.sock.sendto(response, addr)
                self.tx_count += 1

                # 记录响应
                if self.rx_count % 50 == 1:
                    self._log("本机 -> 机器人 (模拟响应)", response,
                             f"状态字: {[f'0x{s:04X}' for s in self.motor_status]}")

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"错误: {e}")

    def start(self):
        """启动"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((LISTEN_HOST, LISTEN_PORT))
        self.sock.settimeout(0.1)

        self._create_log()

        self.running = True
        self.start_time = time.time()

        self.rx_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.rx_thread.start()

        print("=" * 70)
        print("UDP纯抓包模式（带模拟响应）")
        print("=" * 70)
        print(f"监听端口: {LISTEN_PORT}")
        print(f"日志文件: {self.log_filename}")
        print("=" * 70)
        print("等待机器人连接...")
        print("控制字变化会实时显示在此处")
        print("-" * 70)

    def stop(self):
        """停止"""
        self.running = False
        if self.sock:
            self.sock.close()
        if self.log_file:
            self.log_file.write(f"\n\n结束时间: {datetime.now()}\n")
            self.log_file.write(f"总接收: {self.rx_count}, 总发送: {self.tx_count}\n")
            self.log_file.close()

    def print_status(self):
        """打印状态"""
        uptime = time.time() - self.start_time if self.start_time else 0
        print(f"\n运行时间: {int(uptime)}s  收:{self.rx_count} 发:{self.tx_count}")
        print(f"状态字: {[f'0x{s:04X}' for s in self.motor_status]}")


def main():
    print("=" * 70)
    print("法奥机器人UDP命令抓包工具（纯记录模式）")
    print("=" * 70)
    print("本程序用于：")
    print("  1. 接收并记录机器人发送的所有UDP命令")
    print("  2. 返回简单的模拟响应保持通信")
    print("  3. 分析机器人协议格式")
    print("-" * 70)

    capture = UDPCaptureOnly()

    def check_keyboard():
        try:
            import msvcrt
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore')
        except ImportError:
            import select
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        return None

    try:
        capture.start()
        print("\n按 'q' 退出, 's' 显示状态")

        while True:
            key = check_keyboard()
            if key:
                if key.lower() == 'q':
                    break
                elif key.lower() == 's':
                    capture.print_status()
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        capture.stop()
        print(f"\n日志已保存: {capture.log_filename}")


if __name__ == "__main__":
    main()

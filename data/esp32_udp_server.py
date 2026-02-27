#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 扩展轴控制器 UDP 服务器模拟程序
基于实际抓包分析的法奥机器人UDP扩展轴协议

功能：
- 监听UDP端口8211，接收机器人控制指令
- 模拟4轴伺服电机控制（位置模式、回零模式）
- 模拟DI/DO和AI/AO
- 完全符合实际PLC协议数据包格式

数据包格式 (根据实际抓包分析):
- 接收: 154字节 (帧头2 + 4轴×28 + DO16 + AO8 + 帧尾10)
- 发送: 160字节 (帧头2 + 4轴×30 + DI16 + AI8 + 帧尾14)

协议特点:
- 帧头: 0x5A5A
- 控制字: 0x0001=使能
- 状态字: 0x0002=已使能, 0x0003=就绪+已使能
- 运动控制: 通过改变目标位置实现，控制字保持0x0001

使用方法：
1. 运行此脚本: python esp32_udp_server.py
2. 机器人端配置: ExtDevSetUDPComParam("本机IP", 8211, 2)
"""

import socket
import struct
import threading
import time
from dataclasses import dataclass
from typing import List
from datetime import datetime
import sys

# ==================== 协议常量定义 ====================

# 帧头
FRAME_HEADER = 0x5A5A

# 数据包长度 (根据实际抓包分析)
RX_PACKET_SIZE = 154  # 机器人→PLC (帧头2 + 4轴×28 + DO16 + AO8 + 帧尾10)
TX_PACKET_SIZE = 160  # PLC→机器人 (帧头2 + 4轴×30 + DI16 + AI8 + 帧尾14)

# 每轴数据长度
AXIS_CTRL_SIZE = 28   # 控制数据 28字节/轴
AXIS_STATUS_SIZE = 30 # 状态数据 30字节/轴

# 控制字位定义 (根据实际抓包分析)
CTRL_ENABLE      = 0x0001  # bit0: 使能 (0x0001)
# 注意: 实际协议中运动控制通过改变目标位置实现，无需额外控制位

# 回零控制字 (实际协议) - 根据抓包分析
# 抓包显示机器人发送的回零控制字为0x1001
# bit0 (0x0001): 回零使能/启动
# bit12 (0x1000): 回零模式选择
HOME_CTRL_ENABLE = 0x0001  # 回零启动位
HOME_CTRL_MODE   = 0x1000  # 回零模式位
HOME_CTRL_START  = 0x1001  # 回零控制 (0x1001 = 启动+模式)

# 状态字位定义 (根据抓包分析，实际协议简化)
# 抓包显示: 0x0002=已使能, 0x0003=就绪+已使能
STS_ENABLED      = 0x0002  # bit1: 已使能
STS_READY        = 0x0001  # bit0: 就绪
STS_READY_ENABLED = 0x0003 # bit0+bit1: 就绪+已使能 (运行中)

# 回零状态字位定义 (D103) - 根据抓包分析简化
HOME_BUSY        = 0x0001  # bit0: 回零中
HOME_DONE        = 0x0002  # bit1: 回零完成
HOME_ERROR       = 0x0004  # bit2: 回零错误


# ==================== 数据结构定义 ====================

@dataclass
class AxisControl:
    """
    单轴控制数据 (机器人→PLC, 28字节/轴)

    偏移  寄存器  类型   说明
    0-1   D200   INT    控制命令字
    2-5   D201   DINT   目标位置命令 (脉冲)
    6-7   D203   INT    回零命令字
    8-11  D204   DINT   回零高速度命令 (脉冲/s)
    12-15 D206   DINT   回零低速度命令 (脉冲/s)
    16-19 D208   DINT   位置偏置
    20-23 D210   DINT   速度偏置
    24-27 D212   DINT   转矩偏置
    """
    ctrl_word: int = 0         # 控制命令字
    target_pos: int = 0        # 目标位置 (脉冲)
    home_ctrl: int = 0         # 回零命令字
    home_high_speed: int = 0   # 回零高速度 (脉冲/s)
    home_low_speed: int = 0    # 回零低速度 (脉冲/s)
    pos_offset: int = 0        # 位置偏置
    spd_offset: int = 0        # 速度偏置
    torque_offset: int = 0     # 转矩偏置


@dataclass
class AxisStatus:
    """
    单轴状态数据 (PLC→机器人, 30字节/轴)

    根据实际抓包分析:
    偏移  寄存器  类型   说明
    0-1   D100   INT    状态字 (0x0002=已使能, 0x0003=就绪+已使能)
    2-5   D101   DINT   当前位置 (脉冲)
    6-7   D103   INT    回零状态字
    8-11  D104   DINT   寻零速度反馈 (脉冲/s)
    12-15 D106   DINT   爬行速度反馈 (脉冲/s)
    16-17 D108   INT    故障码
    18-21 D109   DINT   随动偏差量
    22-25 D111   DINT   速度反馈 (脉冲/s)
    26-29 D113   DINT   实时转矩
    """
    # 根据实际抓包分析：PLC始终响应0x0002(已使能)状态
    # 机器人期望PLC在上电后就处于就绪状态
    status_word: int = 0x0002     # 默认已使能（与实际PLC行为一致）
    current_pos: int = 0         # 当前位置 (脉冲)
    home_status: int = 0         # 回零状态字
    home_high_speed: int = 0     # 寻零速度反馈
    home_low_speed: int = 0      # 爬行速度反馈
    fault_code: int = 0          # 故障码
    follow_error: int = 0        # 随动偏差量
    speed_feedback: int = 0      # 速度反馈
    torque_feedback: int = 0     # 实时转矩


@dataclass
class AxisSimulator:
    """单轴运动模拟器"""
    # 运动参数
    actual_velocity: float = 0.0      # 实际速度 (脉冲/s)
    target_velocity: float = 0.0      # 目标速度
    max_velocity: float = 1000000.0   # 最大速度 (脉冲/s)
    acceleration: float = 5000000.0   # 加速度 (脉冲/s²)

    # 回零状态
    home_phase: int = 0               # 回零阶段 (0:未开始, 1:快速, 2:慢速, 3:完成)
    home_start_time: float = 0.0      # 回零开始时间
    is_homed: bool = False            # 是否已回零

    # 使能状态
    is_enabled: bool = False          # 使能状态
    enable_time: float = 0.0          # 使能时间

    # 运动状态
    is_moving: bool = False           # 运动中
    in_position: bool = False         # 到位

    # 上一周期控制字
    prev_ctrl: int = 0
    prev_home_ctrl: int = 0


# ==================== UDP服务器类 ====================

class ServoSimulatorUDP:
    """
    伺服驱动器UDP模拟器
    完全按照 法奥机器人UDP扩展轴协议 实现
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8211):
        self.host = host
        self.port = port
        self.sock = None
        self.running = False
        self.robot_addr = None

        # 4轴数据
        self.axis_ctrl: List[AxisControl] = [AxisControl() for _ in range(4)]
        self.axis_status: List[AxisStatus] = [AxisStatus() for _ in range(4)]
        self.axis_sim: List[AxisSimulator] = [AxisSimulator() for _ in range(4)]

        # IO数据
        self.di = [0] * 8   # 8×UINT16 = 128位DI
        self.do = [0] * 8   # 8×UINT16 = 128位DO
        self.ai = [0] * 4   # 4×INT16
        self.ao = [0] * 4   # 4×INT16

        # 帧计数
        self.rx_count = 0   # 接收帧计数 (D176)
        self.tx_count = 0   # 发送帧计数 (D177)

        # 时间戳
        self.start_time = 0
        self.last_rx_time = 0

        # 线程锁 - 保护共享数据
        self.data_lock = threading.Lock()

        # 日志文件
        self.log_file = None
        self.log_filename = ""
        self.log_lock = threading.Lock()

    def _create_log_file(self):
        """创建日志文件"""
        import os
        log_dir = os.path.dirname(os.path.abspath(__file__))
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_filename = os.path.join(log_dir, f"simulator_log_{timestamp}.txt")
        self.log_file = open(self.log_filename, 'w', encoding='utf-8')

        # 写入头部信息
        header = f"""================================================================================
ESP32 UDP伺服模拟器 通信日志
================================================================================
开始时间: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
监听地址: {self.host}:{self.port}
接收包长: {RX_PACKET_SIZE} 字节
发送包长: {TX_PACKET_SIZE} 字节
================================================================================

"""
        self.log_file.write(header)
        self.log_file.flush()
        print(f"日志文件: {self.log_filename}")

    def _log_packet(self, direction: str, data: bytes, addr: tuple = None):
        """记录数据包到日志文件"""
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

            # 解析数据
            parsed = self._parse_packet_for_log(data, direction)

            addr_str = f" (来自 {addr[0]}:{addr[1]})" if addr else ""
            log_entry = f"""
--------------------------------------------------------------------------------
[{timestamp}] {direction}{addr_str}
--------------------------------------------------------------------------------
长度: {len(data)} 字节
原始数据(HEX):
{chr(10).join(hex_lines)}

解析结果:
{parsed}
"""
            self.log_file.write(log_entry)
            self.log_file.flush()

    def _parse_packet_for_log(self, data: bytes, direction: str) -> str:
        """解析数据包用于日志"""
        result = []
        result.append(f"  数据长度: {len(data)} 字节")

        # 检测协议头
        header_offset = 0
        if len(data) >= 2 and data[0:2] == b'\x5a\x5a':
            result.append(f"  协议头: 5A 5A (检测到)")
            header_offset = 2

        if "机器人" in direction or "接收" in direction:
            # 解析机器人发送的控制数据
            for i in range(4):
                offset = header_offset + i * 28
                if offset + 28 > len(data):
                    break
                try:
                    motor_data = struct.unpack_from('<hiHiiiiii', data, offset)
                    ctrl = motor_data[0]
                    target_pos = motor_data[1]
                    home_ctrl = motor_data[2]

                    ctrl_bits = []
                    if ctrl & 0x0001: ctrl_bits.append("使能")
                    if ctrl & 0x0002: ctrl_bits.append("定位")
                    if ctrl & 0x0004: ctrl_bits.append("回零")
                    ctrl_str = "+".join(ctrl_bits) if ctrl_bits else "无"

                    result.append(f"  --- 轴{i+1} ---")
                    result.append(f"    控制字: 0x{ctrl:04X} [{ctrl_str}]")
                    result.append(f"    目标位置: {target_pos}")
                    result.append(f"    回零控制: 0x{home_ctrl:04X}")
                except Exception as e:
                    result.append(f"  --- 轴{i+1} 解析失败: {e} ---")
        else:
            # 解析模拟器发送的状态数据
            for i in range(4):
                offset = header_offset + i * 30
                if offset + 30 > len(data):
                    break
                try:
                    motor_data = struct.unpack_from('<hihiihiii', data, offset)
                    status = motor_data[0]
                    position = motor_data[1]
                    home_status = motor_data[2]
                    fault = motor_data[5]
                    speed = motor_data[7]

                    sts_bits = []
                    if status & 0x0001: sts_bits.append("就绪")
                    if status & 0x0002: sts_bits.append("已使能")
                    sts_str = "+".join(sts_bits) if sts_bits else "无"

                    result.append(f"  --- 轴{i+1} ---")
                    result.append(f"    状态字: 0x{status:04X} [{sts_str}]")
                    result.append(f"    当前位置: {position}")
                    result.append(f"    回零状态: 0x{home_status:04X}")
                    result.append(f"    故障码: {fault}")
                    result.append(f"    速度反馈: {speed}")
                except Exception as e:
                    result.append(f"  --- 轴{i+1} 解析失败: {e} ---")

        return "\n".join(result)

    def start(self):
        """启动服务器"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 增大接收缓冲区，避免丢包
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)  # 1MB
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024 * 1024)  # 1MB

        self.sock.bind((self.host, self.port))
        # 使用超时模式，与代理程序一致
        self.sock.settimeout(0.1)

        self.running = True
        self.start_time = time.time()

        # 创建日志文件
        self._create_log_file()

        # 启动线程
        threading.Thread(target=self._receive_loop, daemon=True).start()
        threading.Thread(target=self._motor_loop, daemon=True).start()
        threading.Thread(target=self._status_loop, daemon=True).start()

        self._print_header()

    def stop(self):
        """停止服务器"""
        self.running = False
        if self.sock:
            self.sock.close()
        if self.log_file:
            self.log_file.write(f"\n\n日志结束时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            self.log_file.close()

    def _print_header(self):
        """打印启动信息"""
        print("=" * 70)
        print("法奥机器人 UDP扩展轴伺服模拟器")
        print("=" * 70)
        print(f"监听地址: {self.host}:{self.port}")
        print(f"接收包长: {RX_PACKET_SIZE} 字节")
        print(f"发送包长: {TX_PACKET_SIZE} 字节")
        print("-" * 70)
        print(f'机器人配置: ExtDevSetUDPComParam("本机IP", {self.port}, 2)')
        print("=" * 70)
        print("等待机器人连接...")
        print()

    # ==================== 数据接收 ====================

    def _receive_loop(self):
        """接收数据循环 - 收到包立即响应"""
        while self.running:
            try:
                # 接收数据（使用超时模式）
                data, addr = self.sock.recvfrom(1024)
                self.robot_addr = addr
                self.rx_count += 1
                self.last_rx_time = time.time()

                # 首次收到数据时打印连接信息
                if self.rx_count == 1:
                    print(f"\n[连接] 机器人已连接: {addr[0]}:{addr[1]}")
                    print(f"       接收数据长度: {len(data)} 字节")

                # 记录接收数据到日志
                self._log_packet("机器人 -> 模拟器", data, addr)

                # 解析数据
                with self.data_lock:
                    self._parse_rx_packet(data)
                    # 立即构建响应
                    response = self._build_tx_packet()

                # 记录发送数据到日志
                self._log_packet("模拟器 -> 机器人", response)

                # 发送响应
                self.sock.sendto(response, addr)
                self.tx_count += 1

                # 每100个包打印一次状态
                if self.rx_count % 100 == 0:
                    print(f"[通信] RX={self.rx_count} TX={self.tx_count}")

            except socket.timeout:
                # 超时是正常的，继续循环
                continue
            except Exception as e:
                if self.running:
                    print(f"[错误] 接收异常: {e}")

    def _process_loop(self):
        """处理接收队列中的数据 - 已弃用"""
        pass

    def _status_loop(self):
        """状态打印循环"""
        while self.running:
            if self.tx_count > 0 and self.tx_count % 50 == 0:
                self._print_status()
            time.sleep(1)

    def _parse_rx_packet(self, data: bytes):
        """
        解析接收数据包 (136字节) - 根据官方文档

        结构 (无帧头):
        0-27:    轴1控制 (28字节)
        28-55:   轴2控制 (28字节)
        56-83:   轴3控制 (28字节)
        84-111:  轴4控制 (28字节)
        112-127: DO输出 (16字节, 8×UINT16)
        128-135: AO输出 (8字节, 4×INT16)
        总计: 136字节

        注：也兼容带帧头0x5A5A的格式
        """
        if len(data) < 2:
            return

        # 检查帧头 - 兼容两种格式
        offset = 0
        if data[0:2] == b'\x5A\x5A':
            offset = 2

        # 解析4轴控制数据
        for i in range(4):
            if offset + AXIS_CTRL_SIZE > len(data):
                break
            self._parse_axis_ctrl(i, data, offset)
            offset += AXIS_CTRL_SIZE

        # 解析DO (8×UINT16 = 16字节)
        if offset + 16 <= len(data):
            for i in range(8):
                self.do[i] = struct.unpack_from('<H', data, offset)[0]
                offset += 2

        # 解析AO (4×INT16 = 8字节)
        if offset + 8 <= len(data):
            for i in range(4):
                self.ao[i] = struct.unpack_from('<h', data, offset)[0]
                offset += 2

    def _parse_axis_ctrl(self, axis: int, data: bytes, offset: int):
        """解析单轴控制数据 (28字节)"""
        # 解包: h(2) + i(4) + H(2) + i(4) + i(4) + i(4) + i(4) + i(4) = 28
        values = struct.unpack_from('<hiHiiiiii', data, offset)

        ctrl = self.axis_ctrl[axis]
        sim = self.axis_sim[axis]

        # 保存上一周期控制字
        old_ctrl = ctrl.ctrl_word
        old_home = ctrl.home_ctrl

        # 更新控制数据
        ctrl.ctrl_word = values[0]
        ctrl.target_pos = values[1]
        ctrl.home_ctrl = values[2]
        ctrl.home_high_speed = values[3]
        ctrl.home_low_speed = values[4]
        ctrl.pos_offset = values[5]
        ctrl.spd_offset = values[6]
        ctrl.torque_offset = values[7]

        # 检测控制字变化
        self._process_ctrl_change(axis, old_ctrl, ctrl.ctrl_word, old_home, ctrl.home_ctrl)

        # 更新上一周期
        sim.prev_ctrl = ctrl.ctrl_word
        sim.prev_home_ctrl = ctrl.home_ctrl

    def _process_ctrl_change(self, axis: int, old_ctrl: int, new_ctrl: int,
                             old_home: int, new_home: int):
        """
        处理控制字变化 - 基于实际抓包分析的协议

        实际协议特点:
        - 控制字: 0x0001=使能，其他位未使用
        - 运动控制: 通过改变目标位置实现
        - 回零控制: 通过回零控制字0x0010触发
        - 状态字: 0x0002=已使能, 0x0003=就绪+已使能
        """
        ctrl = self.axis_ctrl[axis]
        status = self.axis_status[axis]
        sim = self.axis_sim[axis]

        # 打印任何控制字变化
        if old_ctrl != new_ctrl:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            print(f"[{timestamp}] 轴{axis+1} 控制字变化: 0x{old_ctrl:04X} -> 0x{new_ctrl:04X}")

        # 使能状态处理 - 根据实际抓包分析
        # 实际PLC行为：始终响应0x0002(已使能)，机器人发送使能命令后响应0x0003
        if new_ctrl & CTRL_ENABLE:
            if not sim.is_enabled:
                # 首次使能
                sim.is_enabled = True
                sim.enable_time = time.time()
                status.fault_code = 0
                self._log_event(axis, "使能", f"状态字=0x{STS_READY_ENABLED:04X}")
            # 使能后状态字: 0x0003 (就绪+已使能)
            status.status_word = STS_READY_ENABLED  # 0x0003
        else:
            # 未发送使能命令时 - 保持0x0002（已使能/就绪状态）
            # 根据实际抓包：PLC始终响应0x0002，表示伺服已就绪
            if sim.is_enabled:
                sim.is_enabled = False
                sim.actual_velocity = 0
                self._log_event(axis, "去使能", "")
            status.status_word = STS_ENABLED  # 0x0002 - 保持就绪状态

        # 检测回零启动 - 回零控制字为0x1001 (bit0=1表示启动)
        # 根据抓包分析：机器人发送回零控制字0x1001时启动回零
        # 不需要等待控制字使能，回零命令本身就包含使能
        home_rising = (new_home & HOME_CTRL_ENABLE) and not (old_home & HOME_CTRL_ENABLE)
        if home_rising:
            # 收到回零命令时自动使能该轴
            sim.is_enabled = True
            sim.home_phase = 1
            sim.home_start_time = time.time()
            sim.is_homed = False
            status.home_status = HOME_BUSY
            status.status_word = STS_READY_ENABLED  # 0x0003
            self._log_event(axis, "回零启动", f"回零控制=0x{new_home:04X} 高速={ctrl.home_high_speed}")

        # 位置跟踪模式 - 实际协议通过目标位置变化控制运动
        # 当目标位置变化时，自动跟踪到目标位置
        if sim.is_enabled and ctrl.target_pos != status.current_pos:
            sim.is_moving = True
            sim.in_position = False

    def _log_event(self, axis: int, event: str, detail: str):
        """打印事件日志"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        ctrl = self.axis_ctrl[axis]
        status = self.axis_status[axis]
        print(f"[{timestamp}] 轴{axis+1} {event} {detail}")
        print(f"           控制字=0x{ctrl.ctrl_word:04X} 回零控制=0x{ctrl.home_ctrl:04X}")
        print(f"           响应状态字=0x{status.status_word:04X} 回零状态=0x{status.home_status:04X}")

    # ==================== 电机模拟 ====================

    def _motor_loop(self):
        """电机运动模拟循环"""
        last_time = time.perf_counter()

        while self.running:
            current_time = time.perf_counter()
            dt = current_time - last_time
            last_time = current_time

            with self.data_lock:
                for i in range(4):
                    self._simulate_axis(i, dt)

            time.sleep(0.001)  # 1ms周期

    def _simulate_axis(self, axis: int, dt: float):
        """
        模拟单轴运动 - 基于实际协议

        实际协议特点:
        - 状态字: 0x0002=已使能(回零后), 0x0003=就绪+已使能(运行中)
        - 运动控制: 位置跟踪模式，当前位置追踪目标位置
        """
        ctrl = self.axis_ctrl[axis]
        status = self.axis_status[axis]
        sim = self.axis_sim[axis]

        # 未使能时 - 保持0x0002就绪状态
        if not sim.is_enabled:
            sim.actual_velocity *= 0.9
            if abs(sim.actual_velocity) < 1:
                sim.actual_velocity = 0
            status.speed_feedback = int(sim.actual_velocity)
            status.status_word = STS_ENABLED  # 0x0002 - 保持就绪状态
            return

        # 使能状态 - 默认状态字0x0003
        status.status_word = STS_READY_ENABLED  # 0x0003

        # 回零完成后状态字变为0x0002
        if sim.is_homed and sim.home_phase == 3:
            status.status_word = STS_ENABLED  # 0x0002

        # 回零处理
        if sim.home_phase > 0 and sim.home_phase < 3:
            self._simulate_homing(axis, dt)
            return

        # 位置跟踪模式 - 当前位置追踪目标位置
        target = ctrl.target_pos
        current = status.current_pos
        error = target - current

        # 到位判断
        if abs(error) <= 5:
            status.current_pos = target
            sim.actual_velocity = 0
            sim.is_moving = False
            sim.in_position = True
            status.speed_feedback = 0
            status.follow_error = 0
            return

        # 运动中
        sim.is_moving = True
        sim.in_position = False

        # 计算目标速度 (简化梯形曲线)
        max_vel = sim.max_velocity
        decel_dist = (sim.actual_velocity ** 2) / (2 * sim.acceleration) if sim.acceleration > 0 else 0

        if abs(error) <= decel_dist:
            # 减速阶段
            target_vel = error * 10 if abs(error) > 100 else (error / abs(error)) * 1000 if error != 0 else 0
        else:
            # 加速/匀速阶段
            target_vel = max_vel if error > 0 else -max_vel

        # 速度平滑
        self._smooth_velocity(sim, target_vel, dt)

        # 更新位置
        move = int(sim.actual_velocity * dt)
        if abs(move) > abs(error):
            move = error
        status.current_pos += move

        # 更新反馈
        status.speed_feedback = int(sim.actual_velocity)
        status.follow_error = int(error * 0.001)
        status.torque_feedback = int(abs(sim.actual_velocity) * 0.001)

    def _simulate_homing(self, axis: int, dt: float):
        """
        模拟回零过程 - 基于实际协议

        回零完成后状态字从0x0003变为0x0002
        """
        ctrl = self.axis_ctrl[axis]
        status = self.axis_status[axis]
        sim = self.axis_sim[axis]

        current_pos = status.current_pos
        high_speed = ctrl.home_high_speed if ctrl.home_high_speed > 0 else 100000
        low_speed = ctrl.home_low_speed if ctrl.home_low_speed > 0 else 10000

        # 回零中: 状态字0x0003
        status.status_word = STS_READY_ENABLED  # 0x0003
        status.home_status = HOME_BUSY

        # 回零速度反馈
        status.home_high_speed = high_speed
        status.home_low_speed = low_speed

        if sim.home_phase == 1:  # 高速阶段
            if abs(current_pos) > 1000:
                target_vel = -high_speed if current_pos > 0 else high_speed
            else:
                sim.home_phase = 2
                target_vel = -low_speed if current_pos > 0 else low_speed
        elif sim.home_phase == 2:  # 低速阶段
            if abs(current_pos) > 10:
                target_vel = -low_speed if current_pos > 0 else low_speed
            else:
                # 回零完成
                sim.home_phase = 3
                sim.is_homed = True
                sim.is_moving = False
                sim.in_position = True
                sim.actual_velocity = 0
                status.current_pos = 0
                status.home_status = HOME_DONE
                # 回零完成后状态字变为0x0002
                status.status_word = STS_ENABLED  # 0x0002

                duration = time.time() - sim.home_start_time
                self._log_event(axis, "回零完成", f"耗时={duration:.2f}s 状态字=0x{status.status_word:04X}")
                return
        else:
            target_vel = 0

        # 速度平滑
        self._smooth_velocity(sim, target_vel, dt)

        # 更新位置
        status.current_pos += int(sim.actual_velocity * dt)
        status.speed_feedback = int(sim.actual_velocity)

    def _smooth_velocity(self, sim: AxisSimulator, target: float, dt: float):
        """速度平滑处理"""
        diff = target - sim.actual_velocity
        max_delta = sim.acceleration * dt

        if abs(diff) > max_delta:
            sim.actual_velocity += max_delta if diff > 0 else -max_delta
        else:
            sim.actual_velocity = target

    # ==================== 数据发送 ====================

    def _send_loop(self):
        """发送数据循环 - 已弃用，改为同步响应"""
        pass

    def _build_tx_packet(self) -> bytes:
        """
        构建发送数据包 (160字节) - 根据实际抓包分析

        结构:
        0-1:      帧头 0x5A5A (2字节)
        2-31:     轴1状态 (30字节)
        32-61:    轴2状态 (30字节)
        62-91:    轴3状态 (30字节)
        92-121:   轴4状态 (30字节)
        122-137:  DI输入 (16字节, 8×UINT16)
        138-145:  AI输入 (8字节, 4×INT16)
        146-155:  保留区域 (10字节)
        156-157:  帧计数 (2字节, UINT16)
        158-159:  CRC16校验 (2字节)
        总计: 160字节
        """
        data = bytearray()

        # [0-1] 帧头 0x5A5A
        data.extend(struct.pack('<H', FRAME_HEADER))

        # [2-121] 4轴状态数据 (每轴30字节)
        for i in range(4):
            status = self.axis_status[i]
            # 打包: h(2) + i(4) + h(2) + i(4) + i(4) + h(2) + i(4) + i(4) + i(4) = 30字节
            axis_data = struct.pack('<hihiihiii',
                status.status_word,      # 状态字 (2字节)
                status.current_pos,      # 当前位置 (4字节)
                status.home_status,      # 回零状态 (2字节)
                status.home_high_speed,  # 寻零速度反馈 (4字节)
                status.home_low_speed,   # 爬行速度反馈 (4字节)
                status.fault_code,       # 故障码 (2字节)
                status.follow_error,     # 随动偏差 (4字节)
                status.speed_feedback,   # 速度反馈 (4字节)
                status.torque_feedback   # 转矩反馈 (4字节)
            )
            data.extend(axis_data)

        # [122-137] DI输入 (8×UINT16 = 16字节)
        for i in range(8):
            data.extend(struct.pack('<H', self.di[i]))

        # [138-145] AI输入 (4×INT16 = 8字节)
        for i in range(4):
            data.extend(struct.pack('<h', self.ai[i]))

        # [146-155] 保留区域 (10字节) - 全部填0
        data.extend(b'\x00' * 10)

        # [156-157] 帧计数 (2字节)
        data.extend(struct.pack('<H', self.tx_count & 0xFFFF))

        # [158-159] CRC16校验 (计算前158字节)
        crc = self._calc_crc16(bytes(data[:158]))
        data.extend(struct.pack('<H', crc & 0xFFFF))

        # 验证数据包长度
        assert len(data) == TX_PACKET_SIZE, f"数据包长度错误: {len(data)} != {TX_PACKET_SIZE}"

        return bytes(data)

    def _calc_crc16(self, data: bytes) -> int:
        """计算CRC16 (Modbus)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _print_status(self):
        """打印状态信息"""
        uptime = time.time() - self.start_time
        uptime_str = f"{int(uptime//3600):02d}:{int((uptime%3600)//60):02d}:{int(uptime%60):02d}"

        status = self.axis_status[0]
        ctrl = self.axis_ctrl[0]

        # 状态字解析 - 基于实际协议
        sts_bits = []
        if status.status_word & STS_READY:
            sts_bits.append("就绪")
        if status.status_word & STS_ENABLED:
            sts_bits.append("已使能")
        if status.home_status & HOME_DONE:
            sts_bits.append("已回零")
        sts_str = "+".join(sts_bits) if sts_bits else "无"

        print(f"\n[状态] 运行:{uptime_str} RX:{self.rx_count} TX:{self.tx_count}")
        print(f"  轴1: 控制字=0x{ctrl.ctrl_word:04X} 状态字=0x{status.status_word:04X}[{sts_str}]")
        print(f"       位置={status.current_pos} 速度={status.speed_feedback} 目标={ctrl.target_pos}")

    # ==================== IO操作 ====================

    def set_di(self, bit: int, value: bool):
        """设置DI位"""
        if 0 <= bit < 128:
            word = bit // 16
            pos = bit % 16
            if value:
                self.di[word] |= (1 << pos)
            else:
                self.di[word] &= ~(1 << pos)

    def set_ai(self, channel: int, value: int):
        """设置AI值"""
        if 0 <= channel < 4:
            self.ai[channel] = value


# ==================== 主函数 ====================

def main():
    """主函数"""
    port = 8211
    if len(sys.argv) > 1:
        try:
            port = int(sys.argv[1])
        except ValueError:
            print(f"无效端口号: {sys.argv[1]}")
            sys.exit(1)

    server = ServoSimulatorUDP(port=port)

    # 键盘检测
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
        server.start()

        # 初始化DI (模拟原点传感器)
        server.set_di(0, True)
        server.set_ai(0, 2048)

        print("\n按 'q' 退出")
        print("-" * 70)

        while True:
            key = check_keyboard()
            if key and key.lower() == 'q':
                break
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n正在关闭...")
    finally:
        server.stop()
        print("服务器已关闭")


if __name__ == "__main__":
    main()

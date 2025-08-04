import serial
import time
import sys


class TamagawaEncoderReader:
    def __init__(self, port, baudrate=2500000):
        self.port = port
        self.baudrate = baudrate
        self.resolution = 2 ** 23  # 23位分辨率 (8388608)
        self.ser = None

    def connect(self):
        """连接到串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1  # 100ms超时
            )
            print(f"已连接到 {self.ser.name}, 波特率 {self.baudrate}")
            print("开始读取23位绝对值编码器数据...")
            return True
        except serial.SerialException as e:
            print(f"连接失败: {str(e)}")
            return False

    def calculate_bcc(self, data):
        """计算BCC校验(异或校验)"""
        bcc = 0
        for byte in data:
            bcc ^= byte
        return bcc

    def parse_response(self, response):
        """解析编码器响应"""
        if len(response) != 6:
            raise ValueError(f"无效响应长度: {len(response)}字节 (应为6字节)")

        # 验证BCC校验
        calculated_bcc = self.calculate_bcc(response[:5])
        received_bcc = response[5]

        if calculated_bcc != received_bcc:
            raise ValueError(f"BCC校验失败: 计算值={hex(calculated_bcc)}, 接收值={hex(received_bcc)}")

        # 检查固定头字节
        if response[0] != 0x02:
            raise ValueError(f"无效头字节: {hex(response[0])} (应为0x02)")

        # 提取状态标志
        status_byte = response[1]
        counting_error = bool(status_byte & 0x10)  # Bit4: 计数错误标志

        # 提取23位位置值 (小端格式: D0, D1, D2)
        position = (response[4] << 16) | (response[3] << 8) | response[2]
        position &= 0x7FFFFF  # 确保只取23位

        return position, counting_error

    def read_position(self):
        """读取当前位置"""
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("串口未连接")

        # 发送读取命令
        self.ser.write(bytes([0x02]))

        # 读取响应 (6字节)
        response = self.ser.read(6)

        if len(response) == 0:
            raise TimeoutError("未收到响应")

        position, counting_error = self.parse_response(response)

        # 计算角度
        angle = position * 360.0 / self.resolution

        return position, angle, counting_error

    def run(self):
        """主运行循环"""
        if not self.connect():
            return

        try:
            while True:
                try:
                    position, angle, counting_error = self.read_position()

                    # 格式化输出
                    error_flag = " [计数错误!]" if counting_error else ""
                    print(f"位置: {position} (0x{position:06X}) | 角度: {angle:.6f}°{error_flag}")

                    # 控制读取频率
                    time.sleep(0.05)  # 约20Hz采样率

                except Exception as e:
                    print(f"错误: {str(e)}")
                    time.sleep(0.5)  # 出错后稍作等待

        except KeyboardInterrupt:
            print("\n程序已终止")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()


if __name__ == "__main__":
    # 配置参数
    PORT = 'COM21'  # 更改为您的串口号

    # 创建并运行读取器
    reader = TamagawaEncoderReader(PORT)
    reader.run()
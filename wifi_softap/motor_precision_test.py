import requests
import time
import csv
from datetime import datetime
from 读多摩川编码器 import TamagawaEncoderReader
import threading
import logging
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import rcParams
import numpy as np

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

class MotorController:
    """电机控制器 - 通过WiFi HTTP接口控制ESP32"""
    
    def __init__(self, esp32_ip="192.168.4.1", timeout=10):
        self.base_url = f"http://{esp32_ip}"
        self.timeout = timeout
        self.session = requests.Session()
        
    def _send_request(self, endpoint, description=""):
        """发送HTTP请求到ESP32"""
        try:
            url = f"{self.base_url}{endpoint}"
            response = self.session.get(url, timeout=self.timeout)
            response.raise_for_status()
            logger.info(f"{description} - 响应: {response.text}")
            return True, response.text
        except requests.RequestException as e:
            logger.error(f"{description} - 失败: {str(e)}")
            return False, str(e)
    
    def enable_motor(self):
        """使能电机"""
        return self._send_request("/enable", "使能电机")
    
    def disable_motor(self):
        """失能电机"""
        return self._send_request("/disable", "失能电机")
    
    def set_position(self, position):
        """设置电机位置"""
        return self._send_request(f"/set_position?value={position}", f"设置位置到 {position}")
    
    def clear_errors(self):
        """清除电机错误"""
        return self._send_request("/clear", "清除电机错误")

class EncoderStabilityDetector:
    """编码器稳定性检测器"""
    
    def __init__(self, stability_samples=5, max_wait_time=30):
        self.stability_samples = stability_samples  # 连续稳定样本数
        self.max_wait_time = max_wait_time  # 最大等待时间(秒)
        
    def wait_for_stability(self, encoder_reader):
        """
        等待编码器角度小数点后3位稳定
        返回: (是否稳定, 稳定角度值)
        """
        logger.info("等待编码器角度稳定...")
        start_time = time.time()
        stable_readings = []
        
        while time.time() - start_time < self.max_wait_time:
            try:
                position, angle, counting_error = encoder_reader.read_position()
                if counting_error:
                    logger.warning("编码器计数错误，继续等待...")
                    time.sleep(0.1)
                    continue
                
                # 取小数点后三位
                angle_3decimal = round(angle, 3)
                stable_readings.append(angle_3decimal)
                
                # 保持最近的stability_samples个读数
                if len(stable_readings) > self.stability_samples:
                    stable_readings.pop(0)
                
                # 检查是否稳定（所有读数相同）
                if len(stable_readings) == self.stability_samples:
                    if all(reading == stable_readings[0] for reading in stable_readings):
                        logger.info(f"编码器角度稳定: {angle_3decimal:.3f}°")
                        return True, angle_3decimal
                    else:
                        # 清空不稳定的读数，重新开始计数
                        stable_readings.clear()
                
                time.sleep(0.05)  # 20Hz采样
                
            except Exception as e:
                logger.error(f"读取编码器数据失败: {str(e)}")
                time.sleep(0.1)
        
        logger.warning(f"等待{self.max_wait_time}秒后编码器仍未稳定")
        return False, None

class MotorPrecisionTester:
    """电机精度测试器"""
    
    def __init__(self, encoder_port='COM43', esp32_ip="192.168.4.1", reduction_ratio=19.24, enable_plot=False):
        # 初始化组件
        self.motor_controller = MotorController(esp32_ip)
        self.encoder_reader = TamagawaEncoderReader(encoder_port)
        self.stability_detector = EncoderStabilityDetector()
        
        # 测试配置
        self.test_positions = [32 + x * 0.8 for x in range(int((176-32)/0.8) + 1)]  # 32到176，间隔0.8
        self.position_stable_time = 3.0  # 位置稳定等待时间
        
        # 减速比配置
        self.reduction_ratio = reduction_ratio
        self.theoretical_step = (360 / reduction_ratio / 8) * 0.8  # 理论步长
        
        # 数据存储
        self.test_results = []
        self.actual_angles = []  # 用于动态绘图的实际角度数据
        self.angle_errors = []   # 用于动态绘图的误差数据
        
        # 动态绘图配置
        self.enable_plot = enable_plot
        self.fig = None
        self.ax = None
        self.line = None
        self.plot_thread = None
        
    def calculate_theoretical_angle(self, position):
        """计算理论角度"""
        return position * (360 / self.reduction_ratio) / 8.0
    
    def setup_dynamic_plot(self):
        """设置动态绘图"""
        if not self.enable_plot:
            return
        
        plt.ion()  # 开启交互模式
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.ax.set_xlabel('实际角度 (°)')
        self.ax.set_ylabel('角度变化误差 (°)')
        self.ax.set_title(f'实时角度变化误差 (减速比: {self.reduction_ratio}, 理论步长: {self.theoretical_step:.5f}°)')
        self.ax.grid(True, alpha=0.3)
        self.ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        # 初始化空线条
        self.line, = self.ax.plot([], [], 'b-', linewidth=2)
        
        plt.show()
        logger.info("动态绘图已启动")
    
    def update_plot(self):
        """更新动态绘图"""
        if not self.enable_plot or len(self.actual_angles) < 2:
            return
        
        try:
            # 更新线条数据
            self.line.set_data(self.actual_angles[1:], self.angle_errors)
            
            # 自动调整坐标轴范围
            if len(self.actual_angles) > 1:
                self.ax.relim()
                self.ax.autoscale_view()
            
            # 刷新图表
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            logger.error(f"更新绘图失败: {str(e)}")
    
    def test_single_position(self, position):
        """测试单个位置的精度"""
        logger.info(f"\n=== 测试位置: {position} ===")
        
        # 1. 使能电机
        success, msg = self.motor_controller.enable_motor()
        if not success:
            logger.error(f"使能电机失败: {msg}")
            return None
        
        time.sleep(0.5)  # 等待使能完成
        
        # 2. 移动到目标位置
        success, msg = self.motor_controller.set_position(position)
        if not success:
            logger.error(f"设置位置失败: {msg}")
            return None
        
        # 3. 等待位置稳定
        logger.info(f"等待{self.position_stable_time}秒位置稳定...")
        time.sleep(self.position_stable_time)
        
        # 4. 失能电机
        success, msg = self.motor_controller.disable_motor()
        if not success:
            logger.error(f"失能电机失败: {msg}")
        
        time.sleep(0.5)  # 等待失能完成
        
        # 5. 等待编码器稳定并记录
        is_stable, stable_angle = self.stability_detector.wait_for_stability(self.encoder_reader)
        
        if is_stable:
            # 计算角度变化误差（除了第一个点）
            angle_error = 0
            if len(self.actual_angles) > 0:  # 不是第一个点
                actual_change = stable_angle - self.actual_angles[-1]
                angle_error = actual_change - self.theoretical_step
                self.angle_errors.append(angle_error)
            
            # 添加到动态绘图数据
            self.actual_angles.append(stable_angle)
            
            # 更新动态绘图
            self.update_plot()
            
            result = {
                'position': position,
                'actual_angle': stable_angle
            }
            
            if len(self.actual_angles) > 1:
                logger.info(f"位置 {position}: 实际角度={stable_angle:.3f}°, 误差={angle_error:.3f}°")
            else:
                logger.info(f"位置 {position}: 实际角度={stable_angle:.3f}° (起始点)")
            return result
        else:
            logger.error(f"位置 {position}: 编码器角度未稳定")
            return None
    
    def run_test(self):
        """运行完整的精度测试"""
        logger.info("=== 开始电机精度测试 ===")
        logger.info(f"减速比: {self.reduction_ratio}, 理论步长: {self.theoretical_step:.5f}°")
        logger.info(f"测试位置点: {len(self.test_positions)}个点")
        
        # 设置动态绘图
        self.setup_dynamic_plot()
        
        # 连接编码器
        if not self.encoder_reader.connect():
            logger.error("连接编码器失败")
            return False
        
        # 清除电机错误
        self.motor_controller.clear_errors()
        time.sleep(1)
        
        try:
            # 逐个测试每个位置
            for i, position in enumerate(self.test_positions):
                logger.info(f"\n进度: {i+1}/{len(self.test_positions)}")
                
                result = self.test_single_position(position)
                if result:
                    self.test_results.append(result)
                else:
                    logger.warning(f"位置 {position} 测试失败，跳过")
                
                # 测试间隔
                if i < len(self.test_positions) - 1:
                    time.sleep(2)
            
            # 保存测试结果
            self.save_results()
            self.print_summary()
            
            logger.info("=== 精度测试完成 ===")
            return True
            
        except KeyboardInterrupt:
            logger.info("测试被用户中断")
            return False
        except Exception as e:
            logger.error(f"测试过程中发生错误: {str(e)}")
            return False
        finally:
            # 确保电机失能
            self.motor_controller.disable_motor()
            # 关闭编码器连接
            if self.encoder_reader.ser and self.encoder_reader.ser.is_open:
                self.encoder_reader.ser.close()
            # 保持绘图窗口打开
            if self.enable_plot:
                plt.ioff()  # 关闭交互模式
                plt.show()  # 保持窗口显示
    
    def save_results(self):
        """保存测试结果到CSV文件"""
        if not self.test_results:
            logger.warning("没有测试结果需要保存")
            return
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"test_results_{timestamp}.csv"
        
        with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = ['position', 'actual_angle']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            writer.writeheader()
            writer.writerows(self.test_results)
        
        logger.info(f"测试结果已保存到: {filename}")
    
    def print_summary(self):
        """打印测试结果摘要"""
        if not self.test_results:
            return
        
        logger.info("\n=== 测试结果摘要 ===")
        logger.info(f"测试点数: {len(self.test_results)}")
        logger.info("位置 -> 实际角度:")
        for result in self.test_results:
            logger.info(f"  {result['position']} -> {result['actual_angle']:.3f}°")

def main():
    """主函数"""
    print("电机精度测试程序")
    print("确保ESP32热点已启动，编码器已连接")
    
    # 配置参数
    encoder_port = input("请输入编码器串口号 (默认: COM43): ").strip() or 'COM43'
    esp32_ip = input("请输入ESP32 IP地址 (默认: 192.168.4.1): ").strip() or '192.168.4.1'
    
    # 输入减速比
    reduction_ratio_input = input("请输入外部减速比 (默认: 19.24): ").strip()
    if reduction_ratio_input:
        try:
            reduction_ratio = float(reduction_ratio_input)
        except ValueError:
            print("输入无效，使用默认值19.24")
            reduction_ratio = 19.24
    else:
        reduction_ratio = 19.24
    
    # 是否启用动态绘图
    plot_choice = input("是否启用实时动态绘图? (y/n, 默认: y): ").strip().lower()
    enable_plot = plot_choice != 'n'
    
    print(f"使用减速比: {reduction_ratio}")
    theoretical_step = (360 / reduction_ratio / 8) * 0.8
    print(f"理论步长: {theoretical_step:.5f}°")
    print(f"动态绘图: {'启用' if enable_plot else '禁用'}")
    
    # 创建测试器
    tester = MotorPrecisionTester(encoder_port, esp32_ip, reduction_ratio, enable_plot)
    
    # 开始测试
    input("按回车键开始测试...")
    tester.run_test()

if __name__ == "__main__":
    main()
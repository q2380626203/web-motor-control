import tkinter as tk
from tkinter import ttk, scrolledtext, filedialog, messagebox
import threading
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
from datetime import datetime
import csv
import logging
import time
import os
import re
from motor_precision_test import MotorPrecisionTester, MotorController, TamagawaEncoderReader
import json

class MotorPrecisionGUI:
    """电机精度测试GUI界面"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("WiFi电机精度测试系统 v1.0")
        self.root.geometry("1200x800")
        self.root.configure(bg='#f0f0f0')
        
        # 测试器实例
        self.tester = None
        self.test_thread = None
        self.is_testing = False
        
        # 连接管理
        self.motor_controller = None
        self.encoder_reader = None
        self.esp32_connected = False
        self.encoder_connected = False
        self.real_time_thread = None
        self.real_time_running = False
        
        # 数据存储
        self.test_results = []
        self.real_time_data = {'positions': [], 'angles': [], 'errors': [], 'cumulative_errors': []}
        self.start_angle = None  # 起始角度
        
        # 实时统计数据
        self.previous_angle = None  # 用于计算实际步长
        self.error_count = 0  # 误差>0.01次数统计
        
        # 创建界面
        self.create_widgets()
        self.load_default_settings()
        
        # 配置日志
        self.setup_logging()
        
    def create_widgets(self):
        """创建界面组件"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(2, weight=1)
        
        # 左侧控制面板
        self.create_control_panel(main_frame)
        
        # 右侧绘图和数据显示区域
        self.create_display_panel(main_frame)
        
    def create_control_panel(self, parent):
        """创建左侧控制面板"""
        control_frame = ttk.LabelFrame(parent, text="控制面板", padding="10")
        control_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 5))
        
        # 连接设置
        conn_frame = ttk.LabelFrame(control_frame, text="连接设置", padding="5")
        conn_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # ESP32 IP
        ttk.Label(conn_frame, text="ESP32 IP:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.esp32_ip = tk.StringVar(value="192.168.4.1")
        ttk.Entry(conn_frame, textvariable=self.esp32_ip, width=15).grid(row=0, column=1, padx=(5, 0), pady=2)
        
        # 编码器串口
        ttk.Label(conn_frame, text="编码器串口:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.encoder_port = tk.StringVar(value="COM43")
        ttk.Entry(conn_frame, textvariable=self.encoder_port, width=15).grid(row=1, column=1, padx=(5, 0), pady=2)
        
        # 连接状态显示
        status_frame = ttk.Frame(conn_frame)
        status_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(10, 5))
        
        # ESP32连接状态
        ttk.Label(status_frame, text="ESP32状态:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.esp32_status = tk.StringVar(value="未连接")
        self.esp32_status_label = ttk.Label(status_frame, textvariable=self.esp32_status, foreground="red")
        self.esp32_status_label.grid(row=0, column=1, sticky=tk.W, padx=(0, 15))
        
        # 编码器连接状态
        ttk.Label(status_frame, text="编码器状态:").grid(row=0, column=2, sticky=tk.W, padx=(0, 5))
        self.encoder_status = tk.StringVar(value="未连接")
        self.encoder_status_label = ttk.Label(status_frame, textvariable=self.encoder_status, foreground="red")
        self.encoder_status_label.grid(row=0, column=3, sticky=tk.W)
        
        # 连接控制按钮
        button_frame = ttk.Frame(conn_frame)
        button_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(5, 0))
        
        self.connect_button = ttk.Button(button_frame, text="连接", command=self.connect_devices)
        self.connect_button.grid(row=0, column=0, padx=(0, 5), pady=2)
        
        self.disconnect_button = ttk.Button(button_frame, text="断开", command=self.disconnect_devices, state=tk.DISABLED)
        self.disconnect_button.grid(row=0, column=1, pady=2)
        
        # 测试参数
        param_frame = ttk.LabelFrame(control_frame, text="测试参数", padding="5")
        param_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 外部减速比
        ttk.Label(param_frame, text="外部减速比:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.reduction_ratio = tk.StringVar(value="19.24")
        ttk.Entry(param_frame, textvariable=self.reduction_ratio, width=15).grid(row=0, column=1, padx=(5, 0), pady=2)
        
        # 内部电机参数
        ttk.Label(param_frame, text="内部电机参数:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.internal_motor_param = tk.StringVar(value="8")
        ttk.Entry(param_frame, textvariable=self.internal_motor_param, width=15).grid(row=1, column=1, padx=(5, 0), pady=2)
        
        # 用户说明文本
        ttk.Label(param_frame, text="用户说明文本:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.user_description = tk.StringVar(value="测试")
        ttk.Entry(param_frame, textvariable=self.user_description, width=15).grid(row=2, column=1, padx=(5, 0), pady=2)
        
        # 起始位置
        ttk.Label(param_frame, text="起始位置:").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.start_pos = tk.StringVar(value="32")
        ttk.Entry(param_frame, textvariable=self.start_pos, width=15).grid(row=3, column=1, padx=(5, 0), pady=2)
        
        # 目标角度范围
        ttk.Label(param_frame, text="目标角度范围:").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.target_angle_range = tk.StringVar(value="360")
        ttk.Entry(param_frame, textvariable=self.target_angle_range, width=15).grid(row=4, column=1, padx=(5, 0), pady=2)
        
        # 步长间隔
        ttk.Label(param_frame, text="步长间隔:").grid(row=5, column=0, sticky=tk.W, pady=2)
        self.step_interval = tk.StringVar(value="0.8")
        ttk.Entry(param_frame, textvariable=self.step_interval, width=15).grid(row=5, column=1, padx=(5, 0), pady=2)
        
        # 控制按钮
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=2, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.start_button = ttk.Button(button_frame, text="开始测试", command=self.start_test, state=tk.DISABLED)
        self.start_button.grid(row=0, column=0, padx=(0, 5), pady=2)
        
        self.stop_button = ttk.Button(button_frame, text="停止测试", command=self.stop_test, state=tk.DISABLED)
        self.stop_button.grid(row=0, column=1, padx=(0, 5), pady=2)
        
        self.save_button = ttk.Button(button_frame, text="保存数据", command=self.save_data)
        self.save_button.grid(row=1, column=0, padx=(0, 5), pady=2)
        
        self.load_button = ttk.Button(button_frame, text="加载数据", command=self.load_data)
        self.load_button.grid(row=1, column=1, padx=(0, 5), pady=2)
        
        self.generate_report_button = ttk.Button(button_frame, text="生成报告", command=self.generate_test_report)
        self.generate_report_button.grid(row=2, column=0, columnspan=2, pady=(5, 2))
        
        # 电机控制按钮
        motor_frame = ttk.LabelFrame(control_frame, text="电机控制", padding="5")
        motor_frame.grid(row=3, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # 第一行：使能/失能按钮
        self.enable_button = ttk.Button(motor_frame, text="使能电机", command=self.enable_motor, state=tk.DISABLED)
        self.enable_button.grid(row=0, column=0, padx=(0, 5), pady=2)
        
        self.disable_button = ttk.Button(motor_frame, text="失能电机", command=self.disable_motor, state=tk.DISABLED)
        self.disable_button.grid(row=0, column=1, pady=2)
        
        # 第二行：指定位置输入框和标签
        ttk.Label(motor_frame, text="指定位置:").grid(row=1, column=0, sticky=tk.W, pady=(5, 2))
        self.target_position = tk.StringVar(value="32")
        position_entry = ttk.Entry(motor_frame, textvariable=self.target_position, width=10)
        position_entry.grid(row=1, column=1, padx=(5, 0), pady=(5, 2))
        
        # 第三行：移动到位置和清除错误按钮
        self.move_to_position_button = ttk.Button(motor_frame, text="移动到位置", command=self.move_to_position, state=tk.DISABLED)
        self.move_to_position_button.grid(row=2, column=0, padx=(0, 5), pady=2)
        
        self.clear_errors_button = ttk.Button(motor_frame, text="清除错误", command=self.clear_motor_errors, state=tk.DISABLED)
        self.clear_errors_button.grid(row=2, column=1, pady=2)
        
    def create_display_panel(self, parent):
        """创建右侧显示面板"""
        display_frame = ttk.Frame(parent)
        display_frame.grid(row=0, column=1, rowspan=3, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(5, 0))
        display_frame.columnconfigure(0, weight=1)
        display_frame.rowconfigure(0, weight=0)  # 测试参数
        display_frame.rowconfigure(1, weight=0)  # 实时数据
        display_frame.rowconfigure(2, weight=2)  # 绘图区域
        display_frame.rowconfigure(3, weight=1)  # 日志区域
        
        # 测试参数显示
        param_display_frame = ttk.LabelFrame(display_frame, text="测试参数", padding="5")
        param_display_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # 参数显示 - 使用网格布局
        param_grid = ttk.Frame(param_display_frame)
        param_grid.pack(fill=tk.X)
        
        ttk.Label(param_grid, text="减速比:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.param_reduction = tk.StringVar(value="--")
        ttk.Label(param_grid, textvariable=self.param_reduction, foreground="blue").grid(row=0, column=1, sticky=tk.W, padx=(0, 15))
        
        ttk.Label(param_grid, text="理论步长:").grid(row=0, column=2, sticky=tk.W, padx=(0, 5))
        self.param_step = tk.StringVar(value="--")
        ttk.Label(param_grid, textvariable=self.param_step, foreground="blue").grid(row=0, column=3, sticky=tk.W, padx=(0, 15))
        
        ttk.Label(param_grid, text="测试范围:").grid(row=0, column=4, sticky=tk.W, padx=(0, 5))
        self.param_range = tk.StringVar(value="--")
        ttk.Label(param_grid, textvariable=self.param_range, foreground="blue").grid(row=0, column=5, sticky=tk.W)
        
        # 实时数据显示
        data_display_frame = ttk.LabelFrame(display_frame, text="实时数据", padding="5")
        data_display_frame.grid(row=1, column=0, sticky=(tk.W, tk.E), pady=(0, 5))
        
        # 实时数据 - 使用网格布局
        data_grid = ttk.Frame(data_display_frame)
        data_grid.pack(fill=tk.X)
        
        ttk.Label(data_grid, text="当前位置:").grid(row=0, column=0, sticky=tk.W, padx=(0, 5))
        self.current_position = tk.StringVar(value="--")
        ttk.Label(data_grid, textvariable=self.current_position, foreground="red").grid(row=0, column=1, sticky=tk.W, padx=(0, 15))
        
        ttk.Label(data_grid, text="编码器角度:").grid(row=0, column=2, sticky=tk.W, padx=(0, 5))
        self.encoder_angle = tk.StringVar(value="--")
        ttk.Label(data_grid, textvariable=self.encoder_angle, foreground="red").grid(row=0, column=3, sticky=tk.W, padx=(0, 15))
        
        ttk.Label(data_grid, text="实际步长:").grid(row=0, column=4, sticky=tk.W, padx=(0, 5))
        self.actual_step = tk.StringVar(value="--")
        ttk.Label(data_grid, textvariable=self.actual_step, foreground="purple").grid(row=0, column=5, sticky=tk.W, padx=(0, 15))
        
        # 第二行添加角度误差和误差统计
        ttk.Label(data_grid, text="角度误差:").grid(row=1, column=0, sticky=tk.W, padx=(0, 5))
        self.angle_error = tk.StringVar(value="--")
        ttk.Label(data_grid, textvariable=self.angle_error, foreground="green").grid(row=1, column=1, sticky=tk.W, padx=(0, 15))
        
        ttk.Label(data_grid, text="误差>0.01次数:").grid(row=1, column=2, sticky=tk.W, padx=(0, 5))
        self.error_count_var = tk.StringVar(value="0")
        ttk.Label(data_grid, textvariable=self.error_count_var, foreground="orange").grid(row=1, column=3, sticky=tk.W, padx=(0, 15))
        
        ttk.Label(data_grid, text="测试进度:").grid(row=1, column=4, sticky=tk.W, padx=(0, 5))
        self.progress_var = tk.StringVar(value="0/0")
        ttk.Label(data_grid, textvariable=self.progress_var).grid(row=1, column=5, sticky=tk.W)
        
        # 进度条
        self.progress_bar = ttk.Progressbar(data_display_frame, mode='determinate')
        self.progress_bar.pack(fill=tk.X, pady=(5, 0))
        
        # 绘图区域
        plot_frame = ttk.LabelFrame(display_frame, text="实时曲线", padding="5")
        plot_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(0, 5))
        
        # 创建matplotlib图形
        self.fig = Figure(figsize=(8, 4), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlabel('实际角度 (°)')
        self.ax.set_ylabel('角度误差 (°)')
        self.ax.set_title('实时角度误差曲线')
        self.ax.grid(True, alpha=0.3)
        self.ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        # 设置固定的坐标轴范围
        self.ax.set_xlim(0, 360)  # x轴固定0到360度
        self.ax.set_ylim(-0.01, 0.01)  # y轴固定-0.01到0.01度
        
        # 初始化两条线条
        self.angle_error_line, = self.ax.plot([], [], 'b-', linewidth=2, label='角度变化误差')
        self.cumulative_error_line, = self.ax.plot([], [], 'gray', linewidth=2, label='累积误差')
        self.ax.legend()
        
        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 日志显示区域
        log_frame = ttk.LabelFrame(display_frame, text="测试日志", padding="5")
        log_frame.grid(row=3, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, width=60)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        
    def setup_logging(self):
        """设置日志记录"""
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)
        
        # 创建自定义处理器，将日志输出到GUI
        class GUILogHandler(logging.Handler):
            def __init__(self, text_widget):
                super().__init__()
                self.text_widget = text_widget
                
            def emit(self, record):
                msg = self.format(record)
                self.text_widget.insert(tk.END, msg + '\n')
                self.text_widget.see(tk.END)  # 自动滚动到最新内容
        
        gui_handler = GUILogHandler(self.log_text)
        gui_handler.setFormatter(logging.Formatter('%(asctime)s - %(message)s', datefmt='%H:%M:%S'))
        self.logger.addHandler(gui_handler)
        
    def load_default_settings(self):
        """加载默认设置"""
        try:
            with open('gui_settings.json', 'r') as f:
                settings = json.load(f)
                self.esp32_ip.set(settings.get('esp32_ip', '192.168.4.1'))
                self.encoder_port.set(settings.get('encoder_port', 'COM43'))
                self.reduction_ratio.set(settings.get('reduction_ratio', '19.24'))
                self.internal_motor_param.set(settings.get('internal_motor_param', '8'))
                self.user_description.set(settings.get('user_description', '测试'))
                self.start_pos.set(settings.get('start_pos', '32'))
                self.target_angle_range.set(settings.get('target_angle_range', '360'))
                self.step_interval.set(settings.get('step_interval', '0.8'))
        except FileNotFoundError:
            pass  # 使用默认值
            
    def save_settings(self):
        """保存设置"""
        settings = {
            'esp32_ip': self.esp32_ip.get(),
            'encoder_port': self.encoder_port.get(),
            'reduction_ratio': self.reduction_ratio.get(),
            'internal_motor_param': self.internal_motor_param.get(),
            'user_description': self.user_description.get(),
            'start_pos': self.start_pos.get(),
            'target_angle_range': self.target_angle_range.get(),
            'step_interval': self.step_interval.get()
        }
        with open('gui_settings.json', 'w') as f:
            json.dump(settings, f, indent=4)
    
    def calculate_smart_positions(self, start_pos, target_angle_range, step_interval, reduction_ratio, internal_motor_param):
        """
        智能计算测试位置列表，避免跨越0°/360°边界
        
        Args:
            start_pos: 起始位置
            target_angle_range: 目标角度范围
            step_interval: 步长间隔
            reduction_ratio: 外部减速比
            internal_motor_param: 内部电机参数
        
        Returns:
            list: 测试位置列表
        """
        # 计算每个步长对应的角度变化
        angle_per_step = (360 / reduction_ratio / internal_motor_param) * step_interval
        
        # 计算需要的步数
        required_steps = int(target_angle_range / angle_per_step)
        
        # 生成位置列表
        positions = []
        current_pos = start_pos
        
        for i in range(required_steps + 1):
            positions.append(current_pos)
            current_pos += step_interval
        
        # 计算实际的结束位置
        end_pos = positions[-1]
        
        # 检查是否会跨越边界（简单检查）
        start_angle = self.calculate_position_angle(start_pos, reduction_ratio, internal_motor_param)
        end_angle = self.calculate_position_angle(end_pos, reduction_ratio, internal_motor_param)
        
        # 如果跨越了360度边界，警告用户
        if abs(end_angle - start_angle) > 350:  # 近似检查跨越
            self.logger.warning(f"警告：测试范围可能跨越360°边界（{start_angle:.1f}° -> {end_angle:.1f}°）")
        
        return positions
    
    def calculate_position_angle(self, position, reduction_ratio, internal_motor_param):
        """计算位置对应的角度"""
        return (position * 360 / reduction_ratio / internal_motor_param) % 360
            
    def start_test(self):
        """开始测试"""
        if self.is_testing:
            return
            
        try:
            # 验证参数
            reduction_ratio = float(self.reduction_ratio.get())
            internal_motor_param = float(self.internal_motor_param.get())
            user_description = self.user_description.get().strip()
            start_pos = float(self.start_pos.get())
            target_angle_range = float(self.target_angle_range.get())
            step_interval = float(self.step_interval.get())
            
            # 验证参数合理性
            if internal_motor_param <= 0:
                raise ValueError("内部电机参数必须大于0")
            if target_angle_range <= 0 or target_angle_range > 720:
                raise ValueError("目标角度范围必须在0-720度之间")
            if step_interval <= 0:
                raise ValueError("步长间隔必须大于0")
            if not user_description:
                user_description = "测试"
            
            # 智能计算测试位置列表
            positions = self.calculate_smart_positions(
                start_pos, target_angle_range, step_interval, 
                reduction_ratio, internal_motor_param
            )
            
            end_pos = positions[-1]
            self.logger.info(f"开始测试：{user_description}")
            self.logger.info(f"测试参数：起始{start_pos} -> 结束{end_pos}，共{len(positions)}个位置点")
            
            # 更新测试参数显示
            theoretical_step = (360 / reduction_ratio / internal_motor_param) * step_interval
            self.param_reduction.set(f"{reduction_ratio}")
            self.param_step.set(f"{theoretical_step:.5f}°")
            self.param_range.set(f"{start_pos} ~ {end_pos:.1f} (间隔{step_interval}, 范围{target_angle_range}°)")
            
            # 清空之前的数据
            self.test_results = []
            self.real_time_data = {'positions': [], 'angles': [], 'errors': [], 'cumulative_errors': []}
            self.start_angle = None
            
            # 重置实时统计数据
            self.previous_angle = None
            self.error_count = 0
            self.error_count_var.set("0")
            self.actual_step.set("--")
            
            # 设置进度条
            self.progress_bar['maximum'] = len(positions)
            self.progress_bar['value'] = 0
            self.progress_var.set(f"0/{len(positions)}")
            
            # 更新按钮状态
            self.start_button.config(state=tk.DISABLED)
            self.stop_button.config(state=tk.NORMAL)
            self.is_testing = True
            
            # 检查连接状态
            if not self.esp32_connected or not self.encoder_connected:
                messagebox.showerror("连接错误", "请先连接ESP32和编码器")
                self.is_testing = False
                self.start_button.config(state=tk.NORMAL)
                self.stop_button.config(state=tk.DISABLED)
                return
            
            # 停止实时读取，避免冲突
            self.stop_real_time_reading()
            
            # 创建测试器，使用已连接的实例
            self.tester = CustomMotorPrecisionTester(
                encoder_port=self.encoder_port.get(),
                esp32_ip=self.esp32_ip.get(),
                reduction_ratio=reduction_ratio,
                internal_motor_param=internal_motor_param,
                user_description=user_description,
                test_positions=positions,
                gui_callback=self.update_from_test,
                motor_controller=self.motor_controller,
                encoder_reader=self.encoder_reader
            )
            
            # 在新线程中运行测试
            self.test_thread = threading.Thread(target=self.run_test_thread)
            self.test_thread.daemon = True
            self.test_thread.start()
            
        except ValueError as e:
            messagebox.showerror("参数错误", f"请检查输入参数: {str(e)}")
        except Exception as e:
            messagebox.showerror("启动错误", f"启动测试失败: {str(e)}")
            
    def run_test_thread(self):
        """在后台线程中运行测试"""
        try:
            success = self.tester.run_test()
            if success:
                self.root.after(0, lambda: self.logger.info("测试完成"))
            else:
                self.root.after(0, lambda: self.logger.info("测试失败或被中断"))
        except Exception as e:
            self.root.after(0, lambda: self.logger.error(f"测试过程中发生错误: {str(e)}"))
        finally:
            self.root.after(0, self.test_finished)
            
    def test_finished(self):
        """测试完成后的清理工作"""
        self.is_testing = False
        if self.esp32_connected and self.encoder_connected:
            self.start_button.config(state=tk.NORMAL)
        else:
            self.start_button.config(state=tk.DISABLED)
        self.stop_button.config(state=tk.DISABLED)
        
        # 自动生成测试报告
        if len(self.real_time_data['angles']) > 0:
            try:
                self.generate_test_report(auto_save=True)
                self.logger.info("测试报告已自动生成")
            except Exception as e:
                self.logger.error(f"自动生成测试报告失败: {str(e)}")
        
        # 重新启动实时读取
        if self.encoder_connected:
            self.start_real_time_reading()
        
    def stop_test(self):
        """停止测试"""
        if self.tester and self.is_testing:
            self.logger.info("正在停止测试...")
            self.tester.stop_test()  # 调用测试器的停止方法
            self.is_testing = False
            
    def update_from_test(self, position, angle, error=None, progress=None):
        """从测试线程接收更新"""
        def update_gui():
            self.current_position.set(f"{position}")
            self.encoder_angle.set(f"{angle:.3f}°")
            
            # 计算实际步长
            if self.previous_angle is not None:
                actual_step_value = angle - self.previous_angle
                self.actual_step.set(f"{actual_step_value:.3f}°")
            else:
                self.actual_step.set("--")  # 第一个点显示"--"
            
            # 更新上一个角度值
            self.previous_angle = angle
            
            # 设置起始角度
            if self.start_angle is None:
                self.start_angle = angle
            
            if error is not None:
                self.angle_error.set(f"{error:.3f}°")
                self.real_time_data['errors'].append(error)
                
                # 实时统计误差>0.01次数
                if abs(error) > 0.01:
                    self.error_count += 1
                    self.error_count_var.set(str(self.error_count))
            
            # 计算累积误差
            if len(self.real_time_data['positions']) > 0:
                # 理论总角度变化
                start_pos = self.real_time_data['positions'][0]
                theoretical_total_change = (position - start_pos) * (360 / float(self.reduction_ratio.get()) / 8)
                # 累积误差 = 当前实际角度 - (起始角度 + 理论总角度变化)
                cumulative_error = angle - (self.start_angle + theoretical_total_change)
                self.real_time_data['cumulative_errors'].append(cumulative_error)
                
            self.real_time_data['positions'].append(position)
            self.real_time_data['angles'].append(angle)
            
            if progress is not None:
                self.progress_bar['value'] = progress
                self.progress_var.set(f"{progress}/{self.progress_bar['maximum']}")
                
            # 更新图形
            self.update_plot()
            
        self.root.after(0, update_gui)
        
    def update_plot(self):
        """更新实时图形"""
        if len(self.real_time_data['angles']) < 1:
            return
            
        try:
            angles = self.real_time_data['angles']
            errors = self.real_time_data['errors']
            cumulative_errors = self.real_time_data['cumulative_errors']
            
            # 更新角度变化误差曲线（跳过第一个点，因为第一个点没有误差）
            if len(errors) > 0 and len(angles) > 1:
                angle_error_x = angles[1:]  # 跳过第一个点
                self.angle_error_line.set_data(angle_error_x, errors)
            
            # 更新累积误差曲线
            if len(cumulative_errors) > 0:
                cumulative_error_x = angles[:len(cumulative_errors)]
                self.cumulative_error_line.set_data(cumulative_error_x, cumulative_errors)
            
            # 动态调整坐标轴范围以确保两条曲线都清晰可见
            all_errors = []
            if len(errors) > 0:
                all_errors.extend(errors)
            if len(cumulative_errors) > 0:
                all_errors.extend(cumulative_errors)
                
            if all_errors:
                error_min = min(all_errors)
                error_max = max(all_errors)
                error_range = error_max - error_min
                margin = max(0.001, error_range * 0.1)  # 至少1毫度的边距
                self.ax.set_ylim(error_min - margin, error_max + margin)
            else:
                self.ax.set_ylim(-0.01, 0.01)  # 默认范围
            
            # 固定x轴范围
            self.ax.set_xlim(0, 360)  # x轴固定0到360度
                
            self.canvas.draw()
            
        except Exception as e:
            self.logger.error(f"更新图形失败: {str(e)}")
            
    def enable_motor(self):
        """使能电机"""
        if not self.esp32_connected or not self.motor_controller:
            self.logger.error("ESP32未连接，无法使能电机")
            return
            
        try:
            success, msg = self.motor_controller.enable_motor()
            if success:
                self.logger.info("电机使能成功")
            else:
                self.logger.error(f"电机使能失败: {msg}")
        except Exception as e:
            self.logger.error(f"使能电机时发生错误: {str(e)}")
            
    def disable_motor(self):
        """失能电机"""
        if not self.esp32_connected or not self.motor_controller:
            self.logger.error("ESP32未连接，无法失能电机")
            return
            
        try:
            success, msg = self.motor_controller.disable_motor()
            if success:
                self.logger.info("电机失能成功")
            else:
                self.logger.error(f"电机失能失败: {msg}")
        except Exception as e:
            self.logger.error(f"失能电机时发生错误: {str(e)}")
            
    def move_to_position(self):
        """移动到指定位置"""
        try:
            # 获取目标位置
            target_pos = self.target_position.get().strip()
            if not target_pos:
                messagebox.showwarning("输入错误", "请输入目标位置")
                return
                
            try:
                position = float(target_pos)
            except ValueError:
                messagebox.showerror("输入错误", "位置值必须是数字")
                return
            
            # 验证位置范围（可根据实际需要调整）
            if position < 0 or position > 360:
                if not messagebox.askyesno("位置确认", f"位置值 {position} 超出常规范围 (0-360)，确定要继续吗？"):
                    return
            
            self.logger.info(f"开始移动到位置: {position}")
            
            # 检查ESP32连接状态
            if not self.esp32_connected or not self.motor_controller:
                messagebox.showerror("连接错误", "请先连接ESP32")
                return
            
            controller = self.motor_controller
            
            # 1. 使能电机
            success, msg = controller.enable_motor()
            if not success:
                self.logger.error(f"使能电机失败: {msg}")
                messagebox.showerror("操作失败", f"使能电机失败: {msg}")
                return
            
            self.logger.info("电机使能成功")
            time.sleep(0.5)  # 等待使能完成
            
            # 2. 移动到目标位置
            success, msg = controller.set_position(position)
            if success:
                self.logger.info(f"位置设置命令发送成功，移动到: {position}")
                messagebox.showinfo("操作成功", f"电机正在移动到位置: {position}")
            else:
                self.logger.error(f"设置位置失败: {msg}")
                messagebox.showerror("操作失败", f"设置位置失败: {msg}")
            
            # 3. 等待一段时间后失能电机（可选）
            # 注意：这里不自动失能，让用户手动控制
            
        except Exception as e:
            self.logger.error(f"移动到位置时发生错误: {str(e)}")
            messagebox.showerror("操作错误", f"移动到位置时发生错误: {str(e)}")
    
    def clear_motor_errors(self):
        """清除电机错误"""
        try:
            controller = MotorController(self.esp32_ip.get())
            success, msg = controller.clear_errors()
            if success:
                self.logger.info("电机错误清除成功")
                messagebox.showinfo("操作成功", "电机错误已清除")
            else:
                self.logger.error(f"清除电机错误失败: {msg}")
                messagebox.showerror("操作失败", f"清除电机错误失败: {msg}")
        except Exception as e:
            self.logger.error(f"清除电机错误时发生错误: {str(e)}")
            messagebox.showerror("操作错误", f"清除电机错误时发生错误: {str(e)}")
    
    def connect_devices(self):
        """连接ESP32和编码器"""
        self.logger.info("开始连接设备...")
        
        # 连接ESP32
        try:
            self.motor_controller = MotorController(self.esp32_ip.get())
            # 尝试连接测试
            success, msg = self.motor_controller.get_motor_status()
            if success:
                self.esp32_connected = True
                self.esp32_status.set("已连接")
                self.esp32_status_label.config(foreground="green")
                self.logger.info(f"ESP32连接成功: {self.esp32_ip.get()}")
            else:
                self.esp32_connected = False
                self.esp32_status.set("连接失败")
                self.esp32_status_label.config(foreground="red")
                self.logger.error(f"ESP32连接失败: {msg}")
        except Exception as e:
            self.esp32_connected = False
            self.esp32_status.set("连接错误")
            self.esp32_status_label.config(foreground="red")
            self.logger.error(f"ESP32连接错误: {str(e)}")
        
        # 连接编码器
        try:
            self.encoder_reader = TamagawaEncoderReader(self.encoder_port.get())
            # 尝试读取测试
            angle = self.encoder_reader.read_angle()
            if angle is not None:
                self.encoder_connected = True
                self.encoder_status.set("已连接")
                self.encoder_status_label.config(foreground="green")
                self.logger.info(f"编码器连接成功: {self.encoder_port.get()}")
                
                # 启动实时角度读取
                self.start_real_time_reading()
            else:
                self.encoder_connected = False
                self.encoder_status.set("连接失败")
                self.encoder_status_label.config(foreground="red")
                self.logger.error("编码器连接失败: 无法读取角度")
        except Exception as e:
            self.encoder_connected = False
            self.encoder_status.set("连接错误")
            self.encoder_status_label.config(foreground="red")
            self.logger.error(f"编码器连接错误: {str(e)}")
        
        # 更新按钮状态
        if self.esp32_connected or self.encoder_connected:
            self.connect_button.config(state=tk.DISABLED)
            self.disconnect_button.config(state=tk.NORMAL)
        
        # 根据连接状态更新其他按钮
        if self.esp32_connected:
            self.enable_button.config(state=tk.NORMAL)
            self.disable_button.config(state=tk.NORMAL)
            self.move_to_position_button.config(state=tk.NORMAL)
            self.clear_errors_button.config(state=tk.NORMAL)
        
        if self.esp32_connected and self.encoder_connected:
            self.start_button.config(state=tk.NORMAL)
    
    def disconnect_devices(self):
        """断开ESP32和编码器连接"""
        self.logger.info("断开设备连接...")
        
        # 停止实时读取
        self.stop_real_time_reading()
        
        # 断开ESP32连接
        if self.motor_controller:
            try:
                # 可以添加断开连接的清理代码
                self.motor_controller = None
                self.esp32_connected = False
                self.esp32_status.set("未连接")
                self.esp32_status_label.config(foreground="red")
                self.logger.info("ESP32连接已断开")
            except Exception as e:
                self.logger.error(f"断开ESP32连接时发生错误: {str(e)}")
        
        # 断开编码器连接
        if self.encoder_reader:
            try:
                self.encoder_reader.close()
                self.encoder_reader = None
                self.encoder_connected = False
                self.encoder_status.set("未连接")
                self.encoder_status_label.config(foreground="red")
                self.logger.info("编码器连接已断开")
            except Exception as e:
                self.logger.error(f"断开编码器连接时发生错误: {str(e)}")
        
        # 更新按钮状态
        self.connect_button.config(state=tk.NORMAL)
        self.disconnect_button.config(state=tk.DISABLED)
        self.enable_button.config(state=tk.DISABLED)
        self.disable_button.config(state=tk.DISABLED)
        self.move_to_position_button.config(state=tk.DISABLED)
        self.clear_errors_button.config(state=tk.DISABLED)
        self.start_button.config(state=tk.DISABLED)
        
        # 清空实时数据显示
        self.encoder_angle.set("--")
    
    def start_real_time_reading(self):
        """启动实时编码器角度读取"""
        if self.encoder_connected and not self.real_time_running:
            self.real_time_running = True
            self.real_time_thread = threading.Thread(target=self.real_time_reading_thread)
            self.real_time_thread.daemon = True
            self.real_time_thread.start()
            self.logger.info("启动实时角度读取")
    
    def stop_real_time_reading(self):
        """停止实时编码器角度读取"""
        if self.real_time_running:
            self.real_time_running = False
            self.logger.info("停止实时角度读取")
    
    def real_time_reading_thread(self):
        """实时读取编码器角度的后台线程"""
        import time
        while self.real_time_running and self.encoder_connected:
            try:
                if self.encoder_reader:
                    angle = self.encoder_reader.read_angle()
                    if angle is not None:
                        # 更新GUI显示
                        self.root.after(0, lambda a=angle: self.encoder_angle.set(f"{a:.3f}°"))
                    time.sleep(0.1)  # 每100ms读取一次
                else:
                    break
            except Exception as e:
                self.root.after(0, lambda: self.logger.error(f"实时读取角度错误: {str(e)}"))
                break
            
    def save_data(self):
        """保存测试数据"""
        if not self.test_results:
            messagebox.showwarning("警告", "没有测试数据需要保存")
            return
        
        # 生成默认文件名（使用用户说明文本和时间戳）
        user_desc = self.user_description.get().strip()
        if not user_desc:
            user_desc = "测试"
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        default_filename = f"{user_desc}_{timestamp}.csv"
            
        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="保存测试数据",
            initialvalue=default_filename
        )
        
        if filename:
            try:
                with open(filename, 'w', newline='', encoding='utf-8') as csvfile:
                    fieldnames = ['position', 'actual_angle']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    writer.writerows(self.test_results)
                self.logger.info(f"数据已保存到: {filename}")
            except Exception as e:
                messagebox.showerror("保存错误", f"保存数据失败: {str(e)}")
                
    def load_data(self):
        """加载测试数据"""
        filename = filedialog.askopenfilename(
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            title="加载测试数据"
        )
        
        if filename:
            try:
                with open(filename, 'r', encoding='utf-8') as csvfile:
                    reader = csv.DictReader(csvfile)
                    self.test_results = list(reader)
                    
                # 重新计算实时数据用于绘图
                self.real_time_data = {'positions': [], 'angles': [], 'errors': [], 'cumulative_errors': []}
                self.start_angle = None
                
                for i, result in enumerate(self.test_results):
                    position = float(result['position'])
                    angle = float(result['actual_angle'])
                    
                    # 设置起始角度
                    if self.start_angle is None:
                        self.start_angle = angle
                    
                    self.real_time_data['positions'].append(position)
                    self.real_time_data['angles'].append(angle)
                    
                    # 计算累积误差
                    if i > 0:
                        start_pos = float(self.test_results[0]['position'])
                        theoretical_total_change = (position - start_pos) * (360 / float(self.reduction_ratio.get()) / float(self.internal_motor_param.get()))
                        cumulative_error = angle - (self.start_angle + theoretical_total_change)
                        self.real_time_data['cumulative_errors'].append(cumulative_error)
                        
                        # 计算角度变化误差
                        actual_change = angle - float(self.test_results[i-1]['actual_angle'])
                        theoretical_step = 0.8 * (360 / float(self.reduction_ratio.get()) / float(self.internal_motor_param.get()))
                        error = actual_change - theoretical_step
                        self.real_time_data['errors'].append(error)
                    else:
                        # 第一个点的累积误差为0
                        self.real_time_data['cumulative_errors'].append(0.0)
                        
                self.update_plot()
                self.logger.info(f"已加载数据: {filename}, 共{len(self.test_results)}个数据点")
                
            except Exception as e:
                messagebox.showerror("加载错误", f"加载数据失败: {str(e)}")
    
    def sanitize_filename(self, filename):
        """清理文件名，移除不安全字符"""
        # 移除或替换不安全的文件名字符
        filename = re.sub(r'[<>:"/\\|?*]', '_', filename)
        # 移除多余的空格和点
        filename = re.sub(r'\s+', '_', filename.strip())
        filename = filename.strip('.')
        # 限制长度
        if len(filename) > 50:
            filename = filename[:50]
        return filename
    
    def generate_unique_filename(self, base_filename):
        """生成唯一的文件名，避免重名"""
        if not os.path.exists(base_filename):
            return base_filename
        
        base, ext = os.path.splitext(base_filename)
        counter = 1
        while True:
            new_filename = f"{base}_{counter}{ext}"
            if not os.path.exists(new_filename):
                return new_filename
            counter += 1
    
    def generate_test_report(self, auto_save=False):
        """生成综合测试报告图片"""
        if len(self.real_time_data['angles']) == 0:
            if not auto_save:
                messagebox.showwarning("警告", "没有测试数据，无法生成报告")
            return
        
        try:
            # 设置中文字体
            plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
            plt.rcParams['axes.unicode_minus'] = False
            
            # 创建图形，使用4个子图布局
            fig = plt.figure(figsize=(16, 12))
            
            # 子图1：绘制曲线（左上）
            ax1 = plt.subplot(2, 2, 1)
            self._plot_curves(ax1)
            
            # 子图2：测试参数表格（右上）
            ax2 = plt.subplot(2, 2, 2)
            self._plot_parameters_table(ax2)
            
            # 子图3：实时数据统计汇总（左下）
            ax3 = plt.subplot(2, 2, 3)
            self._plot_statistics_summary(ax3)
            
            # 子图4：用户说明文本（右下）
            ax4 = plt.subplot(2, 2, 4)
            self._plot_user_description(ax4)
            
            # 设置整体标题
            user_desc = self.user_description.get().strip() or "测试"
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            fig.suptitle(f'电机精度测试报告 - {user_desc}\n生成时间: {timestamp}', 
                        fontsize=16, fontweight='bold', y=0.95)
            
            # 调整子图间距
            plt.tight_layout(rect=[0, 0, 1, 0.92])
            
            if auto_save:
                # 自动保存
                filename = self._generate_report_filename()
                filepath = self.generate_unique_filename(filename)
                fig.savefig(filepath, dpi=300, bbox_inches='tight', 
                           facecolor='white', edgecolor='none')
                self.logger.info(f"测试报告已保存: {filepath}")
                plt.close(fig)  # 关闭图形以释放内存
            else:
                # 手动保存 - 让用户选择保存位置
                user_desc_clean = self.sanitize_filename(user_desc)
                timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
                default_filename = f"{timestamp_str}_{user_desc_clean}.png"
                
                filename = filedialog.asksaveasfilename(
                    defaultextension=".png",
                    filetypes=[("PNG files", "*.png"), ("All files", "*.*")],
                    title="保存测试报告",
                    initialvalue=default_filename
                )
                
                if filename:
                    fig.savefig(filename, dpi=300, bbox_inches='tight', 
                               facecolor='white', edgecolor='none')
                    self.logger.info(f"测试报告已保存: {filename}")
                    messagebox.showinfo("保存成功", f"测试报告已保存:\n{filename}")
                    plt.close(fig)
                else:
                    plt.show()  # 如果用户取消保存，则显示图形
                    
        except Exception as e:
            self.logger.error(f"生成测试报告失败: {str(e)}")
            if not auto_save:
                messagebox.showerror("生成报告失败", f"生成测试报告时发生错误:\n{str(e)}")
    
    def _generate_report_filename(self):
        """生成报告文件名"""
        user_desc = self.user_description.get().strip() or "测试"
        user_desc_clean = self.sanitize_filename(user_desc)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f"{timestamp}_{user_desc_clean}.png"
    
    def _plot_curves(self, ax):
        """绘制曲线图"""
        angles = self.real_time_data['angles']
        errors = self.real_time_data['errors']
        cumulative_errors = self.real_time_data['cumulative_errors']
        
        # 绘制角度变化误差曲线（跳过第一个点）
        if len(errors) > 0 and len(angles) > 1:
            angle_error_x = angles[1:]
            ax.plot(angle_error_x, errors, 'b-', linewidth=2, label='角度变化误差', alpha=0.8)
        
        # 绘制累积误差曲线
        if len(cumulative_errors) > 0:
            cumulative_error_x = angles[:len(cumulative_errors)]
            ax.plot(cumulative_error_x, cumulative_errors, 'r-', linewidth=2, label='累积误差', alpha=0.8)
        
        ax.set_xlabel('实际角度 (°)')
        ax.set_ylabel('角度误差 (°)')
        ax.set_title('实时角度误差曲线')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax.legend()
        
        # 设置坐标轴范围
        ax.set_xlim(0, 360)
        if len(errors + cumulative_errors) > 0:
            all_errors = errors + cumulative_errors
            error_range = max(all_errors) - min(all_errors)
            margin = max(0.001, error_range * 0.1)
            ax.set_ylim(min(all_errors) - margin, max(all_errors) + margin)
    
    def _plot_parameters_table(self, ax):
        """绘制测试参数表格"""
        ax.axis('off')  # 隐藏坐标轴
        
        # 准备参数数据
        params_data = [
            ['参数名称', '数值'],
            ['外部减速比', f"{self.reduction_ratio.get()}"],
            ['内部电机参数', f"{self.internal_motor_param.get()}"],
            ['起始位置', f"{self.start_pos.get()}"],
            ['目标角度范围', f"{self.target_angle_range.get()}°"],
            ['步长间隔', f"{self.step_interval.get()}"],
            ['ESP32 IP', f"{self.esp32_ip.get()}"],
            ['编码器串口', f"{self.encoder_port.get()}"],
            ['测试点数', f"{len(self.real_time_data['angles'])}"]
        ]
        
        # 创建表格
        table = ax.table(cellText=params_data, loc='center', cellLoc='left')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 1.8)
        
        # 设置表格样式
        for i in range(len(params_data)):
            for j in range(len(params_data[i])):
                cell = table[(i, j)]
                if i == 0:  # 标题行
                    cell.set_facecolor('#4CAF50')
                    cell.set_text_props(weight='bold', color='white')
                else:
                    cell.set_facecolor('#f0f0f0' if i % 2 == 0 else 'white')
        
        ax.set_title('测试参数', fontweight='bold', pad=20)
    
    def _plot_statistics_summary(self, ax):
        """绘制实时数据统计汇总"""
        ax.axis('off')  # 隐藏坐标轴
        
        # 计算统计数据
        errors = self.real_time_data['errors']
        cumulative_errors = self.real_time_data['cumulative_errors']
        
        stats_data = [
            ['统计项目', '数值'],
            ['测试点数', f"{len(self.real_time_data['angles'])}"],
            ['误差>0.01次数', f"{self.error_count}"]
        ]
        
        if errors:
            max_error = max([abs(e) for e in errors])
            avg_error = sum([abs(e) for e in errors]) / len(errors)
            stats_data.extend([
                ['最大角度误差', f"{max_error:.4f}°"],
                ['平均角度误差', f"{avg_error:.4f}°"]
            ])
        
        if cumulative_errors:
            max_cum_error = max([abs(e) for e in cumulative_errors])
            final_cum_error = cumulative_errors[-1] if cumulative_errors else 0
            stats_data.extend([
                ['最大累积误差', f"{max_cum_error:.4f}°"],
                ['最终累积误差', f"{final_cum_error:.4f}°"]
            ])
        
        # 创建表格
        table = ax.table(cellText=stats_data, loc='center', cellLoc='left')
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 1.8)
        
        # 设置表格样式
        for i in range(len(stats_data)):
            for j in range(len(stats_data[i])):
                cell = table[(i, j)]
                if i == 0:  # 标题行
                    cell.set_facecolor('#2196F3')
                    cell.set_text_props(weight='bold', color='white')
                else:
                    cell.set_facecolor('#f0f0f0' if i % 2 == 0 else 'white')
        
        ax.set_title('统计数据汇总', fontweight='bold', pad=20)
    
    def _plot_user_description(self, ax):
        """绘制用户说明文本"""
        ax.axis('off')  # 隐藏坐标轴
        
        user_desc = self.user_description.get().strip()
        if not user_desc:
            user_desc = "无说明文本"
        
        # 添加说明文本
        ax.text(0.5, 0.5, user_desc, ha='center', va='center', 
                fontsize=12, wrap=True, 
                bbox=dict(boxstyle="round,pad=0.5", facecolor='lightyellow', alpha=0.8))
        
        ax.set_title('用户说明', fontweight='bold', pad=20)
    
    def generate_report_button_clicked(self):
        """手动生成测试报告按钮点击事件"""
        self.generate_test_report(auto_save=False)
                
    def on_closing(self):
        """窗口关闭事件"""
        self.save_settings()
        if self.is_testing:
            result = messagebox.askyesno("确认", "测试正在进行中，确定要退出吗？")
            if not result:
                return
        
        # 停止所有后台线程
        self.stop_real_time_reading()
        
        # 断开所有连接
        if self.esp32_connected or self.encoder_connected:
            self.disconnect_devices()
                
        self.root.destroy()

class CustomMotorPrecisionTester(MotorPrecisionTester):
    """自定义的电机精度测试器，支持GUI回调"""
    
    def __init__(self, encoder_port, esp32_ip, reduction_ratio, internal_motor_param, user_description, 
                 test_positions, gui_callback=None, motor_controller=None, encoder_reader=None):
        super().__init__(encoder_port, esp32_ip, reduction_ratio, internal_motor_param, enable_plot=False)
        self.test_positions = test_positions
        self.gui_callback = gui_callback
        self.user_description = user_description
        
        # 如果传入了已连接的实例，使用它们
        if motor_controller is not None:
            self.motor_controller = motor_controller
        if encoder_reader is not None:
            self.encoder = encoder_reader
        
    def test_single_position(self, position):
        """重写单个位置测试方法，添加GUI回调"""
        result = super().test_single_position(position)
        
        if result and self.gui_callback:
            error = 0
            if len(self.actual_angles) > 1:
                actual_change = self.actual_angles[-1] - self.actual_angles[-2]
                error = actual_change - self.theoretical_step
                
            # 回调GUI更新
            progress = len(self.actual_angles)
            self.gui_callback(position, result['actual_angle'], error, progress)
            
        return result

def main():
    """主函数"""
    root = tk.Tk()
    app = MotorPrecisionGUI(root)
    
    # 设置窗口关闭事件
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    # 启动GUI
    root.mainloop()

if __name__ == "__main__":
    main()
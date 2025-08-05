#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试报告生成功能的验证脚本
"""

import sys
import os
import numpy as np
from datetime import datetime

# 添加当前目录到Python路径
sys.path.append(os.path.dirname(__file__))

# 模拟GUI类来测试报告生成功能
class MockMotorPrecisionGUI:
    """模拟GUI类，用于测试报告生成功能"""
    
    def __init__(self):
        # 模拟GUI变量
        self.reduction_ratio = MockVar("19.24")
        self.internal_motor_param = MockVar("8")
        self.step_interval = MockVar("0.8")
        self.start_pos = MockVar("32")
        self.target_angle_range = MockVar("360")
        self.user_description = MockVar("测试报告生成功能")
        self.esp32_ip = MockVar("192.168.4.1")
        self.encoder_port = MockVar("COM43")
        
        # 生成模拟测试数据
        self.generate_mock_data()
    
    def generate_mock_data(self):
        """生成模拟测试数据"""
        # 生成450个测试点（模拟360度测试范围）
        num_points = 450
        
        # 生成位置数据
        positions = np.linspace(32, 32 + 360, num_points)
        
        # 生成角度数据（带一些随机误差）
        theoretical_step = (360 / 19.24 / 8) * 0.8  # 理论步长
        angles = []
        start_angle = 0
        
        for i, pos in enumerate(positions):
            if i == 0:
                angle = start_angle
            else:
                # 添加一些随机误差
                error = np.random.normal(0, 0.002)  # 标准差为0.002度
                angle = angles[-1] + theoretical_step + error
            angles.append(angle)
        
        # 计算误差
        errors = []
        for i in range(1, len(angles)):
            actual_change = angles[i] - angles[i-1]
            error = actual_change - theoretical_step
            errors.append(error)
        
        # 计算累积误差
        cumulative_errors = []
        for i, pos in enumerate(positions):
            if i == 0:
                cumulative_errors.append(0.0)
            else:
                theoretical_total_change = (pos - positions[0]) * theoretical_step / 0.8
                cumulative_error = angles[i] - (angles[0] + theoretical_total_change)
                cumulative_errors.append(cumulative_error)
        
        # 设置实时数据
        self.real_time_data = {
            'positions': positions.tolist(),
            'angles': angles,
            'errors': errors,
            'cumulative_errors': cumulative_errors
        }
        
        print(f"生成了 {num_points} 个模拟测试点")
        print(f"角度范围: {min(angles):.3f}° 到 {max(angles):.3f}°")
        print(f"平均误差: {np.mean([abs(e) for e in errors]):.4f}°")
    
    # 导入报告生成相关方法
    def sanitize_filename(self, filename):
        """清理文件名，移除不安全字符"""
        import re
        filename = re.sub(r'[<>:"/\\|?*]', '_', filename)
        filename = re.sub(r'\s+', '_', filename.strip())
        filename = filename.strip('.')
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
    
    def _generate_report_filename(self):
        """生成报告文件名"""
        user_desc = self.user_description.get().strip() or "测试"
        user_desc_clean = self.sanitize_filename(user_desc)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f"{timestamp}_{user_desc_clean}.png"
    
    def generate_test_report(self, auto_save=False):
        """生成综合测试报告图片"""
        import matplotlib.pyplot as plt
        
        if len(self.real_time_data['angles']) == 0:
            print("警告: 没有测试数据，无法生成报告")
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
            
            # 保存报告
            filename = self._generate_report_filename()
            filepath = self.generate_unique_filename(filename)
            fig.savefig(filepath, dpi=300, bbox_inches='tight', 
                       facecolor='white', edgecolor='none')
            print(f"测试报告已保存: {filepath}")
            
            # 显示图形
            plt.show()
            plt.close(fig)
                    
        except Exception as e:
            print(f"生成测试报告失败: {str(e)}")
            import traceback
            traceback.print_exc()
    
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
        reduction_ratio = self.reduction_ratio.get()
        internal_param = self.internal_motor_param.get()
        theoretical_step = (360 / float(reduction_ratio) / float(internal_param)) * float(self.step_interval.get())
        
        params_data = [
            ['外部减速比', reduction_ratio],
            ['内部电机参数', internal_param],
            ['步长间隔', self.step_interval.get()],
            ['理论步长', f"{theoretical_step:.5f}°"],
            ['起始位置', self.start_pos.get()],
            ['目标角度范围', f"{self.target_angle_range.get()}°"],
            ['测试点数', str(len(self.real_time_data['angles']))],
            ['ESP32 IP', self.esp32_ip.get()],
            ['编码器串口', self.encoder_port.get()]
        ]
        
        # 创建表格
        table = ax.table(cellText=params_data,
                        colLabels=['参数名称', '参数值'],
                        cellLoc='left',
                        loc='center',
                        colWidths=[0.6, 0.4])
        
        # 设置表格样式
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 2)
        
        # 设置表头样式
        for i in range(2):
            table[(0, i)].set_facecolor('#4CAF50')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # 设置数据行样式
        for i in range(1, len(params_data) + 1):
            for j in range(2):
                if i % 2 == 0:
                    table[(i, j)].set_facecolor('#f0f0f0')
                else:
                    table[(i, j)].set_facecolor('white')
        
        ax.set_title('测试参数', fontweight='bold', pad=20)
    
    def _plot_statistics_summary(self, ax):
        """绘制实时数据统计汇总"""
        ax.axis('off')
        
        # 计算统计数据
        angles = self.real_time_data['angles']
        errors = self.real_time_data['errors']
        cumulative_errors = self.real_time_data['cumulative_errors']
        
        if len(errors) == 0:
            ax.text(0.5, 0.5, '无足够数据进行统计', ha='center', va='center', fontsize=12)
            ax.set_title('数据统计汇总', fontweight='bold')
            return
        
        # 统计计算
        error_count_over_threshold = sum(1 for e in errors if abs(e) > 0.01)
        avg_error = np.mean(errors)
        std_error = np.std(errors)
        max_error = max(errors)
        min_error = min(errors)
        abs_avg_error = np.mean([abs(e) for e in errors])
        abs_max_error = max([abs(e) for e in errors])
        
        # 累积误差统计
        if len(cumulative_errors) > 0:
            final_cumulative_error = cumulative_errors[-1]
            max_cumulative_error = max([abs(e) for e in cumulative_errors])
        else:
            final_cumulative_error = 0
            max_cumulative_error = 0
        
        # 测试范围统计
        angle_range = max(angles) - min(angles) if len(angles) > 0 else 0
        
        stats_data = [
            ['测试点总数', f"{len(angles)}"],
            ['测试角度范围', f"{angle_range:.1f}°"],
            ['误差>0.01°次数', f"{error_count_over_threshold}/{len(errors)}"],
            ['平均角度误差', f"{avg_error:.4f}°"],
            ['误差标准差', f"{std_error:.4f}°"],
            ['最大正误差', f"{max_error:.4f}°"],
            ['最大负误差', f"{min_error:.4f}°"],
            ['误差绝对值平均', f"{abs_avg_error:.4f}°"],
            ['误差绝对值最大', f"{abs_max_error:.4f}°"],
            ['最终累积误差', f"{final_cumulative_error:.4f}°"],
            ['最大累积误差', f"{max_cumulative_error:.4f}°"]
        ]
        
        # 创建统计表格
        table = ax.table(cellText=stats_data,
                        colLabels=['统计项目', '数值'],
                        cellLoc='left',
                        loc='center',
                        colWidths=[0.6, 0.4])
        
        # 设置表格样式
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 1.8)
        
        # 设置表头样式
        for i in range(2):
            table[(0, i)].set_facecolor('#2196F3')
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        # 设置数据行样式
        for i in range(1, len(stats_data) + 1):
            for j in range(2):
                if i % 2 == 0:
                    table[(i, j)].set_facecolor('#f0f0f0')
                else:
                    table[(i, j)].set_facecolor('white')
                    
                # 高亮重要数据
                if i == 3 and j == 1:  # 误差>0.01次数
                    if error_count_over_threshold > len(errors) * 0.1:  # 超过10%
                        table[(i, j)].set_facecolor('#ffcdd2')
        
        ax.set_title('数据统计汇总', fontweight='bold', pad=20)
    
    def _plot_user_description(self, ax):
        """绘制用户说明文本区域"""
        ax.axis('off')
        
        # 获取用户说明和测试信息
        user_desc = self.user_description.get().strip() or "无说明"
        
        # 创建说明文本内容
        description_text = f"""用户说明：
{user_desc}

测试配置：
• 外部减速比：{self.reduction_ratio.get()}
• 内部电机参数：{self.internal_motor_param.get()}
• 步长间隔：{self.step_interval.get()}
• 起始位置：{self.start_pos.get()}
• 目标角度范围：{self.target_angle_range.get()}°

连接信息：
• ESP32 IP：{self.esp32_ip.get()}
• 编码器串口：{self.encoder_port.get()}

报告生成时间：
{datetime.now().strftime('%Y年%m月%d日 %H:%M:%S')}"""
        
        # 显示文本
        ax.text(0.05, 0.95, description_text, 
               fontsize=10, 
               verticalalignment='top',
               horizontalalignment='left',
               transform=ax.transAxes,
               bbox=dict(boxstyle="round,pad=0.5", facecolor='#e8f5e8', alpha=0.8))
        
        ax.set_title('测试说明信息', fontweight='bold', pad=20)


class MockVar:
    """模拟tkinter变量"""
    def __init__(self, value):
        self.value = value
    
    def get(self):
        return self.value
    
    def set(self, value):
        self.value = value


def main():
    """主函数 - 测试报告生成功能"""
    print("电机精度测试报告生成功能验证")
    print("=" * 50)
    
    # 创建模拟GUI实例
    mock_gui = MockMotorPrecisionGUI()
    
    print("\n开始生成测试报告...")
    
    # 测试报告生成
    mock_gui.generate_test_report(auto_save=True)
    
    print("\n报告生成测试完成！")
    print("请检查生成的PNG文件是否包含以下内容：")
    print("1. 左上：角度误差曲线图（蓝色线：角度变化误差，红色线：累积误差）")
    print("2. 右上：测试参数表格（绿色表头）")
    print("3. 左下：数据统计汇总表格（蓝色表头）")
    print("4. 右下：用户说明文本区域（绿色背景）")


if __name__ == "__main__":
    main()
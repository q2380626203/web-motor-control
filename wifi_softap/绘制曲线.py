import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams
import os
import glob

# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

class MotorPrecisionAnalyzer:
    """电机精度分析器"""
    
    def __init__(self, reduction_ratio=19.24):
        self.data = None
        self.reduction_ratio = reduction_ratio
        # 计算理论步长：360度/减速比/8 * 0.8 = 每0.8位置值对应的理论角度变化
        self.theoretical_step = (360 / reduction_ratio / 8) * 0.8
        
    def load_latest_csv(self):
        """加载最新的测试结果CSV文件"""
        csv_files = glob.glob("test_results_*.csv")
        if not csv_files:
            raise FileNotFoundError("未找到测试结果CSV文件")
        
        # 获取最新的文件
        latest_file = max(csv_files, key=os.path.getctime)
        print(f"加载数据文件: {latest_file}")
        
        self.data = pd.read_csv(latest_file)
        print(f"数据点数: {len(self.data)}")
        print(f"位置范围: {self.data['position'].min()} 到 {self.data['position'].max()}")
        return latest_file
    
    def load_csv(self, filename):
        """加载指定的CSV文件"""
        if not os.path.exists(filename):
            raise FileNotFoundError(f"文件不存在: {filename}")
        
        print(f"加载数据文件: {filename}")
        self.data = pd.read_csv(filename)
        print(f"数据点数: {len(self.data)}")
        print(f"位置范围: {self.data['position'].min()} 到 {self.data['position'].max()}")
        return filename
    
    def calculate_errors(self):
        """计算角度变化误差"""
        if self.data is None:
            raise ValueError("请先加载数据")
        
        # 对数据按位置排序
        self.data = self.data.sort_values('position').reset_index(drop=True)
        
        # 计算相邻点之间的实际角度变化
        self.data['actual_angle_change'] = self.data['actual_angle'].diff()
        
        # 计算理论角度变化（除了第一个点，其他每个点都应该是理论步长的变化）
        self.data['theoretical_angle_change'] = 0
        self.data.loc[1:, 'theoretical_angle_change'] = self.theoretical_step
        
        # 计算误差（实际变化 - 理论变化）
        self.data['angle_change_error'] = self.data['actual_angle_change'] - self.data['theoretical_angle_change']
        
        # 计算误差百分比（误差除以理论步长，转换为百分比）
        self.data['angle_change_error_percent'] = (self.data['angle_change_error'] / self.theoretical_step) * 100
        
        # 对于第一个点，误差设为0（因为没有前一个点作为参考）
        self.data.loc[0, 'angle_change_error'] = 0
        self.data.loc[0, 'angle_change_error_percent'] = 0
        
        print(f"\n角度变化分析 (减速比: {self.reduction_ratio}, 理论步长: {self.theoretical_step:.5f}°):")
        print(self.data[['position', 'actual_angle', 'actual_angle_change', 'theoretical_angle_change', 'angle_change_error', 'angle_change_error_percent']].to_string())
        
        return self.data
    
    def analyze_specific_range(self, start_pos=32, end_pos=40):
        """分析特定范围的角度变化（如32到40）"""
        if self.data is None:
            raise ValueError("请先加载数据")
        
        # 筛选指定范围的数据
        range_data = self.data[(self.data['position'] >= start_pos) & (self.data['position'] <= end_pos)]
        
        if len(range_data) < 2:
            print(f"警告: 范围 {start_pos} 到 {end_pos} 的数据点不足2个")
            return None
        
        # 计算该范围内的总角度变化
        start_angle = range_data.iloc[0]['actual_angle']
        end_angle = range_data.iloc[-1]['actual_angle']
        actual_total_change = end_angle - start_angle
        
        # 计算理论总变化
        position_change = end_pos - start_pos
        theoretical_total_change = position_change * self.theoretical_step / 0.8
        
        # 计算总误差
        total_error = actual_total_change - theoretical_total_change
        
        print(f"\n=== 范围分析: {start_pos} 到 {end_pos} ===")
        print(f"起始角度: {start_angle:.3f}°")
        print(f"结束角度: {end_angle:.3f}°")
        print(f"实际总变化: {actual_total_change:.3f}°")
        print(f"理论总变化: {theoretical_total_change:.3f}°")
        print(f"总误差: {total_error:.3f}°")
        
        return {
            'start_pos': start_pos,
            'end_pos': end_pos,
            'start_angle': start_angle,
            'end_angle': end_angle,
            'actual_total_change': actual_total_change,
            'theoretical_total_change': theoretical_total_change,
            'total_error': total_error,
            'range_data': range_data
        }
    
    def plot_results(self, save_plot=True):
        """绘制分析结果"""
        if self.data is None:
            raise ValueError("请先加载数据")
        
        # 创建图形
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('电机精度测试分析结果', fontsize=16, fontweight='bold')
        
        # 图1: 位置 vs 实际角度
        ax1.plot(self.data['position'], self.data['actual_angle'], 'b-', linewidth=2)
        ax1.set_xlabel('位置值')
        ax1.set_ylabel('实际角度 (°)')
        ax1.set_title('位置 vs 实际角度')
        ax1.grid(True, alpha=0.3)
        
        # 图2: 角度变化对比
        positions = self.data['position'][1:]  # 跳过第一个点
        actual_changes = self.data['actual_angle_change'][1:]
        theoretical_changes = self.data['theoretical_angle_change'][1:]
        
        x = np.arange(len(positions))
        width = 0.35
        
        ax2.bar(x - width/2, actual_changes, width, label='实际变化', alpha=0.8)
        ax2.bar(x + width/2, theoretical_changes, width, label='理论变化', alpha=0.8)
        ax2.set_xlabel('测试点')
        ax2.set_ylabel('角度变化 (°)')
        ax2.set_title('每步角度变化对比')
        ax2.set_xticks(x)
        ax2.set_xticklabels([f'{pos}' for pos in positions])
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 图3: 角度变化误差
        ax3.plot(self.data['actual_angle'][1:], self.data['angle_change_error'][1:], 'b-', linewidth=2)
        ax3.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax3.set_xlabel('实际角度 (°)')
        ax3.set_ylabel('角度变化误差 (°)')
        ax3.set_title('角度变化与步长的误差值')
        ax3.set_xlim(0, 360)  # x轴固定360度最大
        ax3.set_ylim(-0.01, 0.01)  # y轴固定0.01最大
        ax3.grid(True, alpha=0.3)
        
        # 图4: 累积误差
        cumulative_error = self.data['angle_change_error'].cumsum()
        ax4.plot(self.data['position'], cumulative_error, 'g-', linewidth=2)
        ax4.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        ax4.set_xlabel('位置值')
        ax4.set_ylabel('累积误差 (°)')
        ax4.set_title('累积角度误差')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_plot:
            plot_filename = f"precision_analysis_{pd.Timestamp.now().strftime('%Y%m%d_%H%M%S')}.png"
            plt.savefig(plot_filename, dpi=300, bbox_inches='tight')
            print(f"图表已保存: {plot_filename}")
        
        plt.show()
    
    def print_statistics(self):
        """打印统计信息"""
        if self.data is None:
            raise ValueError("请先加载数据")
        
        # 过滤掉第一个点（因为没有角度变化参考）
        errors = self.data['angle_change_error'][1:]
        errors_percent = self.data['angle_change_error_percent'][1:]
        
        print("\n=== 精度统计分析 ===")
        print(f"测试点数: {len(self.data)}")
        print(f"角度变化步数: {len(errors)}")
        print(f"平均角度变化误差: {errors.mean():.3f}° ({errors_percent.mean():.3f}%)")
        print(f"角度变化误差标准差: {errors.std():.3f}° ({errors_percent.std():.3f}%)")
        print(f"最大正误差: {errors.max():.3f}° ({errors_percent.max():.3f}%)")
        print(f"最大负误差: {errors.min():.3f}° ({errors_percent.min():.3f}%)")
        print(f"误差绝对值平均: {errors.abs().mean():.3f}° ({errors_percent.abs().mean():.3f}%)")
        print(f"误差绝对值最大: {errors.abs().max():.3f}° ({errors_percent.abs().max():.3f}%)")

def main():
    """主函数"""
    print("电机精度测试数据分析程序")
    
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
    
    print(f"使用减速比: {reduction_ratio}")
    theoretical_step = (360 / reduction_ratio / 8) * 0.8
    print(f"理论步长: {theoretical_step:.5f}°")
    
    analyzer = MotorPrecisionAnalyzer(reduction_ratio)
    
    # 选择数据文件
    choice = input("\n选择数据源:\n1. 自动加载最新CSV文件\n2. 手动指定CSV文件\n请输入选择(1/2): ").strip()
    
    try:
        if choice == '2':
            filename = input("请输入CSV文件名: ").strip()
            analyzer.load_csv(filename)
        else:
            analyzer.load_latest_csv()
        
        # 计算误差
        analyzer.calculate_errors()
        
        # 打印统计信息
        analyzer.print_statistics()
        
        # 分析特定范围（例如32到40）
        print("\n" + "="*50)
        start_pos = float(input("请输入分析起始位置 (默认: 32): ").strip() or "32")
        end_pos = float(input("请输入分析结束位置 (默认: 40): ").strip() or "40")
        
        range_analysis = analyzer.analyze_specific_range(start_pos, end_pos)
        
        # 绘制图表
        print("\n生成分析图表...")
        analyzer.plot_results()
        
        print("\n分析完成！")
        
    except Exception as e:
        print(f"错误: {str(e)}")

if __name__ == "__main__":
    main()
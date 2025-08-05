#!/usr/bin/env python3
"""
WiFi电机精度测试系统 - GUI启动器
简化的启动脚本，直接运行图形界面
"""

import sys
import os
import tkinter as tk
from tkinter import messagebox

def check_dependencies():
    """检查必要的依赖"""
    missing_modules = []
    
    try:
        import matplotlib
    except ImportError:
        missing_modules.append('matplotlib')
        
    try:
        import requests
    except ImportError:
        missing_modules.append('requests')
        
    try:
        import serial
    except ImportError:
        missing_modules.append('pyserial')
        
    if missing_modules:
        root = tk.Tk()
        root.withdraw()  # 隐藏主窗口
        messagebox.showerror(
            "缺少依赖",
            f"缺少以下Python模块:\n" + 
            "\n".join(f"- {module}" for module in missing_modules) +
            f"\n\n请使用以下命令安装:\npip install {' '.join(missing_modules)}"
        )
        return False
    
    return True

def main():
    """主函数"""
    print("=" * 50)
    print("    WiFi电机精度测试系统 v1.0")
    print("    成都双创时代科技有限公司")
    print("=" * 50)
    
    # 检查依赖
    if not check_dependencies():
        return
    
    try:
        # 导入并启动GUI
        from motor_precision_gui import main as gui_main
        gui_main()
        
    except ImportError as e:
        print(f"导入错误: {str(e)}")
        print("请确保所有文件都在同一目录下")
        input("按回车键退出...")
        
    except Exception as e:
        print(f"启动失败: {str(e)}")
        input("按回车键退出...")

if __name__ == "__main__":
    main()
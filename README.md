# Web电机控制系统

基于ESP32的Web电机控制系统，通过WiFi SoftAP提供Web界面控制电机运动，专为电机精度测试设计。

## 🚀 功能特性

- **Web界面控制** - 通过浏览器直接控制电机
- **G代码CAN控制** - 支持标准G代码命令通过CAN总线控制电机
- **三种控制模式** - 位置模式、速度模式、力矩模式自动切换
- **实时UART监听** - 监听电机响应和G代码CAN帧
- **CAN总线通信** - 使用UART转CAN与电机驱动器通信
- **WiFi热点模式** - ESP32作为AP，无需外部网络
- **容错机制** - 支持分包传输和数据清理，确保稳定性
- **精度测试优化** - 专为电机精度测试场景设计

## 📋 系统架构

```
[Web浏览器] <--WiFi--> [ESP32 SoftAP] <--UART/CAN--> [电机驱动器] <--> [电机]
                           ↑
                    [G代码CAN控制]
                    (00 01 + ASCII)
```

## 🔧 硬件要求

- **ESP32开发板** (ESP32/ESP32-S2/ESP32-S3/ESP32-C3等)
- **电机驱动器** (支持CAN总线通信)
- **步进/伺服电机**
- **减速器** (当前配置: 8.0位置值 = 18.711019度)

## 📱 控制方式

### Web界面功能
- **角度控制** - 直接输入角度值(-180°~180°)
- **位置控制** - 输入位置值(-100~100)
- **电机状态** - 使能/失能电机
- **错误清除** - 清除电机驱动器错误

### G代码CAN控制
- **G1 X{角度}** - 位置模式，设置目标角度（例：G1 X15, G1 X-45）
- **G1 F{速度}** - 速度模式，设置转速r/s（例：G1 F1.5, G1 F-2.0）
- **G1 T{力矩}** - 力矩模式，设置力矩Nm（例：G1 T0.5, G1 T-1.2）
- **M1** - 使能电机
- **M0** - 失能电机

### CAN帧格式
- **CAN ID**: `00 01` (G代码指令)
- **数据**: 直接ASCII字符（如：G1X10）
- **电机响应**: CAN ID `00 i0` (i≠0)

## ⚙️ 配置说明

### WiFi设置
```c
#define EXAMPLE_ESP_WIFI_SSID      "ESP32-Motor-Control"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
```

### 电机参数
```c
// 减速机换算关系: 8.0 位置值 = 18.711019 度
#define POSITION_TO_DEGREE_RATIO    (18.711019f / 8.0f)
#define DEGREE_TO_POSITION_RATIO    (8.0f / 18.711019f)
```

### UART配置
- **波特率**: 115200
- **数据位**: 8
- **停止位**: 1
- **校验位**: 无

## 🛠️ 编译和烧录

### 环境准备
```bash
# 安装ESP-IDF
git clone https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

### 编译项目
```bash
cd web电机控制
idf.py build
```

### 烧录到ESP32
```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

## 🌐 使用方法

1. **烧录固件**到ESP32开发板
2. **连接WiFi** - 搜索并连接"ESP32-Motor-Control"热点，密码"12345678"
3. **打开浏览器** - 访问 `http://192.168.4.1`
4. **控制电机** - 通过Web界面输入角度或位置值

## 📡 API接口

| 接口 | 方法 | 功能 |
|------|------|------|
| `/` | GET | 主页面 |
| `/set_angle?value=度数` | GET | 设置电机角度 |
| `/set_position?value=位置值` | GET | 设置电机位置 |
| `/enable` | GET | 使能电机 |
| `/disable` | GET | 失能电机 |
| `/clear` | GET | 清除错误 |

## 🔧 CAN指令协议

| 功能 | CAN ID | 数据 |
|------|--------|------|
| 使能电机 | 0x0027 | [0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00] |
| 失能电机 | 0x0027 | [0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00] |
| 位置模式 | 0x002B | [0x03,0x00,0x00,0x00,0x03,0x00,0x00,0x00] |
| 目标位置 | 0x002C | [position(float)] |
| 速度模式 | 0x002B | [0x02,0x00,0x00,0x00,0x01,0x00,0x00,0x00] |
| 目标速度 | 0x002D | [velocity(float)] |

## 📊 技术规格

- **控制精度**: 根据减速比可达0.1度精度
- **响应时间**: <100ms
- **通信协议**: WiFi 802.11 b/g/n + UART/CAN
- **工作电压**: 3.3V (ESP32)
- **功耗**: <500mA (不含电机)

## 🐛 故障排除

### 连接问题
- 确认ESP32正常启动，串口监视器显示WiFi热点已创建
- 检查WiFi密码是否正确
- 确认设备在ESP32的WiFi覆盖范围内

### 电机不响应
- 检查UART连接线是否正确
- 确认电机驱动器电源和参数配置
- 查看串口监视器的CAN指令发送日志

### Web界面异常
- 清除浏览器缓存后重试
- 确认IP地址 192.168.4.1 可访问
- 检查ESP32串口日志是否有HTTP服务错误

## 📁 项目结构

```
web电机控制/
├── main/
│   ├── main.c                   # 主程序和电机初始化
│   ├── motor_control.c          # 电机控制核心模块  
│   ├── motor_control.h          # 电机控制头文件
│   ├── gcode_unified_control.c  # G代码解析和控制
│   ├── gcode_unified_control.h  # G代码控制头文件
│   ├── uart_monitor.c           # UART数据监听和CAN帧处理
│   ├── uart_monitor.h           # UART监听头文件
│   ├── wifi_http_server.c       # WiFi和HTTP服务器
│   ├── web_interface.c          # Web界面HTML生成
│   └── CMakeLists.txt          # 构建配置
├── CMakeLists.txt              # 项目构建配置
├── sdkconfig                   # ESP-IDF配置
└── README.md                   # 项目说明文档
```

## 🔄 更新日志

- **v1.4** (2025-01-06) - 新增G代码CAN控制系统，支持三种电机控制模式，增强容错机制
- **v1.3** (2025-01-XX) - 清理项目结构，移除Python GUI组件，专注Web控制
- **v1.2** (2025-01-XX) - 完成电机精度测试系统全面改进
- **v1.1** (2025-01-XX) - 实现测试报告生成功能  
- **v1.0** (2025-01-XX) - 初始提交：电机精度测试系统

## 📄 开源协议

本项目基于 MIT 协议开源，详见 [LICENSE](LICENSE) 文件。

## 🤝 贡献指南

欢迎提交Issue和Pull Request来完善这个项目！

---

**⚡ 专为电机精度测试而设计的Web控制系统**
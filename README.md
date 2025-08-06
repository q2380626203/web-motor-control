# ESP32 Web与CAN电机控制系统

基于ESP32的智能电机控制系统，集成Web界面控制和G代码CAN总线通信，专为高精度电机测试与控制而设计。

## 🚀 核心功能

- **双控制接口** - Web浏览器控制 + G代码CAN总线控制
- **三种精确控制模式** - 位置控制、速度控制、力矩控制
- **实时数据监控** - UART电机响应监听 + CAN帧数据监听
- **智能减速比计算** - 支持19.2158倍减速器的精确角度转换
- **WiFi热点部署** - 独立运行，无需外部网络依赖
- **容错与稳定性** - 分包传输、数据清理、异常恢复机制

## 🏗️ 系统架构

```
                    WiFi热点控制路径
[Web浏览器] <--WiFi--> [ESP32 SoftAP] <--UART--> [电机驱动器] <--> [电机]
                           |                           ↑
                           |                    (485/CAN转换)
                    CAN总线控制路径
[G代码发送器] <--CAN--> [ESP32 CAN接收] 
                           |
                    [G代码解析器] 
                    (支持位置/速度/力矩模式)
```

## 🔧 硬件配置

### 必需硬件
- **ESP32开发板** (推荐ESP32-WROOM-32)
- **电机驱动器** (支持UART/485通信，带CAN转换功能)
- **伺服电机** (支持位置/速度/力矩控制)
- **减速器** (当前配置: 19.2158:1减速比)

### 引脚连接
```
ESP32引脚配置:
├── UART1 (电机通信)
│   ├── GPIO12 - RXD (接收电机响应)
│   └── GPIO13 - TXD (发送电机指令)
├── CAN总线 (G代码接收)
│   ├── GPIO1  - CAN_TX
│   └── GPIO2  - CAN_RX
└── 电源: 3.3V/5V供电
```

## 📱 双模式控制

### 🌐 Web界面控制
- **精确角度设置** - 输入目标角度值(0-360°)
- **位置值控制** - 直接设置电机位置(-100~100)
- **实时电机状态** - 使能/失能控制
- **故障诊断** - 一键清除驱动器错误
- **监控界面** - 实时显示电机响应数据

### ⚡ G代码CAN控制
支持标准G代码指令，通过CAN总线实时控制：

#### 位置控制模式
- `G1 X{角度}` - 绝对位置控制 (例: `G1 X90`, `G1 X-45`)
- 自动换算减速比，精确定位

#### 速度控制模式  
- `G1 F{转速}` - 设置转速r/s (例: `G1 F1.5`, `G1 F-2.0`)
- 支持正反转，平滑加减速

#### 力矩控制模式
- `G1 T{力矩}` - 设置输出力矩Nm (例: `G1 T0.8`, `G1 T-1.2`) 
- 智能力矩转换: 30Nm外部 → 11Nm内部

#### 电机使能控制
- `M1` - 使能电机 
- `M0` - 失能电机

### 📡 CAN通信协议
```
CAN帧格式:
├── G代码指令帧
│   ├── CAN ID: 0x0001 
│   └── 数据: ASCII字符串 (如: "G1X45")
└── 电机响应帧
    ├── CAN ID: 0x00i0 (i≠0)
    └── 数据: 电机状态/位置反馈
```

## ⚙️ 系统配置

### WiFi热点设置
```c
// WiFi热点配置 (通过sdkconfig配置)
CONFIG_ESP_WIFI_SSID="myssid"              // 默认热点名称
CONFIG_ESP_WIFI_PASSWORD="mypassword"      // 默认密码
CONFIG_ESP_WIFI_CHANNEL=1                  // WiFi信道
CONFIG_EXAMPLE_MAX_STA_CONN=4              // 最大连接数
```

### 减速器参数配置
```c
// 减速比配置 (main/main.c)
#define GEAR_RATIO 19.2158f                // 外部减速比
#define ANGLE_TO_POSITION_SCALE 8.0f       // 内部位置换算: 0-8 对应 0-360°

// 力矩转换系数
#define TORQUE_CONVERSION_FACTOR 0.3667f   // 30Nm外部 -> 11Nm内部
```

### 通信参数
```c
// UART配置 (电机通信)
UART_PORT: UART_NUM_1
BAUD_RATE: 115200
DATA_BITS: 8
STOP_BITS: 1  
PARITY: NONE

// CAN配置 (G代码接收)
TIMING: TWAI_TIMING_CONFIG_500KBITS()
FILTER: TWAI_FILTER_CONFIG_ACCEPT_ALL()
```

## 🛠️ 快速部署

### 环境搭建
```bash
# 1. 安装ESP-IDF v5.0+
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && git checkout release/v5.1
./install.sh esp32
source ./export.sh

# 2. 验证安装
idf.py --version
```

### 编译与烧录
```bash
# 1. 进入项目目录  
cd web和can控制

# 2. 配置目标芯片
idf.py set-target esp32

# 3. 编译项目
idf.py build

# 4. 烧录并监控 (替换为实际串口)
idf.py -p /dev/ttyUSB0 flash monitor
```

### 快速验证
```bash
# 监控启动日志，确认以下信息:
# ✓ WiFi热点: ESP32-Motor-Control 已创建
# ✓ Web服务器: http://192.168.4.1 已启动
# ✓ UART1监听: 电机响应监听已开启
# ✓ CAN监听: G代码接收已就绪
```

## 🌐 操作指南

### Web界面控制
1. **连接WiFi热点** - "myssid" (密码: mypassword)
2. **打开控制面板** - 浏览器访问 `http://192.168.4.1` 
3. **选择控制模式**:
   - 角度控制: 输入0-360°角度值
   - 位置控制: 输入-100~100位置值
   - 电机使能: 启用/禁用电机
4. **监控反馈** - 实时查看电机状态和响应数据

### G代码CAN控制
```bash
# CAN帧发送示例 (使用CAN工具)
# 1. 位置控制: 旋转到90度
CAN ID: 0x0001, Data: "G1X90"

# 2. 速度控制: 1.5r/s转速
CAN ID: 0x0001, Data: "G1F1.5" 

# 3. 力矩控制: 0.8Nm力矩
CAN ID: 0x0001, Data: "G1T0.8"

# 4. 电机使能/失能
CAN ID: 0x0001, Data: "M1"  # 使能
CAN ID: 0x0001, Data: "M0"  # 失能
```

## 📡 HTTP API接口

### Web控制接口
| 端点 | 方法 | 参数 | 功能描述 |
|------|------|------|----------|
| `/` | GET | - | 主控制页面 |
| `/set_angle` | GET | `value={角度}` | 设置电机绝对角度 (0-360°) |
| `/set_position` | GET | `value={位置}` | 设置电机位置值 (-100~100) |
| `/set_velocity` | GET | `value={速度}` | 设置电机转速 (r/s) |
| `/set_torque` | GET | `value={力矩}` | 设置电机输出力矩 (Nm) |
| `/set_mode` | GET | `mode={模式}` | 切换控制模式 (position/velocity/torque) |
| `/enable` | GET | - | 使能电机驱动 |
| `/disable` | GET | - | 失能电机驱动 |
| `/clear` | GET | - | 清除驱动器故障 |

### 示例API调用
```bash
# 角度控制
curl "http://192.168.4.1/set_angle?value=90"

# 位置控制  
curl "http://192.168.4.1/set_position?value=50"

# 速度控制
curl "http://192.168.4.1/set_velocity?value=1.5"

# 力矩控制
curl "http://192.168.4.1/set_torque?value=0.8"

# 切换控制模式
curl "http://192.168.4.1/set_mode?mode=velocity"

# 电机使能控制
curl "http://192.168.4.1/enable"
curl "http://192.168.4.1/disable"
```

## 🔧 底层CAN协议

### 电机驱动器指令 (ESP32 → 驱动器)
| 功能 | CAN ID | 数据格式 | 说明 |
|------|--------|----------|------|
| 使能电机 | 0x0027 | `[0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00]` | 启动电机驱动 |
| 失能电机 | 0x0027 | `[0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00]` | 停止电机驱动 |
| 位置模式 | 0x002B | `[0x03,0x00,0x00,0x00,0x03,0x00,0x00,0x00]` | 切换到位置控制 |
| 速度模式 | 0x002B | `[0x02,0x00,0x00,0x00,0x01,0x00,0x00,0x00]` | 切换到速度控制 |
| 力矩模式 | 0x002B | `[0x04,0x00,0x00,0x00,0x04,0x00,0x00,0x00]` | 切换到力矩控制 |
| 目标位置 | 0x002C | `[position_float_bytes]` | 设置目标位置(float) |
| 目标速度 | 0x002D | `[velocity_float_bytes]` | 设置目标速度(float) |
| 目标力矩 | 0x002E | `[torque_float_bytes]` | 设置目标力矩(float) |

## 📊 技术规格

### 性能参数
- **角度控制精度**: ±0.1° (基于19.2158:1减速比)
- **位置重复精度**: ±0.05° 
- **响应时间**: <50ms (WiFi) / <10ms (CAN)
- **最大转速**: 50 r/s (输出轴)
- **最大力矩**: 30 Nm (输出轴)

### 系统规格  
- **控制器**: ESP32-WROOM-32 (240MHz双核)
- **内存**: 520KB SRAM + 4MB Flash
- **通信**: WiFi 802.11 b/g/n + CAN 2.0B + UART
- **工作电压**: 3.3V (ESP32) + 24V (电机驱动)
- **功耗**: ESP32 <200mA, 系统总计<1A

## 🐛 故障排除

### WiFi连接问题
```bash
# 1. 确认ESP32启动状态
idf.py monitor  # 查看启动日志

# 应显示以下信息:
# I (xxx) WIFI: WiFi SoftAP started
# I (xxx) HTTP_SERVER: Starting server on port: 80
# I (xxx) MAIN: 请连接WiFi热点，然后访问: http://192.168.4.1
```

**解决方法**:
- 检查ESP32电源是否稳定
- 确认WiFi热点名: `myssid`, 密码: `mypassword`  
- 重启路由器WiFi功能，避免信道冲突
- 尝试手机热点测试连接
- 可通过`idf.py menuconfig`修改WiFi配置

### 电机无响应
```bash  
# 检查UART通信状态
# 正常情况应显示:
# I (xxx) UART监听: 接收数据: [电机响应数据]
```

**诊断步骤**:
1. **检查硬件连接**
   - GPIO12(RXD) ↔ 驱动器TX
   - GPIO13(TXD) ↔ 驱动器RX  
   - 确认共地连接
2. **验证驱动器状态**
   - 驱动器电源指示灯
   - 参数配置是否匹配
   - 使能状态检查
3. **UART通信测试**
   - 波特率115200确认
   - 发送测试指令观察响应

### CAN通信异常
```bash
# CAN监听正常应显示:
# I (xxx) CAN监听: 接收G代码: G1X90
# I (xxx) G代码控制: 执行角度控制: 90.000000度
```

**解决方案**:
- 确认CAN总线终端电阻(120Ω)
- 检查CAN_TX(GPIO1)和CAN_RX(GPIO2)连线
- 验证CAN波特率500K设置
- 使用CAN分析仪验证帧格式

### Web界面异常
- **页面无法加载**: 清除浏览器缓存，尝试Chrome隐私模式
- **控制无反应**: F12查看Network错误，检查API返回状态  
- **显示异常**: 确认浏览器支持现代JavaScript

## 📁 代码架构

### 项目文件结构
```
web和can控制/
├── 📂 main/                        # 主程序目录
│   ├── main.c                     # 🚀 系统入口 & 任务调度
│   ├── motor_control.c/.h         # ⚙️  电机控制核心 (UART通信)
│   ├── gcode_unified_control.c/.h # 📝 G代码解析器 (三模式控制)
│   ├── can_monitor.c/.h           # 📡 CAN总线监听器 (新增)
│   ├── uart_monitor.c/.h          # 🔍 UART数据监听器
│   ├── wifi_http_server.c/.h      # 🌐 WiFi热点 + HTTP服务
│   ├── web_interface.c/.h         # 🎨 Web界面HTML生成
│   └── CMakeLists.txt             # 构建配置
├── CMakeLists.txt                 # 项目构建配置
├── sdkconfig                      # ESP-IDF系统配置
└── README.md                      # 项目说明文档
```

### 核心模块说明
- **main.c**: 系统初始化、任务创建、减速比计算
- **motor_control**: 电机驱动器通信、模式切换、参数设置
- **gcode_unified_control**: G代码解析、三种控制模式统一处理  
- **can_monitor**: CAN总线数据接收、G代码指令分发
- **uart_monitor**: 电机响应数据监听、状态反馈
- **wifi_http_server**: SoftAP创建、HTTP服务器、RESTful API
- **web_interface**: 动态HTML生成、前端控制界面

## 📈 版本历史

| 版本 | 日期 | 主要更新 |
|------|------|----------|
| **v1.4** | 2025-01-06 | ✨ 新增完整CAN总线G代码控制系统 |
| | | ⚡ 三种电机控制模式统一架构 |
| | | 🛡️ 增强容错机制和数据清理 |
| **v1.3** | 2024-12-XX | 🧹 项目结构清理，专注Web + CAN控制 |
| **v1.2** | 2024-12-XX | 🎯 电机精度测试系统全面优化 |
| **v1.1** | 2024-11-XX | 📊 测试报告生成功能实现 |
| **v1.0** | 2024-10-XX | 🎉 初始版本：基础Web电机控制 |

## 🔮 开发规划

### 近期计划 (v1.5)
- [ ] 增加电机状态实时反馈显示
- [ ] 优化CAN通信错误处理
- [ ] 增加电机参数自动检测

### 中期计划 (v2.0) 
- [ ] 支持多电机协调控制
- [ ] 增加数据记录与分析功能
- [ ] 移动端App开发

## 📄 开源协议

本项目采用 **MIT License** 开源协议，允许商业和非商业使用。

## 🤝 参与贡献

### 贡献方式
1. **🐛 Bug报告**: [Issues页面](../../issues) 提交问题
2. **💡 功能建议**: 提交Feature Request  
3. **🔧 代码贡献**: Fork → 修改 → Pull Request
4. **📖 文档改进**: 完善README或添加技术文档

### 联系方式
- **项目维护**: [@yaui](../../)
- **技术讨论**: 项目Issues或Discussions
- **应用案例**: 欢迎分享实际应用场景

---

**⚡ 高精度电机控制，Web + CAN双模驱动** | **🚀 专业级ESP32解决方案**
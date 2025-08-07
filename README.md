# ESP32电机控制系统

基于ESP32的电机控制系统，支持Web界面和CAN总线G代码控制。

## 核心功能

- **Web控制**: 通过WiFi热点访问192.168.4.1控制电机
- **CAN控制**: 支持标准G代码命令通过CAN总线控制
- **三种模式**: 位置控制、速度控制、力矩控制
- **实时监控**: UART和CAN数据监听

## 硬件连接

```
ESP32引脚:
├── UART1 (电机通信)
│   ├── GPIO12 - RXD 
│   └── GPIO13 - TXD
├── CAN总线 (G代码)
│   ├── GPIO1 - CAN_TX
│   └── GPIO2 - CAN_RX
└── 电源: 3.3V供电
```

## 快速使用

### 编译烧录
```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### Web控制
1. 连接WiFi热点 "myssid" (密码: mypassword)
2. 浏览器打开 http://192.168.4.1
3. 设置角度/位置/速度/力矩参数

### G代码控制 (CAN)
- `G1 X90` - 位置控制，转到90度
- `G1 F1.5` - 速度控制，1.5r/s
- `G1 T0.8` - 力矩控制，0.8Nm
- `M1`/`M0` - 使能/失能电机

## 系统配置

- **减速比**: 19.2158:1
- **UART波特率**: 115200
- **CAN波特率**: 500K
- **WiFi热点**: 192.168.4.1

## 故障排除

- Web无法访问: 检查WiFi连接和ESP32启动日志
- 电机无响应: 检查GPIO12/13连线和驱动器电源
- CAN无数据: 确认GPIO1/2连线和终端电阻

## 项目结构

```
main/
├── main.c              # 系统入口
├── motor_control.c/h   # 电机控制
├── gcode_unified_control.c/h  # G代码解析
├── can_monitor.c/h     # CAN监听
├── wifi_http_server.c/h # Web服务器
└── web_interface.c/h   # Web界面
```
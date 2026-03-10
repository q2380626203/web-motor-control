#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "motor_control.h"
#include "wifi_http_server.h"
#include "uart_monitor.h"
#include "can_monitor.h"
#include "gcode_unified_control.h"
#include "motor_status_scheduler.h"

// W5500 以太网相关头文件
#include "esp_eth.h"
#include "esp_eth_mac.h"
#include "esp_eth_phy.h"
#include "esp_eth_com.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

// UDP测试相关头文件
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "fr_robot_protocol.h"

// 函数声明
float angle_to_position(float angle_degrees);
float external_velocity_to_internal(float external_velocity);
float external_torque_to_internal(float external_torque);

static const char *TAG = "MAIN";

// W5500 SPI 引脚定义
#define W5500_SPI_HOST      SPI2_HOST
#define W5500_CS_GPIO       GPIO_NUM_4
#define W5500_SCLK_GPIO     GPIO_NUM_3
#define W5500_MISO_GPIO     GPIO_NUM_1  // 恢复原始配置
#define W5500_MOSI_GPIO     GPIO_NUM_2  // 恢复原始配置
#define W5500_INT_GPIO      -1  // 不使用中断引脚，使用轮询模式
#define W5500_PHY_RST_GPIO  -1  // 不使用复位引脚
#define W5500_PHY_ADDR      1
#define W5500_POLL_MS       100 // 轮询周期 100ms

// 全局变量
static motor_controller_t* motor_controller = NULL;  // 主控制器（用于Web界面）
static motor_controller_t* motor_controller_1 = NULL; // 电机1控制器
static motor_controller_t* motor_controller_4 = NULL; // 电机4控制器
static httpd_handle_t web_server = NULL;
static uart_monitor_t* uart_monitor = NULL;
static can_monitor_t* can_monitor = NULL;
static motor_status_scheduler_t* status_scheduler = NULL;
gcode_controller_t* g_gcode_controller = NULL; // G代码控制器（供CAN监听使用）
static char gcode_response_buffer[512]; // G代码响应缓冲区

// 角度映射参数
#define GEAR_RATIO 19.2158f          // 外部减速比
#define ANGLE_TO_POSITION_SCALE 8.0f // 0-8对应0-360度

/**
 * @brief 角度转换为电机位置值
 * @param angle_degrees 输入角度(度) - 外部输出轴角度，支持范围 -360 到 360 度（含边界）
 * @return 电机位置值 - 内部电机需要的位置值
 */
float angle_to_position(float angle_degrees) {
    // 限制角度范围在 -360 到 360 度之间（包含边界值）
    if (angle_degrees < -360.0f) {
        angle_degrees = -360.0f;
    } else if (angle_degrees > 360.0f) {
        angle_degrees = 360.0f;
    }

    // 直接转换角度（支持负值，表示反向旋转）
    // 外部转angle_degrees度，内部需要转 angle_degrees * (减速比/360度)
    float internal_rotations = (angle_degrees / 360.0f) * GEAR_RATIO;

    // 内部转换为位置值：每转1圈对应位置值8
    float motor_position = internal_rotations * ANGLE_TO_POSITION_SCALE;

    return motor_position;
}

/**
 * @brief 外部速度转换为内部电机速度
 * @param external_velocity 外部期望速度 (r/s) - 输出轴转速
 * @return 内部电机需要的速度 (r/s)
 */
float external_velocity_to_internal(float external_velocity) {
    // 外部转1 r/s，内部需要转 减速比 r/s
    return external_velocity * GEAR_RATIO;
}

/**
 * @brief 外部力矩转换为内部电机力矩
 * @param external_torque 外部期望力矩 (Nm) - 输出轴力矩
 * @return 内部电机需要的力矩 (Nm)
 */
float external_torque_to_internal(float external_torque) {
    // 力矩转换系数：30Nm外部 -> 11Nm内部
    // 转换系数 = 11/30 = 0.3667
    return external_torque * 0.3667f;
}

// ========== W5500 以太网初始化函数 ==========

#if CONFIG_ETH_SPI_ETHERNET_W5500
/**
 * @brief 初始化 W5500 以太网
 * @return esp_eth_handle_t 以太网句柄，失败返回 NULL
 */
static esp_eth_handle_t w5500_eth_init(void)
{
    esp_eth_handle_t eth_handle = NULL;

    // 配置 SPI 总线
    spi_bus_config_t buscfg = {
        .miso_io_num = W5500_MISO_GPIO,
        .mosi_io_num = W5500_MOSI_GPIO,
        .sclk_io_num = W5500_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    ESP_LOGI(TAG, "初始化 SPI 总线: MISO=%d, MOSI=%d, SCLK=%d",
             W5500_MISO_GPIO, W5500_MOSI_GPIO, W5500_SCLK_GPIO);

    ESP_ERROR_CHECK(spi_bus_initialize(W5500_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // 配置 SPI 设备接口
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = 2 * 1000 * 1000,  // 2 MHz (降低速度以提高稳定性)
        .queue_size = 20,
        .spics_io_num = W5500_CS_GPIO
    };

    ESP_LOGI(TAG, "配置 W5500 SPI 设备: CS=%d, 时钟=2MHz", W5500_CS_GPIO);

    // 初始化 MAC 和 PHY 配置
    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = W5500_PHY_ADDR;
    phy_config.reset_gpio_num = W5500_PHY_RST_GPIO;

    // 配置 W5500
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(W5500_SPI_HOST, &devcfg);
    w5500_config.int_gpio_num = W5500_INT_GPIO;
    w5500_config.poll_period_ms = W5500_POLL_MS;

    ESP_LOGI(TAG, "创建 W5500 MAC 和 PHY 实例");

    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    // 安装以太网驱动
    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

    // 设置 MAC 地址（可选）
    uint8_t mac_addr[6] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
    ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, mac_addr));

    ESP_LOGI(TAG, "W5500 以太网驱动初始化成功");

    return eth_handle;
}
#endif // CONFIG_ETH_SPI_ETHERNET_W5500

// ========== W5500 以太网事件处理函数 ==========

/** W5500 以太网事件处理 */
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "W5500 以太网连接成功");
        ESP_LOGI(TAG, "W5500 MAC地址: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "W5500 以太网断开连接");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "W5500 以太网已启动");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "W5500 以太网已停止");
        break;
    default:
        break;
    }
}

/** IP地址获取事件处理 */
static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "W5500 获取到IP地址");
    ESP_LOGI(TAG, "~~~~~~~~~~~");
    ESP_LOGI(TAG, "IP地址:" IPSTR, IP2STR(&ip_info->ip));
    ESP_LOGI(TAG, "子网掩码:" IPSTR, IP2STR(&ip_info->netmask));
    ESP_LOGI(TAG, "网关:" IPSTR, IP2STR(&ip_info->gw));
    ESP_LOGI(TAG, "~~~~~~~~~~~");
}

// ========== 法奥机器人协议 UDP 服务 ==========
#define FR_UDP_PORT 8211
static esp_netif_t *g_eth_netif = NULL;

// 客户端地址（机器人）
static struct sockaddr_in g_robot_addr;
static bool g_robot_connected = false;

// 运动更新任务周期 (ms)
#define MOTION_UPDATE_MS 10

static void fr_robot_server_task(void *pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "无法创建 UDP socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(FR_UDP_PORT);

    if (bind(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        ESP_LOGE(TAG, "UDP socket 绑定失败");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "法奥机器人协议服务器启动，监听端口：%d", FR_UDP_PORT);

    // 接收缓冲区
    uint8_t rx_buffer[FR_ROBOT_PACKET_SIZE];
    socklen_t client_addr_len = sizeof(g_robot_addr);

    // 帧计数
    uint16_t frame_count = 0;

    // 初始化协议模块（motor绑定在motor_init_task完成后调用）
    fr_protocol_init();

    while (1) {
        // 接收机器人命令
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0,
                          (struct sockaddr *)&g_robot_addr, &client_addr_len);
        if (len > 0) {
            g_robot_connected = true;

            // 解析命令包
            fr_robot_packet_t cmd_packet;
            if (fr_parse_robot_packet(rx_buffer, len, &cmd_packet)) {
                ESP_LOGD(TAG, "收到机器人命令包，帧计数：%d", cmd_packet.frame_count);

                // 处理各轴命令
                for (int i = 0; i < FR_AXIS_COUNT; i++) {
                    fr_process_axis_command(&cmd_packet.axis_cmd[i], i);
                }

                // 构建 PLC 响应包
                fr_plc_packet_t resp_packet;
                frame_count++;
                fr_build_plc_response(&resp_packet, frame_count);

                // 发送响应
                sendto(sock, (uint8_t*)&resp_packet, FR_PLC_PACKET_SIZE, 0,
                       (struct sockaddr *)&g_robot_addr, client_addr_len);

                ESP_LOGD(TAG, "已发送响应包，帧计数：%d", frame_count);
            } else {
                ESP_LOGW(TAG, "解析机器人命令包失败，长度：%d", len);
            }
        }

        // 定期更新运动状态（每 10ms）
        fr_update_motion(MOTION_UPDATE_MS);
        vTaskDelay(pdMS_TO_TICKS(MOTION_UPDATE_MS));
    }

    close(sock);
    vTaskDelete(NULL);
}

// 电机初始化任务
void motor_init_task(void *pvParameters) {
    // ========== 初始化电机1控制器 ==========
    motor_driver_config_t motor1_config = {
        .tx_pin = GPIO_NUM_11,   // CAN TX引脚（与CAN监听器使用相同引脚）
        .rx_pin = GPIO_NUM_10,   // CAN RX引脚
        .motor_id = 1            // 电机ID 1
    };

    motor_controller_1 = motor_control_init(&motor1_config);
    if (!motor_controller_1) {
        ESP_LOGE(TAG, "电机1控制器初始化失败");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "电机1控制器初始化成功");

    // ========== 初始化电机4控制器 ==========
    motor_driver_config_t motor4_config = {
        .tx_pin = GPIO_NUM_11,   // CAN TX引脚（共享CAN总线）
        .rx_pin = GPIO_NUM_10,   // CAN RX引脚
        .motor_id = 4            // 电机ID 4
    };

    motor_controller_4 = motor_control_init(&motor4_config);
    if (!motor_controller_4) {
        ESP_LOGE(TAG, "电机4控制器初始化失败");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "电机4控制器初始化成功");

    // 设置主控制器为电机1（用于Web界面）
    motor_controller = motor_controller_1;

    // 绑定电机1到法奥协议轴1（等电机初始化完成后才能绑定）
    fr_protocol_set_motor(motor_controller_1);

    // 等待2秒让电机稳定
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 暂时不设置位置模式，后面会设置速度模式
    // motor_control_set_position_mode(motor_controller);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    
    // 启动Web服务器
    web_server = start_webserver(motor_controller);
    if (!web_server) {
        ESP_LOGE(TAG, "Web服务器启动失败");
    }
    
    // 初始化状态查询调度器（使用电机1）
    scheduler_config_t scheduler_config = {
        .frequency = 1.0f,               // 默认1Hz查询频率
        // .uart_port = UART_NUM_1,      // 注释掉：不再需要UART，使用TWAI
        .motor_id = motor1_config.motor_id,  // 使用电机1的ID
        .enable_all_queries = false      // 默认不启动自动查询，等待用户手动启动
    };
    
    status_scheduler = motor_status_scheduler_init(&scheduler_config);
    if (status_scheduler) {
        ESP_LOGI(TAG, "状态查询调度器初始化成功");
        // 将调度器传递给Web服务器
        set_status_scheduler(status_scheduler);
    } else {
        ESP_LOGE(TAG, "状态查询调度器初始化失败");
    }
    
    // 初始化G代码控制器
    gcode_controller_config_t gcode_config = {
        .motor_controller = motor_controller,
        .response_buffer = gcode_response_buffer,
        .response_buffer_size = sizeof(gcode_response_buffer)
    };
    
    g_gcode_controller = gcode_controller_init(&gcode_config);
    if (g_gcode_controller) {
        ESP_LOGI(TAG, "G代码控制器初始化成功");
    } else {
        ESP_LOGE(TAG, "G代码控制器初始化失败");
    }

    // 初始化并启动TWAI电机数据监听器（专门监听电机响应数据）
    // 该监听器复用motor_control已初始化的TWAI驱动，不会重复初始化
    uart_monitor_config_t uart_config = {
        .tx_gpio = GPIO_NUM_11,         // CAN TX引脚（与motor_control相同）
        .rx_gpio = GPIO_NUM_10,         // CAN RX引脚
        .tag = "电机响应监听"             // 日志标签
    };

    uart_monitor = uart_monitor_init(&uart_config);
    if (uart_monitor) {
        if (uart_monitor_start(uart_monitor)) {
            ESP_LOGI(TAG, "TWAI电机数据监听器启动成功");
        } else {
            ESP_LOGE(TAG, "TWAI电机数据监听器启动失败");
        }
    } else {
        ESP_LOGE(TAG, "TWAI电机数据监听器初始化失败");
    }

    // 注意：电机响应现在通过CAN总线接收，由uart_monitor（实际使用TWAI）处理

    // /* 注释掉CAN监听器 - TWAI已经被motor_control初始化，避免重复初始化
    // 如果需要监听CAN数据，应该修改can_monitor使其不重新初始化TWAI驱动
    // 初始化并启动CAN监听器（专门监听G代码CAN数据）
    // can_monitor_config_t can_config = {
    //     .tx_gpio = GPIO_NUM_13,               // CAN TX引脚
    //     .rx_gpio = GPIO_NUM_12,               // CAN RX引脚
    //     .timing_config = TWAI_TIMING_CONFIG_500KBITS(), // 500K波特率
    //     .filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(), // 接收所有消息
    //     .tag = "CAN监听",                    // 日志标签
    //     .gcode_controller = g_gcode_controller // G代码控制器
    // };
    //
    // can_monitor = can_monitor_init(&can_config);
    // if (can_monitor) {
    //     if (can_monitor_start(can_monitor)) {
    //         ESP_LOGI(TAG, "CAN数据监听器启动成功");
    //     } else {
    //         ESP_LOGE(TAG, "CAN数据监听器启动失败");
    //     }
    // } else {
    //     ESP_LOGE(TAG, "CAN数据监听器初始化失败");
    // }
    // */

    ESP_LOGI(TAG, "注意：TWAI已由motor_control模块初始化并管理");
    
    ESP_LOGI(TAG, "电机初始化完成，Web服务器已启动，G代码控制器已就绪");
    ESP_LOGI(TAG, "当前控制电机ID: 1 和 4");
    ESP_LOGI(TAG, "电机1 CAN基础ID + 0x00, 电机4 CAN基础ID + 0x60");
    ESP_LOGI(TAG, "请连接WiFi热点，然后访问: http://192.168.4.1");
    ESP_LOGI(TAG, "CAN总线配置: TX=GPIO11, RX=GPIO10, 500kbps标准帧");
    ESP_LOGI(TAG, "电机控制通过TWAI直接发送CAN指令");
    ESP_LOGI(TAG, "Web界面支持: 位置/速度/力矩模式控制");

    // ========== 所有初始化完成后，设置电机1和电机4 ==========
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========== 开始配置电机运行参数 ==========");
    vTaskDelay(pdMS_TO_TICKS(1000));

    // // 1. 设置电机1为速度模式
    // ESP_LOGI(TAG, "[电机1] 设置为速度模式...");
    // motor_control_set_velocity_mode(motor_controller_1);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 2. 设置电机4为速度模式
    // ESP_LOGI(TAG, "[电机4] 设置为速度模式...");
    // motor_control_set_velocity_mode(motor_controller_4);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 3. 使能电机1
    // ESP_LOGI(TAG, "[电机1] 使能电机...");
    // motor_control_enable(motor_controller_1, true);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 4. 使能电机4
    // ESP_LOGI(TAG, "[电机4] 使能电机...");
    // motor_control_enable(motor_controller_4, true);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // // 5. 设置电机1速度为 20 r/s (直接使用外部值，不转换)
    // float motor1_speed = 20.0f;
    // ESP_LOGI(TAG, "[电机1] 设置速度: %.2f r/s", motor1_speed);
    // motor_control_set_velocity(motor_controller_1, motor1_speed);
    // vTaskDelay(pdMS_TO_TICKS(5));  // 指令间隔5ms

    // // 6. 设置电机4速度为 -20 r/s (直接使用外部值，不转换)
    // float motor4_speed = -20.0f;
    // ESP_LOGI(TAG, "[电机4] 设置速度: %.2f r/s", motor4_speed);
    // motor_control_set_velocity(motor_controller_4, motor4_speed);
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 指令间隔5ms

    // ESP_LOGI(TAG, "========== 电机配置完成！电机1和电机4已开始运行 ==========");
    // ESP_LOGI(TAG, "");

    // 任务完成，删除自己 (使用vTaskDelete确保正确清理)
    ESP_LOGI(TAG, "初始化任务完成，释放任务资源");
    vTaskDelay(pdMS_TO_TICKS(100)); // 确保日志输出完成
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32电机WEB控制系统启动中...");

    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 初始化TCP/IP协议栈和事件循环（原来由WiFi初始化，现在需要手动初始化）
    ESP_LOGI(TAG, "初始化TCP/IP协议栈");
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // WiFi功能暂时关闭，专注测试W5500以太网
    // ESP_LOGI(TAG, "初始化WiFi热点模式");
    // wifi_init_softap();

#if CONFIG_ETH_SPI_ETHERNET_W5500
    // ========== 初始化 W5500 以太网 ==========
    ESP_LOGI(TAG, "开始初始化 W5500 以太网...");

    // 等待 W5500 芯片上电稳定
    ESP_LOGI(TAG, "等待 W5500 上电稳定...");
    vTaskDelay(pdMS_TO_TICKS(500));

    // 注册以太网事件处理器
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    // 初始化 W5500 以太网驱动
    esp_eth_handle_t eth_handle = w5500_eth_init();
    if (eth_handle == NULL) {
        ESP_LOGE(TAG, "W5500 以太网初始化失败");
    } else {
        // 创建以太网网络接口（使用默认配置）
        esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
        esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
        g_eth_netif = eth_netif;

        // 将以太网驱动连接到 TCP/IP 栈
        esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(eth_handle);
        ESP_ERROR_CHECK(esp_netif_attach(eth_netif, glue));

        // 启动以太网驱动
        ESP_ERROR_CHECK(esp_eth_start(eth_handle));
        ESP_LOGI(TAG, "W5500 以太网已启动");

        // 等待以太网连接
        vTaskDelay(pdMS_TO_TICKS(1000));

        // 配置静态IP（直连电脑时需要）
        // ESP32 IP: 172.16.2.100, 电脑需要设置为 172.16.2.x 网段
        esp_netif_ip_info_t ip_info;
        memset(&ip_info, 0, sizeof(ip_info));
        IP4_ADDR(&ip_info.ip,      172,  16, 2, 100);
        IP4_ADDR(&ip_info.netmask, 255, 255, 255,  0);
        IP4_ADDR(&ip_info.gw,      172,  16, 2,   1);

        // 停止DHCP客户端，设置静态IP
        if (esp_netif_dhcpc_stop(eth_netif) == ESP_OK) {
            esp_netif_set_ip_info(eth_netif, &ip_info);
            ESP_LOGI(TAG, "W5500 静态IP已配置: 172.16.2.100");
        } else {
            ESP_LOGW(TAG, "无法停止DHCP客户端，静态IP设置可能失败");
        }

        // 启动法奥机器人协议服务器
        xTaskCreate(fr_robot_server_task, "fr_robot_server", 4096, NULL, 5, NULL);
        ESP_LOGI(TAG, "法奥机器人协议服务器任务已创建，端口: %d", FR_UDP_PORT);
    }
#else
    ESP_LOGI(TAG, "W5500 以太网未启用（需要在 menuconfig 中启用）");
#endif // CONFIG_ETH_SPI_ETHERNET_W5500

    // 创建电机初始化任务 (增加栈大小以避免栈溢出)
    xTaskCreate(motor_init_task, "motor_init", 8192, NULL, 5, NULL);

    ESP_LOGI(TAG, "系统启动完成");
}
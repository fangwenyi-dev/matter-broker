/**
 * @file main.cpp
 * @brief Matter-MQTT Broker 一体化项目入口
 *
 * 启动流程（按实际执行顺序）：
 * 1. NVS 初始化（esp_matter 和 mosquitto 都依赖）
 * 2. 创建消息队列（MQTT 消息队列置于 PSRAM，Matter 事件队列置于内部 RAM）
 * 3. esp_netif 初始化（mosquitto broker 需要 lwip socket）
 * 4. 创建默认 event loop（esp_mqtt_client / WiFi 事件需要）
 * 5. 注册 WiFi/IP 事件处理器（连接成功启动 MQTT 客户端，断开诊断+重连兜底）
 * 6. 启动 MQTT Broker（独立 task，core 1，TCP 1883，阻塞等待绑定结果）
 * 7. 初始化 Matter Bridge（esp_matter node + aggregator）
 * 8. 初始化协议桥接层（双向转换 Matter↔MQTT $SH 协议）
 * 9. 启动协议桥接 task
 * 10. 初始化 mDNS 并设置 hostname（必须在 Matter 启动前）
 * 11. 启动 Matter（esp_matter::start 内部自动初始化 WiFi）
 * 12. 初始化按键（非关键模块，失败不影响 broker）
 *
 * 注意：esp_netif 和 event_loop 需在 Broker 启动前手动初始化，
 *       esp_matter::start() 会复用已有的，不重复创建。
 */
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"  // 需要 WIFI_EVENT 和 IP_EVENT 定义
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"  // 项22: system_monitor_task 喂狗
#include "esp_ota_ops.h"   // P0: esp_ota_mark_app_valid_cancel_rollback（OTA 回滚保护）
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"  // xQueueCreateWithCaps（ESP-IDF 扩展 API）

#include "app_mqtt_broker.h"
#include "app_matter_bridge.h"
#include "app_protocol_bridge.h"
#include "app_button.h"
#include "app_led.h"
#include "mdns.h"   // mDNS：设置 matter-broker.local hostname

static const char *TAG = "main";

// 前向声明（定义在文件末尾）
static void system_monitor_task(void *arg);

// 项10: WiFi 重连失败计数器，超过阈值清除凭据重启进入配网模式
// P2-9 修复：10→20，避免 AP 短暂故障（重启/信号盲区）即清除凭据迫使用户重新配网
static int s_wifi_retry_count = 0;
#define WIFI_RETRY_MAX  20

// P1-5 修复：OTA 回滚保护标志，WiFi 连接成功后标记固件有效（仅执行一次）
static volatile bool s_ota_verified = false;

// ==================== WiFi 事件处理器 ====================
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
ESP_LOGI(TAG, "WiFi 已连接，IP: " IPSTR, IP2STR(&event->ip_info.ip));

// 蓝灯常亮2秒后熄灭（WiFi 连接成功指示）
app_led_set(LED_BLUE, LED_MODE_ON, 2000);

// 项10: 连接成功，重置重连计数
s_wifi_retry_count = 0;

        // 项23: 禁用 WiFi 省电模式（WIFI_PS_NONE），保持射频常开。
        //       原因：IDF 默认 WIFI_PS_MIN_MODEM 会在 beacon 间隙关闭接收机，
        //       导致 Matter keep-alive（MRP）和 mDNS 广播包在 sleep 窗口到达时丢失，
        //       涂鸦/HomeKit App 连续数次未收到响应后判定设备离线。
        //       每次获取 IP 都调用：WiFi 重连后 IDF 不会自动重置 PS 策略。
        //       代价：增加约 30-50mA 功耗，本设备为常供电 broker，可接受。
        esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
        if (ps_err == ESP_OK) {
            ESP_LOGI(TAG, "已禁用 WiFi 省电模式 (WIFI_PS_NONE)");
        } else {
            ESP_LOGW(TAG, "禁用 WiFi 省电模式失败: %s", esp_err_to_name(ps_err));
        }

        // 项12: mdns hostname 已在 app_main 中设置（matter-broker），
        //       espressif/mdns 组件自动注册了 IP_EVENT 处理器，WiFi 获取 IP 后会自动
        //       启用 STA 接口的 mDNS PCB 并广播 A/AAAA 记录，无需在此重复设置。
        // P2-8 修复：删除原 if(!s_mdns_hostname_set) 死代码块（app_main 中成功设置后
        //            s_mdns_hostname_set 必为 true，失败则 esp_restart，此分支永不执行）

        // WiFi 连接成功，启动本地 MQTT 客户端
        app_protocol_bridge_on_wifi_connected();

        // P1-5 修复：WiFi 连接成功后标记新固件有效（取消 OTA 回滚）。
        // sdkconfig.defaults 启用了 CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE，
        // 新固件首次启动处于 pending verify 状态，必须在下次重启前标记 valid。
        // 移到此处（原在 Matter 启动后立即标记），至少验证 WiFi 协议栈正常工作。
        // 仅执行一次，避免 WiFi 重连时重复调用。
        if (!s_ota_verified) {
            s_ota_verified = true;
            esp_err_t ota_err = esp_ota_mark_app_valid_cancel_rollback();
            if (ota_err == ESP_OK) {
                ESP_LOGI(TAG, "新固件已标记为有效（WiFi 连接验证通过，取消 OTA 回滚）");
            } else {
                // 非 OTA 升级启动时返回 ESP_ERR_OTA_VALIDATE_INVALID_STATE，属正常情况
                ESP_LOGD(TAG, "esp_ota_mark_app_valid_cancel_rollback: %s", esp_err_to_name(ota_err));
            }
        }
    }
    else if (event_base == WIFI_EVENT) {
        switch (event_id) {
        case WIFI_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "WiFi STA 已关联到 AP，等待 DHCP 分配 IP...");
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            // 打印断开原因，方便排查配网失败
            // esp_matter 内部会自动重连，此处记录日志 + 重连兜底
            wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;
            // 项10: 递增重连计数
            s_wifi_retry_count++;
            ESP_LOGW(TAG, "WiFi STA 断开, reason=%d, retry=%d/%d",
                     event->reason, s_wifi_retry_count, WIFI_RETRY_MAX);
            // 常见 reason 说明：
            //   201 (NO_AP_FOUND)     : 找不到指定 SSID 的 AP
            //   15  (4WAY_HANDSHAKE_TIMEOUT) : WiFi 密码错误
            //   2   (AUTH_EXPIRE)     : 认证超时
            //   8   (ASSOC_LEAVE)     : 主动断开
            if (event->reason == WIFI_REASON_NO_AP_FOUND) {
                ESP_LOGW(TAG, "  → 找不到 SSID，请检查配网时输入的 WiFi 名称");
            } else if (event->reason == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT) {
                ESP_LOGW(TAG, "  → WiFi 密码错误，请重新配网");
            }

            // 项10: 累计超过阈值，清除 WiFi 凭据并重启进入配网模式
            //       （避免无限重连消耗资源，常见于密码错误/AP 消失场景）
            //       注意：仅清除 nvs.net80211 命名空间（WiFi 凭据），保留 Matter commissioning 数据，
            //       避免已配网的设备丢失 Matter fabric 需重新走 BLE 配网流程。
            if (s_wifi_retry_count >= WIFI_RETRY_MAX) {
                ESP_LOGE(TAG, "WiFi 重连失败已超 %d 次，清除 WiFi 凭据重启进入配网模式", WIFI_RETRY_MAX);
                nvs_handle_t nvs_handle;
                esp_err_t open_err = nvs_open("nvs.net80211", NVS_READWRITE, &nvs_handle);
                bool nvs_ok = false;
                if (open_err == ESP_OK) {
                    esp_err_t erase_err = nvs_erase_all(nvs_handle);
                    if (erase_err != ESP_OK) {
                        ESP_LOGE(TAG, "nvs_erase_all 失败: %s", esp_err_to_name(erase_err));
                    } else {
                        esp_err_t commit_err = nvs_commit(nvs_handle);
                        if (commit_err != ESP_OK) {
                            ESP_LOGE(TAG, "nvs_commit 失败: %s", esp_err_to_name(commit_err));
                        } else {
                            nvs_ok = true;
                        }
                    }
                    nvs_close(nvs_handle);
                } else {
                    ESP_LOGE(TAG, "nvs_open(nvs.net80211) 失败: %s", esp_err_to_name(open_err));
                }
                // P2-12 修复：NVS 清除失败时不重启，重置计数让 esp_matter 继续自动重连，
                // 下次达到阈值再尝试清除。避免 vTaskDelay 阻塞 event loop 和失败-重启循环。
                if (nvs_ok) {
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "WiFi 凭据清除失败，重置重连计数，继续自动重连");
                    s_wifi_retry_count = 0;
                }
            }
            break;
        }
        default:
            break;
        }
    }
}

// 队列大小
#define MQTT_MSG_QUEUE_SIZE     10
#define MATTER_EVENT_QUEUE_SIZE 10

// 全局队列句柄
static QueueHandle_t s_mqtt_msg_queue = NULL;
static QueueHandle_t s_matter_event_queue = NULL;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== Matter-MQTT Broker 启动 ===");
    ESP_LOGI(TAG, "芯片: ESP32-S3-WROOM-1-N16R8");
    ESP_LOGI(TAG, "Flash: 16MB Quad, PSRAM: 8MB Octal");

    // 1. NVS 初始化（esp_matter 和 mosquitto 都需要）
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // 项13: 检查 nvs_flash_erase 返回值并记录日志（原代码用 ESP_ERROR_CHECK 直接 abort）
        esp_err_t erase_err = nvs_flash_erase();
        if (erase_err != ESP_OK) {
            ESP_LOGE(TAG, "nvs_flash_erase 失败: %s，尝试继续初始化", esp_err_to_name(erase_err));
        }
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS 初始化完成");

    // 2. 创建消息队列
    // MQTT 消息队列每条约 4292 字节（data[4096]），10 条 ≈ 42KB，
    // 若用默认 xQueueCreate 会全部占用内部 RAM，与 BLE controller 争内存
    // 导致 BLE_INIT: Malloc failed。改用 xQueueCreateWithCaps 分配到 PSRAM。
    // （ESP-IDF v5.1+ 支持，本项目 v5.5.4）
    s_mqtt_msg_queue = xQueueCreateWithCaps(MQTT_MSG_QUEUE_SIZE, sizeof(mqtt_message_t),
                                             MALLOC_CAP_SPIRAM);
    // Matter 事件队列较小（每项约 40 字节），保持内部 RAM 以保证事件响应速度
    s_matter_event_queue = xQueueCreate(MATTER_EVENT_QUEUE_SIZE, sizeof(matter_event_t));
    if (s_mqtt_msg_queue == NULL || s_matter_event_queue == NULL) {
        ESP_LOGE(TAG, "创建队列失败: mqtt=%p, matter=%p", s_mqtt_msg_queue, s_matter_event_queue);
        // 项14: 清理已创建的队列再重启，避免裸 return 导致后续模块访问空指针
        if (s_mqtt_msg_queue != NULL) {
            vQueueDelete(s_mqtt_msg_queue);
            s_mqtt_msg_queue = NULL;
        }
        if (s_matter_event_queue != NULL) {
            vQueueDelete(s_matter_event_queue);
            s_matter_event_queue = NULL;
        }
        esp_restart();
    }
    ESP_LOGI(TAG, "消息队列创建完成（MQTT 队列在 PSRAM）");

    // 2.5 初始化 esp_netif（mosquitto broker 需要 lwip socket）
    //     注意：esp_matter::start() 也会初始化 esp_netif，但多次调用是安全的
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_LOGI(TAG, "esp_netif 初始化完成");

    // 2.6 创建默认 event loop（esp_mqtt_client 需要注册事件）
    //     esp_matter::start() 也会创建，但多次调用会报错，所以在此前创建
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "event loop 已创建");

    // 2.7 注册 WiFi/IP 事件处理器（监听 WiFi 连接成功 + 断开诊断）
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED,
                                                wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED,
                                                wifi_event_handler, NULL));
    ESP_LOGI(TAG, "WiFi/IP 事件处理器已注册");

    // 3. 启动 MQTT Broker（独立 task，core 1）
    //    Broker 需要 lwip socket，esp_netif 已初始化
    mqtt_broker_config_t broker_config = {
        .host = "0.0.0.0",
        .port = CONFIG_BROKER_PORT,
        .username = CONFIG_MQTT_BROKER_USERNAME,
        .password = CONFIG_MQTT_BROKER_PASSWORD,
        .msg_queue = s_mqtt_msg_queue,
    };
    ESP_ERROR_CHECK(app_mqtt_broker_start(&broker_config));
    ESP_LOGI(TAG, "MQTT Broker 已启动并绑定端口 (端口 %d)", CONFIG_BROKER_PORT);

    // 项9: app_mqtt_broker_start 已阻塞等待绑定结果（1.5s 超时），
    //      start 返回 ESP_OK 即表示 broker 已成功绑定端口并开始接收连接，
    //      无需额外延时。

    // 4. 初始化 Matter Bridge（创建 node + aggregator）
    matter_bridge_config_t matter_config = {
        .event_queue = s_matter_event_queue,
    };
    ESP_ERROR_CHECK(app_matter_bridge_init(&matter_config));
    ESP_LOGI(TAG, "Matter Bridge 已初始化");

    // 5. 初始化协议桥接层
    protocol_bridge_config_t bridge_config = {
        .bridge_sn = CONFIG_GATEWAY_SN,
        .mqtt_msg_queue = s_mqtt_msg_queue,
        .matter_event_queue = s_matter_event_queue,
    };
    ESP_ERROR_CHECK(app_protocol_bridge_init(&bridge_config));
    ESP_LOGI(TAG, "协议桥接层已初始化");

    // 6. 启动协议桥接 task
    ESP_ERROR_CHECK(app_protocol_bridge_start());
    ESP_LOGI(TAG, "协议桥接 task 已启动");

    // 7. 初始化 espressif/mdns 并设置 hostname（必须在 app_matter_bridge_start 之前）
    //    CONFIG_USE_MINIMAL_MDNS=n：单实例方案，espressif/mdns 负责全部 mDNS 功能
    //    - Matter DNS-SD 服务发布（SRV/TXT/AAAA）：App 能发现设备
    //    - matter-broker.local A 查询响应：LoRa 网关能解析 ESP32 的 IP
    //    必须在 app_matter_bridge_start() 之前调用，否则 esp_matter 启动时 mdns 未就绪
    //    ESP32DnssdImpl.cpp L162-168 已注释 mdns_hostname_set(MAC_hex)，避免覆盖 "matter-broker"
    esp_err_t mdns_err = mdns_init();
    if (mdns_err != ESP_OK) {
        // 项11: mdns_init 失败后中止 Matter 启动并重启重试。
        //       Matter 服务发布依赖 mDNS，失败后设备无法被 App 发现，没有继续启动的意义。
        ESP_LOGE(TAG, "mDNS 初始化失败: %s，1 秒后重启重试", esp_err_to_name(mdns_err));
        vTaskDelay(pdMS_TO_TICKS(1000));  // 让日志输出完成
        esp_restart();
    }
    mdns_err = mdns_hostname_set("matter-broker");
    if (mdns_err == ESP_OK) {
        ESP_LOGI(TAG, "mDNS hostname 已设置: matter-broker.local");
    } else {
        ESP_LOGE(TAG, "mDNS hostname 设置失败: %s，1 秒后重启重试", esp_err_to_name(mdns_err));
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    // 8. 启动 Matter
    //    esp_matter::start() 内部会自动初始化 WiFi；esp_netif 和 event_loop 已在前面
    //    手动创建（broker/mqtt client 需要它们），esp_matter 会复用已有的，不重复创建
    ESP_ERROR_CHECK(app_matter_bridge_start());
    ESP_LOGI(TAG, "Matter 已启动，等待 BLE 配网...");

    // P1-5 修复：OTA 回滚保护移到 WiFi 连接成功事件中（wifi_event_handler 的 IP_EVENT_STA_GOT_IP）。
    // 原代码在 Matter 启动后立即标记 valid，但此时 WiFi 未连接、LoRa 未通信，
    // 若新固件有 WiFi/LoRa bug 已标记 valid 无法回滚。
    // 移到 WiFi 连接成功后标记，至少验证 WiFi 协议栈正常。

    // 9. 初始化 LED 指示灯模块（WS2812 幻彩 LED）
    //    蓝灯慢闪表示 WiFi 等待连接，WiFi 连接成功后蓝灯常亮2s
    esp_err_t led_err = app_led_init(CONFIG_LED_GPIO);
    if (led_err != ESP_OK) {
        ESP_LOGW(TAG, "LED 初始化失败: %s，继续运行（指示灯不可用）", esp_err_to_name(led_err));
    } else {
        // 启动时蓝灯慢闪，等待 WiFi 连接
        app_led_set(LED_BLUE, LED_MODE_SLOW_BLINK, 0);
        ESP_LOGI(TAG, "LED 指示灯已初始化");
    }

    // 10. 初始化按键（GPIO0，低电平有效）—— 非关键模块
    //    项17: 降级处理，失败时仅警告并继续运行（设备作为 broker 仍可工作，只是按键不可用）
    esp_err_t btn_err = app_button_init(CONFIG_BUTTON_GPIO);
    if (btn_err != ESP_OK) {
        ESP_LOGW(TAG, "按键初始化失败: %s，继续运行（按键功能不可用）", esp_err_to_name(btn_err));
    } else {
        ESP_LOGI(TAG, "按键已初始化");
    }

    ESP_LOGI(TAG, "=== 系统初始化完成 ===");
    ESP_LOGI(TAG, "Broker: tcp://0.0.0.0:%d", CONFIG_BROKER_PORT);
    ESP_LOGI(TAG, "Bridge SN: %s", CONFIG_GATEWAY_SN);
    ESP_LOGI(TAG, "按键: 2击=LoRa配对, 3击=删除所有子设备, 5击=重置Matter, 长按5s=重置WiFi");
    ESP_LOGI(TAG, "等待 LoRa 网关上线（自动绑定 + 发现设备）");
    ESP_LOGI(TAG, "使用 Apple Home / Google Home 扫描配网二维码进行配对");

    // 打印初始内存状态
    ESP_LOGI(TAG, "=== 内存状态 ===");
    ESP_LOGI(TAG, "内部 RAM: free=%u, min=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "PSRAM:     free=%u, min=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "总 heap:   free=%u, min=%u",
             (unsigned)esp_get_free_heap_size(),
             (unsigned)esp_get_minimum_free_heap_size());

    // 启动系统资源监控 task（每 30 秒打印 heap + 检查网关离线状态）
// 栈 4096：check_gateway_offline 使用快照模式会在栈上分配 dev_sns[12][32]=384B，
// 加上 taskENTER_CRITICAL 和函数调用栈，3072 偏紧，4096 留余量。
    // P2-10 修复：优先级 1→2，高于 IDLE，确保看门狗喂狗和离线检测及时调度
    BaseType_t monitor_ret = xTaskCreate(system_monitor_task, "sys_monitor", 4096, NULL, 2, NULL);
    if (monitor_ret != pdPASS) {
        ESP_LOGW(TAG, "创建 system_monitor task 失败（不影响主功能）");
    }
}

// ==================== 系统资源监控 task ====================
static void system_monitor_task(void *arg)
{
    (void)arg;
    // 项22: 注册到任务看门狗（Task Watchdog Timer），定期喂狗。
    //       若项目未启用 CONFIG_ESP_TASK_WDT，esp_task_wdt_add 返回 ESP_ERR_NOT_SUPPORTED，
    //       此时跳过喂狗，task 仍正常运行。
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK) {
        ESP_LOGW(TAG, "system_monitor 注册看门狗失败: %s（继续运行，不喂狗）",
                 esp_err_to_name(wdt_err));
    }

    // 30 秒检查一次：内存监控 + 网关离线检测（超时阈值 900s=15分钟，30s 检查粒度足够）。
    // 项22: 原代码 vTaskDelay(30s) 会导致看门狗超时（默认 TWDT 超时 5s），
    //       改为 30 次 1s 短延时，每次喂狗。
    int tick = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // 项22: 每秒喂狗
        if (wdt_err == ESP_OK) {
            esp_task_wdt_reset();
        }
        tick++;
        if (tick >= 30) {
            tick = 0;
            ESP_LOGI(TAG, "[监控] heap: internal=%u, psram=%u, total_free=%u, min=%u",
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                     (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                     (unsigned)esp_get_free_heap_size(),
                     (unsigned)esp_get_minimum_free_heap_size());
            // 检查 LoRa 网关离线状态
            // s_mqtt_client_started / s_mqtt_client_starting 已通过
            // s_mqtt_client_lock 保护跨 task 访问（app_protocol_bridge.cpp）。
            app_protocol_bridge_check_gateway_offline();
        }
    }
}

/**
 * @file app_protocol_bridge.cpp
 * @brief Matter↔MQTT 协议桥接层实现
 *
 * 完整 $SH 协议处理，参考 huijian-gateway 的 Python 实现。
 *
 * 关键设计：
 * 1. 多网关支持：每个 LoRa 设备记录所属网关 SN，控制时发到正确的 gateway/{gw_sn}/req
 * 2. 设备过滤：跳过 SN 以 "1001" 开头的设备、model 包含 "gateway" 的设备
 * 3. 005 双格式：支持直接字段（position/battery）和 attrs 数组两种格式
 * 4. 位置语义：Matter→LoRa 方向，event.value 已经是反转后的 LoRa 值（0=关闭, 100=打开）
 */
#include "app_protocol_bridge.h"
#include "app_matter_bridge.h"
#include "app_led.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include <platform/PlatformManager.h>  // P3-3 修复：chip::StackLock（bridge_task 调用 Matter API 时加锁）
#include "freertos/task.h"  // taskENTER_CRITICAL（离线检测跨 task 保护）
#include "freertos/semphr.h"  // SemaphoreHandle_t（P2-7 信号量等待 task 退出）
#include "esp_timer.h"        // esp_timer_get_time（pending_003 超时清理）
#include <string.h>
#include <stdio.h>
#include <stdlib.h>  // strtol（parse_number_field 改进）
#include <errno.h>   // ERANGE（strtol 溢出检查）

// QueueSet 容量依赖 main.cpp 中的队列深度定义
// 若队列深度改变，QueueSet 容量自动跟随
#ifndef MQTT_MSG_QUEUE_SIZE
#define MQTT_MSG_QUEUE_SIZE 10
#endif
#ifndef MATTER_EVENT_QUEUE_SIZE
#define MATTER_EVENT_QUEUE_SIZE 10
#endif

static const char *TAG = "protocol_bridge";

// ==================== $SH 协议常量（参考 huijian-gateway const.py） ====================
#define PROTOCOL_HEAD               "$SH"
#define TOPIC_GATEWAY_REQ_FMT       "gateway/%s/req"   // 下发命令主题（用 LoRa 网关 SN）
#define TOPIC_GATEWAY_RSP           "gateway/rpt_rsp"   // 状态上报主题
#define ATTRIBUTE_W_TRAVEL          "w_travel"
#define ATTRIBUTE_WIND_LOCK_MODE    "rwp_wind_lock_mode"
#define MAX_COMMAND_ID              999999
#define MAX_GATEWAYS                10
#define DEVICE_SN_PREFIX_GATEWAY    "1001"  // 网关设备 SN 前缀，需跳过
// 网关离线判定超时（秒）。LoRa 网关通常每分钟上报 001/002/005，
// 超过 15 分钟（900 秒）无任何消息视为离线（兼顾网络抖动、网关心跳间隔
// 和 LoRa 信号波动导致的临时失联，避免频繁误判离线）
#define GATEWAY_OFFLINE_TIMEOUT_SEC 900

// ==================== Task 配置 ====================
// 优化：stack 8192→10240（cJSON 解析 + Matter 事件处理需要更多空间）
// P1-D 修复：priority 8→4（低于 Matter task 优先级 5，避免抢占 CASE 会话导致 App 离线）
#define BRIDGE_TASK_PRIORITY        4
#define BRIDGE_TASK_STACK_SIZE      10240
#define BRIDGE_TASK_CORE            0

// ==================== 内部状态 ====================
static char s_bridge_sn[32] = {0};
static QueueHandle_t s_mqtt_msg_queue = NULL;
static QueueHandle_t s_matter_event_queue = NULL;
static TaskHandle_t s_bridge_task = NULL;
static volatile bool s_bridge_running = false;  // 跨 task 共享，需 volatile 防止编译器优化
// P2-7 修复：用信号量替代 eTaskGetState 轮询，避免 use-after-free
// bridge_task 退出前 give，stop 中 take 等待
static SemaphoreHandle_t s_bridge_exit_sem = NULL;

// 本地 MQTT 客户端（连接到本地 broker，用于发布命令）
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
// s_mqtt_client_started：在 MQTT_EVENT_CONNECTED 中置 true，DISCONNECTED 中复位
// s_mqtt_client_starting：防止 on_wifi_connected 重入时重复调用 esp_mqtt_client_start
static volatile bool s_mqtt_client_started = false;
static volatile bool s_mqtt_client_starting = false;
// P2-4 修复：s_mqtt_client_started/starting 的"检查-设置"序列加锁保护，避免 TOCTOU 竞态
static portMUX_TYPE s_mqtt_client_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_command_id = 1;
// s_command_id 跨 task 并发递增（bridge_task 和按键 task）需自旋锁保护
static portMUX_TYPE s_command_id_lock = portMUX_INITIALIZER_UNLOCKED;

// ==================== 003 操作类型追踪 ====================
// LoRa 网关的 003 响应不包含 bind 字段，无法从响应本身判断是配对还是解绑。
// 通过记录发送 003 命令时的 command_id → bind 映射，在收到响应时查找，
// 替代原有的"设备已存在→解绑"推断逻辑（该逻辑在重新配对已存在设备时会误判为解绑）。
typedef struct {
    int command_id;       // 发送 003 命令时的 id
    int bind;             // 1=配对, 0=解绑
    int64_t timestamp_us; // 记录时刻（用于超时清理）
    bool in_use;
} pending_003_op_t;

#define MAX_PENDING_003_OPS 4
#define PENDING_003_TIMEOUT_SEC 120  // 2 分钟超时（配对/解绑响应通常在数秒内返回）

static pending_003_op_t s_pending_003_ops[MAX_PENDING_003_OPS] = {0};
static portMUX_TYPE s_pending_003_lock = portMUX_INITIALIZER_UNLOCKED;

// ==================== 多网关管理 ====================
typedef struct {
    char gateway_sn[32];
    bool online;
    uint32_t last_seen;  // tick
    bool in_use;
} gateway_entry_t;

static gateway_entry_t s_gateways[MAX_GATEWAYS] = {0};

// s_gateways 跨 task 访问自旋锁（register_gateway 在 bridge_task，
// check_gateway_offline 在 system_monitor_task）
static portMUX_TYPE s_gateways_lock = portMUX_INITIALIZER_UNLOCKED;

// ==================== 设备→网关映射 ====================
// 每个设备记录其所属的 LoRa 网关 SN，控制时发到正确的主题
// 同时缓存最新的电压(mV)和状态(0=关闭,1=打开)供调试和扩展使用
typedef struct {
    char device_sn[32];
    char gateway_sn[32];   // 所属 LoRa 网关 SN
    uint16_t voltage_mv;   // 电池电压（毫伏），HA 集成上报值×10
    uint8_t  state;        // 0=关闭, 1=打开
    uint32_t add_seq;      // 添加顺序序号（用于删除最后添加的子设备）
    bool in_use;
} device_gateway_entry_t;

#define MAX_DEVICE_ENTRIES CONFIG_MAX_BRIDGED_DEVICES
static device_gateway_entry_t s_device_gateway_map[MAX_DEVICE_ENTRIES] = {0};
static uint32_t s_add_seq_counter = 0;  // 全局添加序号（递增）

// s_device_gateway_map 跨 task 访问自旋锁
// 访问者：bridge_task（register_device_gateway/find_gateway_for_device/handle_ctype_005 缓存/UNBIND_LAST）、
//         system_monitor_task（check_gateway_offline）
// 临界区内禁止调用 Matter API / MQTT 发布 / 递归加锁的函数，需用快照模式
static portMUX_TYPE s_device_map_lock = portMUX_INITIALIZER_UNLOCKED;

// ==================== 辅助函数 ====================

// 电池电压原始值有效范围（超出视为异常，避免 uint16 溢出和错误显示）
// 12V锂电池: 正常工作范围 9.5V-12.6V（raw 95-126），放宽到 8V-14V（raw 80-140）容错
#define BATTERY_RAW_MIN 80
#define BATTERY_RAW_MAX 140

/**
 * @brief 从 cJSON 字段解析数值（兼容字符串和数字类型）
 *
 * 网关上报的 battery/voltage 字段可能是数字（105）或字符串（"105"）。
 * HA 集成 mqtt_handler.py 用 float() 兼容两种，此处同样兼容。
 *
 * @param obj cJSON 对象（已 GetObjectItem 的结果）
 * @return 解析后的整数值，-1 表示字段不存在或类型不匹配
 */
static int parse_number_field(cJSON *obj)
{
    if (obj == NULL) {
        return -1;
    }
    if (cJSON_IsNumber(obj)) {
        return obj->valueint;
    }
    if (cJSON_IsString(obj)) {
        const char *start = obj->valuestring;
        char *endptr = NULL;
        errno = 0;
        long val = strtol(start, &endptr, 10);
        // endptr == start 表示无任何数字被解析（纯非数字字符串），返回 -1
        // 这样可区分 "abc"（返回 -1）和 "0"（返回 0），避免与合法值 0 混淆
        // 部分可解析（如 "10.5"）返回整数部分，与原 atoi 行为一致
        if (endptr == start) {
            return -1;
        }
        // P3-6 修复：检查 strtol 溢出（ERANGE），超长数字字符串返回 -1
        if (errno == ERANGE) {
            return -1;
        }
        return (int)val;
    }
    return -1;
}

static int next_command_id(void)
{
    int id;
    // bridge_task 和按键 task 并发调用，临界区内仅 ++ 和比较，极短
    taskENTER_CRITICAL(&s_command_id_lock);
    id = s_command_id++;
    if (s_command_id > MAX_COMMAND_ID) {
        s_command_id = 1;
    }
    taskEXIT_CRITICAL(&s_command_id_lock);
    return id;
}

/**
 * @brief 记录待处理的 003 操作（发送 003 命令时调用）
 *
 * LoRa 网关的 003 响应不包含 bind 字段，需要通过 command_id 查找发送时的 bind 值。
 */
static void record_pending_003(int command_id, int bind)
{
    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_pending_003_lock);
    // 查找已有记录（同一 command_id 更新）
    int free_slot = -1;
    for (int i = 0; i < MAX_PENDING_003_OPS; i++) {
        if (s_pending_003_ops[i].in_use && s_pending_003_ops[i].command_id == command_id) {
            s_pending_003_ops[i].bind = bind;
            s_pending_003_ops[i].timestamp_us = now;
            taskEXIT_CRITICAL(&s_pending_003_lock);
            return;
        }
        if (!s_pending_003_ops[i].in_use && free_slot < 0) {
            free_slot = i;
        }
    }
    // 占用新空位
    if (free_slot >= 0) {
        s_pending_003_ops[free_slot].command_id = command_id;
        s_pending_003_ops[free_slot].bind = bind;
        s_pending_003_ops[free_slot].timestamp_us = now;
        s_pending_003_ops[free_slot].in_use = true;
    }
    // 顺便清理过期记录
    for (int i = 0; i < MAX_PENDING_003_OPS; i++) {
        if (s_pending_003_ops[i].in_use &&
            (now - s_pending_003_ops[i].timestamp_us) > (int64_t)PENDING_003_TIMEOUT_SEC * 1000000LL) {
            s_pending_003_ops[i].in_use = false;
        }
    }
    taskEXIT_CRITICAL(&s_pending_003_lock);
}

/**
 * @brief 查找并消费待处理的 003 操作（收到 003 响应时调用）
 *
 * @return bind 值（1=配对, 0=解绑），-1 表示未找到匹配的记录
 */
static int lookup_pending_003(int command_id)
{
    int bind = -1;
    int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_pending_003_lock);
    for (int i = 0; i < MAX_PENDING_003_OPS; i++) {
        if (s_pending_003_ops[i].in_use && s_pending_003_ops[i].command_id == command_id) {
            // 检查未超时
            if ((now - s_pending_003_ops[i].timestamp_us) <= (int64_t)PENDING_003_TIMEOUT_SEC * 1000000LL) {
                bind = s_pending_003_ops[i].bind;
            }
            s_pending_003_ops[i].in_use = false;  // 消费（读后即清除）
            break;
        }
    }
    taskEXIT_CRITICAL(&s_pending_003_lock);
    return bind;
}

/**
 * @brief 注册/更新 LoRa 网关
 *
 * 注意：s_gateways 被 bridge_task（本函数）和 system_monitor_task
 * （check_gateway_offline）跨 task 访问，更新 online/last_seen 需用
 * 临界区保护，避免 monitor task 读到 online=true 但 last_seen=旧值的竞态。
 */
static void register_gateway(const char *gw_sn)
{
    // P-Bug6 修复：合并"查找已有"和"查找空位"为单次扫描+单临界区，
    // 消除原两个独立循环之间的竞态窗口。
    // 原实现：第一轮循环查已有→退出临界区→第二轮循环查空位→退出临界区。
    // 若两个 task 同时调用，都可能在第一轮未找到→第二轮各占一个空位→同一 SN 出现两个槽位。
    // 修复：单次临界区内完成"查找已有 OR 查找空位并占位"，原子不可分。
    //
    // 注意：s_gateways 被 bridge_task（本函数）和 system_monitor_task
    // （check_gateway_offline）跨 task 访问，更新 online/last_seen 需用
    // 临界区保护，避免 monitor task 读到 online=true 但 last_seen=旧值的竞态。

    bool found_existing = false;
    bool was_offline = false;
    int new_slot = -1;

    taskENTER_CRITICAL(&s_gateways_lock);
    for (int i = 0; i < MAX_GATEWAYS; i++) {
        if (s_gateways[i].in_use && strcmp(s_gateways[i].gateway_sn, gw_sn) == 0) {
            // 找到已有记录，更新 online/last_seen
            found_existing = true;
            was_offline = !s_gateways[i].online;
            s_gateways[i].online = true;
            s_gateways[i].last_seen = xTaskGetTickCount();
            break;
        }
        if (!s_gateways[i].in_use && new_slot < 0) {
            // 记录第一个空位（但不立即占位，等确认不存在后再占）
            new_slot = i;
        }
    }
    if (!found_existing && new_slot >= 0) {
        // 未找到已有记录，占位新空位
        strncpy(s_gateways[new_slot].gateway_sn, gw_sn, sizeof(s_gateways[new_slot].gateway_sn) - 1);
        s_gateways[new_slot].gateway_sn[sizeof(s_gateways[new_slot].gateway_sn) - 1] = '\0';
        s_gateways[new_slot].online = true;
        s_gateways[new_slot].last_seen = xTaskGetTickCount();
        s_gateways[new_slot].in_use = true;
    }
    taskEXIT_CRITICAL(&s_gateways_lock);

    if (found_existing) {
        // 网关从离线恢复时，同步更新该网关下所有子设备的 Matter Reachable=true
        if (was_offline) {
            ESP_LOGI(TAG, "LoRa 网关恢复在线: %s", gw_sn);
            // 快照模式：临界区内收集设备 SN，临界区外调用 Matter API
            char dev_sns[MAX_DEVICE_ENTRIES][32];
            int dev_count = 0;
            taskENTER_CRITICAL(&s_device_map_lock);
            for (int j = 0; j < MAX_DEVICE_ENTRIES; j++) {
                if (s_device_gateway_map[j].in_use &&
                    strcmp(s_device_gateway_map[j].gateway_sn, gw_sn) == 0) {
                    strncpy(dev_sns[dev_count], s_device_gateway_map[j].device_sn,
                            sizeof(dev_sns[dev_count]) - 1);
                    dev_sns[dev_count][sizeof(dev_sns[dev_count]) - 1] = '\0';
                    dev_count++;
                }
            }
            taskEXIT_CRITICAL(&s_device_map_lock);

            for (int k = 0; k < dev_count; k++) {
                uint16_t ep_id, mode_ep_id;
                if (app_matter_bridge_find_endpoints(dev_sns[k], &ep_id, &mode_ep_id) == ESP_OK) {
                    app_matter_bridge_update_reachable(ep_id, true);
                    // Bug 修复：同时更新模式端点 Reachable，否则 5005 设备的模式开关
                    // 在网关离线恢复后仍显示离线
                    if (mode_ep_id != 0) {
                        app_matter_bridge_update_reachable(mode_ep_id, true);
                    }
                }
            }
        }
        return;
    }

    if (new_slot < 0) {
        ESP_LOGW(TAG, "网关表已满，无法注册 %s", gw_sn);
        return;
    }

    ESP_LOGI(TAG, "注册 LoRa 网关: %s", gw_sn);
    // 网关不再创建 Matter 端点（移除虚拟设备），仅记录到网关表
}

/**
 * @brief 记录设备→网关映射
 */
static void register_device_gateway(const char *device_sn, const char *gw_sn)
{
    bool table_full = false;
    // 查找已有记录 + 新建记录，全程临界区保护（查找-占用必须原子，避免多 task 同时占同一空位）
    taskENTER_CRITICAL(&s_device_map_lock);
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (s_device_gateway_map[i].in_use &&
            strcmp(s_device_gateway_map[i].device_sn, device_sn) == 0) {
            strncpy(s_device_gateway_map[i].gateway_sn, gw_sn,
                    sizeof(s_device_gateway_map[i].gateway_sn) - 1);
            s_device_gateway_map[i].gateway_sn[sizeof(s_device_gateway_map[i].gateway_sn) - 1] = '\0';
            taskEXIT_CRITICAL(&s_device_map_lock);
            return;
        }
    }
    // 新建记录
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (!s_device_gateway_map[i].in_use) {
            strncpy(s_device_gateway_map[i].device_sn, device_sn,
                    sizeof(s_device_gateway_map[i].device_sn) - 1);
            s_device_gateway_map[i].device_sn[sizeof(s_device_gateway_map[i].device_sn) - 1] = '\0';
            strncpy(s_device_gateway_map[i].gateway_sn, gw_sn,
                    sizeof(s_device_gateway_map[i].gateway_sn) - 1);
            s_device_gateway_map[i].gateway_sn[sizeof(s_device_gateway_map[i].gateway_sn) - 1] = '\0';
            s_device_gateway_map[i].add_seq = ++s_add_seq_counter;  // 记录添加顺序
            s_device_gateway_map[i].in_use = true;
            taskEXIT_CRITICAL(&s_device_map_lock);
            return;
        }
    }
    table_full = true;
    taskEXIT_CRITICAL(&s_device_map_lock);
    if (table_full) {
        ESP_LOGW(TAG, "设备→网关映射表已满，无法注册 dev=%s gw=%s", device_sn, gw_sn);
    }
}

/**
 * @brief 查找设备所属的 LoRa 网关 SN（线程安全，传出拷贝）
 *
 * 返回指针会有竞态（调用方在临界区外使用时可能被其他 task 修改），
 * 改为拷贝到调用方提供的缓冲区，临界区内完成拷贝。
 *
 * @param device_sn 设备 SN
 * @param gw_sn_buf 输出缓冲区，存放网关 SN
 * @param buf_size  缓冲区大小
 * @return true 找到，false 未找到
 */
static bool find_gateway_for_device(const char *device_sn, char *gw_sn_buf, size_t buf_size)
{
    if (gw_sn_buf == NULL || buf_size == 0) return false;
    gw_sn_buf[0] = '\0';
    bool found = false;
    taskENTER_CRITICAL(&s_device_map_lock);
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (s_device_gateway_map[i].in_use &&
            strcmp(s_device_gateway_map[i].device_sn, device_sn) == 0) {
            strncpy(gw_sn_buf, s_device_gateway_map[i].gateway_sn, buf_size - 1);
            gw_sn_buf[buf_size - 1] = '\0';
            found = true;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_device_map_lock);
    return found;
}

/**
 * @brief 设备过滤：跳过网关设备和 1001* 设备
 * 参考 huijian-gateway mqtt_handler.py L962-971
 */
static bool should_skip_device(const char *device_sn, const char *model)
{
    // 跳过 SN 以 "1001" 开头的设备（网关本身）
    if (strncmp(device_sn, DEVICE_SN_PREFIX_GATEWAY, 4) == 0) {
        return true;
    }
    // 跳过 model 包含 "gateway" 或 "网关" 的设备
    if (model) {
        if (strstr(model, "gateway") != NULL || strstr(model, "Gateway") != NULL ||
            strstr(model, "GATEWAY") != NULL || strstr(model, "网关") != NULL) {
            return true;
        }
    }
    return false;
}

/**
 * @brief 生成设备显示名称
 * 参考 huijian-gateway const.py get_device_display_name()
 */
static void get_device_display_name(char *buf, size_t buf_size,
                                     const char *gw_sn, const char *dev_sn)
{
    const char *short_gw = strlen(gw_sn) >= 4 ? gw_sn + strlen(gw_sn) - 4 : gw_sn;
    const char *short_dev = strlen(dev_sn) >= 4 ? dev_sn + strlen(dev_sn) - 4 : dev_sn;
    snprintf(buf, buf_size, "开窗器 %s-%s", short_gw, short_dev);
}

// ==================== $SH 消息发布 ====================

/**
 * @brief 通过本地 MQTT 客户端发布 JSON 字符串到指定主题
 *
 * Fix-Bug4: 提取 publish_sh_to_gateway 和 send_bind_command 的公共 MQTT 发布逻辑，
 * 避免锁/快照/发布代码重复，未来修改发布逻辑只需改一处。
 *
 * @param topic 目标主题
 * @param json_str JSON 字符串（调用方负责 free）
 * @param log_label 日志标签（如 "$SH 消息" / "配对命令" / "解绑命令"），用于日志区分
 */
static void publish_mqtt_json(const char *topic, const char *json_str, const char *log_label)
{
    int data_len = strlen(json_str);
    // P2-A 修复：s_mqtt_client 指针加锁保护，防止 stop 中销毁导致 use-after-free。
    // 锁仅保护指针读写（临界区极短），publish 在临界区外用局部指针执行。
    taskENTER_CRITICAL(&s_mqtt_client_lock);
    esp_mqtt_client_handle_t client = s_mqtt_client;
    bool client_connected = s_mqtt_client_started;
    taskEXIT_CRITICAL(&s_mqtt_client_lock);
    if (client == NULL) {
        ESP_LOGW(TAG, "MQTT 客户端未启动，%s丢失: topic=%s", log_label, topic);
    } else {
        if (!client_connected) {
            ESP_LOGW(TAG, "MQTT 客户端未连接，%s进 outbox 延迟发送: topic=%s", log_label, topic);
        }
        int rc = esp_mqtt_client_publish(client, topic, json_str, data_len, 1, 0);
if (rc < 0) {
// 返回值 <0 表示发布失败（客户端未连接/outbox 满等）。
ESP_LOGW(TAG, "发布失败(rc=%d)，%s丢失: topic=%s", rc, log_label, topic);
} else {
ESP_LOGD(TAG, "发布%s: topic=%s payload=%s", log_label, topic, json_str);
// 绿灯单闪：ESP → LoRa 网关消息
app_led_flash(LED_GREEN);
}
    }
}

/**
 * @brief 通过 MQTT 发布 $SH 消息到指定 LoRa 网关
 *
 * @param gw_sn LoRa 网关 SN（用于主题和 sn 字段）
 * @param ctype 消息类型
 * @param data 数据对象（调用方负责创建，此函数负责释放）
 */
static void publish_sh_to_gateway(const char *gw_sn, const char *ctype, cJSON *data)
{
    if (s_mqtt_client == NULL || data == NULL || gw_sn == NULL) {
        if (data) cJSON_Delete(data);
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "cJSON_CreateObject 失败 (root), 释放 data 避免泄漏");
        cJSON_Delete(data);  // 释放调用方传入的 data，避免内存泄漏
        return;
    }
    cJSON_AddStringToObject(root, "head", PROTOCOL_HEAD);
    cJSON_AddStringToObject(root, "ctype", ctype);
    cJSON_AddNumberToObject(root, "id", next_command_id());
    cJSON_AddStringToObject(root, "sn", gw_sn);  // sn 字段必须是 LoRa 网关 SN
    // 使用 AddItemToObject 而非 AddItemReferenceToObject，转让所有权
    // 这样 cJSON_Delete(root) 会同时释放 data
    cJSON_AddItemToObject(root, "data", data);

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        char topic[128];
        snprintf(topic, sizeof(topic), TOPIC_GATEWAY_REQ_FMT, gw_sn);
        // Fix-Bug4: 使用公共发布辅助函数，消除重复的锁/快照/发布代码
        publish_mqtt_json(topic, json_str, "$SH 消息");
        free(json_str);
    }

    cJSON_Delete(root);  // 同时释放 data（所有权已转让给 root）
}

/**
 * @brief 发送设备控制命令（ctype=004）
 *
 * @param device_sn LoRa 设备 SN
 * @param gw_sn LoRa 网关 SN
 * @param value 控制值（"100"=打开, "0"=关闭, "101"=暂停, "0"-"100"=定位）
 */
static void send_device_control(const char *device_sn, const char *gw_sn, const char *value)
{
    cJSON *data = cJSON_CreateObject();
    if (data == NULL) {
        ESP_LOGE(TAG, "cJSON_CreateObject 失败 (send_device_control data)");
        return;
    }
    cJSON_AddStringToObject(data, "sn", device_sn);
    cJSON_AddStringToObject(data, "attribute", ATTRIBUTE_W_TRAVEL);
    cJSON_AddStringToObject(data, "value", value);

    publish_sh_to_gateway(gw_sn, "004", data);
    ESP_LOGI(TAG, "发送控制命令: gw=%s dev=%s value=%s", gw_sn, device_sn, value);
}

/**
 * @brief 发送配对/解绑命令（ctype=003）
 *
 * 与 HA 集成保持一致：顶层和 data 内都设置 bind 字段。
 * - bind=1: 配对模式（data.sn=FFFFFFFFFFFF）
 * - bind=0: 解绑设备（data.sn=具体设备SN）
 *
 * @param gw_sn LoRa 网关 SN
 * @param dev_sn 设备 SN（配对时传 "FFFFFFFFFFFF"）
 * @param bind 1=配对, 0=解绑
 */
static void send_bind_command(const char *gw_sn, const char *dev_sn, int bind)
{
    if (s_mqtt_client == NULL || gw_sn == NULL || dev_sn == NULL) {
        return;
    }

    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "cJSON_CreateObject 失败 (send_bind_command root)");
        return;
    }
    cJSON_AddStringToObject(root, "head", PROTOCOL_HEAD);
    cJSON_AddStringToObject(root, "ctype", "003");
    int cmd_id = next_command_id();
    record_pending_003(cmd_id, bind);  // 记录 003 操作类型，供响应查找
    cJSON_AddNumberToObject(root, "id", cmd_id);
    cJSON_AddStringToObject(root, "sn", gw_sn);
    cJSON_AddNumberToObject(root, "bind", bind);  // 顶层 bind 字段

    cJSON *data = cJSON_CreateObject();
    if (data == NULL) {
        ESP_LOGE(TAG, "cJSON_CreateObject 失败 (send_bind_command data)");
        cJSON_Delete(root);  // root 已创建，需释放避免泄漏
        return;
    }
    cJSON_AddNumberToObject(data, "bind", bind);
    cJSON_AddStringToObject(data, "devtype", "curtain_ctr");
    cJSON_AddStringToObject(data, "sn", dev_sn);
    cJSON_AddItemToObject(root, "data", data);

    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        char topic[128];
        snprintf(topic, sizeof(topic), TOPIC_GATEWAY_REQ_FMT, gw_sn);
        // Fix-Bug4: 使用公共发布辅助函数，消除重复的锁/快照/发布代码
        publish_mqtt_json(topic, json_str, bind ? "配对命令" : "解绑命令");
        free(json_str);
    }
    cJSON_Delete(root);
}

/**
 * @brief 触发网关配对模式（60秒）
 */
static void send_start_pairing(const char *gw_sn)
{
    send_bind_command(gw_sn, "FFFFFFFFFFFF", 1);
}

// ==================== Matter→MQTT 方向处理 ====================

// 前向声明：send_unbind_device 定义在 handle_ctype_003 之后，但 handle_matter_event 需要调用
static void send_unbind_device(const char *gw_sn, const char *dev_sn);

/**
 * @brief 处理 Matter 事件（WindowCovering 位置变更）
 *
 * event.value 是已经反转的 LoRa 值（0=关闭, 100=打开）
 */
static void handle_matter_event(const matter_event_t *event)
{
    switch (event->type) {
    case MATTER_EVENT_LIFT_CHANGED: {
        // event->value 是 LoRa 值（0=关闭, 100=打开）
        // 范围校验：超范围记 ESP_LOGW 并丢弃，防止异常值下发到 LoRa 网关
        if (event->value < 0 || event->value > 100) {
            ESP_LOGW(TAG, "LIFT_CHANGED value=%d 超出范围[0,100]，丢弃: dev=%s",
                     event->value, event->device_sn);
            break;
        }
        char value_str[8];
        snprintf(value_str, sizeof(value_str), "%d", (int)event->value);

        // 查找设备所属的 LoRa 网关（线程安全，传出拷贝）
        char gw_sn[32];
        if (!find_gateway_for_device(event->device_sn, gw_sn, sizeof(gw_sn))) {
            ESP_LOGW(TAG, "未找到设备 %s 的网关，无法发送控制命令", event->device_sn);
            return;
        }
        send_device_control(event->device_sn, gw_sn, value_str);
        break;
    }
    case MATTER_EVENT_STOP_MOTION: {
        // event->value = 101（LoRa 停止命令值）
        // 校验 ==101，防止异常值下发
        if (event->value != 101) {
            ESP_LOGW(TAG, "STOP_MOTION value=%d != 101，丢弃: dev=%s",
                     event->value, event->device_sn);
            break;
        }
        char value_str[8];
        snprintf(value_str, sizeof(value_str), "%d", (int)event->value);
        char gw_sn[32];
        if (!find_gateway_for_device(event->device_sn, gw_sn, sizeof(gw_sn))) {
            ESP_LOGW(TAG, "未找到设备 %s 的网关，无法发送停止命令", event->device_sn);
            return;
        }
        send_device_control(event->device_sn, gw_sn, value_str);
        break;
    }
    case MATTER_EVENT_TILT_TOGGLE: {
        // event->value = 200（LoRa 内倒命令值）
        // 校验 ==200，防止异常值下发
        if (event->value != 200) {
            ESP_LOGW(TAG, "TILT_TOGGLE value=%d != 200，丢弃: dev=%s",
                     event->value, event->device_sn);
            break;
        }
        char value_str[8];
        snprintf(value_str, sizeof(value_str), "%d", (int)event->value);
        char gw_sn[32];
        if (!find_gateway_for_device(event->device_sn, gw_sn, sizeof(gw_sn))) {
            ESP_LOGW(TAG, "未找到设备 %s 的网关，无法发送内倒命令", event->device_sn);
            return;
        }
        send_device_control(event->device_sn, gw_sn, value_str);
        break;
    }
    case MATTER_EVENT_MODE_CHANGED: {
        // 窗锁模式变更：event->value = 0(内开内倒) 或 1(平开)
        // 发送 ctype=004, attribute=rwp_wind_lock_mode
        if (event->value != 0 && event->value != 1) {
            ESP_LOGW(TAG, "MODE_CHANGED value=%d 无效，丢弃: dev=%s",
                     event->value, event->device_sn);
            break;
        }
        char gw_sn[32];
        if (!find_gateway_for_device(event->device_sn, gw_sn, sizeof(gw_sn))) {
            ESP_LOGW(TAG, "未找到设备 %s 的网关，无法发送模式命令", event->device_sn);
            return;
        }
        // 构造模式控制命令（attribute=rwp_wind_lock_mode）
        cJSON *data = cJSON_CreateObject();
        if (data == NULL) {
            ESP_LOGE(TAG, "cJSON_CreateObject 失败 (MODE_CHANGED)");
            break;
        }
        cJSON_AddStringToObject(data, "sn", event->device_sn);
        cJSON_AddStringToObject(data, "attribute", ATTRIBUTE_WIND_LOCK_MODE);
        char value_str[4];
        snprintf(value_str, sizeof(value_str), "%d", (int)event->value);
        cJSON_AddStringToObject(data, "value", value_str);
        publish_sh_to_gateway(gw_sn, "004", data);
        ESP_LOGI(TAG, "发送窗锁模式命令: gw=%s dev=%s mode=%s",
                 gw_sn, event->device_sn, value_str);
        break;
    }
    case MATTER_EVENT_DEVICE_ADDED:
        ESP_LOGI(TAG, "Matter 设备已添加: ep=%u sn=%s", event->endpoint_id, event->device_sn);
        break;
    case MATTER_EVENT_DEVICE_REMOVED:
        ESP_LOGI(TAG, "Matter 设备已移除: ep=%u sn=%s", event->endpoint_id, event->device_sn);
        break;
    case MATTER_EVENT_COMMISSIONING_COMPLETE: {
        // P-HomeKit6 修复：配网完成后主动向所有已注册网关请求设备列表（002）。
        // 首次配网时 LoRa 网关可能在配网前就上报了 002，但当时未配网无法创建端点。
        // 配网完成后主动请求 002，加速端点创建，避免用户等待下一个 LoRa 上报周期。
        ESP_LOGI(TAG, "Matter 配网完成，向所有已注册网关请求设备列表");
        for (int i = 0; i < MAX_GATEWAYS; i++) {
            char gw_sn[32] = {0};
            bool online = false;
            taskENTER_CRITICAL(&s_gateways_lock);
            if (s_gateways[i].in_use) {
                strncpy(gw_sn, s_gateways[i].gateway_sn, sizeof(gw_sn) - 1);
                gw_sn[sizeof(gw_sn) - 1] = '\0';
                online = s_gateways[i].online;
            }
            taskEXIT_CRITICAL(&s_gateways_lock);
            if (online && gw_sn[0] != '\0') {
                cJSON *req_data = cJSON_CreateObject();
                if (req_data != NULL) {
                    publish_sh_to_gateway(gw_sn, "002", req_data);
                    ESP_LOGI(TAG, "配网完成，已请求设备列表: gw=%s", gw_sn);
                }
            }
        }
        break;
    }
    default:
        break;
    }
}

// ==================== MQTT→Matter 方向处理 ====================

/**
 * @brief 处理 ctype=001 网关绑定
 *
 * LoRa 网关上线 → 发 001（带 version/model/userid）
 * 桥接器回复 001(errcode=0, uuid=MAC) 到 gateway/{gw_sn}/req
 * 然后发送 002 请求设备列表
 */
static void handle_ctype_001(const char *gw_sn, cJSON *data, int msg_id)
{
    // 检查是否是网关信息上报（带 version/model/userid 字段）
    bool is_gateway_info = (cJSON_GetObjectItem(data, "version") != NULL ||
                            cJSON_GetObjectItem(data, "model") != NULL ||
                            cJSON_GetObjectItem(data, "userid") != NULL ||
                            cJSON_GetObjectItem(data, "vesion") != NULL);  // 注意拼写错误兼容

    if (is_gateway_info) {
        // 网关信息上报（带 version/model/userid/vesion 字段），回复 001
        ESP_LOGI(TAG, "收到网关绑定: %s", gw_sn);
        // P-Bug5: register_gateway 内部调用 Matter API，需持有 StackLock
        {
            chip::DeviceLayer::StackLock lock;
            register_gateway(gw_sn);
        }

        // 回复 001(errcode=0, uuid=bridge_sn) — MQTT 操作不需要 StackLock
        cJSON *resp_data = cJSON_CreateObject();
        if (resp_data == NULL) {
            ESP_LOGE(TAG, "cJSON_CreateObject 失败 (001 resp_data)");
        } else {
            cJSON_AddNumberToObject(resp_data, "errcode", 0);
            cJSON_AddStringToObject(resp_data, "uuid", s_bridge_sn);
            publish_sh_to_gateway(gw_sn, "001", resp_data);
            ESP_LOGI(TAG, "已回复网关绑定确认: %s", gw_sn);
        }

        // 主动请求设备列表（发 002）
        cJSON *req_data = cJSON_CreateObject();
        if (req_data == NULL) {
            ESP_LOGE(TAG, "cJSON_CreateObject 失败 (002 req_data)");
        } else {
            publish_sh_to_gateway(gw_sn, "002", req_data);
            ESP_LOGI(TAG, "已请求设备列表: %s", gw_sn);
        }
    } else {
        // 非 is_gateway_info：检查是否有 errcode（网关响应）
        cJSON *errcode = cJSON_GetObjectItem(data, "errcode");
        if (errcode) {
            // P3-3 修复：用 parse_number_field 兼容字符串/数字类型 errcode
            int err_val = parse_number_field(errcode);
            if (err_val == 0) {
                ESP_LOGI(TAG, "网关绑定成功: %s", gw_sn);
                {
                    chip::DeviceLayer::StackLock lock;
                    register_gateway(gw_sn);
                }
            } else {
                ESP_LOGW(TAG, "网关绑定失败: %s errcode=%d", gw_sn, err_val);
            }
        } else {
            // P2-5 修复：无 errcode 且无网关信息字段 = 网关主动发起的纯绑定请求
            // 参考 HA 网关 mqtt_handler.py：此情况应回复 001(errcode=0) 完成绑定
            ESP_LOGI(TAG, "收到网关纯绑定请求: %s", gw_sn);
            {
                chip::DeviceLayer::StackLock lock;
                register_gateway(gw_sn);
            }
            cJSON *resp_data = cJSON_CreateObject();
            if (resp_data != NULL) {
                cJSON_AddNumberToObject(resp_data, "errcode", 0);
                cJSON_AddStringToObject(resp_data, "uuid", s_bridge_sn);
                publish_sh_to_gateway(gw_sn, "001", resp_data);
                ESP_LOGI(TAG, "已回复纯绑定确认: %s", gw_sn);
            } else {
                ESP_LOGE(TAG, "cJSON_CreateObject 失败 (纯绑定 resp_data)");
            }
        }
    }
}

/**
 * @brief 处理 ctype=002 设备列表
 *
 * LoRa 网关主动发送 002（带 devices 数组）作为状态上报。
 * 桥接器遍历设备列表，为每个新设备创建 Matter 端点，
 * 为已有设备更新位置和电池属性，然后回复 002(errcode=0)。
 */
static void handle_ctype_002(const char *gw_sn, cJSON *data, int msg_id)
{
    // P-Bug5 优化：分两阶段处理，cJSON 解析在 StackLock 外完成，Matter API 在 StackLock 内调用。
    // 原实现在 StackLock 内遍历 12 个设备的 cJSON 数组 + 逐个调用 Matter API，
    // 可能阻塞 Matter 事件循环数秒，导致 CASE 会话超时、App 掉线。

    // Phase 1: 预解析设备数据到栈数组（无 StackLock）
    typedef struct {
        char sn[32];
        int r_travel;   // -1 = 无效/未上报
        int battery;    // -1 = 无效/未上报
    } pre_parsed_device_t;

    pre_parsed_device_t parsed_devs[MAX_DEVICE_ENTRIES];
    int device_count = 0;

    cJSON *devices = cJSON_GetObjectItem(data, "devices");
    if (cJSON_IsArray(devices)) {
        cJSON *device;
        cJSON_ArrayForEach(device, devices) {
            if (device_count >= MAX_DEVICE_ENTRIES) break;

            cJSON *sn_obj = cJSON_GetObjectItem(device, "sn");
            if (!cJSON_IsString(sn_obj)) continue;

            const char *dev_sn = sn_obj->valuestring;
            const char *model = NULL;
            cJSON *model_obj = cJSON_GetObjectItem(device, "model");
            if (cJSON_IsString(model_obj)) {
                model = model_obj->valuestring;
            }

            if (should_skip_device(dev_sn, model)) {
                ESP_LOGD(TAG, "跳过设备: sn=%s model=%s", dev_sn, model ? model : "N/A");
                continue;
            }

            strncpy(parsed_devs[device_count].sn, dev_sn, sizeof(parsed_devs[0].sn) - 1);
            parsed_devs[device_count].sn[sizeof(parsed_devs[0].sn) - 1] = '\0';

            int pos = parse_number_field(cJSON_GetObjectItem(device, "r_travel"));
            parsed_devs[device_count].r_travel = (pos >= 0 && pos <= 100) ? pos : -1;

            int volt = parse_number_field(cJSON_GetObjectItem(device, "battery"));
            if (volt >= 0 && (volt < BATTERY_RAW_MIN || volt > BATTERY_RAW_MAX)) {
                ESP_LOGW(TAG, "设备 %s 电池电压 %d 超范围，丢弃", dev_sn, volt);
                volt = -1;
            }
            parsed_devs[device_count].battery = volt;

            device_count++;
        }
    } else if (devices != NULL) {
        // devices 字段存在但非数组，格式错误
        ESP_LOGW(TAG, "002 devices 字段非数组: gw=%s", gw_sn);
        cJSON *err_resp = cJSON_CreateObject();
        if (err_resp == NULL) {
            ESP_LOGE(TAG, "cJSON_CreateObject 失败 (002 errcode=1 resp)");
        } else {
            cJSON_AddNumberToObject(err_resp, "errcode", 1);
            publish_sh_to_gateway(gw_sn, "002", err_resp);
        }
        return;
    }
    // devices == NULL：网关状态上报（无 devices），正常回复 errcode=0

    // Phase 2: 调用 Matter API 处理设备
    // 优化：StackLock 按设备逐个获取，而非整个循环持锁。
    // 原实现在 StackLock 内遍历所有设备调用 add_device（含 endpoint::create +
    // window_covering::add + endpoint::enable，每个耗时数十毫秒），
    // N 个设备持续阻塞 Matter 事件循环，可能导致 CASE Session 超时、App 显示离线。
    // 现改为：register_gateway 单独持锁；每个设备单独持锁，设备间释放锁让出事件循环。

    // Phase 2a: 注册网关（StackLock）
    {
        chip::DeviceLayer::StackLock lock;
        register_gateway(gw_sn);
    }

    // Phase 2b: 逐设备处理（每设备单独 StackLock）
    // 设备间添加延迟，避免同时创建多个端点导致 ReportData 过大（>1000字节），
    // HomeKit 无法处理会拆除订阅。每个设备单独创建+报告，让 HomeKit 逐步发现。
    int new_count = 0;
    for (int i = 0; i < device_count; i++) {
        const char *dev_sn = parsed_devs[i].sn;
        int init_pos = parsed_devs[i].r_travel;
        int init_voltage = parsed_devs[i].battery;

        // 记录设备→网关映射（非 Matter API，使用自身锁保护，不需要 StackLock）
        register_device_gateway(dev_sn, gw_sn);

        // 检查 Matter 端点是否已存在（使用设备表锁，不需要 StackLock）
        uint16_t ep_id;
        bool endpoint_ready = false;
        bool just_created = false;  // 标记本次迭代是否创建了新端点
        if (app_matter_bridge_find_endpoint(dev_sn, &ep_id) == ESP_OK) {
            ESP_LOGD(TAG, "设备 %s 端点已存在: ep=%u", dev_sn, ep_id);
            endpoint_ready = true;
        } else {
            // 创建新 Matter 端点（需要 StackLock）
            char dev_name[64];
            get_device_display_name(dev_name, sizeof(dev_name), gw_sn, dev_sn);

            {
                chip::DeviceLayer::StackLock lock;
                if (app_matter_bridge_add_device(dev_sn, dev_name, &ep_id) == ESP_OK) {
                    new_count++;
                    just_created = true;
                    ESP_LOGI(TAG, "新设备端点: ep=%u sn=%s name=%s", ep_id, dev_sn, dev_name);
                    endpoint_ready = true;
                } else {
                    ESP_LOGW(TAG, "创建 Matter 端点失败: dev=%s gw=%s", dev_sn, gw_sn);
                }
            }
        }

        // 更新位置和电池（需要 StackLock）
        if (endpoint_ready) {
            chip::DeviceLayer::StackLock lock;
            if (init_pos >= 0) {
                app_matter_bridge_update_position(ep_id, (uint8_t)init_pos);
            }
            if (init_voltage >= 0) {
                app_matter_bridge_update_battery(ep_id, (uint16_t)(init_voltage * 100), -1);
            }
        }

        // 仅在本次迭代创建了新端点且还有后续设备时延迟 3 秒，
        // 让 HomeKit 有时间处理 PartsList 变更和属性报告，
        // 避免多个设备同时创建导致 ReportData 过大（>1000字节）引发订阅拆除。
        // 修复：原条件 new_count > 0 会在首个新设备后对所有后续设备（含已存在的）
        // 都延迟 3 秒，12 设备仅 1 新增时产生 11×3s=33s 不必要延迟。
        if (just_created && i < device_count - 1) {
            ESP_LOGI(TAG, "设备间延迟 3 秒，等待 HomeKit 处理前一个端点...");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
    // StackLock 已释放

    ESP_LOGI(TAG, "设备列表处理完成: gw=%s 总计=%d 新增=%d", gw_sn, device_count, new_count);

    // Phase 3: 回复 002(errcode=0) — 协议要求，网关需要确认
    cJSON *resp_data = cJSON_CreateObject();
    if (resp_data == NULL) {
        ESP_LOGE(TAG, "cJSON_CreateObject 失败 (002 resp_data)");
    } else {
        cJSON_AddNumberToObject(resp_data, "errcode", 0);
        publish_sh_to_gateway(gw_sn, "002", resp_data);
        ESP_LOGI(TAG, "已回复设备列表确认: %s", gw_sn);
    }
}

/**
 * @brief 发送解绑命令（ctype=003, bind=0）
 *
 * 解绑指定的 LoRa 子设备。
 */
static void send_unbind_device(const char *gw_sn, const char *dev_sn)
{
    send_bind_command(gw_sn, dev_sn, 0);
}

/**
 * @brief 本地强制移除设备（Matter 端点 + 映射表）
 *
 * 参照 HA 网关 gateway.py 的做法：发送解绑命令后不等响应，
 * 固定等待后本地直接删除。此函数用于 3 击删除的兜底清理，
 * 确保即使网关不回复 003 响应，Matter 端点也能被正确移除。
 *
 * 注意：调用此函数前应已发送解绑命令并等待了一段时间。
 */
static void local_remove_device(const char *dev_sn)
{
    if (dev_sn == NULL || dev_sn[0] == '\0') {
        return;
    }

    // 1. 移除 Matter 端点（需要 StackLock）
    {
        chip::DeviceLayer::StackLock lock;
        uint16_t ep_id;
        if (app_matter_bridge_find_endpoint(dev_sn, &ep_id) == ESP_OK) {
            esp_err_t rm_err = app_matter_bridge_remove_device(ep_id);
            if (rm_err == ESP_OK) {
                ESP_LOGI(TAG, "本地清理: 已移除 Matter 端点 ep=%u sn=%s", ep_id, dev_sn);
            } else {
                ESP_LOGW(TAG, "本地清理: 移除 Matter 端点失败 ep=%u sn=%s err=0x%x",
                         ep_id, dev_sn, rm_err);
            }
        }
    }
    // StackLock 已释放

    // 2. 清理设备→网关映射表
    // 注意：ESP_LOG 不能在 taskENTER_CRITICAL 临界区内调用，
    // newlib 的 log lock 在临界区内获取会触发 lock_acquire_generic 的 abort() 检测。
    bool map_cleaned = false;
    taskENTER_CRITICAL(&s_device_map_lock);
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (s_device_gateway_map[i].in_use &&
            strcmp(s_device_gateway_map[i].device_sn, dev_sn) == 0) {
            s_device_gateway_map[i].in_use = false;
            s_device_gateway_map[i].device_sn[0] = '\0';
            s_device_gateway_map[i].gateway_sn[0] = '\0';
            s_device_gateway_map[i].voltage_mv = 0;
            s_device_gateway_map[i].state = 0;
            s_device_gateway_map[i].add_seq = 0;
            map_cleaned = true;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_device_map_lock);
    if (map_cleaned) {
        ESP_LOGI(TAG, "本地清理: 已清理设备映射 dev=%s", dev_sn);
    }

/**
 * @brief 检查设备是否仍在映射表中（用于兜底清理判断）
 */
static bool is_device_in_map(const char *dev_sn)
{
    bool found = false;
    taskENTER_CRITICAL(&s_device_map_lock);
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (s_device_gateway_map[i].in_use &&
            strcmp(s_device_gateway_map[i].device_sn, dev_sn) == 0) {
            found = true;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_device_map_lock);
    return found;
}

/**
 * @brief 处理 ctype=003 设备配对/解绑响应
 *
 * bind=1 且 errcode=0：配对成功，添加端点
 * bind=0 且 errcode=0：解绑成功，移除端点
 *
 * P-Bug8 修复：LoRa 网关的 003 响应中不包含 bind 字段，
 * 无法从响应本身判断是配对还是解绑。
 * 修复策略：通过 command_id → bind 映射追踪发送时的操作类型：
 *   - send_bind_command 发送时调用 record_pending_003(cmd_id, bind) 记录
 *   - 收到响应时调用 lookup_pending_003(msg_id) 查找并消费记录
 *   - 查找失败时回退到设备存在性推断（兼容异常情况）
 */
static void handle_ctype_003(const char *gw_sn, cJSON *data, int msg_id)
{
    cJSON *errcode = cJSON_GetObjectItem(data, "errcode");
    cJSON *sn_obj = cJSON_GetObjectItem(data, "sn");
    cJSON *bind_obj = cJSON_GetObjectItem(data, "bind");

    // P3-3 修复：用 parse_number_field 兼容字符串/数字类型 errcode
    int err_val = parse_number_field(errcode);
    if (!errcode || err_val != 0) {
        ESP_LOGW(TAG, "配对/解绑失败: gw=%s errcode=%d", gw_sn, err_val);
        return;
    }

    if (!cJSON_IsString(sn_obj)) {
        ESP_LOGW(TAG, "003 响应缺少 sn 字段");
        return;
    }

    const char *dev_sn = sn_obj->valuestring;

    // 配对响应：sn=FFFFFFFFFFFF 时需要从其他字段拿真实 SN
    // 实际网关固件配对成功后会上报新设备 SN（非 FFFFFFFFFFFF）
    if (strcmp(dev_sn, "FFFFFFFFFFFF") == 0) {
        ESP_LOGW(TAG, "配对响应 sn=FFFFFFFFFFFF，等待 002/005 上报实际设备 SN");
        return;
    }

    // 判断是配对(bind=1)还是解绑(bind=0)
    // P2-8 修复：用 parse_number_field 兼容字符串/数字类型 bind（与 errcode 一致）
    int bind_val = parse_number_field(bind_obj);
    if (bind_val < 0) {
        // P-Bug8 修复：bind 字段缺失时，优先通过 command_id 查找发送时记录的操作类型。
        // 原推断逻辑"设备已存在→解绑"在重新配对已存在设备时会误判为解绑，
        // 导致已配对设备的 Matter 端点被错误移除并引发崩溃。
        bind_val = lookup_pending_003(msg_id);
        if (bind_val >= 0) {
            ESP_LOGI(TAG, "003 响应无 bind 字段，通过命令ID查找: bind=%d (id=%d)", bind_val, msg_id);
        } else {
            // 回退推断：无命令记录时通过设备是否存在来推断（兼容旧逻辑）
            char existing_gw[32] = {0};
            bool device_exists = find_gateway_for_device(dev_sn, existing_gw, sizeof(existing_gw));
            bind_val = device_exists ? 0 : 1;
            ESP_LOGW(TAG, "003 响应无 bind 字段且无命令记录，推断为 %s (dev=%s exists=%d)",
                     bind_val == 0 ? "解绑" : "配对", dev_sn, device_exists ? 1 : 0);
        }
    }

    if (bind_val == 0) {
        // 解绑成功：移除 Matter 端点 + 清理设备→网关映射表
        ESP_LOGI(TAG, "设备解绑成功: gw=%s dev=%s", gw_sn, dev_sn);
        // P-Bug5: remove_device 是 Matter API，需持有 StackLock
        // 用嵌套作用域确保 StackLock 在映射表清理前释放，避免持锁期间进入自旋锁
        {
            chip::DeviceLayer::StackLock lock;
            uint16_t ep_id;
            if (app_matter_bridge_find_endpoint(dev_sn, &ep_id) == ESP_OK) {
                esp_err_t rm_err = app_matter_bridge_remove_device(ep_id);
                if (rm_err == ESP_OK) {
                    ESP_LOGI(TAG, "已移除 Matter 端点: ep=%u sn=%s", ep_id, dev_sn);
                } else {
                    ESP_LOGW(TAG, "移除 Matter 端点失败: ep=%u sn=%s err=0x%x",
                             ep_id, dev_sn, rm_err);
                }
            } else {
                ESP_LOGW(TAG, "解绑的设备未在 Matter 端点表中: %s", dev_sn);
            }
        }
        // StackLock 已释放，清理映射表不需要 Matter API
        // 清理设备→网关映射表条目，防止映射表空间泄漏
        // 注意：ESP_LOG 不能在 taskENTER_CRITICAL 临界区内调用，
        // newlib 的 log lock 在临界区内获取会触发 lock_acquire_generic 的 abort() 检测。
        bool map_cleaned = false;
        taskENTER_CRITICAL(&s_device_map_lock);
        for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
            if (s_device_gateway_map[i].in_use &&
                strcmp(s_device_gateway_map[i].device_sn, dev_sn) == 0) {
                s_device_gateway_map[i].in_use = false;
                s_device_gateway_map[i].device_sn[0] = '\0';
                s_device_gateway_map[i].gateway_sn[0] = '\0';
                s_device_gateway_map[i].voltage_mv = 0;
                s_device_gateway_map[i].state = 0;
                s_device_gateway_map[i].add_seq = 0;
                map_cleaned = true;
                break;
            }
        }
        taskEXIT_CRITICAL(&s_device_map_lock);
        if (map_cleaned) {
            ESP_LOGI(TAG, "已清理设备映射: dev=%s gw=%s", dev_sn, gw_sn);
        }
    } else {
        // 配对成功：添加 Matter 端点
        ESP_LOGI(TAG, "设备配对成功: gw=%s dev=%s", gw_sn, dev_sn);
        // 配对成功，关闭绿灯（停止快闪）
        app_led_off(LED_GREEN);
        register_device_gateway(dev_sn, gw_sn);

        char dev_name[64];
        get_device_display_name(dev_name, sizeof(dev_name), gw_sn, dev_sn);

        // P-Bug5: add_device 是 Matter API，需持有 StackLock
        chip::DeviceLayer::StackLock lock;
        uint16_t ep_id;
        esp_err_t add_err = app_matter_bridge_add_device(dev_sn, dev_name, &ep_id);
        if (add_err == ESP_OK) {
            ESP_LOGI(TAG, "已创建 Matter 端点: ep=%u sn=%s", ep_id, dev_sn);
        } else if (add_err == ESP_ERR_INVALID_STATE) {
            // 设备已存在（重新配对场景），端点保持不变，无需重复创建
            ESP_LOGI(TAG, "设备 %s 端点已存在，无需重复创建", dev_sn);
        } else {
            ESP_LOGW(TAG, "创建 Matter 端点失败: dev=%s gw=%s err=%s",
                     dev_sn, gw_sn, esp_err_to_name(add_err));
        }
    }
}

/**
 * @brief 处理 ctype=005 设备状态上报
 *
 * 支持两种格式：
 * 1. 直接字段：{"sn":"xxx","position":80,"battery":105}
 * 2. attrs 数组：{"sn":"xxx","attrs":[{"attribute":"r_travel","value":"80"}]}
 */
static void handle_ctype_005(const char *gw_sn, cJSON *data, int msg_id)
{
    // P-Bug5: register_gateway 内部调用 Matter API（add_gateway），需持有 StackLock
    {
        chip::DeviceLayer::StackLock lock;
        register_gateway(gw_sn);
    }

    cJSON *dev_sn_obj = cJSON_GetObjectItem(data, "sn");
    if (!cJSON_IsString(dev_sn_obj)) {
        ESP_LOGW(TAG, "005 上报缺少 sn 字段");
        return;
    }

    const char *dev_sn = dev_sn_obj->valuestring;

    // 设备过滤：跳过网关设备（SN 以 "1001" 开头）
    // 防止网关自身的 005 上报被当作子设备处理，占用映射表空间
    if (should_skip_device(dev_sn, NULL)) {
        ESP_LOGD(TAG, "005 上报跳过网关设备: sn=%s", dev_sn);
        return;
    }

    int position = -1;
    int voltage = -1;
    int state = -1;

    // 格式1：直接字段 position/r_travel/voltage/battery/state
    // 字段名对照 HA 集成 mqtt_handler.py：
    //   - 顶层电池字段名是 "battery"（原始值÷10=伏特，如 105 → 10.5V）
    //   - attrs 数组中 attribute 名是 "voltage"（同样÷10）
    //   - 本项目内部统一用毫伏，转换：原始值 × 100 = 毫伏
    // 用 parse_number_field 兼容字符串/数字类型（HA 集成 float() 同样兼容）
    cJSON *position_obj = cJSON_GetObjectItem(data, "position");
    int pos_val = parse_number_field(position_obj);
    // 上限校验：position > 100 表示未校准/离线标记（与 r_travel 一致），
    // 不覆盖有效 position，避免无效值进入后续处理
    if (pos_val >= 0 && pos_val <= 100) position = pos_val;
    cJSON *r_travel_obj = cJSON_GetObjectItem(data, "r_travel");
    int rt_val = parse_number_field(r_travel_obj);
    // P3-2 修复：添加 r_travel <= 100 范围检查，与 attrs 数组分支和 handle_ctype_002 一致。
    // r_travel=255 等超范围值表示设备未校准/离线标记，不覆盖有效 position。
    if (rt_val >= 0 && rt_val <= 100) {
        // r_travel 覆盖 position 时记录日志，便于排查字段冲突
        if (position >= 0 && rt_val != pos_val) {
            ESP_LOGW(TAG, "r_travel(%d) 覆盖 position(%d): dev=%s", rt_val, pos_val, dev_sn);
        }
        position = rt_val;
    }

    // battery：HA 集成中 ctype=005 顶层实际字段名（mqtt_handler.py:1124）
    cJSON *battery_obj = cJSON_GetObjectItem(data, "battery");
    int bat_val = parse_number_field(battery_obj);
    // voltage：兼容部分固件直接发 voltage 字段的情况
    cJSON *voltage_obj = cJSON_GetObjectItem(data, "voltage");
    int vol_val = parse_number_field(voltage_obj);
    // 范围校验：先对直接字段做范围检查，再选择有效值。
    // 原实现在选择值后才做范围校验，当 voltage 超范围但 battery 有效时，
    // voltage 被选中后在后续范围校验中被丢弃，有效的 battery 值永远不会被使用。
    if (vol_val >= 0 && (vol_val < BATTERY_RAW_MIN || vol_val > BATTERY_RAW_MAX)) {
        ESP_LOGW(TAG, "voltage(%d) 超范围[%d,%d]，丢弃", vol_val, BATTERY_RAW_MIN, BATTERY_RAW_MAX);
        vol_val = -1;
    }
    if (bat_val >= 0 && (bat_val < BATTERY_RAW_MIN || bat_val > BATTERY_RAW_MAX)) {
        ESP_LOGW(TAG, "battery(%d) 超范围[%d,%d]，丢弃", bat_val, BATTERY_RAW_MIN, BATTERY_RAW_MAX);
        bat_val = -1;
    }
    // 两字段同时有效时检查值一致性，优先采用 voltage
    if (bat_val >= 0 && vol_val >= 0 && bat_val != vol_val) {
        ESP_LOGW(TAG, "battery(%d) 与 voltage(%d) 值不一致，采用 voltage", bat_val, vol_val);
    }
    if (vol_val >= 0) {
        voltage = vol_val;
    } else if (bat_val >= 0) {
        voltage = bat_val;
    }

    cJSON *state_obj = cJSON_GetObjectItem(data, "state");
    int st_val = parse_number_field(state_obj);
    // P3-8 修复：state 语义应为 0=关闭/1=打开，校验范围（与 attrs 数组分支一致）
    if (st_val >= 0 && st_val <= 1) state = st_val;

    // 格式2：attrs 数组
    cJSON *attrs = cJSON_GetObjectItem(data, "attrs");
    if (cJSON_IsArray(attrs)) {
        cJSON *attr;
        cJSON_ArrayForEach(attr, attrs) {
            cJSON *attr_name = cJSON_GetObjectItem(attr, "attribute");
            cJSON *attr_value = cJSON_GetObjectItem(attr, "value");
            if (cJSON_IsString(attr_name) && attr_value) {
                const char *name = attr_name->valuestring;
                // P2-5 修复：统一用 parse_number_field 替代 atoi
                // atoi("abc")=0 无法区分无效和合法值 0，parse_number_field 返回 -1 表示无效
                int val_int = parse_number_field(attr_value);

                if (strcmp(name, "r_travel") == 0) {
                    // P1-5 修复：r_travel 值 >100 表示未校准/离线标记，不覆盖有效 position
                    if (val_int >= 0 && val_int <= 100) position = val_int;
                } else if (strcmp(name, "voltage") == 0) {
                    // P1-5 修复：voltage 超出有效范围不覆盖
                    if (val_int >= BATTERY_RAW_MIN && val_int <= BATTERY_RAW_MAX) voltage = val_int;
                } else if (strcmp(name, "state") == 0) {
                    // P3-8 修复：state 语义应为 0=关闭/1=打开，校验范围
                    if (val_int >= 0 && val_int <= 1) state = val_int;
                }
            }
        }
    }

    // 记录设备→网关映射（确保后续控制命令能找到网关）
    register_device_gateway(dev_sn, gw_sn);

    // 电池电压范围校验（防御性）：直接字段已在上方校验，此处仅对 attrs 数组
    // 分支设置的 voltage 做兜底检查。异常值（如网关故障发 700）会导致 uint16 溢出和错误显示。
    if (voltage >= 0 && (voltage < BATTERY_RAW_MIN || voltage > BATTERY_RAW_MAX)) {
        ESP_LOGW(TAG, "电池电压原始值 %d 超出范围[%d,%d]，丢弃", voltage, BATTERY_RAW_MIN, BATTERY_RAW_MAX);
        voltage = -1;
    }

    // 缓存 voltage 和 state 到设备表（供调试和扩展使用）
    // HA 集成中 voltage 原始值÷10 得到伏特，这里存毫伏
    taskENTER_CRITICAL(&s_device_map_lock);
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (s_device_gateway_map[i].in_use &&
            strcmp(s_device_gateway_map[i].device_sn, dev_sn) == 0) {
            if (voltage >= 0) {
                s_device_gateway_map[i].voltage_mv = (uint16_t)(voltage * 100);
            }
            if (state >= 0) {
                s_device_gateway_map[i].state = (uint8_t)state;
            }
            break;
        }
    }
    taskEXIT_CRITICAL(&s_device_map_lock);

    // 打印完整状态上报
    ESP_LOGI(TAG, "状态上报: gw=%s dev=%s pos=%d volt=%d state=%d",
             gw_sn, dev_sn, position, voltage, state);

    // 端点存在性处理（与属性更新解耦）
    // 关键修复：原逻辑把端点创建包裹在 position<=100 条件内，导致 pos=255
    // （未校准/离线设备）永远走 else if 分支，find_endpoint 失败时静默丢弃，
    // 设备无法在 App 中显示。修复后：先确保端点存在，再按属性有效性分别更新。
    // P-Bug5 优化：StackLock 按操作粒度获取，避免长时间阻塞 Matter 事件循环。
    // find_endpoint 使用设备表锁，不需要 StackLock；add_device 单独持锁；
    // update_position/update_battery 合并持锁。
    uint16_t ep_id;
    bool endpoint_ready = false;
    bool just_created = false;
    if (app_matter_bridge_find_endpoint(dev_sn, &ep_id) == ESP_OK) {
        endpoint_ready = true;
    } else {
        ESP_LOGW(TAG, "收到未知设备状态: dev=%s，尝试创建端点", dev_sn);
        char dev_name[64];
        get_device_display_name(dev_name, sizeof(dev_name), gw_sn, dev_sn);
        {
            chip::DeviceLayer::StackLock lock;
            if (app_matter_bridge_add_device(dev_sn, dev_name, &ep_id) == ESP_OK) {
                endpoint_ready = true;
                just_created = true;
            } else {
                ESP_LOGW(TAG, "创建端点失败: dev=%s（可能未配网，等配网后重启自动创建）", dev_sn);
            }
        }
    }

    // Fix-Bug1: 新端点创建后延迟 1 秒，让 HomeKit 处理 PartsList 变更和属性报告。
    // 005 是单设备消息，但网关批量配对后可能连续发送多条 005，
    // 不延迟会导致多个端点同时创建，ReportData 过大（>1000字节）引发订阅拆除。
    // 延迟 1 秒（短于 002 的 3 秒，因为 005 是单设备而非批量）。
    if (just_created) {
        ESP_LOGI(TAG, "新端点创建后延迟 1 秒，等待 HomeKit 处理...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // 端点就绪后，按属性有效性分别更新
    if (endpoint_ready) {
        chip::DeviceLayer::StackLock lock;
        if (position >= 0 && position <= 100) {
            app_matter_bridge_update_position(ep_id, (uint8_t)position);
        }
        if (voltage >= 0) {
            app_matter_bridge_update_battery(ep_id, (uint16_t)(voltage * 100), -1);
        }
    }
}

// ==================== MQTT 消息解析与分发 ====================

/**
 * @brief 处理 MQTT 消息（来自 broker 的 $SH 协议消息）
 */
static void handle_mqtt_message(const mqtt_message_t *msg)
{
    if (msg->data_len <= 0) {
        return;
    }

    // 过滤自身发布的消息：mosquitto broker 对所有 PUBLISH 都回调（handle_publish.c:274），
    // 包括本地 MQTT 客户端（client_id=s_bridge_sn）发出的 001/002 响应。
    // 不过滤会导致 002 响应被自己再次处理 → 又回复 002 → 无限循环（Bug1）。
    if (s_bridge_sn[0] != '\0' && strcmp(msg->client_id, s_bridge_sn) == 0) {
        ESP_LOGD(TAG, "过滤自身发布的消息: topic=%s", msg->topic);
        return;
    }

    ESP_LOGD(TAG, "收到 MQTT: topic=%s data=%.*s", msg->topic, msg->data_len, msg->data);

    // 绿灯单闪：LoRa 网关 → ESP 消息
    app_led_flash(LED_GREEN);

    // 解析 JSON
    cJSON *root = cJSON_ParseWithLength(msg->data, msg->data_len);
    if (root == NULL) {
        ESP_LOGW(TAG, "JSON 解析失败");
        return;
    }

    // 校验 $SH 协议头
    cJSON *head = cJSON_GetObjectItem(root, "head");
    if (!cJSON_IsString(head) || strcmp(head->valuestring, PROTOCOL_HEAD) != 0) {
        cJSON_Delete(root);
        return;
    }

    // 获取消息类型和网关 SN
    cJSON *ctype_obj = cJSON_GetObjectItem(root, "ctype");
    cJSON *sn_obj = cJSON_GetObjectItem(root, "sn");
    cJSON *id_obj = cJSON_GetObjectItem(root, "id");
    cJSON *data = cJSON_GetObjectItem(root, "data");

    if (!cJSON_IsString(ctype_obj) || !cJSON_IsString(sn_obj)) {
        cJSON_Delete(root);
        return;
    }

    const char *ctype = ctype_obj->valuestring;
    const char *gw_sn = sn_obj->valuestring;
    int msg_id = cJSON_IsNumber(id_obj) ? id_obj->valueint : 0;

    bool data_created_locally = false;
    if (!cJSON_IsObject(data)) {
        data = cJSON_CreateObject();  // 空数据对象
        if (data == NULL) {
            ESP_LOGE(TAG, "cJSON_CreateObject 失败 (空 data)");
            cJSON_Delete(root);
            return;
        }
        data_created_locally = true;
    }

    ESP_LOGD(TAG, "处理 $SH: ctype=%s gw=%s id=%d", ctype, gw_sn, msg_id);

    // P1-4 修复：cJSON 解析已在 StackLock 外完成。
    // P-Bug5 优化：StackLock 从 handle_mqtt_message 移到各 handle_ctype_* 函数内部，
    // 仅在调用 Matter API（add_gateway/add_device/update_*/remove_device）时持锁。
    // 原 handle_mqtt_message 全程持锁会导致 handle_ctype_002 的 cJSON_ArrayForEach
    // 循环（最多 12 个设备）阻塞 Matter 事件循环数秒，引发 CASE 会话超时、App 掉线。

    // 分发到对应的处理函数（各函数内部自行管理 StackLock）
    if (strcmp(ctype, "001") == 0) {
        handle_ctype_001(gw_sn, data, msg_id);
    } else if (strcmp(ctype, "002") == 0) {
        handle_ctype_002(gw_sn, data, msg_id);
    } else if (strcmp(ctype, "003") == 0) {
        handle_ctype_003(gw_sn, data, msg_id);
    } else if (strcmp(ctype, "005") == 0) {
        handle_ctype_005(gw_sn, data, msg_id);
    } else if (strcmp(ctype, "004") == 0) {
        // 004 响应（设备控制结果），暂时只记录日志
        // P3-3 修复：用 parse_number_field 兼容字符串/数字类型 errcode
        cJSON *errcode = cJSON_GetObjectItem(data, "errcode");
        int err_val = parse_number_field(errcode);
        ESP_LOGD(TAG, "控制响应: gw=%s errcode=%d", gw_sn, err_val);
    } else {
        ESP_LOGD(TAG, "未处理的 ctype=%s", ctype);
    }

    if (data_created_locally) {
        cJSON_Delete(data);  // 释放本地创建的空对象
    }
    cJSON_Delete(root);
}

// ==================== MQTT 客户端事件回调 ====================
// 本地 MQTT 客户端仅用于发布命令（gateway/{gw}/req），不处理接收消息。
// 所有消息接收由 mosquitto broker 的 on_broker_message 回调统一处理，
// 推入队列后由 bridge_task 处理。此处订阅 rpt_rsp 会导致同一条消息
// 被重复处理（broker 回调 + 本地客户端事件），故删除订阅。

static void mqtt_client_event_handler(void *handler_args, esp_event_base_t base,
                                       int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;
    (void)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        // P2-4 修复：加锁保护 s_mqtt_client_started/starting 的读写，
        // 避免与 on_wifi_connected 中的检查-设置序列竞态
        taskENTER_CRITICAL(&s_mqtt_client_lock);
        s_mqtt_client_started = true;
        s_mqtt_client_starting = false;  // 清除启动中标志
        taskEXIT_CRITICAL(&s_mqtt_client_lock);
        ESP_LOGI(TAG, "本地 MQTT 客户端已连接（仅用于发布命令）");
        break;
    case MQTT_EVENT_DISCONNECTED:
        taskENTER_CRITICAL(&s_mqtt_client_lock);
        s_mqtt_client_started = false;
        // P3-9 修复：同步清除 starting 标志。
        // 认证失败（密码错误/broker 拒绝）只触发 DISCONNECTED 无 CONNECTED，
        // 若不清除 starting，后续 on_wifi_connected 走 L1373 分支返回 ESP_OK 拒绝重试，
        // MQTT 客户端永远无法启动，直到重启。
        s_mqtt_client_starting = false;
        taskEXIT_CRITICAL(&s_mqtt_client_lock);
        ESP_LOGW(TAG, "本地 MQTT 客户端断开，将自动重连");
        break;
    default:
        break;
    }
}

// ==================== Bridge Task ====================

static void bridge_task(void *arg)
{
    ESP_LOGI(TAG, "协议桥接 task 已启动");

    // 使用 QueueSet 同时监听两个队列
    // QueueSet 容量必须 = 所有加入队列的深度之和（不是成员数量！）
    // main.cpp 中两个队列深度均为 10，故 QueueSet 容量为 20
    // 若容量不足，多消息并发时 xQueueSend 会触发 QueueSet 内部断言失败 abort
    QueueSetHandle_t queue_set = xQueueCreateSet(MQTT_MSG_QUEUE_SIZE + MATTER_EVENT_QUEUE_SIZE);
    if (queue_set == NULL) {
        ESP_LOGE(TAG, "创建 QueueSet 失败");
        // P2-12 修复：错误分支退出前 give 信号量，避免 stop 等待 2s 超时
        if (s_bridge_exit_sem != NULL) xSemaphoreGive(s_bridge_exit_sem);
        s_bridge_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (xQueueAddToSet(s_mqtt_msg_queue, queue_set) != pdPASS) {
        ESP_LOGE(TAG, "xQueueAddToSet(s_mqtt_msg_queue) 失败");
        vQueueDelete(queue_set);
        if (s_bridge_exit_sem != NULL) xSemaphoreGive(s_bridge_exit_sem);
        s_bridge_task = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (xQueueAddToSet(s_matter_event_queue, queue_set) != pdPASS) {
        ESP_LOGE(TAG, "xQueueAddToSet(s_matter_event_queue) 失败");
        vQueueDelete(queue_set);
        if (s_bridge_exit_sem != NULL) xSemaphoreGive(s_bridge_exit_sem);
        s_bridge_task = NULL;
        vTaskDelete(NULL);
        return;
    }

    mqtt_message_t mqtt_msg;
    matter_event_t matter_event;

    while (s_bridge_running) {
        QueueSetMemberHandle_t member = xQueueSelectFromSet(queue_set, pdMS_TO_TICKS(1000));

        if (member == s_mqtt_msg_queue) {
            if (xQueueReceive(s_mqtt_msg_queue, &mqtt_msg, 0) == pdTRUE) {
                // P1-4 修复：StackLock 已移到 handle_mqtt_message 内部分发处，
                // 让 cJSON 解析在锁外完成，避免阻塞 Matter 事件循环。
                // handle_mqtt_message 内部调用 Matter API（add_gateway/add_device/
                // update_reachable）时会自动加锁。
                handle_mqtt_message(&mqtt_msg);
            }
        } else if (member == s_matter_event_queue) {
            if (xQueueReceive(s_matter_event_queue, &matter_event, 0) == pdTRUE) {
                handle_matter_event(&matter_event);
            }
        }
    }

    // 移除成员队列后再删除 QueueSet，避免其他 task 向仍注册在 QueueSet 中的
    // 队列发送消息时访问已释放的 QueueSet 内部数据结构（FreeRTOS 规范要求）
    xQueueRemoveFromSet(s_mqtt_msg_queue, queue_set);
    xQueueRemoveFromSet(s_matter_event_queue, queue_set);
    vQueueDelete(queue_set);
    s_bridge_task = NULL;
    // P2-7 修复：通知 stop 调用方 task 已退出（替代 eTaskGetState 轮询）
    if (s_bridge_exit_sem != NULL) {
        xSemaphoreGive(s_bridge_exit_sem);
    }
    vTaskDelete(NULL);
}

// ==================== 公共 API ====================

esp_err_t app_protocol_bridge_init(const protocol_bridge_config_t *config)
{
    if (config == NULL || config->bridge_sn == NULL) {
        ESP_LOGE(TAG, "无效的配置参数");
        return ESP_ERR_INVALID_ARG;
    }

    strncpy(s_bridge_sn, config->bridge_sn, sizeof(s_bridge_sn) - 1);
    s_bridge_sn[sizeof(s_bridge_sn) - 1] = '\0';
    s_mqtt_msg_queue = config->mqtt_msg_queue;
    s_matter_event_queue = config->matter_event_queue;

    // 创建本地 MQTT 客户端（连接到本地 broker，用于发布命令）
    // 注意：不立即启动，等待 WiFi 连接成功后再启动
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.hostname = CONFIG_MQTT_BROKER_HOST;
    mqtt_cfg.broker.address.port = CONFIG_BROKER_PORT;
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg.credentials.client_id = s_bridge_sn;
    mqtt_cfg.credentials.username = CONFIG_MQTT_BROKER_USERNAME;
    mqtt_cfg.credentials.authentication.password = CONFIG_MQTT_BROKER_PASSWORD;

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "创建 MQTT 客户端失败");
        return ESP_FAIL;
    }

    // IDF v5.5.4 中 ESP_EVENT_ANY_ID 是 int 类型，需要强制转换为 esp_mqtt_event_id_t
    esp_mqtt_client_register_event(s_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID,
                                    mqtt_client_event_handler, NULL);

    ESP_LOGI(TAG, "协议桥接层已初始化: bridge_sn=%s（MQTT 客户端等待 WiFi 连接）", s_bridge_sn);
    return ESP_OK;
}

esp_err_t app_protocol_bridge_on_wifi_connected(void)
{
    if (s_mqtt_client == NULL) {
        ESP_LOGE(TAG, "MQTT 客户端未初始化，忽略 WiFi 连接事件");
        return ESP_ERR_INVALID_STATE;
    }

    // P2-4 修复：检查-设置序列加锁，避免 WiFi 重连时 IP_EVENT 重复触发导致双重启动
    taskENTER_CRITICAL(&s_mqtt_client_lock);
    if (s_mqtt_client_started) {
        taskEXIT_CRITICAL(&s_mqtt_client_lock);
        return ESP_OK;  // 已经连接
    }
    if (s_mqtt_client_starting) {
        taskEXIT_CRITICAL(&s_mqtt_client_lock);
        ESP_LOGW(TAG, "MQTT 客户端正在启动中，忽略重复调用");
        return ESP_OK;
    }
    s_mqtt_client_starting = true;
    taskEXIT_CRITICAL(&s_mqtt_client_lock);

    ESP_LOGI(TAG, "WiFi 已连接，启动本地 MQTT 客户端...");

    // 启动本地 MQTT 客户端
    esp_err_t err = esp_mqtt_client_start(s_mqtt_client);
    if (err != ESP_OK) {
        taskENTER_CRITICAL(&s_mqtt_client_lock);
        s_mqtt_client_starting = false;
        taskEXIT_CRITICAL(&s_mqtt_client_lock);
        ESP_LOGE(TAG, "启动 MQTT 客户端失败: %s", esp_err_to_name(err));
        return err;
    }

    // 注意：s_mqtt_client_started 在 MQTT_EVENT_CONNECTED 事件中才置位
    // 避免在连接建立前就标记为已连接（认证失败/broker 未就绪等情况）
    ESP_LOGI(TAG, "本地 MQTT 客户端启动中，等待连接 %s:%d ...",
             CONFIG_MQTT_BROKER_HOST, CONFIG_BROKER_PORT);
    return ESP_OK;
}

esp_err_t app_protocol_bridge_start(void)
{
    if (s_bridge_running) {
        return ESP_OK;
    }

    // 注意：MQTT 客户端不在此处启动，等待 WiFi 连接成功后由事件处理器触发
    // bridge_task 会处理队列消息，MQTT 客户端连接成功后才能发布消息

    // P2-7 修复：创建退出信号量（用于 stop 等待 bridge_task 退出）
    if (s_bridge_exit_sem == NULL) {
        s_bridge_exit_sem = xSemaphoreCreateBinary();
        if (s_bridge_exit_sem == NULL) {
            ESP_LOGE(TAG, "创建 bridge_exit_sem 失败");
            return ESP_ERR_NO_MEM;
        }
    }

    s_bridge_running = true;

    BaseType_t ret = xTaskCreatePinnedToCore(
        bridge_task,
        "proto_bridge",
        BRIDGE_TASK_STACK_SIZE,
        NULL,
        BRIDGE_TASK_PRIORITY,
        &s_bridge_task,
        BRIDGE_TASK_CORE
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 bridge task 失败");
        s_bridge_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "协议桥接 task 已创建, core=%d", BRIDGE_TASK_CORE);
    return ESP_OK;
}

void app_protocol_bridge_stop(void)
{
    s_bridge_running = false;
    // P2-7 修复：用信号量替代 eTaskGetState 轮询，避免 use-after-free
    // bridge_task 退出前会 give s_bridge_exit_sem，此处 take 等待
    if (s_bridge_exit_sem != NULL && s_bridge_task != NULL) {
        // 清除可能残留的 token
        xSemaphoreTake(s_bridge_exit_sem, 0);
        // 等待 bridge_task 退出（最多 2 秒，大于 queue select 超时 1s）
        BaseType_t took = xSemaphoreTake(s_bridge_exit_sem, pdMS_TO_TICKS(2000));
        if (took != pdTRUE) {
            ESP_LOGW(TAG, "等待 bridge_task 退出超时（2s），强制销毁 mqtt_client");
        }
    }
    if (s_mqtt_client) {
        // P2-A 修复：加锁保存指针并置 NULL，解锁后再 stop+destroy，
        // 防止按键 task 在 publish 时 use-after-free
        taskENTER_CRITICAL(&s_mqtt_client_lock);
        esp_mqtt_client_handle_t client = s_mqtt_client;
        s_mqtt_client = NULL;
        taskEXIT_CRITICAL(&s_mqtt_client_lock);
        esp_mqtt_client_stop(client);
        esp_mqtt_client_destroy(client);
    }
    // P2-6 修复：重置 MQTT 状态标志，防止 stop+init 后客户端无法启动
    taskENTER_CRITICAL(&s_mqtt_client_lock);
    s_mqtt_client_started = false;
    s_mqtt_client_starting = false;
    taskEXIT_CRITICAL(&s_mqtt_client_lock);
}

void app_protocol_bridge_check_gateway_offline(void)
{
    TickType_t now = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(GATEWAY_OFFLINE_TIMEOUT_SEC * 1000);
    // P3-4 修复：离线超过 1 小时（3600s）清理表项，允许新网关复用槽位
    TickType_t cleanup_ticks = pdMS_TO_TICKS(3600 * 1000);
    for (int i = 0; i < MAX_GATEWAYS; i++) {
        // 合并 check+update 为单个临界区，消除 TOCTOU 竞态：
        // 原实现先读后写分两个临界区，中间 register_gateway 可能把 online 恢复为 true，
        // 随后被本函数错误覆盖为 false。现在临界区内完成判定+置位，原子不可分。
        char gw_sn_buf[32] = {0};
        bool should_mark_offline = false;
        bool should_cleanup = false;
        taskENTER_CRITICAL(&s_gateways_lock);
        if (s_gateways[i].in_use && s_gateways[i].online &&
            (now - s_gateways[i].last_seen) > timeout_ticks) {
            s_gateways[i].online = false;
            should_mark_offline = true;
            strncpy(gw_sn_buf, s_gateways[i].gateway_sn, sizeof(gw_sn_buf) - 1);
            gw_sn_buf[sizeof(gw_sn_buf) - 1] = '\0';
        }
        // P3-4: 离线超过 1 小时清理表项（in_use=false），允许新网关复用
        // P3-4 修复：先拷贝 gw_sn_buf 再清除，避免日志打印空 SN
        if (s_gateways[i].in_use && !s_gateways[i].online &&
            (now - s_gateways[i].last_seen) > cleanup_ticks) {
            // 仅在 should_mark_offline 未填充 gw_sn_buf 时拷贝，避免重复写入
            // （两个分支填充的是同一网关 SN，覆盖也无害，此处仅为语义清晰）
            if (!should_mark_offline) {
                strncpy(gw_sn_buf, s_gateways[i].gateway_sn, sizeof(gw_sn_buf) - 1);
                gw_sn_buf[sizeof(gw_sn_buf) - 1] = '\0';
            }
            s_gateways[i].in_use = false;
            s_gateways[i].gateway_sn[0] = '\0';
            should_cleanup = true;
        }
        taskEXIT_CRITICAL(&s_gateways_lock);

        if (should_cleanup) {
            ESP_LOGI(TAG, "LoRa 网关离线超过 1 小时，清理网关表项: %s", gw_sn_buf);
            // P3-1 修复：不再清理 s_device_gateway_map（设备→网关映射表）。
            // 原因：Matter 端点仍存在，清理映射表会导致网关恢复上报时，
            // register_gateway 的 INVALID_STATE 分支无法从映射表找到子设备 SN，
            // 无法更新子设备 Reachable=true，导致子设备永久显示离线。
            // 映射表保留无害：网关恢复后 002 会刷新映射，相同 SN 会更新映射，
            // 不同 SN 不会复用旧映射（按 device_sn 精确匹配）。
        }

        if (!should_mark_offline) {
            continue;
        }

        ESP_LOGW(TAG, "LoRa 网关离线（%ds 无消息）: %s",
                 GATEWAY_OFFLINE_TIMEOUT_SEC, gw_sn_buf);
        // 不删除 Matter 端点，避免 App 中设备频繁消失/出现；
        // 网关恢复上报时 register_gateway 会重新标记 online=true
        // 网关本身不再有 Matter 端点（已移除虚拟设备），仅更新子设备 Reachable

        // P3-9 修复：update_reachable 是 Matter API（attribute::update +
        // MatterReportingAttributeChangeCallback），必须持有 StackLock，
        // 否则在 debug 构建中触发 chipDie abort。
        // 锁层次规则：先完成所有 taskENTER_CRITICAL 和 find_endpoint（设备表锁），
        // 再加 StackLock 调用 Matter API，避免在 StackLock 内进入自旋锁。

        // 1. StackLock 外：临界区内收集子设备 SN 快照
        char dev_sns[MAX_DEVICE_ENTRIES][32];
        int dev_count = 0;
        taskENTER_CRITICAL(&s_device_map_lock);
        for (int j = 0; j < MAX_DEVICE_ENTRIES; j++) {
            if (s_device_gateway_map[j].in_use &&
                strcmp(s_device_gateway_map[j].gateway_sn, gw_sn_buf) == 0) {
                strncpy(dev_sns[dev_count], s_device_gateway_map[j].device_sn,
                        sizeof(dev_sns[dev_count]) - 1);
                dev_sns[dev_count][sizeof(dev_sns[dev_count]) - 1] = '\0';
                dev_count++;
            }
        }
        taskEXIT_CRITICAL(&s_device_map_lock);

        // 2. StackLock 外：收集子设备端点 ID（find_endpoints 内部用设备表锁，不与 StackLock 冲突）
        // Bug 修复：同时收集主端点和模式端点 ID，网关离线时两者都需标记为不可达
        uint16_t dev_ep_ids[MAX_DEVICE_ENTRIES * 2];
        int dev_ep_count = 0;
        for (int k = 0; k < dev_count; k++) {
            uint16_t ep_id, mode_ep_id;
            if (app_matter_bridge_find_endpoints(dev_sns[k], &ep_id, &mode_ep_id) == ESP_OK) {
                dev_ep_ids[dev_ep_count++] = ep_id;
                if (mode_ep_id != 0) {
                    dev_ep_ids[dev_ep_count++] = mode_ep_id;
                }
            }
        }

        // 3. 加 StackLock：批量调用所有 Matter API（update_reachable）
        chip::DeviceLayer::StackLock lock;
        for (int k = 0; k < dev_ep_count; k++) {
            app_matter_bridge_update_reachable(dev_ep_ids[k], false);
        }
    }
}

void app_protocol_bridge_start_pairing(void)
{
    // 遍历所有已注册网关，对在线网关发送配对命令
    for (int i = 0; i < MAX_GATEWAYS; i++) {
        char gw_sn[32] = {0};
        bool online = false;
        taskENTER_CRITICAL(&s_gateways_lock);
        if (s_gateways[i].in_use && s_gateways[i].online) {
            strncpy(gw_sn, s_gateways[i].gateway_sn, sizeof(gw_sn) - 1);
            online = true;
        }
        taskEXIT_CRITICAL(&s_gateways_lock);

        if (!online) {
            continue;
        }

        send_start_pairing(gw_sn);
        ESP_LOGI(TAG, "按键触发网关配对模式: %s", gw_sn);
    }
}

void app_protocol_bridge_delete_last_device(void)
{
    // 遍历所有设备映射，找到 add_seq 最大的子设备（最后添加的）
    char last_dev_sn[32] = {0};
    char last_gw_sn[32] = {0};
    uint32_t max_seq = 0;
    bool found = false;

    taskENTER_CRITICAL(&s_device_map_lock);
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (s_device_gateway_map[i].in_use) {
            if (!found || s_device_gateway_map[i].add_seq > max_seq) {
                found = true;
                max_seq = s_device_gateway_map[i].add_seq;
                strncpy(last_dev_sn, s_device_gateway_map[i].device_sn, sizeof(last_dev_sn) - 1);
                last_dev_sn[sizeof(last_dev_sn) - 1] = '\0';
                strncpy(last_gw_sn, s_device_gateway_map[i].gateway_sn, sizeof(last_gw_sn) - 1);
                last_gw_sn[sizeof(last_gw_sn) - 1] = '\0';
            }
        }
    }
    taskEXIT_CRITICAL(&s_device_map_lock);

    if (found) {
        ESP_LOGI(TAG, "3击删除最后子设备: gw=%s dev=%s (add_seq=%lu)", last_gw_sn, last_dev_sn, max_seq);
        send_unbind_device(last_gw_sn, last_dev_sn);

        // 兜底清理：参照 HA 网关做法，等待3秒后检查设备是否仍存在，
        // 若仍在则本地强制移除（网关可能未回复003响应）
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (is_device_in_map(last_dev_sn)) {
            ESP_LOGW(TAG, "网关未回复解绑响应，本地强制移除: %s", last_dev_sn);
            local_remove_device(last_dev_sn);
        }
    } else {
        ESP_LOGW(TAG, "没有可删除的子设备");
    }
}

void app_protocol_bridge_delete_all_devices(void)
{
    // P-Del1: 3击改为删除所有 LoRa 子设备
    // 遍历所有设备映射，逐个发送解绑命令（ctype=003, bind=0）
    // 使用快照模式：临界区内复制所有 device_sn + gateway_sn，临界区外发送 MQTT
    char dev_sns[MAX_DEVICE_ENTRIES][32];
    char gw_sns[MAX_DEVICE_ENTRIES][32];
    int dev_count = 0;

    taskENTER_CRITICAL(&s_device_map_lock);
    for (int i = 0; i < MAX_DEVICE_ENTRIES; i++) {
        if (s_device_gateway_map[i].in_use) {
            strncpy(dev_sns[dev_count], s_device_gateway_map[i].device_sn,
                    sizeof(dev_sns[dev_count]) - 1);
            dev_sns[dev_count][sizeof(dev_sns[dev_count]) - 1] = '\0';
            strncpy(gw_sns[dev_count], s_device_gateway_map[i].gateway_sn,
                    sizeof(gw_sns[dev_count]) - 1);
            gw_sns[dev_count][sizeof(gw_sns[dev_count]) - 1] = '\0';
            dev_count++;
        }
    }
    taskEXIT_CRITICAL(&s_device_map_lock);

    if (dev_count == 0) {
        ESP_LOGW(TAG, "映射表中没有子设备，检查是否有残留 Matter 端点...");
        // 重启后映射表为空，但 Matter 端点持久化在 NVS 中
        // 直接从 Matter node 枚举并移除所有桥接端点
        int matter_removed = app_matter_bridge_remove_all_devices();
        if (matter_removed > 0) {
            ESP_LOGI(TAG, "已清理 %d 个残留 Matter 端点", matter_removed);
        } else {
            ESP_LOGI(TAG, "没有可删除的子设备");
        }
        return;
    }

    ESP_LOGI(TAG, "3击删除所有子设备: 共 %d 个", dev_count);
    for (int i = 0; i < dev_count; i++) {
        ESP_LOGI(TAG, "发送解绑命令 [%d/%d]: gw=%s dev=%s", i + 1, dev_count, gw_sns[i], dev_sns[i]);
        send_unbind_device(gw_sns[i], dev_sns[i]);
        // 设备间延迟 200ms，避免 MQTT 报文拥堵
        if (i < dev_count - 1) {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
    ESP_LOGI(TAG, "所有解绑命令已发送: %d 个设备", dev_count);

    // 兜底清理：参照 HA 网关 gateway.py 做法，发送解绑命令后不等响应，
    // 固定等待后本地强制删除残留设备。网关可能对某些解绑命令不回复
    // （通讯失败、网关忙等），此时 handle_ctype_003 不会被触发，
    // Matter 端点会残留。等待3秒（足够网关处理+回复）后检查映射表，
    // 对仍存在的设备本地强制移除 Matter 端点和映射表条目。
    ESP_LOGI(TAG, "等待3秒后执行兜底清理...");
    vTaskDelay(pdMS_TO_TICKS(3000));

    int cleanup_count = 0;
    for (int i = 0; i < dev_count; i++) {
        if (is_device_in_map(dev_sns[i])) {
            ESP_LOGW(TAG, "网关未回复解绑响应，本地强制移除: %s", dev_sns[i]);
            local_remove_device(dev_sns[i]);
            cleanup_count++;
        }
    }
    if (cleanup_count > 0) {
        ESP_LOGI(TAG, "兜底清理完成: 强制移除 %d 个设备", cleanup_count);
    } else {
        ESP_LOGI(TAG, "所有设备已通过003响应正常移除");
    }

    // 最终扫描：清理可能残留的 Matter 端点（重启后 NVS 中遗留的端点）
    // 即使映射表中的设备已全部移除，Matter node 中可能仍有未追踪的端点
    int final_sweep = app_matter_bridge_remove_all_devices();
    if (final_sweep > 0) {
        ESP_LOGI(TAG, "最终扫描: 清理 %d 个残留端点", final_sweep);
    }
}

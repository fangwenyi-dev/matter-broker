/**
 * @file app_matter_bridge.cpp
 * @brief Matter Bridge 桥接器实现
 *
 * 使用 esp_matter 创建 node + aggregator，为每个 LoRa 子设备
 * 动态创建 bridged_node + WindowCovering 端点。
 *
 * 关键设计：位置语义反转
 * - Matter WindowCovering: 0%=完全打开, 100%=完全关闭
 * - LoRa 协议: 0=关闭, 100=打开
 * - Matter→LoRa: lora_value = 100 - matter_percent
 * - LoRa→Matter: matter_percent = 100 - lora_value
 *
 * API 参考：esp-matter main 分支
 * - node::create(config, attribute_callback, identify_callback)
 * - esp_matter::start(event_callback)
 * - endpoint::aggregator::create() 创建桥接父端点
 * - endpoint::bridged_node::create() 创建桥接子端点
 * - endpoint::window_covering::add() 添加 WindowCovering device type
 * - endpoint::enable() 启用端点（自动通知控制器）
 * - attribute::update() 更新属性值
 */
#include "app_matter_bridge.h"
#include "esp_log.h"
#include "esp_matter.h"
#include "esp_matter_core.h"
#include "esp_matter_attribute_utils.h"
#include "esp_matter_command.h"
#include "esp_matter_data_model.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <setup_payload/OnboardingCodesUtil.h>
#include <setup_payload/SetupPayload.h>
#include <app/ConcreteCommandPath.h>
#include <app/server/Server.h>
#include <app/reporting/reporting.h>
#include <lib/core/TLVReader.h>
#include <platform/PlatformManager.h>
#include <string.h>
#include <app-common/zap-generated/cluster-objects.h>

// Descriptor cluster ID 和 PartsList attribute ID（Matter 规范）
// 用于在 CASE Session 建立后通知控制器重新读取 PartsList，发现新端点
#define DESCRIPTOR_CLUSTER_ID       0x001D
#define DESCRIPTOR_ATTR_PARTS_LIST  0x0003

// 使用 esp_matter 命名空间
using namespace esp_matter;
using namespace chip::app::Clusters;

static const char *TAG = "app_matter_bridge";

// WindowCovering cluster 和 attribute ID（Matter 规范）
// WindowCovering cluster ID = 0x0102
// 注意：Matter 规范使用 percent100ths（0-10000），不是 percent（0-100）
// 参见 SDK AttributeIds.h 和参考项目 062101 的实现
#define ATTR_TYPE                          0x0000   // Type (uint8): 0x00=Lift, 0x01=Tilt
#define ATTR_CONFIG_STATUS                 0x0007   // ConfigStatus (uint8): bit0=Operational, bit3=LiftPositionAware
#define ATTR_OPERATIONAL_STATUS            0x000A   // OperationalStatus (bitmap8, 只读，SDK 内部管理)
#define ATTR_TARGET_LIFT_PERCENT_100THS    0x000B   // TargetPositionLiftPercent100ths (uint16, 0-10000)
#define ATTR_END_PRODUCT_TYPE              0x000D   // EndProductType (uint8): 0xFF=Unknown
#define ATTR_CURRENT_LIFT_PERCENT_100THS   0x000E   // CurrentPositionLiftPercent100ths (uint16, 0-10000)
#define ATTR_TARGET_TILT_PERCENT_100THS    0x000C   // TargetPositionTiltPercent100ths (uint16, 0-10000)
#define ATTR_CURRENT_TILT_PERCENT_100THS   0x000F   // CurrentPositionTiltPercent100ths (uint16, 0-10000)
#define ATTR_FEATURE_MAP                   0xFFFD   // FeatureMap (bitmap32): global attribute

// ========== 缺失的 WindowCovering 强制属性 ID ==========
// esp_matter SDK 的 feature::position_aware_lift/tilt::add() 仅创建了
// TargetPosition*Percent100ths 和 CurrentPosition*Percent100ths，
// 漏掉了以下 Matter 规范强制属性。HomeKit 读取这些属性时返回
// UnsupportedAttribute，导致滑块不渲染且添加设备时卡住。
#define ATTR_CURRENT_POSITION_LIFT         0x0003   // CurrentPositionLift (nullable uint16)
#define ATTR_CURRENT_POSITION_TILT         0x0004   // CurrentPositionTilt (nullable uint16)
#define ATTR_NUM_ACTUATIONS_LIFT           0x0005   // NumberOfActuationsLift (uint16)
#define ATTR_NUM_ACTUATIONS_TILT           0x0006   // NumberOfActuationsTilt (uint16)
#define ATTR_CURRENT_LIFT_PERCENTAGE       0x0008   // CurrentPositionLiftPercentage (nullable uint8)
#define ATTR_CURRENT_TILT_PERCENTAGE       0x0009   // CurrentPositionTiltPercentage (nullable uint8)
#define ATTR_INSTALLED_OPEN_LIMIT_LIFT     0x0010   // InstalledOpenLimitLift (uint16)
#define ATTR_INSTALLED_CLOSED_LIMIT_LIFT   0x0011   // InstalledClosedLimitLift (uint16)
#define ATTR_INSTALLED_OPEN_LIMIT_TILT     0x0012   // InstalledOpenLimitTilt (uint16)
#define ATTR_INSTALLED_CLOSED_LIMIT_TILT   0x0013   // InstalledClosedLimitTilt (uint16)

// OnOff cluster（Matter 规范 cluster ID = 0x0006）
#define ON_OFF_CLUSTER_ID             0x0006
#define ATTR_ON_OFF                    0x0000   // OnOff attribute (bool)

// WindowCovering feature flags（参见 SDK Enums.h）
// bit0=Lift, bit2=PositionAwareLift（仅升降，无 Tilt）
#define WC_FEATURE_LIFT                 0x01
#define WC_FEATURE_POSITION_AWARE_LIFT  0x04

// Type 枚举值（参见 SDK Enums.h Type enum）
// 0x00=RollerShade（仅升降），用于开窗器升降控制
// 内倒功能通过 OnOff cluster 实现（独立开关按钮），不再依赖 Tilt feature
#define WC_TYPE_LIFT                    0x00   // RollerShade（仅升降）

// EndProductType 枚举值（Matter 规范 Enums.h）
// 0x00=RollerShade，仅渲染 Lift 滑块 + 开/关/停止按钮
#define WC_END_PRODUCT_TYPE_WINDOW      0x00   // RollerShade（仅升降）

// ConfigStatus 位（Matter 规范 Enums.h：bit3=LiftPositionAware）
#define WC_CONFIG_STATUS_OPERATIONAL          0x01
#define WC_CONFIG_STATUS_LIFT_POSITION_AWARE  0x08

// 内部状态
static QueueHandle_t s_event_queue = NULL;
static node_t *s_node = NULL;
static endpoint_t *s_aggregator = NULL;
static uint16_t s_aggregator_endpoint_id = 0;
// P3-4 修复：配网完成标志，替代配网后重启
// 配网阶段（commissioning mode）内部 RAM 紧张，创建端点会崩溃
// 配网完成后 BLE 释放，内存恢复，LoRa 网关下次上报时正常创建端点
// P-Bug7 修复：跨 task 访问（Matter 事件循环写，bridge_task 读）加自旋锁保护，
// volatile 不保证内存序，portMUX_TYPE 确保 ESP32 双核同步。
static volatile bool s_commissioning_complete = false;
static portMUX_TYPE s_commissioning_lock = portMUX_INITIALIZER_UNLOCKED;

// CASE Session 通知去重时间戳（文件作用域，配网完成时重置为 0）
// 原为 kSecureSessionEstablished 内 static 局部变量，配网完成时无法重置。
// Bug：PASE 配网会话触发 kSecureSessionEstablished 设置时间戳，配网完成后
// HomeKit CASE Session 再次触发时在 30s 窗口内被跳过，导致 PartsList 通知
// 不发送，HomeKit 发现不了桥接端点，卡在添加界面。
static int64_t s_last_session_notify_us = 0;

// 设备 SN ↔ 端点 ID 映射表
#define MAX_BRIDGED_DEVICES CONFIG_MAX_BRIDGED_DEVICES
typedef struct {
    uint16_t endpoint_id;
    char device_sn[32];
    bool in_use;
    // 端点刚创建标志（per-feature 独立）：跳过 SDK 默认 delegate 触发的首次属性变更
    // Lift 和 Tilt 各自独立，避免 Lift 先触发清除后 Tilt 首次变更不被跳过导致意外内倒
    bool just_added_lift;  // 跳过首次 TargetPositionLiftPercent100ths 变更
    // 模式选择端点 ID（独立 bridged_node，OnOff 开关：平开/内倒模式）
    uint16_t mode_ep_id;
    // 内倒 OnOff 自动重置标志：防止 auto-set-back 触发递归回调
    volatile bool updating_onoff_niedao;
    // P2-3 修复：记录端点创建时间，just_added 标志在 2000ms 后自动失效
    // 防止 SDK 异步触发首次 PRE_UPDATE 时标志已被清除导致误发 LoRa 命令
    volatile int64_t added_time_us;
    // per-endpoint 状态标志（原全局变量，移到 entry 避免跨端点干扰）
    volatile bool updating_from_mqtt;        // 当前属性更新来自内部 update_position（MQTT→Matter 方向）
    // P1-4: StopMotion 跳过标志改为带时间戳，避免 target==current 时残留
    // StopMotion 设置后 500ms 内的 TargetPosition 变更被跳过，超时自动失效
    volatile bool skip_next_target_update;   // StopMotion 命令触发，下一次 TargetPosition 变更应跳过
    volatile int64_t skip_target_update_time_us;  // skip_next_target_update 设置时刻（esp_timer_get_time）
    // Reachable 属性去重：避免网关频繁上下线时产生报告风暴
    // reachable_initialized=false 时首次调用强制报告（初始化 data version）
    // 后续仅当 online != last_reachable 时才报告
    bool reachable_initialized;
    bool last_reachable;
    // P-Opt2: 属性值缓存——网关每 2 秒上报 002，值未变化时跳过 Matter 属性写入
    // 避免每 2 秒 5 次 attribute::update 调用 + 6 行日志刷屏
    // sentinel -1 = 未初始化（首次更新强制通过）
    int16_t last_lora_position;   // 上次写入的 LoRa 位置 (0-100, -1=未初始化)
    int32_t last_voltage_mv;      // 上次写入的电池电压 (mV, -1=未初始化)
} bridged_device_entry_t;

static bridged_device_entry_t s_device_table[MAX_BRIDGED_DEVICES] = {0};

// s_device_table 跨多核 task 访问自旋锁
// 访问者：bridge_task（add_device/add_gateway/remove_device/find_*/update_*）、
//         system_monitor_task（经 check_gateway_offline 调用 find_endpoint/update_reachable）、
//         Matter 事件循环 task（app_attribute_update_cb/app_event_cb/命令回调 find_by_endpoint）
static portMUX_TYPE s_device_table_lock = portMUX_INITIALIZER_UNLOCKED;

// ==================== 位置语义反转 ====================

/** LoRa → Matter: LoRa 0=关闭/100=打开 → Matter 0%=打开/100%=关闭 */
static inline uint8_t lora_to_matter_percent(uint8_t lora_pos)
{
    return 100 - lora_pos;
}

/** Matter → LoRa: Matter 0%=打开/100%=关闭 → LoRa 0=关闭/100=打开 */
static inline uint8_t matter_to_lora_value(uint8_t matter_percent)
{
    return 100 - matter_percent;
}

/**
 * @brief 检查 Matter 是否已配网完成（可安全创建端点）
 *
 * 配网阶段（commissioning mode）内部 RAM 紧张，此时创建端点会因
 * AES 加密初始化分配内部 DMA 内存失败而 abort 崩溃。
 * 配网完成后 BLE 释放，内存恢复，可安全创建端点。
 */
static bool is_matter_commissioned(void)
{
    bool val;
    taskENTER_CRITICAL(&s_commissioning_lock);
    val = s_commissioning_complete;
    taskEXIT_CRITICAL(&s_commissioning_lock);
    return val;
}

// ==================== 设备表操作 ====================
// 注意：find_* 函数返回 entry 指针，调用方必须在临界区内使用，
// 或在临界区内复制所需字段到栈变量后，临界区外使用。
// 临界区内禁止调用 Matter API（attribute::update 等会递归加锁导致死锁）。

static bridged_device_entry_t *find_free_entry(void)
{
    for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
        if (!s_device_table[i].in_use) {
            return &s_device_table[i];
        }
    }
    return NULL;
}

static bridged_device_entry_t *find_by_endpoint(uint16_t endpoint_id)
{
    for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
        if (s_device_table[i].in_use && s_device_table[i].endpoint_id == endpoint_id) {
            return &s_device_table[i];
        }
    }
    return NULL;
}

static bridged_device_entry_t *find_by_sn(const char *sn)
{
    for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
        if (s_device_table[i].in_use && strcmp(s_device_table[i].device_sn, sn) == 0) {
            return &s_device_table[i];
        }
    }
    return NULL;
}

/**
 * @brief 清空设备表项（重置所有字段）
 *
 * 用于 add_device/add_gateway 失败路径和 remove_device 的统一清理。
 * 调用方必须在 s_device_table_lock 临界区内调用。
 */
static void clear_device_entry(bridged_device_entry_t *entry)
{
    if (entry == NULL) return;
    entry->in_use = false;
    entry->endpoint_id = 0;
    entry->device_sn[0] = '\0';
    entry->just_added_lift = false;
    entry->mode_ep_id = 0;
    entry->updating_onoff_niedao = false;
    entry->updating_from_mqtt = false;
    entry->skip_next_target_update = false;
    entry->skip_target_update_time_us = 0;
    entry->added_time_us = 0;
    entry->reachable_initialized = false;
    entry->last_reachable = false;
    entry->last_lora_position = -1;
    entry->last_voltage_mv = -1;
}

static void push_matter_event(matter_event_type_t type, uint16_t endpoint_id,
                              const char *sn, int32_t value)
{
    if (s_event_queue == NULL) return;

    matter_event_t event = {};
    event.type = type;
    event.endpoint_id = endpoint_id;
    event.value = value;
    if (sn) {
        strncpy(event.device_sn, sn, sizeof(event.device_sn) - 1);
        event.device_sn[sizeof(event.device_sn) - 1] = '\0';
    }

    if (xQueueSend(s_event_queue, &event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Matter 事件队列已满，丢弃: type=%d ep=%u", type, endpoint_id);
    }
}

// ==================== Matter 回调 ====================

/**
 * @brief Matter 属性更新回调
 *
 * 当 Matter 属性被外部（如 Apple Home/Tuya）修改时触发。
 * WindowCovering 的 GoToLiftPercentage 命令会更新 TargetPositionLiftPercentage。
 *
 * 此回调在 Matter 事件循环中执行，不能阻塞。
 */
/**
 * @brief WindowCovering StopMotion 命令 user callback
 *
 * SDK 收到 StopMotion(0x02) 时会调用此回调（在 SDK 默认回调之前）。
 * SDK 默认回调会把 target=current，无法通过属性回调区分停止和移动到当前位置，
 * 因此必须注册命令级回调才能正确发送 LoRa 101（暂停）。
 *
 * 回调签名见 esp_matter_data_model.h: callback_t
 */
static esp_err_t app_stop_motion_command_cb(const chip::app::ConcreteCommandPath &command_path,
                                             chip::TLV::TLVReader &tlv_data,
                                             void *opaque_ptr)
{
    uint16_t endpoint_id = command_path.mEndpointId;
    // 临界区内查找+复制 device_sn 快照，避免并发访问 s_device_table
    char device_sn[32] = {0};
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry = find_by_endpoint(endpoint_id);
    if (entry != NULL) {
        strncpy(device_sn, entry->device_sn, sizeof(device_sn) - 1);
        // per-endpoint 标志：让 app_attribute_update_cb 跳过下一次 TargetPosition 变更
        // StopMotion 默认回调会设置 target=current，避免触发额外的 LoRa 控制命令
        // P1-4 修复：记录时间戳，若 SDK 因 target==current 不触发 PRE_UPDATE，
        // 标志会在 500ms 后自动失效，避免残留导致后续用户操作被跳过
        entry->skip_next_target_update = true;
        entry->skip_target_update_time_us = esp_timer_get_time();
    }
    taskEXIT_CRITICAL(&s_device_table_lock);

    if (device_sn[0] == '\0') {
        return ESP_OK; // 不是桥接设备
    }

    ESP_LOGI(TAG, "Matter 停止命令: ep=%u sn=%s → lora=101", endpoint_id, device_sn);
    push_matter_event(MATTER_EVENT_STOP_MOTION, endpoint_id, device_sn, 101);
    return ESP_OK;
}

/**
 * @brief 子设备内倒功能（LoRa value=200）
 *
 * 内倒功能通过 OnOff cluster 实现（开关式按钮）。
 * 用户在 HomeKit 中点击内倒开关 → OnOff 属性变更为 true
 * → POST_UPDATE 中发送 LoRa value=200 内倒命令
 * → 自动重置为 false（开关表现为按钮：按一下触发内倒，然后自动弹回）
 */

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type,
                                          uint16_t endpoint_id,
                                          uint32_t cluster_id,
                                          uint32_t attribute_id,
                                          esp_matter_attr_val_t *val,
                                          void *priv_data)
{
    if (type == attribute::POST_UPDATE) {
        // OnOff cluster - 内倒动作 (WindowCovering endpoint) 和 模式选择 (mode endpoint)
        if (cluster_id == OnOff::Id && attribute_id == ATTR_ON_OFF) {
            bool is_niedao = false;
            bool is_mode = false;
            char dev_sn[32] = {0};
            bool onoff_val = (val != NULL) ? val->val.b : false;

            taskENTER_CRITICAL(&s_device_table_lock);
            for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
                if (s_device_table[i].in_use) {
                    if (s_device_table[i].endpoint_id == endpoint_id) {
                        is_niedao = true;
                        strncpy(dev_sn, s_device_table[i].device_sn, sizeof(dev_sn) - 1);
                        break;
                    }
                    if (s_device_table[i].mode_ep_id == endpoint_id) {
                        is_mode = true;
                        strncpy(dev_sn, s_device_table[i].device_sn, sizeof(dev_sn) - 1);
                        break;
                    }
                }
            }
            taskEXIT_CRITICAL(&s_device_table_lock);

            if (is_niedao && onoff_val) {
                // 内倒动作：OnOff 被打开 → 发送 LoRa 200
                ESP_LOGI(TAG, "OnOff 内倒触发: ep=%u sn=%s → lora=200", endpoint_id, dev_sn);
                push_matter_event(MATTER_EVENT_TILT_TOGGLE, endpoint_id, dev_sn, 200);

                // 自动重置为 OFF（使开关表现为按钮：按一下触发内倒，然后自动弹回）
                taskENTER_CRITICAL(&s_device_table_lock);
                bridged_device_entry_t *e = find_by_endpoint(endpoint_id);
                if (e != NULL) e->updating_onoff_niedao = true;
                taskEXIT_CRITICAL(&s_device_table_lock);

                esp_matter_attr_val_t off_val = esp_matter_bool(false);
                attribute::update(endpoint_id, OnOff::Id, ATTR_ON_OFF, &off_val);

                taskENTER_CRITICAL(&s_device_table_lock);
                bridged_device_entry_t *e2 = find_by_endpoint(endpoint_id);
                if (e2 != NULL) e2->updating_onoff_niedao = false;
                taskEXIT_CRITICAL(&s_device_table_lock);
            }

            if (is_mode) {
                // 模式选择：OnOff=true→平开(1), OnOff=false→内倒(0)
                uint8_t mode_val = onoff_val ? 1 : 0;
                ESP_LOGI(TAG, "OnOff 模式变更: ep=%u sn=%s mode=%u (0=内倒,1=平开)",
                         endpoint_id, dev_sn, mode_val);
                push_matter_event(MATTER_EVENT_MODE_CHANGED, endpoint_id, dev_sn, mode_val);
            }
        }
        return ESP_OK;
    }

    if (type != attribute::PRE_UPDATE) {
        return ESP_OK;
    }

    // 临界区内查找+复制快照，避免并发访问 s_device_table
    // - per-endpoint 标志从 entry 复制到栈变量（skip_*  读后立即清除语义；
    //   updating_from_mqtt 仅复制不清除，由 update_position 自己重置）
    // - just_added_lift/tilt per-feature 独立标志，仅在对应分支内清除
    char device_sn[32] = {0};
    bool just_added_lift = false;
    int64_t added_time_us = 0;
    bool updating_from_mqtt = false;
    bool skip_next_target_update = false;
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry = find_by_endpoint(endpoint_id);
    if (entry != NULL) {
        strncpy(device_sn, entry->device_sn, sizeof(device_sn) - 1);
        just_added_lift = entry->just_added_lift;
        added_time_us = entry->added_time_us;
        updating_from_mqtt = entry->updating_from_mqtt;
        // P1-4: 带超时的 skip 检查——超过 500ms 视为过期，不跳过
        skip_next_target_update = entry->skip_next_target_update;
        if (skip_next_target_update) {
            int64_t elapsed_ms = (esp_timer_get_time() - entry->skip_target_update_time_us) / 1000;
            if (elapsed_ms > 500) {
                skip_next_target_update = false;  // 超时失效
            }
            entry->skip_next_target_update = false;  // 读后立即清除
            entry->skip_target_update_time_us = 0;
        }
    } else {
        // 主端点未找到，检查是否为模式选择端点
        for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
            if (s_device_table[i].in_use && s_device_table[i].mode_ep_id == endpoint_id) {
                strncpy(device_sn, s_device_table[i].device_sn, sizeof(device_sn) - 1);
                break;
            }
        }
    }
    taskEXIT_CRITICAL(&s_device_table_lock);

    if (device_sn[0] == '\0') {
        return ESP_OK; // 不是桥接设备，忽略
    }

    // WindowCovering cluster (0x0102) — 所有桥接端点均为子设备
    if (cluster_id == WindowCovering::Id) {
        if (attribute_id == ATTR_TARGET_LIFT_PERCENT_100THS) {
            // 跳过内部 update_position 触发的变更（MQTT→Matter 方向）
            // 避免 LoRa 上报 → Matter 更新 → 又发 LoRa 命令 的控制回环
            if (updating_from_mqtt) {
                return ESP_OK;
            }

            // 跳过 StopMotion 命令触发的 TargetPosition 变更
            // StopMotion 默认回调会设置 target=current，避免重复下发 LoRa 命令
            if (skip_next_target_update) {
                ESP_LOGI(TAG, "跳过 StopMotion 触发的 TargetPosition 变更: ep=%u", endpoint_id);
                return ESP_OK;
            }

            // 跳过端点刚创建时 SDK 默认 delegate 触发的首次 TargetPosition 变更
            // 否则会立即下发 LoRa 控制命令（value=0），导致开窗器在添加时意外动作
            // per-feature 独立：仅清除 Lift 标志，不影响 Tilt 标志
            // P2-3 修复：增加超时机制，防止 SDK 不触发首次变更时标志永久残留
            // P-HomeKit4 修复：超时从 5000ms 缩短到 2000ms
            if (just_added_lift) {
                int64_t added_elapsed_ms = (esp_timer_get_time() - added_time_us) / 1000;
                // 无论是否超时都清除标志（读后即清除语义，防止标志残留）
                taskENTER_CRITICAL(&s_device_table_lock);
                bridged_device_entry_t *e = find_by_endpoint(endpoint_id);
                if (e != NULL) e->just_added_lift = false;
                taskEXIT_CRITICAL(&s_device_table_lock);
                // P-HomeKit4 修复：窗口从 5000ms 缩短到 2000ms。
                // SDK 默认 delegate 的首次 TargetPosition 变更通常在 enable() 后 <100ms 内同步触发。
                // 2000ms 足够覆盖 SDK 异步延迟，同时减少已配网重启场景下误跳过 HomeKit 首次命令的概率。
                if (added_elapsed_ms < 2000) {
                    ESP_LOGI(TAG, "跳过端点初始化的 TargetPosition 变更: ep=%u", endpoint_id);
                    return ESP_OK;
                }
                // 超时：SDK 未触发首次变更，视为用户真实控制命令，继续处理
                ESP_LOGW(TAG, "just_added_lift 超时，视为用户命令继续处理: ep=%u", endpoint_id);
                // 不 return，继续走后面的控制逻辑
            }

            // val 类型校验：TargetPositionLiftPercent100ths 必须是 uint16/nullable uint16
            if (val == NULL || (val->type != ESP_MATTER_VAL_TYPE_UINT16 &&
                                val->type != ESP_MATTER_VAL_TYPE_NULLABLE_UINT16)) {
                ESP_LOGW(TAG, "位置属性类型异常: ep=%u type=%d，跳过", endpoint_id, val ? val->type : -1);
                return ESP_OK;
            }

            // P1-1 修复：nullable<uint16_t> null 值=0xFFFF，若 SDK 发送 null 会算出无效 LoRa 值
            if (val->val.u16 == 0xFFFF) {
                ESP_LOGW(TAG, "Lift 位置属性为 null，跳过: ep=%u", endpoint_id);
                return ESP_OK;
            }

            uint16_t matter_percent_100ths = val->val.u16;
            uint8_t matter_percent = (uint8_t)(matter_percent_100ths / 100);
            uint8_t lora_value = matter_to_lora_value(matter_percent);

            ESP_LOGI(TAG, "Matter 位置控制: ep=%u sn=%s matter=%u%% (raw=%u) → lora=%u",
                     endpoint_id, device_sn, matter_percent,
                     matter_percent_100ths, lora_value);

            push_matter_event(MATTER_EVENT_LIFT_CHANGED, endpoint_id, device_sn, lora_value);
        }
    }

    // OnOff cluster - 跳过内倒自动重置触发的更新
    if (cluster_id == OnOff::Id && attribute_id == ATTR_ON_OFF) {
        bool skip = false;
        taskENTER_CRITICAL(&s_device_table_lock);
        bridged_device_entry_t *e = find_by_endpoint(endpoint_id);
        if (e != NULL && e->updating_onoff_niedao) {
            skip = true;
        }
        taskEXIT_CRITICAL(&s_device_table_lock);
        if (skip) {
            return ESP_OK;
        }
    }

    return ESP_OK;
}

/**
 * @brief Matter 事件回调
 *
 * 处理配网完成等事件。
 */
static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete: {
        // P3-4 修复：移除配网后重启逻辑，改为设置标志
        // 原重启逻辑会中断 HomeKit 配网后续操作（CASE Session 建立、属性读取、订阅），
        // 导致 HomeKit 报告 "pairing failed"
        // 配网完成后 BLE 释放，内存恢复，LoRa 网关下次上报时正常创建端点
        // P-Bug7 修复：加锁写入 s_commissioning_complete
        taskENTER_CRITICAL(&s_commissioning_lock);
        s_commissioning_complete = true;
        taskEXIT_CRITICAL(&s_commissioning_lock);
        ESP_LOGI(TAG, "Matter 配网完成，已启用端点创建");

        // Bug 修复：重置 CASE Session 去重时间戳。
        // PASE 配网会话触发 kSecureSessionEstablished 时设置了 s_last_session_notify_us，
        // 配网完成后 HomeKit 建立 CASE Session 时若在 30s 窗口内，PartsList 通知会被跳过，
        // 导致 HomeKit 发现不了桥接端点，卡在添加界面。
        // 重置为 0 确保下一次 kSecureSessionEstablished 必定触发通知。
        s_last_session_notify_us = 0;

        // P-HomeKit6 修复：配网完成后主动请求设备列表，加速端点创建。
        // 首次配网时 LoRa 网关可能在配网前就上报了 002，但当时未配网无法创建端点。
        // 推送 COMMISSIONING_COMPLETE 事件，协议桥接层收到后向所有已注册网关发送 002。
        push_matter_event(MATTER_EVENT_COMMISSIONING_COMPLETE, 0, NULL, 0);
        break;
    }
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Matter 配网会话开始");
        break;
    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Matter 配网会话结束");
        break;
    case chip::DeviceLayer::DeviceEventType::kSecureSessionEstablished: {
        // 安全会话建立（PASE 配网阶段 / CASE 控制器连接）
        // 关键修复：端点在 CASE Session 建立之前创建时，endpoint::enable() 的
        // PartsList 通知因无活跃 session 而丢失。
        // 控制器首次建立 CASE Session 后不会主动重新读取，导致看不到动态创建的端点。
        // CASE Session 建立后通知 PartsList + Reachable（触发控制器重新发现端点并读取属性）
        if (is_matter_commissioned()) {
            // P-HomeKit3 修复：CASE Session 去重，30 秒内不重复通知。
            // HomeKit 从后台恢复或网络抖动会频繁重建 CASE Session，每次都批量通知
            // 会产生流量尖峰，与 HomeKit 自身的属性读取竞争网络带宽，
            // 导致「正在更新」时间延长。30 秒窗口覆盖正常重连间隔。
            //
            // Bug 修复：s_last_session_notify_us 在 kCommissioningComplete 中被重置为 0，
            // 确保配网完成后 HomeKit 的首次 CASE Session 必定触发 PartsList 通知。
            // 原代码使用 static 局部变量，配网完成时无法重置，导致 PASE → CASE
            // 在 30s 内被去重跳过，HomeKit 卡在添加界面。
            int64_t now_us = esp_timer_get_time();
            int64_t last = s_last_session_notify_us;
            s_last_session_notify_us = now_us;
            if (now_us - last < 30 * 1000000LL) {
                ESP_LOGI(TAG, "CASE Session 建立（<30s 内重复，跳过通知）");
                break;
            }
            ESP_LOGI(TAG, "Matter CASE Session 建立，通知 PartsList");

            // 通知 aggregator 的 PartsList 变更（触发控制器重新发现端点）
            if (s_aggregator_endpoint_id != 0) {
                MatterReportingAttributeChangeCallback(
                    s_aggregator_endpoint_id, DESCRIPTOR_CLUSTER_ID, DESCRIPTOR_ATTR_PARTS_LIST);
            }

            // P-ReportData1 修复：移除 CASE Session 中的批量 Reachable 通知。
            //
            // 原代码在 CASE Session 建立时批量通知所有端点的 Reachable 属性，
            // 叠加 HomeKit 自身订阅请求的属性读取，导致首次订阅 ReportData
            // 超过 1000 字节（实测 4 条 ~1200 字节），HomeKit 拆除订阅
            // （status 0x7d = InvalidSubscription），配网后"正在更新"时间延长。
            //
            // 移除原因：
            // 1. PartsList 通知已足够触发 HomeKit 发现端点并读取属性
            // 2. Reachable 属性值已正确存储在 attribute store 中（update_reachable 设置）
            // 3. HomeKit 读取端点时自动获取 Reachable 值，无需额外通知
            // 4. 减少 dirty 属性数量，降低首次订阅 ReportData 大小
            //
            // 原 P-Speed1 回退的原因（HomeKit 重连后不读取属性）实际由
            // 订阅被拆除引起——拆除后 HomeKit 无活跃订阅，无法接收通知。
            // 修复 ReportData 超限后订阅不再被拆除，HomeKit 正常读取属性。
        }
        break;
    }
    default:
        break;
    }
}

// ==================== 公共 API ====================

esp_err_t app_matter_bridge_init(const matter_bridge_config_t *config)
{
    if (config == NULL || config->event_queue == NULL) {
        ESP_LOGE(TAG, "无效的配置参数");
        return ESP_ERR_INVALID_ARG;
    }

    s_event_queue = config->event_queue;

    // 创建 node（含 root node device type，自动放在 endpoint 0）
    node::config_t node_config;
    // vendor_id / product_id 通过 sdkconfig 中的 CONFIG_CHIP_DEVICE_VENDOR_ID / CONFIG_CHIP_DEVICE_PRODUCT_ID 设置
    // 若需运行时修改，可在 node 创建后通过 esp_matter::attribute::update() 更新

    s_node = node::create(&node_config, app_attribute_update_cb, NULL);
    if (s_node == NULL) {
        ESP_LOGE(TAG, "创建 Matter node 失败");
        return ESP_FAIL;
    }

    // 创建 aggregator 端点（桥接设备的父端点）
    // aggregator 是 Matter Bridge 的必需组件，用于管理动态子端点
    endpoint::aggregator::config_t aggregator_config;
    s_aggregator = endpoint::aggregator::create(s_node, &aggregator_config,
                                                 ENDPOINT_FLAG_NONE, NULL);
    if (s_aggregator == NULL) {
        ESP_LOGE(TAG, "创建 aggregator 端点失败");
        return ESP_FAIL;
    }

    // 启用 aggregator 端点
    esp_err_t err = endpoint::enable(s_aggregator);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启用 aggregator 失败: %s", esp_err_to_name(err));
        return err;
    }

    s_aggregator_endpoint_id = endpoint::get_id(s_aggregator);
    ESP_LOGI(TAG, "Matter Bridge node + aggregator 已创建, aggregator_ep=%u",
             s_aggregator_endpoint_id);

    return ESP_OK;
}

esp_err_t app_matter_bridge_start(void)
{
    if (s_node == NULL) {
        ESP_LOGE(TAG, "Bridge 未初始化");
        return ESP_ERR_INVALID_STATE;
    }

    // esp_matter::start() 接受事件回调，不是 node 参数
    esp_err_t err = esp_matter::start(app_event_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启动 Matter 失败: %s", esp_err_to_name(err));
        return err;
    }

    // P3-4 修复：启动后检查是否已配网（重启场景）
    // 已配网设备（如 OTA 升级后重启）无需等待 kCommissioningComplete 事件
    // P-Bug7 修复：加锁写入 s_commissioning_complete
    if (chip::Server::GetInstance().GetFabricTable().FabricCount() > 0) {
        taskENTER_CRITICAL(&s_commissioning_lock);
        s_commissioning_complete = true;
        taskEXIT_CRITICAL(&s_commissioning_lock);
        ESP_LOGI(TAG, "设备已配网，端点创建已启用");
    }

    // 方案2B：双保险 - 确保WiFi为STA模式（不开启softAP）
    // 配合 sdkconfig.defaults 中 CONFIG_ENABLE_WIFI_AP=n + CONFIG_ESP_WIFI_SOFTAP_SUPPORT=n
    // 此处检查运行时WiFi模式，防止NVS残留或其他组件开启APSTA
    wifi_mode_t cur_mode = WIFI_MODE_NULL;
    if (esp_wifi_get_mode(&cur_mode) == ESP_OK && cur_mode != WIFI_MODE_STA) {
        ESP_LOGW(TAG, "检测到 WiFi 非 STA 模式 (0x%x)，尝试强制 STA", cur_mode);
        esp_err_t set_err = esp_wifi_set_mode(WIFI_MODE_STA);
        if (set_err != ESP_OK) {
            ESP_LOGW(TAG, "强制 STA 失败: %s（依赖 Kconfig 禁用 softAP）", esp_err_to_name(set_err));
        } else {
            ESP_LOGI(TAG, "已强制设为 STA 模式");
        }
    } else {
        ESP_LOGI(TAG, "WiFi 模式检查通过: STA");
    }

    // 方案1：打印配网二维码和手动配对码
    // esp_matter::start() 内部不打印二维码，需应用层显式调用 PrintOnboardingCodes
    // 项目使用 BLE 配网，传入 kBLE flag
    ESP_LOGI(TAG, "===== Matter 配网信息（请用 Apple Home / Google Home 扫码）=====");
    PrintOnboardingCodes(chip::RendezvousInformationFlags(chip::RendezvousInformationFlag::kBLE));
    ESP_LOGI(TAG, "===================================================================");

    ESP_LOGI(TAG, "Matter 已启动，等待 BLE 配网...");
    return ESP_OK;
}

esp_err_t app_matter_bridge_add_device(const char *device_sn, const char *device_name,
                                        uint16_t *endpoint_id)
{
    if (device_sn == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // P2-9 修复：s_node/s_aggregator NULL 检查，防止 init 失败后 add_device 崩溃
    // 与 add_gateway 的检查对称（L942）
    if (s_node == NULL || s_aggregator == NULL) {
        ESP_LOGE(TAG, "Matter node 未初始化，无法创建子设备端点");
        return ESP_ERR_INVALID_STATE;
    }

    // 配网阶段内部 RAM 紧张，创建端点会因 AES 内存分配失败而崩溃
    // 未配网完成时拒绝创建，等配网完成后 LoRa 再次上报时创建
    if (!is_matter_commissioned()) {
        ESP_LOGW(TAG, "Matter 未配网，跳过创建子设备端点: sn=%s（配网后重启自动创建）", device_sn);
        return ESP_ERR_INVALID_STATE;
    }

    // 临界区内检查重复+占位，避免并发 add_device 同一 SN
    taskENTER_CRITICAL(&s_device_table_lock);
    if (find_by_sn(device_sn) != NULL) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        ESP_LOGW(TAG, "设备 %s 已存在", device_sn);
        return ESP_ERR_INVALID_STATE;
    }
    bridged_device_entry_t *entry = find_free_entry();
    if (entry == NULL) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        ESP_LOGE(TAG, "设备表已满（最大 %d）", MAX_BRIDGED_DEVICES);
        return ESP_ERR_NO_MEM;
    }
    // 先占位，防止并发添加同一 SN
    entry->in_use = true;
    taskEXIT_CRITICAL(&s_device_table_lock);

    // 1. 创建 bridged_node 端点（桥接设备的容器）
    //    flags 必须包含 ENDPOINT_FLAG_DESTROYABLE | ENDPOINT_FLAG_BRIDGE
    endpoint::bridged_node::config_t bridged_node_config{};
    endpoint_t *endpoint = endpoint::bridged_node::create(s_node, &bridged_node_config,
                                                           ENDPOINT_FLAG_DESTROYABLE | ENDPOINT_FLAG_BRIDGE,
                                                           NULL);
    if (endpoint == NULL) {
        ESP_LOGE(TAG, "创建 bridged_node 端点失败");
        taskENTER_CRITICAL(&s_device_table_lock);
        clear_device_entry(entry);
        taskEXIT_CRITICAL(&s_device_table_lock);
        return ESP_FAIL;
    }

    // 2. 添加 WindowCovering device type（cluster 0x0102）
    //    feature_flags = Lift + PositionAwareLift（仅升降，无 Tilt）
    //    - Lift/PositionAwareLift: 升降位置控制（开窗器主功能）
    //    内倒功能通过 OnOff cluster 实现（独立开关按钮），不再使用 Tilt feature
    endpoint::window_covering::config_t wc_config(WC_END_PRODUCT_TYPE_WINDOW);
    wc_config.window_covering.type = WC_TYPE_LIFT;
    // ConfigStatus = 0x09 (Operational + LiftPositionAware)
    wc_config.window_covering.config_status = WC_CONFIG_STATUS_OPERATIONAL |
                                              WC_CONFIG_STATUS_LIFT_POSITION_AWARE;
    wc_config.window_covering.feature_flags = WC_FEATURE_LIFT | WC_FEATURE_POSITION_AWARE_LIFT;
    // 初始化 Lift 位置属性（0=完全打开升降，与 LoRa 0=关闭 反转后一致）
    wc_config.window_covering.features.position_aware_lift.target_position_lift_percent_100ths = nullable<uint16_t>(0);
    wc_config.window_covering.features.position_aware_lift.current_position_lift_percent_100ths = nullable<uint16_t>(0);
    esp_err_t err = endpoint::window_covering::add(endpoint, &wc_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "添加 WindowCovering device type 失败: %s", esp_err_to_name(err));
        endpoint::destroy(s_node, endpoint);
        taskENTER_CRITICAL(&s_device_table_lock);
        clear_device_entry(entry);
        taskEXIT_CRITICAL(&s_device_table_lock);
        return err;
    }

    // 2.1 补全 WindowCovering 缺失的 Matter 规范强制属性
    //     esp_matter SDK 的 feature::position_aware_lift::add() 仅创建了
    //     TargetPositionLiftPercent100ths 和 CurrentPositionLiftPercent100ths，
    //     漏掉了 PositionAwareLift feature 的 2 个强制属性。
    //     HomeKit 读取这些属性时返回 UnsupportedAttribute，导致滑块不渲染。
    //
    //     P-ReportData2 优化：移除 3 个非强制属性（NumberOfActuationsLift、
    //     CurrentPositionLiftPercentage、CurrentPositionLift），减少首次订阅
    //     ReportData 数据量约 60-90 字节，缩短 HomeKit "正在更新"时间。
    //     - NumberOfActuationsLift: 执行次数计数器，HomeKit 不渲染
    //     - CurrentPositionLiftPercentage: 已有 Percent100ths，Percentage 冗余
    //     - CurrentPositionLift: 绝对位置，HomeKit 不直接渲染
    {
        cluster_t *wc_cluster = cluster::get(endpoint, WindowCovering::Id);
        if (wc_cluster == NULL) {
            ESP_LOGE(TAG, "获取 WindowCovering cluster 失败，无法补全属性");
        } else {
            // --- PositionAwareLift 强制属性 ---
            // InstalledOpenLimitLift (0x0010): uint16 (全开位置=0)
            esp_matter::attribute::create(wc_cluster, ATTR_INSTALLED_OPEN_LIMIT_LIFT,
                ATTRIBUTE_FLAG_NONE, esp_matter_uint16(0));
            // InstalledClosedLimitLift (0x0011): uint16 (全关位置=10000)
            esp_matter::attribute::create(wc_cluster, ATTR_INSTALLED_CLOSED_LIMIT_LIFT,
                ATTRIBUTE_FLAG_NONE, esp_matter_uint16(10000));

            ESP_LOGI(TAG, "已补全 WindowCovering 强制属性 (2个): ep=%u", endpoint::get_id(endpoint));
        }
    }

    // 3. 设置父端点为 aggregator（关键！让 aggregator 的 parts_list 包含此端点）
    err = endpoint::set_parent_endpoint(endpoint, s_aggregator);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "设置父端点失败: %s", esp_err_to_name(err));
        endpoint::destroy(s_node, endpoint);
        taskENTER_CRITICAL(&s_device_table_lock);
        clear_device_entry(entry);
        taskEXIT_CRITICAL(&s_device_table_lock);
        return err;
    }

    // 4. 注册 WindowCovering StopMotion 命令 user callback
    uint16_t ep_id_for_cmd = endpoint::get_id(endpoint);
    command_t *stop_cmd = command::get(ep_id_for_cmd, WindowCovering::Id, 0x0002);
    if (stop_cmd == NULL) {
        ESP_LOGE(TAG, "无法获取 StopMotion 命令句柄，端点创建成功但停止功能不可用: ep=%u", ep_id_for_cmd);
    } else {
        command::set_user_callback(stop_cmd, app_stop_motion_command_cb);
        ESP_LOGI(TAG, "已注册 StopMotion 命令回调: ep=%u", ep_id_for_cmd);
    }

    // 4.5 添加 OnOff cluster（内倒动作）——仅 SN 前缀为 5005 的设备支持内倒
    //    5005 = 内开内倒窗，支持内倒动作（LoRa value=200）
    //    5001/5002/5003 = 普通升降窗，不支持内倒
    //    HomeKit 在设备详情页渲染为开关按钮，用户点击开关 → OnOff=true
    //    → POST_UPDATE 中发送 LoRa value=200 内倒命令
    //    → 自动重置为 false（开关表现为按钮：按一下触发内倒，然后自动弹回）
    bool support_niedao = (strncmp(device_sn, "5005", 4) == 0);
    if (support_niedao) {
        cluster::on_off::config_t on_off_config{};
        on_off_config.on_off = false;
        cluster_t *on_off_cluster = cluster::on_off::create(endpoint, &on_off_config,
                                                             CLUSTER_FLAG_SERVER);
        if (on_off_cluster == NULL) {
            ESP_LOGE(TAG, "创建 OnOff cluster 失败: ep=%u", ep_id_for_cmd);
        } else {
            // 注册 On/Off/Toggle 命令（SDK on_off::create() 仅创建 Off 命令）
            cluster::on_off::command::create_on(on_off_cluster);
            cluster::on_off::command::create_off(on_off_cluster);
            cluster::on_off::command::create_toggle(on_off_cluster);
            ESP_LOGI(TAG, "已添加 OnOff cluster (内倒): ep=%u sn=%s", ep_id_for_cmd, device_sn);
        }
    } else {
        ESP_LOGI(TAG, "设备不支持内倒，跳过 OnOff cluster: sn=%s", device_sn);
    }

    // 4.6 添加 PowerSource cluster（电池电压）
    //    cluster::power_source::create() 内部会根据 feature_flags 自动调用
    //    feature::battery::add()，后者已创建 BatChargeLevel/BatReplacementNeeded/
    //    BatReplaceability 属性。不能再显式调用 feature::battery::add()。
    // P1-7 修复：status 从 0 (Unavailable) 改为 1 (Active)。
    //   HomeKit 仅当 PowerSource.Status=Active 时才创建 BatteryService 并显示电池百分比。
    //   原值 0 导致 HomeKit 忽略整个 PowerSource cluster，电池信息不显示。
    //   参考 Matter 规范 PowerSource.StatusEnum：0=Unavailable, 1=Active, 2=Fault。
    cluster::power_source::config_t ps_config{};
    ps_config.status = 1;  // Active：电源活跃，HomeKit 会创建 BatteryService
    ps_config.order = 0;
    ps_config.feature_flags = cluster::power_source::feature::battery::get_id();
    cluster_t *ps_cluster = cluster::power_source::create(endpoint, &ps_config,
                                                           CLUSTER_FLAG_SERVER);
    if (ps_cluster == NULL) {
        ESP_LOGE(TAG, "创建 PowerSource cluster 失败: ep=%u", ep_id_for_cmd);
    } else {
        // P-HomeKit5 修复（已回退）：初始值用 0 而非 null。
        // nullable<uint32_t>() = null，HomeKit 不创建 BatteryService，电池信息完全不显示。
        // nullable<uint32_t>(0) = 值 0（0mV），HomeKit 创建 BatteryService 并显示 "0V"，
        // LoRa 首次上报后立即更新为真实值。"0V" 闪烁是可接受的临时状态。
        cluster::power_source::attribute::create_bat_voltage(ps_cluster,
            nullable<uint32_t>(0), nullable<uint32_t>(0), nullable<uint32_t>(0xFFFF));
        cluster::power_source::attribute::create_bat_percent_remaining(ps_cluster,
            nullable<uint8_t>(0), nullable<uint8_t>(0), nullable<uint8_t>(200));
        ESP_LOGI(TAG, "已添加 PowerSource cluster (Battery): ep=%u", ep_id_for_cmd);
    }

    // 4.65 移除 ModeSelect cluster，改用独立 OnOff 端点实现模式选择
    //     ModeSelect cluster 在 HomeKit 中不渲染模式选择器 UI
    //     改用独立 bridged_node + OnOff 开关：ON=平开(1), OFF=内倒(0)

    // 4.7 添加 BridgedDeviceBasicInformation 属性
    // SDK 的 bridged_device_basic_information::create() 仅创建 Reachable 和 UniqueID(空)，
    // 以下属性需手动添加，影响 HomeKit 设备详情页显示和认证状态：
    //   - VendorName  → HomeKit "生产企业"
    //   - VendorID    → HomeKit MFi 认证检查（缺失会显示"未认证"警告）
    //   - ProductName → HomeKit "设备名称"（首页图标下方文字）
    //   - ProductID   → HomeKit MFi 认证检查（缺失会显示"未认证"警告）
    //   - SerialNumber → HomeKit "序列号"
    //   - HardwareVersion → HomeKit "固件版本"（硬件）
    //   - SoftwareVersion → HomeKit "固件版本"（软件）
    //
    // P-Cert1 修复：添加 VendorID 和 ProductID 属性。
    // 原因：HomeKit 对动态添加的桥接设备会检查 BDI 簇的 VendorID/ProductID 进行认证校验。
    // 缺失这两个属性时，HomeKit 显示"此配件尚未经过认证可与homekit配合使用"。
    // 使用与根节点相同的 VendorID/ProductID（来自 sdkconfig），使桥接设备继承根设备认证状态。
    cluster_t *bdi_cluster = cluster::get(endpoint, BridgedDeviceBasicInformation::Id);
    // P2-8 修复：display_name 声明提升到函数作用域，L924 的 ESP_LOGI 在 if 块外使用
    const char *display_name = (device_name != NULL) ? device_name : device_sn;
    if (bdi_cluster != NULL) {
        // VendorName（生产企业）：Matter 规范 max 32 字节
        // "成都慧尖科技有限公司" UTF-8 编码 = 10×3 = 30 字节，符合限制
        // HomeKit 设备详情页显示为"生产企业"字段
        const char *vendor_str = "成都慧尖科技有限公司";
        char vendor_name[32] = {0};
        strncpy(vendor_name, vendor_str, sizeof(vendor_name) - 1);
        cluster::bridged_device_basic_information::attribute::create_vendor_name(
            bdi_cluster, vendor_name, strlen(vendor_name));

        // VendorID：HomeKit 用此属性检查 MFi 认证状态
        // 使用 sdkconfig 中配置的 VendorID（与根节点一致）
        cluster::bridged_device_basic_information::attribute::create_vendor_id(
            bdi_cluster, CONFIG_DEVICE_VENDOR_ID);

        // ProductName（设备名称）：首页图标显示名，缺失时 HomeKit 回退到设备类型名"窗帘"
        char product_name[32] = {0};
        strncpy(product_name, display_name, sizeof(product_name) - 1);
        if (strlen(display_name) >= sizeof(product_name)) {
            ESP_LOGW(TAG, "设备名过长已截断: sn=%s orig=%s", device_sn, display_name);
        }
        cluster::bridged_device_basic_information::attribute::create_product_name(
            bdi_cluster, product_name, strlen(product_name));

        // ProductID：HomeKit 用此属性检查 MFi 认证状态
        // 使用 sdkconfig 中配置的 ProductID（与根节点一致）
        cluster::bridged_device_basic_information::attribute::create_product_id(
            bdi_cluster, CONFIG_DEVICE_PRODUCT_ID);

        // SerialNumber（序列号）：直接使用 LoRa 设备 SN
        // HomeKit 设备详情页显示为"序列号"字段
        char serial_num[32] = {0};
        strncpy(serial_num, device_sn, sizeof(serial_num) - 1);
        cluster::bridged_device_basic_information::attribute::create_serial_number(
            bdi_cluster, serial_num, strlen(serial_num));

        // HardwareVersion：HomeKit 设备详情页显示硬件版本，缺失显示"未知"
        cluster::bridged_device_basic_information::attribute::create_hardware_version(bdi_cluster, 1);
        // SoftwareVersion：HomeKit 设备详情页显示固件版本号
        cluster::bridged_device_basic_information::attribute::create_software_version(bdi_cluster, 1);
        ESP_LOGI(TAG, "已添加 BDI 属性: vendor=%s name=%s sn=%s vid=0x%04X pid=0x%04X",
                 vendor_name, product_name, serial_num, CONFIG_DEVICE_VENDOR_ID, CONFIG_DEVICE_PRODUCT_ID);
    } else {
        ESP_LOGW(TAG, "获取 BridgedDeviceBasicInformation cluster 失败，设备名将为默认值");
    }

    // 5. 在启用端点前先记录到设备表（just_added_lift=true）
    //    确保 SDK 在 endpoint::enable() 触发的首次 TargetPosition 变更时，
    //    find_by_endpoint() 能找到 entry 并通过 just_added_lift 跳过，避免开窗器意外动作。
    uint16_t ep_id = endpoint::get_id(endpoint);
    taskENTER_CRITICAL(&s_device_table_lock);
    entry->endpoint_id = ep_id;
    strncpy(entry->device_sn, device_sn, sizeof(entry->device_sn) - 1);
    entry->device_sn[sizeof(entry->device_sn) - 1] = '\0';
    // 标记刚创建，跳过 SDK 默认 delegate 的首次 TargetPosition 变更
    entry->just_added_lift = true;
    // 显式重置所有运行时标志（防止 slot 复用时残留旧值）
    entry->mode_ep_id = 0;
    entry->updating_onoff_niedao = false;
    entry->updating_from_mqtt = false;
    entry->skip_next_target_update = false;
    entry->skip_target_update_time_us = 0;
    entry->reachable_initialized = false;
    entry->last_reachable = false;
    entry->last_lora_position = -1;
    entry->last_voltage_mv = -1;
    // P1-A 修复：added_time_us 必须在 endpoint::enable() 之前设置。
    // enable() 可能同步触发首次 PRE_UPDATE 回调，若此时 added_time_us=0，
    // 超时判断 (esp_timer_get_time() - 0) / 1000 必然 > 5000ms，会走超时路径
    // 清除 just_added 标志并误发 LoRa 命令，导致开窗器意外动作。
    entry->added_time_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_device_table_lock);

    // 6. 启用端点（SDK 自动通知 PartsList 变更 + 标记所有属性为 dirty）
    //    SDK 的 endpoint::enable() 内部调用：
    //    a) invoke_init_callbacks_internal() — 运行各 cluster 的 init 回调
    //       ⚠️ WindowCovering 的 init 回调会从 NVS 读取 ConfigStatus 并覆盖为 0x00！
    //       必须在 enable() 之后重新设置 ConfigStatus，否则 HomeKit 看到 Operational=0
    //    b) MatterReportingAttributeChangeCallback(ep_id) — 标记新端点所有属性为 dirty
    //    c) report_parts_list_change_internal(ep) — 通知 aggregator 的 PartsList 变更
    err = endpoint::enable(endpoint);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "启用端点失败: %s", esp_err_to_name(err));
        endpoint::destroy(s_node, endpoint);
        taskENTER_CRITICAL(&s_device_table_lock);
        clear_device_entry(entry);
        taskEXIT_CRITICAL(&s_device_table_lock);
        return err;
    }

    // 7. 在 enable() 之后重新设置关键属性并验证
    //    原因：NONVOLATILE 属性（ConfigStatus、Mode、CurrentPosition*）在
    //    attribute::create() 时会从 NVS 加载旧值。当端点 ID 被复用（先解绑再配对）
    //    时，NVS 中残留的旧值会覆盖 config 中的新值。
    //    SDK 的 MatterWindowCoveringPluginServerInitCallback 也会将 ConfigStatus
    //    重置为 0x00（覆盖 config 中的 0x09），必须重新设置。
    //
    //    优化：非 NONVOLATILE 属性（Type、EndProductType、FeatureMap）不会被 NVS
    //    覆盖，且 SDK init callback 不会修改它们。enable() 后直接读取验证即可，
    //    无需强制 update。这样可以减少不必要的 attribute::update() 调用，
    //    减少 ReportData 报文大小（每次 update 都会标记属性为 dirty），
    //    避免 ReportData 超过 1000 字节导致 HomeKit 订阅被拆除。

    // 7a. 验证 FeatureMap（非 NONVOLATILE，不应被覆盖）
    //     SDK 在 cluster create 时已用 config->feature_flags 初始化，
    //     enable() 后 SDK init callback 不会修改它。
    //     注意：attribute::update() 在 enable() 后会返回 ESP_ERR_INVALID_ARG，
    //     因为 SDK init callback 改变了属性的内部类型表示。
    //     但值本身是正确的（cluster create 时设置），无需 update。
    //     此处仅读取验证，不调用 update。

    // 7b. 强制重设 ConfigStatus（NONVOLATILE + SDK init callback 会重置为 0x00）
    esp_matter_attr_val_t cs_val = esp_matter_bitmap8(
        WC_CONFIG_STATUS_OPERATIONAL | WC_CONFIG_STATUS_LIFT_POSITION_AWARE);
    esp_err_t cs_err = attribute::update(ep_id, WindowCovering::Id,
                                          ATTR_CONFIG_STATUS, &cs_val);
    if (cs_err != ESP_OK) {
        ESP_LOGW(TAG, "更新 ConfigStatus 失败: ep=%u err=%s", ep_id, esp_err_to_name(cs_err));
    }

    // 7c. 初始化 Lift 位置 data version（NONVOLATILE，可能被 NVS 旧值覆盖）
    esp_matter_attr_val_t lift_val = esp_matter_nullable_uint16(nullable<uint16_t>(0));
    esp_err_t lift_err = attribute::update(ep_id, WindowCovering::Id,
                                            ATTR_CURRENT_LIFT_PERCENT_100THS, &lift_val);
    if (lift_err != ESP_OK) {
        ESP_LOGW(TAG, "初始化 Lift data version 失败: ep=%u err=%s",
                 ep_id, esp_err_to_name(lift_err));
    }

    // P-ReportData2: CurrentPositionLiftPercentage 已移除，不再初始化

    // 7f. 验证日志：读取所有关键属性的实际值
    {
        esp_matter_attr_val_t verify_val = esp_matter_invalid(NULL);

        // FeatureMap（仅读取验证，不调用 update）
        if (attribute::get_val(ep_id, WindowCovering::Id, ATTR_FEATURE_MAP, &verify_val) == ESP_OK) {
            ESP_LOGI(TAG, "[验证] FeatureMap=0x%04lX (期望0x0005) ep=%u", verify_val.val.u32, ep_id);
        } else {
            ESP_LOGE(TAG, "[验证] FeatureMap 读取失败！ep=%u", ep_id);
        }
        // ConfigStatus
        if (attribute::get_val(ep_id, WindowCovering::Id, ATTR_CONFIG_STATUS, &verify_val) == ESP_OK) {
            ESP_LOGI(TAG, "[验证] ConfigStatus=0x%02X (期望0x09) ep=%u", verify_val.val.u8, ep_id);
        }
        // Type（仅读取验证）
        if (attribute::get_val(ep_id, WindowCovering::Id, ATTR_TYPE, &verify_val) == ESP_OK) {
            ESP_LOGI(TAG, "[验证] Type=0x%02X (期望0x00) ep=%u", verify_val.val.u8, ep_id);
        }
        // EndProductType（仅读取验证）
        if (attribute::get_val(ep_id, WindowCovering::Id, ATTR_END_PRODUCT_TYPE, &verify_val) == ESP_OK) {
            ESP_LOGI(TAG, "[验证] EndProductType=0x%02X (期望0x00) ep=%u", verify_val.val.u8, ep_id);
        }
        // CurrentPositionLiftPercent100ths
        if (attribute::get_val(ep_id, WindowCovering::Id, ATTR_CURRENT_LIFT_PERCENT_100THS, &verify_val) == ESP_OK) {
            ESP_LOGI(TAG, "[验证] CurrentLift%%=0x%04X ep=%u", verify_val.val.u16, ep_id);
        }
        ESP_LOGI(TAG, "[验证] ===== WindowCovering 属性验证完成 ep=%u =====", ep_id);
    }

    // 8. 显式更新 Reachable=true
    app_matter_bridge_update_reachable(ep_id, true);

    // 9. 创建模式选择端点（独立 bridged_node + OnOff 开关）——仅 5005 设备
    //    HomeKit 中显示为独立开关配件：ON=平开模式(1), OFF=内倒模式(0)
    //    用户切换时触发 MQTT 命令：rwp_wind_lock_mode
    //    仅 SN 前缀为 5005 的内开内倒窗需要模式选择
    if (support_niedao) {
        endpoint::bridged_node::config_t mode_bridged_config{};
        endpoint_t *mode_endpoint = endpoint::bridged_node::create(s_node, &mode_bridged_config,
                                                                     ENDPOINT_FLAG_DESTROYABLE | ENDPOINT_FLAG_BRIDGE,
                                                                     NULL);
        if (mode_endpoint == NULL) {
            ESP_LOGE(TAG, "创建模式选择 bridged_node 端点失败: ep=%u", ep_id);
        } else {
            // 添加 OnOff Light device type (0x0100)，HomeKit 渲染为开关
            endpoint::add_device_type(mode_endpoint, ESP_MATTER_ON_OFF_LIGHT_DEVICE_TYPE_ID,
                                       ESP_MATTER_ON_OFF_LIGHT_DEVICE_TYPE_VERSION);

            // 创建 OnOff cluster
            cluster::on_off::config_t mode_on_off_config{};
            mode_on_off_config.on_off = false;  // 默认内倒模式
            cluster_t *mode_on_off_cluster = cluster::on_off::create(mode_endpoint, &mode_on_off_config,
                                                                      CLUSTER_FLAG_SERVER);
            if (mode_on_off_cluster != NULL) {
                cluster::on_off::command::create_on(mode_on_off_cluster);
                cluster::on_off::command::create_off(mode_on_off_cluster);
                cluster::on_off::command::create_toggle(mode_on_off_cluster);
            }

            // 添加 BDI 属性（最小化）
            cluster_t *mode_bdi = cluster::get(mode_endpoint, BridgedDeviceBasicInformation::Id);
            if (mode_bdi != NULL) {
                const char *mode_display_name = (device_name != NULL) ? device_name : device_sn;
                char mode_name[32] = {0};
                strncpy(mode_name, mode_display_name, sizeof(mode_name) - 1);
                cluster::bridged_device_basic_information::attribute::create_product_name(
                    mode_bdi, mode_name, strlen(mode_name));
                char mode_serial[32] = {0};
                strncpy(mode_serial, device_sn, sizeof(mode_serial) - 1);
                cluster::bridged_device_basic_information::attribute::create_serial_number(
                    mode_bdi, mode_serial, strlen(mode_serial));
                // 认证属性
                const char *vendor_str = "成都慧尖科技有限公司";
                char vendor_name[32] = {0};
                strncpy(vendor_name, vendor_str, sizeof(vendor_name) - 1);
                cluster::bridged_device_basic_information::attribute::create_vendor_name(
                    mode_bdi, vendor_name, strlen(vendor_name));
                cluster::bridged_device_basic_information::attribute::create_vendor_id(
                    mode_bdi, CONFIG_DEVICE_VENDOR_ID);
                cluster::bridged_device_basic_information::attribute::create_product_id(
                    mode_bdi, CONFIG_DEVICE_PRODUCT_ID);
                cluster::bridged_device_basic_information::attribute::create_hardware_version(mode_bdi, 1);
                cluster::bridged_device_basic_information::attribute::create_software_version(mode_bdi, 1);
            }

            // 设置父端点为 aggregator
            esp_err_t mode_parent_err = endpoint::set_parent_endpoint(mode_endpoint, s_aggregator);
            if (mode_parent_err != ESP_OK) {
                ESP_LOGE(TAG, "设置模式端点父端点失败: %s", esp_err_to_name(mode_parent_err));
            }

            // 启用模式端点
            esp_err_t mode_enable_err = endpoint::enable(mode_endpoint);
            if (mode_enable_err != ESP_OK) {
                ESP_LOGE(TAG, "启用模式端点失败: %s", esp_err_to_name(mode_enable_err));
            } else {
                uint16_t mode_ep_id = endpoint::get_id(mode_endpoint);
                // 记录到设备表
                taskENTER_CRITICAL(&s_device_table_lock);
                entry->mode_ep_id = mode_ep_id;
                taskEXIT_CRITICAL(&s_device_table_lock);
                // 设置 Reachable=true
                app_matter_bridge_update_reachable(mode_ep_id, true);
                ESP_LOGI(TAG, "创建模式选择端点: mode_ep=%u sn=%s (平开/内倒开关)",
                         mode_ep_id, device_sn);
            }
        }
    } else {
        ESP_LOGI(TAG, "设备不支持模式选择，跳过模式端点: sn=%s", device_sn);
    }

    // 10. 延迟 PartsList 二次通知（核心修复）
    //    问题：SDK 在 endpoint::enable() 中发送的 PartsList 变更通知，
    //    可能因控制器订阅重建而丢失。
    //
    //    日志证据：
    //    T=350341 ReportData(1204B) 发送 → 重传4次
    //    T=354161 StatusResponse(0x7d) 订阅被拆除
    //    T=357191 新订阅建立
    //
    //    2秒延迟在旧订阅重传期间触发，被旧订阅消费。
    //    增加到5秒，确保在新订阅建立后才触发 PartsList 通知。
    //
    //    定时器回调运行在 Matter 事件循环线程，已持有 StackLock。
    if (s_aggregator_endpoint_id != 0) {
        chip::DeviceLayer::SystemLayer().StartTimer(
            chip::System::Clock::Seconds32(5),
            [](chip::System::Layer *layer, void *callback_context) {
                uint16_t agg_ep = s_aggregator_endpoint_id;
                MatterReportingAttributeChangeCallback(
                    agg_ep, DESCRIPTOR_CLUSTER_ID, DESCRIPTOR_ATTR_PARTS_LIST);
                ESP_LOGI(TAG, "延迟 PartsList 二次通知: aggregator_ep=%u", agg_ep);
            },
            nullptr);
    }

    // P1-3 + P2-3 修复：added_time_us 已在 enable() 前设置，此处无需再设。
    // just_added 标志保持 true，SDK 异步触发首次 PRE_UPDATE 时通过超时判断跳过；
    // 2000ms 后超时失效，用户命令正常执行（避免永久跳过）。

    ESP_LOGI(TAG, "创建桥接端点: ep=%u sn=%s name=%s parent=aggregator(%u)",
             ep_id, device_sn, display_name, s_aggregator_endpoint_id);

    if (endpoint_id) {
        *endpoint_id = ep_id;
    }

    push_matter_event(MATTER_EVENT_DEVICE_ADDED, ep_id, device_sn, 0);
    return ESP_OK;
}

esp_err_t app_matter_bridge_remove_device(uint16_t endpoint_id)
{
    // 临界区内查找+复制快照，然后清除表项
    char device_sn[32] = {0};
    uint16_t mode_ep_to_destroy = 0;
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry = find_by_endpoint(endpoint_id);
    if (entry == NULL) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        return ESP_ERR_NOT_FOUND;
    }
    strncpy(device_sn, entry->device_sn, sizeof(device_sn) - 1);
    mode_ep_to_destroy = entry->mode_ep_id;
    clear_device_entry(entry);
    taskEXIT_CRITICAL(&s_device_table_lock);

    // 销毁模式选择端点（如果存在）
    // 注意：endpoint::destroy() 内部已调用 disable()，无需显式调用 disable()。
    // 重复调用 disable() 会触发两次 MatterReportingAttributeChangeCallback(PartsList)，
    // 增加不必要的报告开销，并可能延迟 HomeKit 端设备删除。
    if (mode_ep_to_destroy != 0) {
        endpoint_t *mode_ep = endpoint::get(s_node, mode_ep_to_destroy);
        if (mode_ep) {
            endpoint::destroy(s_node, mode_ep);
        }
    }

    endpoint_t *endpoint = endpoint::get(s_node, endpoint_id);
    if (endpoint) {
        endpoint::destroy(s_node, endpoint);
    }

    ESP_LOGI(TAG, "移除桥接端点: ep=%u mode_ep=%u sn=%s", endpoint_id, mode_ep_to_destroy, device_sn);
    push_matter_event(MATTER_EVENT_DEVICE_REMOVED, endpoint_id, device_sn, 0);

    return ESP_OK;
}

int app_matter_bridge_remove_all_devices(void)
{
    if (s_node == NULL) {
        ESP_LOGW(TAG, "remove_all: node 未初始化");
        return 0;
    }

    // 收集所有需要移除的端点 ID（临界区外不能调用 Matter API，先收集再销毁）
    // 最多收集 MAX_BRIDGED_DEVICES * 2 个端点（每个设备可能有主端点 + 模式端点）
    uint16_t eps_to_remove[MAX_BRIDGED_DEVICES * 2];
    int ep_count = 0;

    // 1. 从 s_device_table 收集已知端点（含模式端点）
    taskENTER_CRITICAL(&s_device_table_lock);
    for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
        if (s_device_table[i].in_use) {
            eps_to_remove[ep_count++] = s_device_table[i].endpoint_id;
            if (s_device_table[i].mode_ep_id != 0) {
                eps_to_remove[ep_count++] = s_device_table[i].mode_ep_id;
            }
        }
    }
    taskEXIT_CRITICAL(&s_device_table_lock);

    // 2. 从 Matter node 枚举所有端点，补充 s_device_table 中没有的
    //    esp_matter 不持久化动态端点结构（仅持久化 min_unused_endpoint_id 和属性值），
    //    重启后只有 root + aggregator，但 HomeKit 仍缓存旧端点。
    //    枚举确保能找到当前内存中的所有端点。
    {
        chip::DeviceLayer::StackLock lock;
        endpoint_t *ep = endpoint::get_first(s_node);
        while (ep != NULL && ep_count < (int)(sizeof(eps_to_remove) / sizeof(eps_to_remove[0]))) {
            uint16_t ep_id = endpoint::get_id(ep);
            // 跳过 root endpoint (0) 和 aggregator endpoint
            if (ep_id != 0 && ep_id != s_aggregator_endpoint_id) {
                // 检查是否已在列表中
                bool found = false;
                for (int i = 0; i < ep_count; i++) {
                    if (eps_to_remove[i] == ep_id) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    eps_to_remove[ep_count++] = ep_id;
                }
            }
            ep = endpoint::get_next(ep);
        }
    }

    ESP_LOGI(TAG, "remove_all: 发现 %d 个端点需要移除", ep_count);

    // 3. 逐个禁用并销毁端点（需要 StackLock）
    int removed = 0;
    if (ep_count > 0) {
        chip::DeviceLayer::StackLock lock;
        for (int i = 0; i < ep_count; i++) {
            endpoint_t *endpoint = endpoint::get(s_node, eps_to_remove[i]);
            if (endpoint != NULL) {
                // destroy() 内部已调用 disable()，无需显式调用
                endpoint::destroy(s_node, endpoint);
                removed++;
                ESP_LOGI(TAG, "remove_all: 已销毁端点 ep=%u", eps_to_remove[i]);
            }
        }
    }

    // 4. 清空整个 s_device_table（不需要 StackLock）
    // 注意：ESP_LOG 不能在 taskENTER_CRITICAL 临界区内调用，
    // newlib 的 log lock 在临界区内获取会触发 lock_acquire_generic 的 abort() 检测。
    int cleaned_count = 0;
    taskENTER_CRITICAL(&s_device_table_lock);
    for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
        if (s_device_table[i].in_use) {
            clear_device_entry(&s_device_table[i]);
            cleaned_count++;
        }
    }
    taskEXIT_CRITICAL(&s_device_table_lock);
    if (cleaned_count > 0) {
        ESP_LOGI(TAG, "remove_all: 清理了 %d 个设备表项", cleaned_count);
    }

    // 5. 关键：无论是否销毁了端点，都主动通知 PartsList 变更
    //    场景：重启后 esp_matter 内存中没有桥接端点（动态端点不持久化），
    //    但 HomeKit 仍缓存旧端点。如果不通知，HomeKit 需要等订阅超时后
    //    自行清理（可能数分钟）。发送 PartsList 报告后，HomeKit 下次
    //    订阅报告周期会立即收到最新 PartsList（不含旧端点），快速移除卡片。
    {
        chip::DeviceLayer::StackLock lock;
        ESP_LOGI(TAG, "remove_all: 通知 PartsList 变更 (aggregator_ep=%u, root_ep=0)", s_aggregator_endpoint_id);
        // 通知 aggregator 的 PartsList
        if (s_aggregator_endpoint_id != 0) {
            MatterReportingAttributeChangeCallback(
                s_aggregator_endpoint_id, DESCRIPTOR_CLUSTER_ID, DESCRIPTOR_ATTR_PARTS_LIST);
        }
        // 通知 root endpoint 的 PartsList
        MatterReportingAttributeChangeCallback(
            0, DESCRIPTOR_CLUSTER_ID, DESCRIPTOR_ATTR_PARTS_LIST);
    }

    ESP_LOGI(TAG, "remove_all: 完成，共移除 %d 个端点，已通知 PartsList", removed);
    return removed;
}

esp_err_t app_matter_bridge_update_position(uint16_t endpoint_id, uint8_t lora_position)
{
    // 范围校验：lora_position 必须在 0-100 范围内
    if (lora_position > 100) {
        ESP_LOGW(TAG, "update_position 忽略无效位置: ep=%u lora=%u（>100）", endpoint_id, lora_position);
        return ESP_ERR_INVALID_ARG;
    }

    // P-Opt2: 值未变化跳过——网关每 2 秒上报 002，位置相同时跳过属性写入
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *cache_entry = find_by_endpoint(endpoint_id);
    if (cache_entry != NULL && cache_entry->last_lora_position == (int16_t)lora_position) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        return ESP_OK;  // 值未变化，跳过
    }
    if (cache_entry != NULL) {
        cache_entry->last_lora_position = (int16_t)lora_position;
    }
    taskEXIT_CRITICAL(&s_device_table_lock);

    // 反转：LoRa 位置 → Matter 百分比
    uint8_t matter_percent = lora_to_matter_percent(lora_position);
    uint16_t matter_percent_100ths = (uint16_t)matter_percent * 100;

    ESP_LOGI(TAG, "更新 Matter 位置: ep=%u lora=%u → matter=%u%% (raw=%u)",
             endpoint_id, lora_position, matter_percent, matter_percent_100ths);

    esp_matter_attr_val_t val = esp_matter_nullable_uint16(nullable<uint16_t>(matter_percent_100ths));

    // 设置方向标志，让 app_attribute_update_cb 跳过此次 TargetPosition 变更，
    // 避免 LoRa 上报 → Matter 更新 → 又发 LoRa 命令 的控制回环
    // per-endpoint：通过 endpoint_id 查找对应 entry 设置标志，避免影响其他端点。
    // 注意：updating_from_mqtt=true 时，回调在 just_added_lift 检查前 early return，
    //       因此 just_added_lift/tilt 仅对外部触发的首次变更生效（如控制器拖滑块）。
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry = find_by_endpoint(endpoint_id);
    if (entry == NULL) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        ESP_LOGW(TAG, "update_position 找不到 entry: ep=%u（端点可能已被移除）", endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }
    entry->updating_from_mqtt = true;
    taskEXIT_CRITICAL(&s_device_table_lock);

    // 1. 更新 TargetPositionLiftPercent100ths (0x000B)
    esp_err_t target_err = attribute::update(endpoint_id,
                                              WindowCovering::Id,
                                              ATTR_TARGET_LIFT_PERCENT_100THS,
                                              &val);
    if (target_err != ESP_OK) {
        ESP_LOGW(TAG, "更新目标位置失败: %s（继续更新 Current）", esp_err_to_name(target_err));
    }

    // 2. 更新 CurrentPositionLiftPercent100ths (0x000E)
    //    即使 Target 更新失败也继续更新 Current，避免两者不一致
    esp_err_t current_err = attribute::update(endpoint_id,
                                               WindowCovering::Id,
                                               ATTR_CURRENT_LIFT_PERCENT_100THS,
                                               &val);
    if (current_err != ESP_OK) {
        ESP_LOGW(TAG, "更新当前位置失败: %s", esp_err_to_name(current_err));
    }

    // P-ReportData2: CurrentPositionLiftPercentage 已移除，不再更新
    // HomeKit 使用 CurrentPositionLiftPercent100ths 渲染滑块位置

    // OperationalStatus (0x000A) 是 Matter 规范中的只读属性，由 SDK 内部管理。
    // SDK 在收到 GoToLiftPercentage 命令时会自动设置 OperationalStatus 为运动中，
    // 运动完成后自动恢复为静止。应用层不应直接写入此属性。
    // 原代码使用 esp_matter_uint8() (type=8) 更新 bitmap8 类型属性 (type=16)，
    // 导致 ESP_ERR_INVALID_ARG (258) 错误日志。已移除此调用。

    // 清除方向标志（仅当之前成功设置时）
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry_clear = find_by_endpoint(endpoint_id);
    if (entry_clear != NULL) {
        entry_clear->updating_from_mqtt = false;
    }
    taskEXIT_CRITICAL(&s_device_table_lock);

    return (target_err != ESP_OK && current_err != ESP_OK) ? target_err : ESP_OK;
}

// PowerSource cluster 属性 ID（参见 SDK AttributeIds.h）
#define ATTR_BAT_VOLTAGE           0x000B   // BatVoltage (uint32, 单位 mV)
#define ATTR_BAT_PERCENT_REMAINING 0x000C   // BatPercentRemaining (uint8, 0-200, 0%=0, 100%=200)
#define ATTR_BAT_CHARGE_LEVEL      0x000E   // BatChargeLevel (enum8: 0=Ok, 1=Warning, 2=Critical)

// BatChargeLevel 枚举值（Matter 规范 PowerSource.BatChargeLevelEnum）
// HomeKit 用此属性显示电池图标（满/低/危），不更新会导致图标始终显示"满"(Ok)
#define BAT_CHARGE_LEVEL_OK        0x00
#define BAT_CHARGE_LEVEL_WARNING   0x01
#define BAT_CHARGE_LEVEL_CRITICAL  0x02

// 电池电压范围（用于估算百分比），单位：原始上报值（÷10 得到伏特）
// 12V锂电池: 正常工作范围 9.5V-12.6V（raw 95-126），放宽到 8V-14V（raw 80-140）容错
#define BATTERY_VOLTAGE_RAW_MIN  80
#define BATTERY_VOLTAGE_RAW_MAX  140

esp_err_t app_matter_bridge_update_battery(uint16_t endpoint_id, uint16_t voltage_mv, int8_t percent)
{
    // 如果 percent=-1，根据 voltage_mv 自动估算
    // voltage_mv = raw_voltage * 100（handle_ctype_005 中转换）
    // raw_voltage 范围 95-120，对应 voltage_mv 范围 9500-12000
    uint8_t bat_percent;
    if (percent >= 0 && percent <= 100) {
        bat_percent = (uint8_t)percent;
    } else {
        // 从 voltage_mv 反算 raw_voltage，再用线性映射估算百分比
        uint16_t raw_voltage = voltage_mv / 100;
        if (raw_voltage <= BATTERY_VOLTAGE_RAW_MIN) {
            bat_percent = 0;
        } else if (raw_voltage >= BATTERY_VOLTAGE_RAW_MAX) {
            bat_percent = 100;
        } else {
            bat_percent = (uint8_t)((raw_voltage - BATTERY_VOLTAGE_RAW_MIN) * 100 /
                                     (BATTERY_VOLTAGE_RAW_MAX - BATTERY_VOLTAGE_RAW_MIN));
        }
    }

    // P-Opt2: 值未变化跳过——电压和估算百分比均未变化时跳过属性写入
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *cache_entry = find_by_endpoint(endpoint_id);
    if (cache_entry != NULL && cache_entry->last_voltage_mv == (int32_t)voltage_mv) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        return ESP_OK;  // 值未变化，跳过
    }
    if (cache_entry != NULL) {
        cache_entry->last_voltage_mv = (int32_t)voltage_mv;
    }
    taskEXIT_CRITICAL(&s_device_table_lock);

    ESP_LOGI(TAG, "更新 Matter 电池: ep=%u voltage=%umV → percent=%u%%",
             endpoint_id, voltage_mv, bat_percent);

    // 更新 BatVoltage (0x000B)，类型 nullable uint32
    // BatVoltage 创建时为 nullable（ATTRIBUTE_FLAG_NULLABLE），必须用 nullable 值更新
    esp_matter_attr_val_t volt_val = esp_matter_nullable_uint32(nullable<uint32_t>(voltage_mv));
    esp_err_t volt_err = attribute::update(endpoint_id, PowerSource::Id,
                                       ATTR_BAT_VOLTAGE, &volt_val);
    if (volt_err != ESP_OK) {
        ESP_LOGW(TAG, "更新电池电压失败: %s（继续更新百分比）", esp_err_to_name(volt_err));
    }

    // 更新 BatPercentRemaining (0x000C)，类型 nullable uint8
    // Matter 规范：BatPercentRemaining 范围 0-200（0%=0, 100%=200）
    // bat_percent 是 0-100 的百分比值，需 ×2 转换为规范值
    // BatVoltage 和 BatPercentRemaining 是独立属性，电压失败不应阻止百分比更新
    esp_matter_attr_val_t pct_val = esp_matter_nullable_uint8(nullable<uint8_t>(bat_percent * 2));
    esp_err_t pct_err = attribute::update(endpoint_id, PowerSource::Id,
                             ATTR_BAT_PERCENT_REMAINING, &pct_val);
    if (pct_err != ESP_OK) {
        ESP_LOGW(TAG, "更新电池百分比失败: %s（继续更新电量等级）", esp_err_to_name(pct_err));
    }

    // 更新 BatChargeLevel (0x000E)，类型 enum8
    // HomeKit 用此属性显示电池图标（满/低/危）。
    // cluster::power_source::create() 内部 feature::battery::add() 创建此属性，
    // 默认值 0 (Ok)。若不更新，低电量时图标仍显示"满"，误导用户。
    //
    // 映射规则（参考 Matter 规范 BatChargeLevelEnum）：
    //   BatChargeLevel 是 HomeKit "低电量"指示的唯一依据：
    //     Ok(0)      → 不显示低电量，首页无感叹号
    //     Warning(1) → 显示"电池电量低"，首页出现感叹号
    //     Critical(2) → 显示"电池电量危急"，首页出现感叹号
    //
    //   业务需求：电量 < 20% 才显示"电池电量低"
    //   0-9%   → Critical (2)：电量极低，需立即更换电池
    //   10-19% → Warning  (1)：电量低，提示用户更换电池
    //   ≥20%   → Ok       (0)：电量正常
    uint8_t bat_charge_level;
    if (bat_percent < 10) {
        bat_charge_level = BAT_CHARGE_LEVEL_CRITICAL;
    } else if (bat_percent < 20) {
        bat_charge_level = BAT_CHARGE_LEVEL_WARNING;
    } else {
        bat_charge_level = BAT_CHARGE_LEVEL_OK;
    }
    esp_matter_attr_val_t cl_val = esp_matter_enum8(bat_charge_level);
    esp_err_t cl_err = attribute::update(endpoint_id, PowerSource::Id,
                                          ATTR_BAT_CHARGE_LEVEL, &cl_val);
    if (cl_err != ESP_OK) {
        ESP_LOGW(TAG, "更新电池电量等级失败: %s（不影响百分比显示）", esp_err_to_name(cl_err));
    }

    return ESP_OK;
}

esp_err_t app_matter_bridge_update_reachable(uint16_t endpoint_id, bool online)
{
    // 去重优化：仅当 Reachable 值变化或首次调用时才更新+报告。
    //
    // 原实现每次调用都强制 MatterReportingAttributeChangeCallback，网关频繁
    // 上下线时会产生报告风暴（N 个设备 × M 次 = N*M 个报告）。
    // 优化后：在 entry 中记录 last_reachable，仅变化时报告。
    // 首次调用（reachable_initialized=false）强制报告以初始化 data version。
    //
    // 注意：CASE Session 建立时会批量通知 Reachable（P-Speed1 回退），
    // 确保 HomeKit 重连后能获取端点 Reachable 状态并读取端点属性。

    bool should_report = true;
    bool is_mode_ep = false;
    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry = find_by_endpoint(endpoint_id);
    if (entry == NULL) {
        // 主端点未找到，检查是否为模式选择端点（mode_ep_id 字段）
        for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
            if (s_device_table[i].in_use && s_device_table[i].mode_ep_id == endpoint_id) {
                entry = &s_device_table[i];
                is_mode_ep = true;
                break;
            }
        }
    }
    if (entry == NULL) {
        // 端点不在设备表中（可能已被移除），不调用 Matter API 避免对不存在的端点操作
        taskEXIT_CRITICAL(&s_device_table_lock);
        ESP_LOGW(TAG, "update_reachable 找不到 entry: ep=%u（端点可能已被移除）", endpoint_id);
        return ESP_ERR_NOT_FOUND;
    }
    if (!is_mode_ep && entry->reachable_initialized && entry->last_reachable == online) {
        // 值未变化，跳过更新和报告（仅主端点去重，模式端点总是报告）
        should_report = false;
    }
    if (!is_mode_ep) {
        entry->reachable_initialized = true;
        entry->last_reachable = online;
    }
    taskEXIT_CRITICAL(&s_device_table_lock);

    if (!should_report) {
        ESP_LOGD(TAG, "Reachable 未变化，跳过报告: ep=%u online=%d", endpoint_id, online);
        return ESP_OK;
    }

    // 关键作用：更新 Reachable 属性并强制触发属性报告下发到控制器。
    //
    // 根本问题：bridged_node::create() 创建的 BridgedDeviceBasicInformation 簇，
    // 其 Reachable 属性默认值就是 true。当端点在线时调用 update(true)，
    // SDK 的 set_val_internal() 检测到新值==旧值，返回 ESP_ERR_NOT_FINISHED，
    // 跳过报告下发（参见 esp_matter_data_model.cpp:734-737）。
    // 控制器永远收不到 Reachable 报告 → 读取时无 data version → 显示离线。
    //
    // 解决方案（两步）：
    // 1. 仅当值真正变化时才调用 attribute::update() 更新存储值（触发 PRE_UPDATE 回调等）
    // 2. 无论值是否变化，都调用 MatterReportingAttributeChangeCallback() 强制标记属性为脏
    //    该 API 直接通知 IM 引擎属性变更，绕过 set_val_internal 的值比较检查
    //
    // 对于在线设备：值未变化（true→true），update 被跳过，但 report callback 仍触发报告
    // 对于离线恢复：值变化（false→true），update 更新值，report callback 也触发报告
    esp_matter_attr_val_t val = esp_matter_bool(online);
    esp_err_t err = attribute::update(endpoint_id, BridgedDeviceBasicInformation::Id,
                                       BridgedDeviceBasicInformation::Attributes::Reachable::Id, &val);
    // attribute::update 在值未变化时返回 ESP_OK（内部把 NOT_FINISHED 转为 OK），
    // 这里不判断 err，继续强制报告

    // 强制触发属性报告下发（绕过值变化检查）
    MatterReportingAttributeChangeCallback(
        endpoint_id, BridgedDeviceBasicInformation::Id,
        BridgedDeviceBasicInformation::Attributes::Reachable::Id);

    ESP_LOGI(TAG, "更新 Reachable: ep=%u online=%d (update err=%s)",
             endpoint_id, online, esp_err_to_name(err));
    return ESP_OK;
}

esp_err_t app_matter_bridge_find_endpoint(const char *device_sn, uint16_t *endpoint_id)
{
    if (device_sn == NULL || endpoint_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry = find_by_sn(device_sn);
    if (entry == NULL) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        return ESP_ERR_NOT_FOUND;
    }
    *endpoint_id = entry->endpoint_id;
    taskEXIT_CRITICAL(&s_device_table_lock);
    return ESP_OK;
}

esp_err_t app_matter_bridge_find_endpoints(const char *device_sn, uint16_t *endpoint_id, uint16_t *mode_ep_id)
{
    if (device_sn == NULL || endpoint_id == NULL || mode_ep_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_device_table_lock);
    bridged_device_entry_t *entry = find_by_sn(device_sn);
    if (entry == NULL) {
        taskEXIT_CRITICAL(&s_device_table_lock);
        return ESP_ERR_NOT_FOUND;
    }
    *endpoint_id = entry->endpoint_id;
    *mode_ep_id = entry->mode_ep_id;
    taskEXIT_CRITICAL(&s_device_table_lock);
    return ESP_OK;
}

int app_matter_bridge_device_count(void)
{
    int count = 0;
    taskENTER_CRITICAL(&s_device_table_lock);
    for (int i = 0; i < MAX_BRIDGED_DEVICES; i++) {
        if (s_device_table[i].in_use) {
            count++;
        }
    }
    taskEXIT_CRITICAL(&s_device_table_lock);
    return count;
}

bool app_matter_bridge_is_initialized(void)
{
    return (s_node != NULL);
}

/**
 * @file app_matter_bridge.h
 * @brief Matter Bridge 桥接器 - 基于 esp_matter
 *
 * 为每个 LoRa 子设备动态创建 Matter WindowCovering 虚拟端点。
 * 处理 Matter 属性变更回调，将控制命令转发给协议桥接层。
 *
 * 关键设计：位置语义反转
 * - LoRa 协议: 0=关闭, 100=打开
 * - Matter WindowCovering: 0%=完全打开, 100%=完全关闭
 * - 转换公式: matter_percent = 100 - lora_value
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Matter 事件类型
 */
typedef enum {
    MATTER_EVENT_NONE = 0,
    MATTER_EVENT_LIFT_CHANGED,       /**< WindowCovering 升降位置变更 */
    MATTER_EVENT_DEVICE_ADDED,       /**< 新设备添加 */
    MATTER_EVENT_DEVICE_REMOVED,     /**< 设备移除 */
    MATTER_EVENT_STOP_MOTION,        /**< WindowCovering 停止命令（value=101） */
    MATTER_EVENT_TILT_TOGGLE,        /**< 子设备内倒命令（value=200，Tilt 滑块触发） */
    MATTER_EVENT_MODE_CHANGED,       /**< 窗锁模式变更（value=0=内倒, 1=平开） */
    MATTER_EVENT_COMMISSIONING_COMPLETE, /**< Matter 配网完成，请求协议桥接层主动向所有已注册网关请求设备列表 */
} matter_event_type_t;

/**
 * @brief Matter 事件结构体（推入队列供协议桥接层处理）
 */
typedef struct {
    matter_event_type_t type;        /**< 事件类型 */
    uint16_t endpoint_id;            /**< Matter 端点 ID */
    char device_sn[32];             /**< 对应的 LoRa 设备 SN */
    int32_t value;                  /**< 属性值（LoRa 值 0-100, 0=关闭 100=打开） */
} matter_event_t;

/**
 * @brief Bridge 启动配置
 */
typedef struct {
    QueueHandle_t event_queue;       /**< 事件队列，Matter 属性变更推入此队列 */
} matter_bridge_config_t;

/**
 * @brief 初始化 Matter Bridge
 *
 * 创建 node + aggregator 端点，注册属性回调。
 * vendor_id / product_id 通过 sdkconfig 的 CONFIG_DEVICE_VENDOR_ID /
 * CONFIG_DEVICE_PRODUCT_ID 配置，不支持运行时修改。
 */
esp_err_t app_matter_bridge_init(const matter_bridge_config_t *config);

/**
 * @brief 启动 Matter（开始 BLE 广播和配网）
 */
esp_err_t app_matter_bridge_start(void);

/**
 * @brief 为 LoRa 子设备创建 Matter WindowCovering 虚拟端点
 *
 * 在 aggregator 下创建 bridged_node 端点，添加 WindowCovering device type。
 * 调用 endpoint::enable() 自动通知 Matter 控制器端点变更。
 *
 * @param device_sn LoRa 设备 SN
 * @param device_name 设备显示名称
 * @param endpoint_id [out] 返回创建的 Matter 端点 ID
 * @return ESP_OK 成功
 */
esp_err_t app_matter_bridge_add_device(const char *device_sn, const char *device_name, uint16_t *endpoint_id);

/**
 * @brief 移除 Matter 虚拟端点
 */
esp_err_t app_matter_bridge_remove_device(uint16_t endpoint_id);

/**
 * @brief 移除所有桥接子设备端点
 *
 * 枚举 Matter node 下所有端点（跳过 root 和 aggregator），
 * 逐个禁用并销毁。用于"3击删除所有子设备"功能。
 * 重启后 s_device_table 为空但 Matter 端点持久化在 NVS 中，
 * 此函数直接从 Matter node 枚举端点，确保彻底清理。
 *
 * @return 移除的端点数量
 */
int app_matter_bridge_remove_all_devices(void);

/**
 * @brief 更新 Matter WindowCovering 位置（MQTT→Matter 方向）
 *
 * 将 LoRa 位置（0=关闭, 100=打开）反转为 Matter 百分比（0%=打开, 100%=关闭）
 *
 * @param endpoint_id Matter 端点 ID
 * @param lora_position LoRa 位置值 0-100（0=关闭, 100=打开）
 * @return ESP_OK 成功
 */
esp_err_t app_matter_bridge_update_position(uint16_t endpoint_id, uint8_t lora_position);

/**
 * @brief 更新子设备电池电压（LoRa 上报 → Matter PowerSource cluster）
 *
 * @param endpoint_id Matter 端点 ID
 * @param voltage_mv 电池电压（毫伏，如 10500 = 10.5V）
 * @param percent 电池百分比 0-100（-1 表示自动估算）
 * @return ESP_OK 成功
 */
esp_err_t app_matter_bridge_update_battery(uint16_t endpoint_id, uint16_t voltage_mv, int8_t percent);

/**
 * @brief 更新端点 Reachable 属性（BridgedDeviceBasicInformation cluster）
 *
 * Tuya/Apple Home 等控制器通过 Reachable 属性判断桥接设备在线/离线状态。
 * 端点创建时必须调用本函数初始化簇的 data version，否则控制器读取会
 * 报 "unknown cluster - no data version available" 错误，设备显示离线。
 *
 * @param endpoint_id Matter 端点 ID
 * @param online true=在线, false=离线
 * @return ESP_OK 成功
 */
esp_err_t app_matter_bridge_update_reachable(uint16_t endpoint_id, bool online);

/**
 * @brief 通过 device_sn 查找 endpoint_id
 */
esp_err_t app_matter_bridge_find_endpoint(const char *device_sn, uint16_t *endpoint_id);

/**
 * @brief 通过 device_sn 查找主端点 ID 和模式端点 ID
 *
 * 模式端点（mode_ep_id）仅 5005 系列设备有，其他设备 mode_ep_id=0。
 * 用于网关上下线时同时更新主端点和模式端点的 Reachable 属性。
 *
 * @param device_sn LoRa 设备 SN
 * @param endpoint_id [out] 主端点 ID
 * @param mode_ep_id [out] 模式端点 ID（0=无模式端点）
 * @return ESP_OK 成功，ESP_ERR_NOT_FOUND 设备不存在
 */
esp_err_t app_matter_bridge_find_endpoints(const char *device_sn, uint16_t *endpoint_id, uint16_t *mode_ep_id);

/**
 * @brief 获取当前已桥接的设备数量
 */
int app_matter_bridge_device_count(void);

/**
 * @brief 检查 Bridge 是否已初始化（node 已创建）
 */
bool app_matter_bridge_is_initialized(void);

#ifdef __cplusplus
}
#endif

/**
 * @file app_protocol_bridge.h
 * @brief Matter↔MQTT 协议桥接层
 *
 * 实现 $SH MQTT 协议与 Matter 属性的双向转换：
 * - Matter→MQTT：用户操作 Apple Home → Matter 属性变更 → 发送 $SH 004 控制命令
 * - MQTT→Matter：LoRa 设备状态上报 → 更新对应 Matter 端点位置
 *
 * 完整 $SH 协议处理：
 * - 001：网关绑定（自动回复 errcode=0，记录网关 SN）
 * - 002：设备列表（解析 devices 数组，为每个设备创建 Matter 端点）
 * - 003：设备配对
 * - 004：设备控制（Matter→LoRa 方向）
 * - 005：设备状态上报（双格式：直接字段 + attrs 数组）
 *
 * 协议参考：e:\AI\huijian-gateway\ha-window-controller-gateway\custom_components\window_controller_gateway\const.py
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_matter_bridge.h"
#include "app_mqtt_broker.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 协议桥接配置
 */
typedef struct {
    const char *bridge_sn;          /**< 本桥接器的序列号（用于 001 响应的 uuid 字段） */
    QueueHandle_t mqtt_msg_queue;   /**< MQTT 消息队列（broker 推入） */
    QueueHandle_t matter_event_queue; /**< Matter 事件队列（bridge 推入） */
} protocol_bridge_config_t;

/**
 * @brief 初始化协议桥接层（创建 MQTT 客户端，但不启动）
 */
esp_err_t app_protocol_bridge_init(const protocol_bridge_config_t *config);

/**
 * @brief 启动协议桥接 task（等待 WiFi 连接后自动启动 MQTT 客户端）
 */
esp_err_t app_protocol_bridge_start(void);

/**
 * @brief WiFi 连接成功后启动 MQTT 客户端（内部调用，由事件处理器触发）
 */
esp_err_t app_protocol_bridge_on_wifi_connected(void);

/**
 * @brief 停止协议桥接
 */
void app_protocol_bridge_stop(void);

/**
 * @brief 检查 LoRa 网关离线状态（超时未上报则标记离线）
 *
 * 由系统监控 task 定期调用。网关超过 GATEWAY_OFFLINE_TIMEOUT 秒未上报
 * 任何消息则标记为离线（online=false）。下次该网关上报时 register_gateway
 * 会重新标记为在线。
 * 注意：不删除已离线网关的 Matter 端点，避免 App 中设备频繁消失/出现。
 */
void app_protocol_bridge_check_gateway_offline(void);

/**
 * @brief 启动所有已注册网关的 LoRa 配对模式
 *
 * 发送 003 类型（ctype=003, bind=1）命令到所有在线网关，
 * 触发网关进入配对模式（60 秒内可配对子设备）。
 */
void app_protocol_bridge_start_pairing(void);

/**
 * @brief 删除最后添加的子设备
 *
 * 遍历所有网关下最后添加的子设备，发送解绑命令（ctype=003, bind=0）。
 */
void app_protocol_bridge_delete_last_device(void);

/**
 * @brief 删除所有 LoRa 子设备
 *
 * 遍历所有已注册网关下的所有子设备，逐个发送解绑命令（ctype=003, bind=0）。
 * 用于 3 击按键触发。设备间延迟 200ms 避免 MQTT 报文拥堵。
 */
void app_protocol_bridge_delete_all_devices(void);

#ifdef __cplusplus
}
#endif

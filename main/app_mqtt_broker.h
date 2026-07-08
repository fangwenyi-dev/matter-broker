/**
 * @file app_mqtt_broker.h
 * @brief MQTT Broker 封装 - 基于 espressif/mosquitto
 *
 * 在独立 FreeRTOS task 中运行 mosq_broker_run()，并提供消息回调机制。
 * Broker 监听 TCP 1883 端口，供 LoRa 网关和本地 MQTT 客户端连接。
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief MQTT 消息结构体（从 broker 收到的消息）
 */
typedef struct {
    char client_id[64];   /**< 客户端 ID */
    char topic[128];      /**< 主题 */
char data[4096];      /**< 负载数据（002 设备列表消息可达 1.5KB+，12 设备时需更大缓冲。
队列已在 PSRAM，4096×10=40KB 不占内部 RAM） */
    int  data_len;        /**< 负载长度 */
    int  qos;             /**< QoS 级别 */
    int  retain;          /**< retain 标志 */
} mqtt_message_t;

/**
 * @brief Broker 启动配置
 *
 * 契约说明：
 * - host / username / password 字符串会被内部拷贝到静态数组，调用者可在
 *   app_mqtt_broker_start() 返回后释放原字符串。
 * - username / password 最大长度 31 字节（不含终止符），超长返回 ESP_ERR_INVALID_ARG。
 * - username 与 password 必须同时为空（匿名模式）或同时非空（认证模式），
 *   只有一个非空返回 ESP_ERR_INVALID_ARG。
 * - port 有效范围 1-65535，否则返回 ESP_ERR_INVALID_ARG。
 * - msg_queue 必须以 sizeof(mqtt_message_t) 为 item size 创建，建议置于 PSRAM
 *   （单条约 4.2KB，10 条 ≈ 42KB，避免占用内部 RAM 影响 BLE controller），
 *   且在 broker 运行期间不得销毁。
 */
typedef struct {
    const char *host;        /**< 监听地址，默认 "0.0.0.0"（NULL 等价于 "0.0.0.0"） */
    int port;                /**< 监听端口，默认 1883（有效范围 1-65535） */
    const char *username;    /**< 认证用户名，NULL 或空串表示允许匿名（与 password 同空同非空） */
    const char *password;    /**< 认证密码（与 username 同空同非空） */
    QueueHandle_t msg_queue; /**< 消息队列，broker 收到消息后推入此队列 */
} mqtt_broker_config_t;

/**
 * @brief 启动 MQTT Broker
 *
 * 创建独立 FreeRTOS task 运行 mosq_broker_run()。该函数会阻塞等待 broker
 * 绑定端口的结果（最多 1.5 秒）：返回 ESP_OK 表示 broker 已成功绑定监听端口
 * 并开始接收连接；返回 ESP_FAIL 表示创建 task 失败、绑定端口失败或等待超时。
 *
 * @param config Broker 配置
 * @return ESP_OK 成功，ESP_FAIL 失败，ESP_ERR_INVALID_ARG 参数非法
 */
esp_err_t app_mqtt_broker_start(const mqtt_broker_config_t *config);

/**
 * @brief 停止 MQTT Broker
 */
void app_mqtt_broker_stop(void);

/**
 * @brief 检查 Broker 是否正在运行
 */
bool app_mqtt_broker_is_running(void);

#ifdef __cplusplus
}
#endif

/**
 * @file app_mqtt_broker.cpp
 * @brief MQTT Broker 封装实现
 *
 * 在独立 task 中运行 mosq_broker_run()，通过 message callback 将
 * 收到的消息推入 FreeRTOS Queue 供协议桥接层处理。
 */
#include "app_mqtt_broker.h"
#include "mosq_broker.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "app_mqtt_broker";

// Broker task 配置
#define BROKER_TASK_PRIORITY    10
#define BROKER_TASK_STACK_SIZE  16384
#define BROKER_TASK_CORE        1   // 固定到 core 1，与 Matter task 隔离

// 内部状态
static volatile bool s_broker_running = false;  // 跨 task 共享，需 volatile
static TaskHandle_t s_broker_task = NULL;
static QueueHandle_t s_msg_queue = NULL;
// 认证凭据和 host 使用静态数组，避免依赖调用者字符串生命周期
static char s_auth_username[32] = {0};
static char s_auth_password[32] = {0};
static char s_broker_host[64] = {0};
// 静态配置（task 长期访问，避免悬垂指针）
static mqtt_broker_config_t s_config;

// 项1: 启动结果回传信号量。start 中 take（等待绑定结果），task 中 give（run 返回时）。
//      mosq_broker_run 绑定成功会阻塞，绑定失败会快速返回，因此：
//      take 成功 = run 快速返回 = 绑定失败；take 超时 = run 阻塞 = 绑定成功。
static SemaphoreHandle_t s_broker_start_sem = NULL;
// 项4: stop 等待 task 退出信号量。stop 中 take，task 退出前 give。
static SemaphoreHandle_t s_broker_stop_sem = NULL;
// 项7: start 入口加锁，保护 s_broker_running 检查与设置，防止并发调用
static portMUX_TYPE s_broker_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief mosquitto 消息回调
 *
 * 由 broker 在其 task 上下文中调用，将消息复制到队列。
 * 注意：此回调在 broker task 中执行，不能阻塞。
 */
static void on_broker_message(char *client, char *topic, char *data, int len, int qos, int retain)
{
    if (s_msg_queue == NULL) {
        return;
    }

    // P2-10 修复：改为 static 避免每次回调在栈上分配 ~4.2KB
    // 安全性：broker_task 单线程运行，mosquitto 回调不可重入，static 无竞态
    static mqtt_message_t msg;
    memset(&msg, 0, sizeof(msg));
    strncpy(msg.client_id, client ? client : "", sizeof(msg.client_id) - 1);
    msg.client_id[sizeof(msg.client_id) - 1] = '\0';
    strncpy(msg.topic, topic ? topic : "", sizeof(msg.topic) - 1);
    msg.topic[sizeof(msg.topic) - 1] = '\0';

    // 防止溢出
    // 注意：data 可能为 NULL（空消息），此时 copy_len 必须为 0，避免越界
    // 注意：len 必须为正数，否则 msg.data[copy_len] 会以负索引写入（缓冲区下溢）
    int copy_len = (data != NULL && len > 0) ? len : 0;
    if (copy_len >= (int)sizeof(msg.data)) {
        copy_len = sizeof(msg.data) - 1;
        ESP_LOGE(TAG, "消息超长已截断: topic=%s len=%d max=%zu",
                 topic ? topic : "(null)", len, sizeof(msg.data));
    }
    if (copy_len > 0) {
        memcpy(msg.data, data, copy_len);
    }
    msg.data[copy_len] = '\0';
    msg.data_len = copy_len;
    msg.qos = qos;
    msg.retain = retain;

    // 非阻塞方式推入队列，队列满则丢弃（避免 broker task 阻塞）
    if (xQueueSend(s_msg_queue, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "消息队列已满，丢弃消息: topic=%s", msg.topic);
    }
}

/**
 * @brief mosquitto 连接认证回调
 *
 * 验证客户端的用户名和密码。返回 0 接受连接，非 0 拒绝。
 */
static int on_broker_connect(const char *client_id, const char *username,
                             const char *password, int password_len)
{
    if (s_auth_username[0] == '\0' || s_auth_password[0] == '\0') {
        return 0;  // 未配置认证，允许匿名
    }
    if (username == NULL || password == NULL || password_len < 0) {
        ESP_LOGW(TAG, "认证失败: 缺少用户名或密码, client=%s", client_id ? client_id : "?");
        return -1;
    }
    // P-HomeKit7 修复：username 也用 strlen+memcmp 恒定时间比较（与 password 一致），
    // 避免 strcmp 的 early-return 泄露匹配前缀长度（timing attack）。
    size_t auth_user_len = strlen(s_auth_username);
    if (auth_user_len == strlen(username) &&
        memcmp(username, s_auth_username, auth_user_len) == 0 &&
        strlen(s_auth_password) == (size_t)password_len &&
        memcmp(password, s_auth_password, password_len) == 0) {
        ESP_LOGI(TAG, "认证成功: client=%s user=%s", client_id ? client_id : "?", username);
        return 0;
    }
    ESP_LOGW(TAG, "认证失败: client=%s user=%s",
             client_id ? client_id : "?", username ? username : "(null)");
    return -1;
}

/**
 * @brief Broker task 入口
 */
static void broker_task(void *arg)
{
    mqtt_broker_config_t *config = (mqtt_broker_config_t *)arg;

    struct mosq_broker_config broker_cfg = {
        .host = config->host ? config->host : "0.0.0.0",
        .port = config->port,
        .tls_cfg = NULL,               // TCP 模式，不使用 TLS
        .handle_message_cb = on_broker_message,
        .handle_connect_cb = on_broker_connect,
    };

    ESP_LOGI(TAG, "启动 mosquitto broker: %s:%d", broker_cfg.host, broker_cfg.port);

    // mosq_broker_run() 是阻塞函数，直到 mosq_broker_stop() 被调用；
    // 若绑定端口失败（如端口被占用），会快速返回非 0 错误码。
    int rc = mosq_broker_run(&broker_cfg);
    ESP_LOGI(TAG, "mosquitto broker 退出, rc=%d", rc);

    // 项1: 如果 start 还在等待绑定结果（s_broker_running 仍为 true 且信号量存在），
    //      说明 run 在 start 阶段快速返回 → 绑定失败，通知 start。
    //      注意：正常 stop 时 s_broker_running 也为 true，此时 give 是无害的
    //      （start 已返回，无人 take，信号量残留会被下次 start 用 take(0) 清除）。
    if (s_broker_running && s_broker_start_sem != NULL) {
        xSemaphoreGive(s_broker_start_sem);
    }

    // P2-D 修复：s_broker_running 写入加锁，与其他写入点（L248-255/L270/L286）保持一致
    taskENTER_CRITICAL(&s_broker_lock);
    s_broker_running = false;
    s_broker_task = NULL;
    taskEXIT_CRITICAL(&s_broker_lock);

    // 项4: 通知 stop 调用方 task 已退出
    if (s_broker_stop_sem != NULL) {
        xSemaphoreGive(s_broker_stop_sem);
    }

    vTaskDelete(NULL);
}

esp_err_t app_mqtt_broker_start(const mqtt_broker_config_t *config)
{
    // 项7: 加锁保护入口检查，防止并发调用 start
    taskENTER_CRITICAL(&s_broker_lock);
    if (s_broker_running) {
        taskEXIT_CRITICAL(&s_broker_lock);
        ESP_LOGW(TAG, "Broker 已在运行");
        return ESP_OK;
    }
    taskEXIT_CRITICAL(&s_broker_lock);

    if (config == NULL || config->msg_queue == NULL) {
        ESP_LOGE(TAG, "无效的配置参数");
        return ESP_ERR_INVALID_ARG;
    }

    // 项6: port 范围校验
    if (config->port < 1 || config->port > 65535) {
        ESP_LOGE(TAG, "无效的端口: %d (有效范围 1-65535)", config->port);
        return ESP_ERR_INVALID_ARG;
    }

    // 项2: 凭据一致性校验（两者同空=匿名，同非空=认证，只一非空=非法）
    bool has_user = (config->username != NULL && config->username[0] != '\0');
    bool has_pass = (config->password != NULL && config->password[0] != '\0');
    if (has_user != has_pass) {
        ESP_LOGE(TAG, "凭据不一致: username/password 必须同时为空或同时非空");
        return ESP_ERR_INVALID_ARG;
    }

    // Fix-Bug3: 默认弱密码运行时警告。
    // Kconfig 默认 admin/admin，broker 监听 0.0.0.0 暴露在局域网，
    // 生产部署必须修改。此处不阻止启动（开发/测试需要默认值），
    // 但以 ESP_LOGW 显著警告，避免用户忽视安全隐患。
    if (has_user && has_pass &&
        strcmp(config->username, "admin") == 0 &&
        strcmp(config->password, "admin") == 0) {
        ESP_LOGW(TAG, "========================================");
        ESP_LOGW(TAG, "警告: 正在使用默认弱密码 admin/admin！");
        ESP_LOGW(TAG, "Broker 监听 0.0.0.0:%d，暴露在局域网中。", config->port);
        ESP_LOGW(TAG, "生产部署请通过 menuconfig 修改 MQTT_BROKER_USERNAME/PASSWORD。");
        ESP_LOGW(TAG, "========================================");
    }

    // 项5: 凭据长度校验（s_auth_username[32] / s_auth_password[32] 减去终止符最多 31 字节）
    if (has_user && strlen(config->username) > sizeof(s_auth_username) - 1) {
        ESP_LOGE(TAG, "username 过长 (%zu > %zu)", strlen(config->username), sizeof(s_auth_username) - 1);
        return ESP_ERR_INVALID_ARG;
    }
    if (has_pass && strlen(config->password) > sizeof(s_auth_password) - 1) {
        ESP_LOGE(TAG, "password 过长 (%zu > %zu)", strlen(config->password), sizeof(s_auth_password) - 1);
        return ESP_ERR_INVALID_ARG;
    }

    // 项1: 创建/复用启动信号量
    if (s_broker_start_sem == NULL) {
        s_broker_start_sem = xSemaphoreCreateBinary();
        if (s_broker_start_sem == NULL) {
            ESP_LOGE(TAG, "创建 start 信号量失败");
            return ESP_FAIL;
        }
    } else {
        // 清除可能残留的 token（上次 broker 退出时 task give 的）
        xSemaphoreTake(s_broker_start_sem, 0);
    }

    // 项4: 创建/复用 stop 信号量（start 失败时也需要等待 task 退出）
    if (s_broker_stop_sem == NULL) {
        s_broker_stop_sem = xSemaphoreCreateBinary();
        if (s_broker_stop_sem == NULL) {
            ESP_LOGE(TAG, "创建 stop 信号量失败");
            return ESP_FAIL;
        }
    } else {
        xSemaphoreTake(s_broker_stop_sem, 0);
    }

    s_msg_queue = config->msg_queue;
    // 复制凭据到静态数组，避免依赖调用者字符串生命周期
    if (config->username) {
        strncpy(s_auth_username, config->username, sizeof(s_auth_username) - 1);
        s_auth_username[sizeof(s_auth_username) - 1] = '\0';
    } else {
        s_auth_username[0] = '\0';
    }
    if (config->password) {
        strncpy(s_auth_password, config->password, sizeof(s_auth_password) - 1);
        s_auth_password[sizeof(s_auth_password) - 1] = '\0';
    } else {
        s_auth_password[0] = '\0';
    }

    // 拷贝配置到静态变量（task 需要长期访问）
    s_config = *config;
    // 项3: username/password 重定向到静态数组，避免浅拷贝导致悬垂指针
    s_config.username = s_auth_username;
    s_config.password = s_auth_password;
    // host 字符串复制到静态数组，避免依赖调用者指针生命周期（防止野指针）
    // P1-5 修复：NULL 时使用 "0.0.0.0"（INADDR_ANY），而非空串，符合头文件契约
    if (config->host != NULL) {
        strncpy(s_broker_host, config->host, sizeof(s_broker_host) - 1);
        s_broker_host[sizeof(s_broker_host) - 1] = '\0';
    } else {
        strcpy(s_broker_host, "0.0.0.0");
    }
    s_config.host = s_broker_host;

    // 项7: 标记运行中（临界区内，与入口检查配对）
    // P1-4 修复：检查与设置在同一临界区完成，消除 TOCTOU 竞态
    taskENTER_CRITICAL(&s_broker_lock);
    if (s_broker_running) {
        taskEXIT_CRITICAL(&s_broker_lock);
        ESP_LOGW(TAG, "Broker 已在运行（并发 start 被拒绝）");
        return ESP_OK;
    }
    s_broker_running = true;
    taskEXIT_CRITICAL(&s_broker_lock);

    BaseType_t ret = xTaskCreatePinnedToCore(
        broker_task,
        "mqtt_broker",
        BROKER_TASK_STACK_SIZE,
        &s_config,
        BROKER_TASK_PRIORITY,
        &s_broker_task,
        BROKER_TASK_CORE
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 broker task 失败");
        taskENTER_CRITICAL(&s_broker_lock);
        s_broker_running = false;
        taskEXIT_CRITICAL(&s_broker_lock);
        return ESP_FAIL;
    }

    // 项1: 等待 broker 绑定结果（最多 1.5 秒）。
    // mosq_broker_run 绑定成功会阻塞运行，绑定失败会快速返回（<100ms）：
    //   take 成功（task give 了）→ run 快速返回 → 绑定失败
    //   take 超时 → run 仍在阻塞 → 绑定成功
    // 1.5 秒足够检测绑定失败，不会显著拖慢正常启动
    BaseType_t took = xSemaphoreTake(s_broker_start_sem, pdMS_TO_TICKS(1500));
    if (took == pdTRUE) {
        ESP_LOGE(TAG, "Broker 绑定失败（端口 %d 可能被占用）", config->port);
        // task 已 give start_sem，接下来会 give stop_sem 后退出，等待其退出
        BaseType_t stop_took = xSemaphoreTake(s_broker_stop_sem, pdMS_TO_TICKS(2000));
        if (stop_took == pdTRUE) {
            // task 已正常退出，安全复位 running
            taskENTER_CRITICAL(&s_broker_lock);
            s_broker_running = false;
            taskEXIT_CRITICAL(&s_broker_lock);
        } else {
            // P1-6 修复：超时后 task 未退出，保持 running=true 防止重复创建 task
            // 需重启设备恢复（与 stop 超时逻辑一致）
            ESP_LOGE(TAG, "Broker task 未在 2s 内退出，保持 running=true，需重启恢复");
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Broker task 已创建并绑定成功, core=%d, stack=%d", BROKER_TASK_CORE, BROKER_TASK_STACK_SIZE);
    return ESP_OK;
}

void app_mqtt_broker_stop(void)
{
    taskENTER_CRITICAL(&s_broker_lock);
    if (!s_broker_running) {
        taskEXIT_CRITICAL(&s_broker_lock);
        return;
    }
    taskEXIT_CRITICAL(&s_broker_lock);

    ESP_LOGI(TAG, "停止 broker...");

    // 项4: stop_sem 在 start 中已创建（s_broker_running=true 说明 start 成功过）
    //      清除可能残留的 token，确保 take 能正确等待本次 task 退出
    if (s_broker_stop_sem != NULL) {
        xSemaphoreTake(s_broker_stop_sem, 0);
    } else {
        // 理论上不会走到这里（start 成功则 stop_sem 必已创建），兜底处理
        ESP_LOGW(TAG, "stop 信号量为空，无法等待 task 退出");
        mosq_broker_stop();
        return;
    }

    mosq_broker_stop();

    // 项4: 等待 broker_task 退出（最多 2 秒）
    BaseType_t took = xSemaphoreTake(s_broker_stop_sem, pdMS_TO_TICKS(2000));
    if (took != pdTRUE) {
        // P2-9 修正：超时不复位 s_broker_running，保持 true。
        // 复位会导致后续 start 创建重复 task，旧 task 退出时污染状态。
        // 超时说明 task 未正常退出，保持 true 强制重启恢复。
        ESP_LOGW(TAG, "等待 broker task 退出超时（2s），s_broker_running 保持 true，建议重启恢复");
    } else {
        ESP_LOGI(TAG, "Broker task 已退出");
    }
}

bool app_mqtt_broker_is_running(void)
{
    return s_broker_running;
}

/**
 * @file app_button.cpp
 * @brief 按键功能实现
 *
 * 基于 GPIO 中断 + 软件计时实现多击和长按检测。
 *
 * 检测逻辑：
 * - 按下时刻记录时间，释放时刻计算按压时长
 * - 如果按压时长 >= 5s → 长按事件（仅清除 WiFi）
 * - 如果按压时长 < 5s → 短击计数 +1
 * - 两次短击间隔 > 600ms → 结束击数判定
 * - 2击 → 启动 LoRa 配对
 * - 3击 → 删除所有 LoRa 子设备
 * - 5击 → 仅重置 Matter
 *
 * 硬件约束（严格遵守）：
 * - 5击操作仅用于重置Matter，不得触发WiFi重置
 * - 长按5秒操作用于仅清除WiFi，不重置Matter
 */
#include "app_button.h"
#include "app_led.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_matter.h"
#include "app_protocol_bridge.h"
#include <platform/PlatformManager.h>  // P1-3 修复：action_reset_matter 调用 Matter API 需持 StackLock
#include <string.h>

static const char *TAG = "app_button";

// 按键参数
#define LONG_PRESS_MS           5000    // 长按阈值 5 秒
#define MULTI_CLICK_INTERVAL_MS 600     // 两次击键最大间隔 600ms
#define DEBOUNCE_MS             50      // 消抖 50ms
// 项18: 从 4096 提升到 6144，因为 action_reset_matter 调用链较深
//       （esp_matter::factory_reset → chip::Server::ScheduleFactoryReset 等深栈）
#define BUTTON_TASK_STACK       6144
// P1-7 修复：5→4，低于 Matter task 优先级 5，避免 factory_reset 时互相影响
#define BUTTON_TASK_PRIORITY    4

// GPIO 事件
typedef enum {
    BTN_EVENT_PRESS,     // 按下
    BTN_EVENT_RELEASE,   // 释放
} btn_event_t;

static int s_gpio_num = -1;
static QueueHandle_t s_event_queue = NULL;

// 多击检测状态
static int s_click_count = 0;
static int64_t s_press_time = 0;      // 按下时刻（微秒）
static int64_t s_last_release = 0;    // 上次释放时刻
static bool s_long_press_fired = false;
// 项16: 重复初始化守卫，防止 app_button_init 被多次调用导致重复创建队列/task
static bool s_initialized = false;

// ==================== 动作实现 ====================

/**
 * @brief 启动 LoRa 配对模式
 */
static void action_lora_pairing(void)
{
    ESP_LOGI(TAG, ">>> 动作: 启动 LoRa 配对模式 <<<");
    // 绿灯快闪，配对成功或8秒超时后熄灭
    app_led_set(LED_GREEN, LED_MODE_FAST_BLINK, 8000);
    app_protocol_bridge_start_pairing();
}

/**
 * @brief 删除所有 LoRa 子设备（3击触发）
 *
 * 遍历所有网关下的所有子设备，逐个发送解绑命令（ctype=003, bind=0）。
 * 设备间延迟 200ms 避免 MQTT 报文拥堵。
 */
static void action_delete_all_devices(void)
{
    ESP_LOGI(TAG, ">>> 动作: 删除所有 LoRa 子设备 <<<");
    // 红灯快闪3秒
    app_led_set(LED_RED, LED_MODE_FAST_BLINK, 3000);
    app_protocol_bridge_delete_all_devices();
}

/**
 * @brief 仅重置 Matter（5击触发）
 *
 * 删除 Matter fabric 数据，但不影响 WiFi 连接。
 * 重置后设备需要重新配网。
 *
 * 注意：esp_matter::factory_reset() 是异步的，内部调用
 * chip::Server::GetInstance().ScheduleFactoryReset()，
 * 完成后会自动重启（见 esp_matter_core.cpp 注释
 * "This also restarts after completion"）。
 * 因此不能立即 esp_restart()，否则会中断 factory_reset 流程。
 */
static void action_reset_matter(void)
{
    ESP_LOGI(TAG, ">>> 动作: 仅重置 Matter（5击）<<<");
    // 蓝灯快闪（持续到重启）
    app_led_set(LED_BLUE, LED_MODE_FAST_BLINK, 0);

    // P1-3 修复：factory_reset 是 Matter API，调用时必须持有 StackLock，
    // 否则与 Matter 事件循环并发可能触发 chipDie abort。
    // 但 factory_reset() 是异步的（内部 ScheduleFactoryReset 调度事件到 Matter 事件循环），
    // StackLock 作用域必须仅保护 factory_reset() 调用本身，调用返回后立即释放锁，
    // 让 Matter 事件循环能获取 StackLock 执行 ScheduleFactoryReset 事件完成实际重置。
    // 原 P1-3 修复将 StackLock 作用于整个函数（含 for 循环 vTaskDelay），
    // vTaskDelay 不会释放 mutex，导致 Matter 事件循环无法获取锁执行 factory_reset 事件 → 死锁。
    {
        chip::DeviceLayer::StackLock lock;
        // 调用 esp_matter 的 factory reset
        // 这会清除 Matter commissioning 数据（fabric、operational credentials 等）
        // 但不会清除 WiFi 凭证
        // 异步执行，完成后会自动重启
        esp_matter::factory_reset();
    }
    // 锁已释放，Matter 事件循环可执行 ScheduleFactoryReset

    ESP_LOGI(TAG, "Matter factory_reset 已调度，等待完成自动重启...");
    // 项25: 兜底等待改为循环短延时（30 次 100ms = 3s），每次打印心跳日志，
    //       避免长时间无日志让用户以为死机。正常情况下 factory_reset 内部会
    //       自动重启，不会执行到循环结束。
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (i % 10 == 9) {
            ESP_LOGI(TAG, "等待 factory_reset 完成... (%d00ms)", i + 1);
        }
    }
    ESP_LOGW(TAG, "factory_reset 未自动重启，手动重启");
    esp_restart();
}

/**
 * @brief 仅清除 WiFi 凭证（长按 5s 触发）
 *
 * 清除 NVS 中的 WiFi SSID/密码，但不重置 Matter fabric 数据。
 * 重启后设备进入 BLE 配网模式等待新 WiFi 凭证。
 */
static void action_reset_wifi(void)
{
    ESP_LOGI(TAG, ">>> 动作: 仅清除 WiFi 凭证（长按 5s）<<<");
    // 蓝灯慢闪（持续到重启，重启后 WiFi 连接成功时蓝灯常亮2s）
    app_led_set(LED_BLUE, LED_MODE_SLOW_BLINK, 0);

    // 打开 NVS 命名空间，清除 WiFi 凭证
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("nvs.net80211", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        // 擦除整个 nvs.net80211 命名空间
        // WiFi 凭证的 key 名是 ESP-IDF 私有的，无法按名删除，必须用 nvs_erase_all
        // P3-7 修复：检查 nvs_erase_all 返回值
        esp_err_t erase_err = nvs_erase_all(nvs_handle);
        if (erase_err != ESP_OK) {
            ESP_LOGE(TAG, "nvs_erase_all 失败: %s，WiFi 凭证未清除，不重启", esp_err_to_name(erase_err));
            nvs_close(nvs_handle);
            return;
        }
        // P3-7 修复：检查 nvs_commit 返回值
        // 若 commit 失败，WiFi 凭证实际未持久化清除，重启后仍会自动连接旧 WiFi
        // 此时不重启，避免误导用户以为已清除
        esp_err_t commit_err = nvs_commit(nvs_handle);
        if (commit_err != ESP_OK) {
            ESP_LOGE(TAG, "nvs_commit 失败: %s，WiFi 凭证未清除，不重启", esp_err_to_name(commit_err));
            nvs_close(nvs_handle);
            return;
        }
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "NVS WiFi 凭证已清除");
    } else {
        // nvs_open 失败说明 WiFi 凭证无法清除，重启也不会进入配网模式
        // 不重启，避免用户误以为已清除
        ESP_LOGE(TAG, "无法打开 nvs.net80211 命名空间: %s，WiFi 凭证未清除，不重启", esp_err_to_name(err));
        return;
    }

    // 同时断开当前 WiFi 连接
    // Fix-Bug6: 检查返回值，失败时记录警告但不阻止重启
    // （NVS 凭据已清除，重启后必然进入配网模式，WiFi 断开失败不影响结果）
    esp_err_t disc_err = esp_wifi_disconnect();
    if (disc_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_disconnect 失败: %s（不影响重启清除凭据）", esp_err_to_name(disc_err));
    }
    esp_err_t stop_err = esp_wifi_stop();
    if (stop_err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_stop 失败: %s（不影响重启清除凭据）", esp_err_to_name(stop_err));
    }
    // 项19: esp_wifi_stop 是异步操作，延时 200ms 让其完成后再重启，
    //       避免重启时 WiFi 状态机未清理导致下次启动异常
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(TAG, "WiFi 凭证已清除，重启进入配网模式...");
    esp_restart();
}

// ==================== 击键判定 ====================

/**
 * @brief 结束一次击键序列，根据击数执行动作
 */
static void finalize_click_sequence(void)
{
    if (s_click_count == 0) return;

    ESP_LOGI(TAG, "击键序列结束: %d 击", s_click_count);

    switch (s_click_count) {
    case 1:
        // 单击无动作（设计如此），静默忽略
        break;
    case 2:
        action_lora_pairing();
        break;
    case 3:
        action_delete_all_devices();
        break;
    case 5:
        action_reset_matter();
        break;
    default:
        ESP_LOGW(TAG, "未定义的击数: %d", s_click_count);
        break;
    }

    s_click_count = 0;
}

// ==================== GPIO 中断和 Task ====================

static void IRAM_ATTR button_isr_handler(void *arg)
{
    int gpio_num = (int)(intptr_t)arg;
    int level = gpio_get_level((gpio_num_t)gpio_num);
    btn_event_t event = (level == 0) ? BTN_EVENT_PRESS : BTN_EVENT_RELEASE;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(s_event_queue, &event, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void button_task(void *arg)
{
    btn_event_t event;

    while (true) {
        // 项24: 队列超时设为 MULTI_CLICK_INTERVAL_MS(600ms)，用于多击间隔判定。
        //       副作用：长按检测（else 分支）的轮询粒度也是 600ms，即长按触发最多
        //       延迟 600ms。对 5s 长按阈值而言相对误差约 12%，可接受。
        //       缩短超时会破坏 600ms 多击间隔语义（5击需 4×600ms 窗口），故保持现状。
        if (xQueueReceive(s_event_queue, &event, pdMS_TO_TICKS(MULTI_CLICK_INTERVAL_MS)) == pdTRUE) {
            int64_t now = esp_timer_get_time() / 1000; // 毫秒

            if (event == BTN_EVENT_PRESS) {
                // 按下：记录时间
                s_press_time = now;
                s_long_press_fired = false;
                ESP_LOGD(TAG, "按键按下");
            }
            else if (event == BTN_EVENT_RELEASE) {
                // 释放：计算按压时长
                if (s_press_time == 0) continue;

                int64_t press_duration = now - s_press_time;
                s_press_time = 0;

                // 消抖
                if (press_duration < DEBOUNCE_MS) continue;

                ESP_LOGD(TAG, "按键释放, 按压时长: %lld ms", press_duration);

                // 长按判定（>= 5s）
                if (press_duration >= LONG_PRESS_MS && !s_long_press_fired) {
                    s_long_press_fired = true;
                    s_click_count = 0;  // 取消击数计数
                    action_reset_wifi();
                    continue;
                }

                // 长按已触发（持续按压检测已执行 action_reset_wifi），跳过短击计数
                // 防止 action_reset_wifi 未重启时 s_click_count 被污染
                if (s_long_press_fired) {
                    s_long_press_fired = false;
                    continue;
                }

                // 短击：计数 +1
                s_click_count++;
                s_last_release = now;

                ESP_LOGI(TAG, "短击 #%d", s_click_count);

                // 5击提前判定（第 5 击直接触发，不等超时）
                if (s_click_count == 5) {
                    finalize_click_sequence();
                    // 项23: finalize 后 continue，避免后续逻辑执行
                    //       （s_click_count 已被 finalize 置 0，不 continue 也无副作用，
                    //        但显式 continue 语义更清晰，防止未来误加逻辑）
                    continue;
                }
            }
        }
        else {
            // 队列超时：检查是否需要结束击键序列
            if (s_click_count > 0 && s_click_count < 5) {
                // 距上次释放已超过间隔阈值，结束序列
                // P2-13 修复：> 改为 >=，避免 tick 粒度边界导致多等 600ms
                int64_t now = esp_timer_get_time() / 1000;
                if (now - s_last_release >= MULTI_CLICK_INTERVAL_MS) {
                    finalize_click_sequence();
                }
            }

            // 长按持续检测（按键一直按着不放的情况）
            if (s_press_time > 0 && !s_long_press_fired) {
                int64_t now = esp_timer_get_time() / 1000;
                if (now - s_press_time >= LONG_PRESS_MS) {
                    s_long_press_fired = true;
                    s_click_count = 0;
                    ESP_LOGI(TAG, "长按 >= 5s 检测到（持续按压）");
                    action_reset_wifi();
                    // P2-11 修复：清除按压时间戳，避免 action_reset_wifi 失败返回后
                    // s_press_time 残留导致后续状态机异常
                    s_press_time = 0;
                }
            }
        }
    }
}

// ==================== 公共 API ====================

esp_err_t app_button_init(int gpio_num)
{
    // 项16: 重复调用保护，已初始化则直接返回
    if (s_initialized) {
        ESP_LOGW(TAG, "按键模块已初始化，跳过重复初始化");
        return ESP_OK;
    }

    s_gpio_num = gpio_num;

    // 创建事件队列
    s_event_queue = xQueueCreate(10, sizeof(btn_event_t));
    if (s_event_queue == NULL) {
        ESP_LOGE(TAG, "创建事件队列失败");
        return ESP_FAIL;
    }

    // 配置 GPIO
    // P2-7 修复：intr_type 初始设为 DISABLE，避免 gpio_config 后到 isr_handler_add 前
    // 的窗口期内电平抖动触发未注册的中断。handler 注册后再用 gpio_set_intr_type 启用。
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << gpio_num),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config 失败: %s", esp_err_to_name(err));
        // P1-5 修复：清理已创建的队列
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return err;
    }

    // 安装 GPIO ISR 服务
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "安装 GPIO ISR 服务失败: %s", esp_err_to_name(err));
        // P1-5 修复：清理队列
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return err;
    }

    // 注册中断处理
    err = gpio_isr_handler_add((gpio_num_t)gpio_num, button_isr_handler, (void *)(intptr_t)gpio_num);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add 失败: %s", esp_err_to_name(err));
        // P1-5 修复：清理队列
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return err;
    }

    // P2-7 修复：handler 注册成功后再启用 ANYEDGE 中断
    // 检查返回值：失败时记录警告但继续运行（中断不启用仅影响按键响应，不影响系统稳定性）
    esp_err_t intr_err = gpio_set_intr_type((gpio_num_t)gpio_num, GPIO_INTR_ANYEDGE);
    if (intr_err != ESP_OK) {
        ESP_LOGW(TAG, "gpio_set_intr_type 失败: %s（按键可能无响应）", esp_err_to_name(intr_err));
    }

    // 创建按键检测 task
    BaseType_t ret = xTaskCreate(
        button_task,
        "btn_detect",
        BUTTON_TASK_STACK,
        NULL,
        BUTTON_TASK_PRIORITY,
        NULL
    );
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建按键 task 失败");
        // P1-5 修复：清理队列 + 注销 ISR handler + 关闭中断
        gpio_isr_handler_remove((gpio_num_t)gpio_num);
        gpio_intr_disable((gpio_num_t)gpio_num);
        vQueueDelete(s_event_queue);
        s_event_queue = NULL;
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "按键模块已初始化: GPIO%d (低电平有效)", gpio_num);
    ESP_LOGI(TAG, "  2击 → LoRa 配对");
    ESP_LOGI(TAG, "  3击 → 删除所有子设备");
    ESP_LOGI(TAG, "  5击 → 仅重置 Matter");
    ESP_LOGI(TAG, "  长按5s → 仅清除 WiFi");

    return ESP_OK;
}

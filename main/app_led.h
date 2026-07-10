/**
 * @file app_led.h
 * @brief LED 指示灯模块（WS2812 幻彩 LED）
 *
 * 单颗 WS2812 5050 幻彩 LED，通过 RMT 驱动，按优先级显示不同颜色：
 * - 蓝灯：WiFi/Matter 状态
 * - 绿灯：LoRa 通信状态
 * - 红灯：删除操作状态
 *
 * LED 模式：
 * - 常亮：状态持续
 * - 快闪（5Hz）：操作进行中
 * - 慢闪（1Hz）：等待中
 * - 单闪（50ms）：消息收发
 *
 * 优先级（单颗 LED 同一时间只能显示一种颜色）：
 * - 单闪 > 快闪 > 慢闪 > 常亮 > 关闭
 * - 同模式时：红 > 绿 > 蓝
 *
 * 线程安全：所有 API 可跨 task 调用，内部用 portMUX_TYPE 保护。
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    LED_BLUE = 0,    /**< 蓝灯：WiFi/Matter 状态 */
    LED_GREEN = 1,   /**< 绿灯：LoRa 通信状态 */
    LED_RED = 2,     /**< 红灯：删除操作状态 */
    LED_COLOR_COUNT
} led_color_t;

typedef enum {
    LED_MODE_OFF = 0,       /**< 关闭 */
    LED_MODE_ON,            /**< 常亮 */
    LED_MODE_FAST_BLINK,    /**< 快闪（5Hz，100ms 周期） */
    LED_MODE_SLOW_BLINK,    /**< 慢闪（1Hz，1000ms 周期） */
    LED_MODE_FLASH,         /**< 单闪（50ms 后自动熄灭） */
} led_mode_t;

/**
 * @brief 初始化 LED 指示灯模块（WS2812）
 *
 * 配置 RMT TX 通道驱动 WS2812，创建 LED 控制任务（50ms 周期刷新）。
 *
 * @param gpio_num WS2812 数据线连接的 GPIO 编号（-1 禁用）
 * @return ESP_OK 成功
 */
esp_err_t app_led_init(int gpio_num);

/**
 * @brief 设置 LED 模式
 *
 * @param color LED 颜色（逻辑颜色，单颗 LED 按优先级显示）
 * @param mode LED 模式
 * @param duration_ms 持续时间（ms），0=持续直到下次设置
 */
void app_led_set(led_color_t color, led_mode_t mode, uint32_t duration_ms);

/**
 * @brief 关闭 LED
 */
void app_led_off(led_color_t color);

/**
 * @brief 单闪 LED（50ms 后自动熄灭）
 *
 * 如果 LED 当前处于快闪/慢闪模式，则跳过（不干扰配对/删除等操作指示）。
 */
void app_led_flash(led_color_t color);

#ifdef __cplusplus
}
#endif

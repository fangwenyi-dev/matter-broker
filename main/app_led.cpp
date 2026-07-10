/**
 * @file app_led.cpp
 * @brief LED 指示灯模块实现（WS2812 幻彩 LED）
 *
 * 使用 ESP-IDF RMT 驱动单颗 WS2812 5050 幻彩 LED。
 * RMT 分辨率 10MHz（100ns/tick），用 bytes encoder 编码 WS2812 时序。
 *
 * WS2812 时序（10MHz 分辨率）：
 * - T0H: 4 ticks (400ns), T0L: 8 ticks (800ns)
 * - T1H: 8 ticks (800ns), T1L: 4 ticks (400ns)
 * - Reset: >50us 低电平（rmt_tx_wait_all_done 提供自然间隔）
 *
 * 数据格式：GRB，每色 8bit，MSB first
 *
 * 单颗 LED 优先级显示策略：
 * - 每个"逻辑颜色"（蓝/绿/红）独立维护状态
 * - 每个 tick 根据优先级选出当前应显示的颜色
 * - 优先级：FLASH(4) > FAST_BLINK(3) > SLOW_BLINK(2) > ON(1) > OFF(0)
 * - 同模式时颜色优先级：RED(3) > GREEN(2) > BLUE(1)
 * - 闪烁/单闪的"灭"相位期间，该颜色不参与显示竞争
 *
 * 线程安全：app_led_set / app_led_off / app_led_flash 使用 portMUX_TYPE 保护。
 */
#include "app_led.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "app_led";

#define LED_TASK_STACK    2048
#define LED_TASK_PRIORITY 2
#define LED_TICK_MS       50  // 刷新周期

// 快闪：100ms on, 100ms off（5Hz）
#define FAST_BLINK_PERIOD_MS  200
// 慢闪：500ms on, 500ms off（1Hz）
#define SLOW_BLINK_PERIOD_MS  1000
// 单闪持续时间
#define FLASH_DURATION_MS     50

// WS2812 RMT 配置
#define RMT_RESOLUTION_HZ     (10 * 1000 * 1000)  // 10MHz → 100ns/tick
#define WS2812_LED_NUM        1
#define WS2812_RESET_DELAY_MS 1  // rmt_tx_wait_all_done 后额外等待，确保 >50us reset

// LED 亮度（0-255），5050 透明 LED 适度亮度
#define LED_BRIGHTNESS        80

// WS2812 颜色表（GRB 格式：[Green, Red, Blue]）
static const uint8_t s_color_table[LED_COLOR_COUNT][3] = {
    [LED_BLUE]  = {0,             0,             LED_BRIGHTNESS},  // 蓝
    [LED_GREEN] = {LED_BRIGHTNESS, 0,             0},               // 绿
    [LED_RED]   = {0,             LED_BRIGHTNESS, 0},               // 红
};

typedef struct {
    led_mode_t mode;
    uint32_t duration_ms;      // 0 = 持续直到下次设置
    int64_t start_time_us;     // 模式开始时刻
    bool valid;                // 模块是否已初始化
} led_state_t;

static led_state_t s_leds[LED_COLOR_COUNT] = {};
static portMUX_TYPE s_led_lock = portMUX_INITIALIZER_UNLOCKED;

// RMT 句柄
static rmt_channel_handle_t s_rmt_chan = NULL;
static rmt_encoder_handle_t s_encoder = NULL;
static bool s_initialized = false;

// ==================== WS2812 底层驱动 ====================

/**
 * @brief 发送一帧像素数据到 WS2812
 *
 * @param grb 3 字节数据（Green, Red, Blue）
 */
static void ws2812_send(const uint8_t grb[3])
{
    if (s_rmt_chan == NULL || s_encoder == NULL) return;

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
        .flags = {
            .queue_nonblocking = 0,  // 阻塞模式，确保时序正确
        },
    };
    rmt_transmit(s_rmt_chan, s_encoder, grb, 3, &tx_config);
    // 等待发送完成（WS2812 需要精确时序，不能并发）
    rmt_tx_wait_all_done(s_rmt_chan, 10);
    // 额外等待确保 reset 时序（>50us 低电平）
    // rmt_tx_wait_all_done 返回后 RMT 已停止输出，总线为低电平
    // 50ms 刷新周期远大于 50us reset 需求，但首次发送后加保险
}

/**
 * @brief 设置 LED 颜色（物理）
 */
static void set_pixel(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t grb[3] = {g, r, b};  // WS2812 格式：GRB
    ws2812_send(grb);
}

static void set_pixel_off(void)
{
    uint8_t grb[3] = {0, 0, 0};
    ws2812_send(grb);
}

static void set_pixel_color(led_color_t color)
{
    if (color < 0 || color >= LED_COLOR_COUNT) {
        set_pixel_off();
        return;
    }
    ws2812_send(s_color_table[color]);
}

// ==================== 优先级计算 ====================

/**
 * @brief 计算指定颜色在当前时刻的显示优先级
 *
 * @return >0 表示该颜色当前应点亮（返回值为优先级分数），0 表示不点亮
 */
static int compute_display_priority(led_state_t *led, int color_index, int64_t now_us)
{
    if (!led->valid || led->mode == LED_MODE_OFF) return 0;

    // 检查持续时间超时
    int64_t elapsed_ms = (now_us - led->start_time_us) / 1000;
    if (led->duration_ms > 0 && elapsed_ms >= (int64_t)led->duration_ms) {
        // 超时，模式已失效（led_task 会清理）
        return 0;
    }

    // 颜色优先级：RED(3) > GREEN(2) > BLUE(1)
    int color_pri = LED_COLOR_COUNT - color_index;

    switch (led->mode) {
    case LED_MODE_OFF:
        return 0;
    case LED_MODE_ON:
        return 1 * 10 + color_pri;
    case LED_MODE_FLASH:
    case LED_MODE_FAST_BLINK:
    case LED_MODE_SLOW_BLINK: {
        // 计算闪烁相位
        int64_t period_ms;
        int mode_pri;
        if (led->mode == LED_MODE_FLASH) {
            period_ms = 0;  // FLASH 不闪烁，持续亮直到超时
            mode_pri = 4;
        } else if (led->mode == LED_MODE_FAST_BLINK) {
            period_ms = FAST_BLINK_PERIOD_MS;
            mode_pri = 3;
        } else {
            period_ms = SLOW_BLINK_PERIOD_MS;
            mode_pri = 2;
        }

        if (led->mode == LED_MODE_FLASH) {
            // FLASH 模式：持续亮，由 duration_ms 超时关闭
            return mode_pri * 10 + color_pri;
        }

        // 闪烁模式：判断当前是否在 on 相位
        int64_t phase_ms = elapsed_ms % period_ms;
        if (phase_ms < period_ms / 2) {
            // on 相位
            return mode_pri * 10 + color_pri;
        } else {
            // off 相位，不点亮
            return 0;
        }
    }
    }
    return 0;
}

// ==================== LED 控制任务 ====================

static void led_task(void *arg)
{
    while (true) {
        int64_t now_us = esp_timer_get_time();

        led_color_t display_color = LED_COLOR_COUNT;
        int best_priority = 0;

        taskENTER_CRITICAL(&s_led_lock);
        // 检查超时并计算优先级
        for (int i = 0; i < LED_COLOR_COUNT; i++) {
            led_state_t *led = &s_leds[i];

            // 检查持续时间超时，清理过期模式
            if (led->valid && led->duration_ms > 0) {
                int64_t elapsed_ms = (now_us - led->start_time_us) / 1000;
                if (elapsed_ms >= (int64_t)led->duration_ms) {
                    led->mode = LED_MODE_OFF;
                    led->duration_ms = 0;
                }
            }

            int pri = compute_display_priority(led, i, now_us);
            if (pri > best_priority) {
                best_priority = pri;
                display_color = (led_color_t)i;
            }
        }
        taskEXIT_CRITICAL(&s_led_lock);

        // 驱动 LED（在临界区外，RMT 发送不需要锁保护）
        if (display_color == LED_COLOR_COUNT) {
            set_pixel_off();
        } else {
            set_pixel_color(display_color);
        }

        vTaskDelay(pdMS_TO_TICKS(LED_TICK_MS));
    }
}

// ==================== 公共 API ====================

esp_err_t app_led_init(int gpio_num)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "LED 模块已初始化，跳过重复初始化");
        return ESP_OK;
    }

    if (gpio_num < 0) {
        ESP_LOGI(TAG, "LED 已禁用 (gpio=-1)");
        s_leds[LED_BLUE].valid = false;
        s_leds[LED_GREEN].valid = false;
        s_leds[LED_RED].valid = false;
        s_initialized = true;
        return ESP_OK;
    }

    // 配置 RMT TX 通道
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = (gpio_num_t)gpio_num,
        .clk_src = RMT_CLK_SRC_DEFAULT,  // 通常为 APB 80MHz
        .resolution_hz = RMT_RESOLUTION_HZ,  // 10MHz
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .io_od_mode = 0,
            .allow_pd = 0,
            .init_level = 0,
        },
    };
    esp_err_t err = rmt_new_tx_channel(&tx_config, &s_rmt_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT TX 通道创建失败: %s", esp_err_to_name(err));
        return err;
    }

    // 创建 WS2812 bytes encoder
    // WS2812 时序（10MHz = 100ns/tick）：
    //   bit0: T0H=400ns(4 ticks) high, T0L=800ns(8 ticks) low
    //   bit1: T1H=800ns(8 ticks) high, T1L=400ns(4 ticks) low
    rmt_bytes_encoder_config_t encoder_config = {
        .bit0 = {
            .duration0 = 4,
            .level0 = 1,
            .duration1 = 8,
            .level1 = 0,
        },
        .bit1 = {
            .duration0 = 8,
            .level0 = 1,
            .duration1 = 4,
            .level1 = 0,
        },
        .flags = {
            .msb_first = 1,  // WS2812 MSB first
        },
    };
    err = rmt_new_bytes_encoder(&encoder_config, &s_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT encoder 创建失败: %s", esp_err_to_name(err));
        rmt_del_channel(s_rmt_chan);
        s_rmt_chan = NULL;
        return err;
    }

    // 启用 RMT 通道
    err = rmt_enable(s_rmt_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RMT 通道启用失败: %s", esp_err_to_name(err));
        rmt_del_encoder(s_encoder);
        rmt_del_channel(s_rmt_chan);
        s_encoder = NULL;
        s_rmt_chan = NULL;
        return err;
    }

    // 初始化 LED 状态
    taskENTER_CRITICAL(&s_led_lock);
    for (int i = 0; i < LED_COLOR_COUNT; i++) {
        s_leds[i].mode = LED_MODE_OFF;
        s_leds[i].duration_ms = 0;
        s_leds[i].start_time_us = esp_timer_get_time();
        s_leds[i].valid = true;
    }
    taskEXIT_CRITICAL(&s_led_lock);

    // 初始熄灭
    set_pixel_off();

    // 创建 LED 控制任务
    BaseType_t ret = xTaskCreate(led_task, "led_ctrl", LED_TASK_STACK, NULL,
                                 LED_TASK_PRIORITY, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "创建 LED task 失败");
        rmt_del_encoder(s_encoder);
        rmt_del_channel(s_rmt_chan);
        s_encoder = NULL;
        s_rmt_chan = NULL;
        return ESP_FAIL;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "LED 指示灯模块已初始化 (WS2812, GPIO%d, 亮度=%d)", gpio_num, LED_BRIGHTNESS);
    return ESP_OK;
}

void app_led_set(led_color_t color, led_mode_t mode, uint32_t duration_ms)
{
    if (color < 0 || color >= LED_COLOR_COUNT) return;

    taskENTER_CRITICAL(&s_led_lock);
    led_state_t *led = &s_leds[color];
    if (!led->valid) {
        taskEXIT_CRITICAL(&s_led_lock);
        return;
    }

    led->mode = mode;
    // 单闪模式默认 50ms
    if (mode == LED_MODE_FLASH && duration_ms == 0) {
        led->duration_ms = FLASH_DURATION_MS;
    } else {
        led->duration_ms = duration_ms;
    }
    led->start_time_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_led_lock);
}

void app_led_off(led_color_t color)
{
    app_led_set(color, LED_MODE_OFF, 0);
}

void app_led_flash(led_color_t color)
{
    if (color < 0 || color >= LED_COLOR_COUNT) return;

    taskENTER_CRITICAL(&s_led_lock);
    led_state_t *led = &s_leds[color];
    if (!led->valid) {
        taskEXIT_CRITICAL(&s_led_lock);
        return;
    }
    // 快闪/慢闪模式下不干扰（配对/删除等操作指示优先）
    if (led->mode == LED_MODE_FAST_BLINK || led->mode == LED_MODE_SLOW_BLINK) {
        taskEXIT_CRITICAL(&s_led_lock);
        return;
    }
    led->mode = LED_MODE_FLASH;
    led->duration_ms = FLASH_DURATION_MS;
    led->start_time_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&s_led_lock);
}

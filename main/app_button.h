/**
 * @file app_button.h
 * @brief 按键功能模块
 *
 * 支持：
 * - 2击：启动 LoRa 配对模式
 * - 3击：删除所有 LoRa 子设备（发送解绑命令）
 * - 5击：仅重置 Matter（不重置 WiFi）
 * - 长按 5 秒：仅清除 WiFi 凭证（不重置 Matter）
 *
 * 硬件约束：
 * - 5击操作仅用于重置Matter，不得触发WiFi重置
 * - 长按5秒操作用于仅清除WiFi，不重置Matter
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化按键模块
 *
 * 配置 GPIO 中断，启动按键检测 task。
 *
 * @param gpio_num 按键 GPIO 编号（默认 GPIO0，低电平有效）
 * @return ESP_OK 成功
 */
esp_err_t app_button_init(int gpio_num);

#ifdef __cplusplus
}
#endif

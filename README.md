# Matter MQTT Broker

基于 ESP32-S3 的 Matter 网关固件，将 LoRa 子设备桥接到 Matter 网络（HomeKit/Google Home/Alexa）。

## 功能

- **Matter 桥接**：为每个 LoRa 子设备动态创建 Matter WindowCovering 端点
- **MQTT 协议**：与 HA 网关通过 MQTT 通信，支持配对/解绑/状态上报/控制下发
- **HomeKit 兼容**：双滑块 UI（Lift 升降 + Tilt 内倒），电池电量显示
- **OTA 升级**：支持 Matter OTA 固件更新
- **按键控制**：2 击配对、3 击解绑、5 击重置 Matter

## 硬件

- ESP32-S3
- LoRa 模块（串口通信）
- 按键（GPIO）

## 编译

需要 ESP-IDF v5.5.4：

```bash
# 克隆仓库
git clone https://github.com/fangwenyi-dev/matter-broker.git
cd matter-broker

# 设置 ESP-IDF 环境
. $IDF_PATH/export.sh

# 编译
idf.py build

# 烧录
idf.py -p PORT flash
```

## 配置

关键配置项在 `sdkconfig.defaults` 中，烧录后可通过 `menuconfig` 修改：

- MQTT 服务器地址
- LoRa 串口引脚
- 按键 GPIO
- Matter 设备信息（Vendor ID、Product ID 等）

## 架构

```
┌─────────────────────────────────────┐
│           HomeKit / Matter          │
│         (Lift + Tilt 双滑块)         │
└──────────────┬──────────────────────┘
               │ Matter Protocol
┌──────────────┴──────────────────────┐
│         ESP32-S3 (本固件)            │
│  ┌─────────┐  ┌──────────────────┐  │
│  │  Matter  │  │   MQTT Broker    │  │
│  │  Bridge  │←→│  (Mosquitto)     │  │
│  └─────────┘  └───────┬──────────┘  │
│                       │ MQTT         │
│  ┌────────────────────┴──────────┐  │
│  │     Protocol Bridge           │  │
│  │  (配对/解绑/控制/状态)         │  │
│  └───────────────────────────────┘  │
└──────────────────────────────────────┘
               │ LoRa (串口)
┌──────────────┴──────────────────────┐
│         HA 网关 (MQTT Client)        │
│  ┌─────────────────────────────┐    │
│  │  LoRa 子设备管理             │    │
│  │  (开窗器/窗帘/传感器)        │    │
│  └─────────────────────────────┘    │
└─────────────────────────────────────┘
```

## 文件结构

```
├── main/
│   ├── main.cpp              # 主程序入口
│   ├── app_matter_bridge.*   # Matter 桥接（端点创建/属性管理）
│   ├── app_mqtt_broker.*     # MQTT Broker（Mosquitto）
│   ├── app_protocol_bridge.* # 协议桥接（MQTT↔Matter 转换）
│   ├── app_button.*          # 按键处理（配对/解绑/重置）
│   └── idf_component.yml     # 组件依赖声明
├── components/
│   ├── espressif__esp_matter/ # ESP-Matter SDK (v1.5.0)
│   └── espressif__mosquitto/  # Mosquitto MQTT Broker
├── partitions.csv             # 分区表
├── sdkconfig.defaults         # 默认配置
└── CMakeLists.txt             # 顶层 CMake
```

## Matter WindowCovering 配置

| 属性 | 值 | 说明 |
|---|---|---|
| Type | 0x08 (TiltBlindLiftAndTilt) | 升降+倾斜型 |
| EndProductType | 0x0A (InteriorBlind) | 室内百叶，支持双滑块 |
| FeatureMap | 0x17 | Lift+PALift+Tilt+PATilt |
| ConfigStatus | 0x19 | Operational+LiftPA+TiltPA |
| Lift 滑块 | 0-100% | 控制窗户升降开合 |
| Tilt 滑块 | 0-100% | 控制窗户内倒 |

## License

MIT

# main 组件构建定义
idf_component_register(
    SRCS
        "main.cpp"
        "app_mqtt_broker.cpp"
        "app_matter_bridge.cpp"
        "app_protocol_bridge.cpp"
        "app_button.cpp"
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        nvs_flash
        mqtt
        esp_matter
        driver
        mdns           # mDNS：设置 matter-broker.local hostname
        app_update     # P0: esp_ota_mark_app_valid_cancel_rollback（OTA 回滚保护）
)

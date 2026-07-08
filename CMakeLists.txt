# ESP32-S3 Matter-MQTT Broker 一体化项目
# 顶层 CMakeLists.txt
cmake_minimum_required(VERSION 3.16)

# 引入 ESP-IDF 构建系统
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(matter_mqtt_broker)

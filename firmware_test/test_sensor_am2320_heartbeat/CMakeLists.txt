# CMakeLists.txt for ESP-PICO Sensor Test (AM2320 + Heartbeat)
cmake_minimum_required(VERSION 3.16)

# Set the target SoC for ESP32-C6
set(IDF_TARGET "esp32c6")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sensor_am2320_heartbeat_test) 
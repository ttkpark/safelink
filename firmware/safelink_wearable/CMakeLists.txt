# CMakeLists.txt for SafeLink Wearable
cmake_minimum_required(VERSION 3.16)

# Set the target SoC
set(IDF_TARGET "esp32c6")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(safelink_wearable) 
idf_component_register(SRCS "data_manager.c"
                    INCLUDE_DIRS "include"
                    REQUIRES spiffs freertos esp_timer ble_scanner ble_server safety_analyzer power_manager) 
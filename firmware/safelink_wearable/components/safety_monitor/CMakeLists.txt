idf_component_register(SRCS "safety_monitor.c"
                    INCLUDE_DIRS "include"
                    REQUIRES freertos esp_timer sensor_manager ble_service) 
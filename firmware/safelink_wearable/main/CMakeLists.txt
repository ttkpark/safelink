idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi bt app_update spiffs
                             sensor_manager ble_service power_manager 
                             safety_monitor data_manager) 
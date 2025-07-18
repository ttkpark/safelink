idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash esp_wifi bt app_update spiffs
                             ble_scanner ble_server audio_manager 
                             device_manager safety_analyzer alert_manager
                             power_manager data_manager) 
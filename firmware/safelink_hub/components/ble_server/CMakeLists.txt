idf_component_register(SRCS "ble_server.c"
                    INCLUDE_DIRS "include"
                    REQUIRES bt freertos esp_timer ble_scanner) 
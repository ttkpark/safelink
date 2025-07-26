idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver i2c_driver esp_adc_cal) 
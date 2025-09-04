idf_component_register(
    SRCS "i2c_driver.c" "max3020x.c" "safelink_band.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_adc driver
)

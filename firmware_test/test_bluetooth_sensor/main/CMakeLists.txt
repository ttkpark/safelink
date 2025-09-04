idf_component_register(
    SRCS "test_bluetooth_sensor.c" "bluetooth.c" "i2c.c" "analog.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_adc driver esp_timer
)

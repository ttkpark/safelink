idf_component_register(
    SRCS "test_bluetooth_sensor.c" "bluetooth.c" "i2c.c" "analog.c"
    INCLUDE_DIRS "." "C:/Espressif/frameworks/esp-idf-v5.5/components/bt/host/nimble/esp-hci/include"
    REQUIRES bt nvs_flash esp_adc driver
)

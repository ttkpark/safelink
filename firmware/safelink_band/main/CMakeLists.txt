idf_component_register(
    SRCS "safelink_band.c" "bluetooth.c" "i2c.c" "analog.c"
    INCLUDE_DIRS "." 
    "C:/Espressif/frameworks/esp-idf-v5.5/components/bt/host/nimble/nimble/porting/nimble/include"
    "C:/Espressif/frameworks/esp-idf-v5.5/components/bt/host/nimble/nimble/nimble/include"
    "C:/Espressif/frameworks/esp-idf-v5.5/components/bt/porting/npl/freertos/include"
    REQUIRES bt nvs_flash esp_adc driver
)

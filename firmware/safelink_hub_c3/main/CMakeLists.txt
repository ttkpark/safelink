idf_component_register(
    SRCS
        "i2c.c"
        "mic_i2s.c"
        "sensor.c"
        "safelink_wearable_hub.c"
        "data_manager.c"
        "bluetooth.c"
        "dfplayer_mini.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_timer bt driver esp_adc
)

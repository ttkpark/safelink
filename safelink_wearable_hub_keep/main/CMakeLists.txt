idf_component_register(
    SRCS "safelink_wearable_hub.c" "main.c" "dfplayer_mini.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_adc driver
)

idf_component_register(
    SRCS "main.c" "dfplayer_mini.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_adc driver
)

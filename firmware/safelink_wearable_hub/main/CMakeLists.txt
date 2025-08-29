idf_component_register(
    SRCS
        "i2c.c"
        "mic_i2s.c"
        "sensor.c"
        "safelink_wearable_hub.c"
        "data_manager.c"
        "bluetooth.c"
        "dfplayer_mini.c"
        # 개발용 시뮬레이터 (필요 시 활성화)
        # "test_simulator.c"
        # 예전 테스트 메인 (미사용)
        # "main.c"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash esp_timer bt driver esp_adc
)

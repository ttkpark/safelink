/*
 * ESP32-C3 SuperMini LED Blink Test
 * 
 * ESP32-C3 SuperMini의 내장 LED를 1초 주기로 깜빡이는 테스트 프로그램
 * 내장 LED는 보통 GPIO8에 연결되어 있습니다.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "ESP32C3_LED_BLINK";

/* ESP32-C3 SuperMini 내장 LED GPIO 핀 */
#define LED_GPIO_PIN    8

/* Blink 설정 */
#define BLINK_DELAY_MS  1000  // 1초 (1000ms)

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 SuperMini LED Blink Test Starting...");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Built LED GPIO: GPIO_%d", LED_GPIO_PIN);
    
    /* GPIO 설정 */
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "GPIO initialization completed successfully");
    } else {
        ESP_LOGE(TAG, "GPIO initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    /* LED 초기 상태를 OFF로 설정 */
    gpio_set_level(LED_GPIO_PIN, 0);
    ESP_LOGI(TAG, "LED initialized to OFF state");
    
    ESP_LOGI(TAG, "Starting LED blink with %dms interval...", BLINK_DELAY_MS);
    
    /* LED 깜빡임 루프 */
    bool led_state = false;
    while (1) {
        led_state = !led_state;  // LED 상태 토글
        
        gpio_set_level(LED_GPIO_PIN, led_state);
        
        ESP_LOGI(TAG, "LED: %s", led_state ? "ON" : "OFF");
        
        /* 1초 대기 */
        vTaskDelay(pdMS_TO_TICKS(BLINK_DELAY_MS));
    }
} 
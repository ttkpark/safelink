#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

// Custom includes
#include "bluetooth.h"
#include "i2c.h"
#include "analog.h"
#include "sensor.h"

static const char *TAG = "SAFELINK_BAND";

// Global variables
static EventGroupHandle_t sensor_event_group = NULL;



// GPIO task (LED 상태 표시)
static void gpio_task(void *pvParameters)
{
    ESP_LOGI(TAG, "GPIO task started");
    
    // GPIO 초기화 (LED용)
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_NUM_2), // GPIO2를 LED로 사용
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    bool led_state = false;
    
    while (1) {
        // Hub 연결 상태에 따라 LED 깜빡임
        if (bluetooth_is_hub_connected()) {
            // 연결됨: 빠른 깜빡임
            gpio_set_level(GPIO_NUM_2, led_state);
            led_state = !led_state;
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            // 연결 안됨: 느린 깜빡임
            gpio_set_level(GPIO_NUM_2, led_state);
            led_state = !led_state;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// Main application entry point
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32C3 Band Application (GATT Client)");
    
    // Event group 생성
    sensor_event_group = xEventGroupCreate();
    if (!sensor_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }
    
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "=== INITIALIZING MODULES ===");
    
    // I2C 초기화
    ESP_LOGI(TAG, "Initializing I2C...");
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }
    
    // Analog 모듈 초기화
    ESP_LOGI(TAG, "Initializing Analog module...");
    ret = analog_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Analog initialization failed");
        return;
    }
    
    // Bluetooth 초기화
    ESP_LOGI(TAG, "Initializing Bluetooth (GATT Client)...");
    ret = bluetooth_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth initialization failed");
        return;
    }
    
    ESP_LOGI(TAG, "=== ALL MODULES INITIALIZED ===");
    
    // 센서 모듈 초기화
    ESP_LOGI(TAG, "Initializing sensor module...");
    ret = sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor module initialization failed");
        return;
    }
    
    // 심박수 측정 시작
    ESP_LOGI(TAG, "Starting heart rate measurement...");
    heart_rate_measurement_start();
    
    // 센서 태스크들 생성
    ESP_LOGI(TAG, "Creating sensor tasks...");
    ret = sensor_create_tasks(sensor_event_group);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create sensor tasks");
        return;
    }
    
    // GPIO 태스크 (LED 상태 표시)
    BaseType_t task_ret = xTaskCreate(
        gpio_task,
        "gpio_task",
        2048,
        NULL,
        2,
        NULL
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GPIO task");
        return;
    }
    
    ESP_LOGI(TAG, "=== APPLICATION STARTED ===");
    ESP_LOGI(TAG, "Band is now scanning for Hub devices...");
    ESP_LOGI(TAG, "Target Hub name: %s", TARGET_HUB_NAME);
    ESP_LOGI(TAG, "=== END STARTUP INFO ===");
}

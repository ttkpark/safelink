#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "bluetooth.h"

static const char *TAG = "SAFELINK_TEST";

// Noise measurement task
static void noise_measurement_task(void *arg)
{
    ESP_LOGI(TAG, "Noise measurement task started");
    
    while (1) {
        // Measure noise level every 1 second
        measure_noise_level();
        
        // Check DFPlayer status every 10 seconds
        static uint32_t dfplayer_counter = 0;
        if (++dfplayer_counter >= 10) {
            check_dfplayer_status();
            dfplayer_counter = 0;
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Test task
static void test_task(void *arg)
{
    ESP_LOGI(TAG, "Test task started");
    
    while (1) {
        // Check if BLE is connected
        bluetooth_state_t ble_state = bluetooth_get_state();
        
        if (ble_state == BLUETOOTH_STATE_CONNECTED) {
            // Send test data every 2 seconds
            static uint32_t counter = 0;
            char test_data[32];
            snprintf(test_data, sizeof(test_data), "Test Data %lu", counter++);
            
            esp_err_t ret = bluetooth_send_test_data(test_data);
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "Test data sent successfully");
            } else {
                ESP_LOGW(TAG, "Failed to send test data");
            }
            
                         // Display health sensor data every 5 seconds
             static uint32_t health_counter = 0;
             if (++health_counter >= 3) { // 2초 * 3 = 6초마다
                 health_sensor_data_t health_data;
                 if (bluetooth_get_health_data(&health_data) == ESP_OK) {
                     ESP_LOGI(TAG, "Health Data - Temp: %.2f°C, Hum: %.2f%%, Body: %.2f°C, SpO2: %.2f%%, HR: %d BPM, Noise: %.1fdB", 
                              health_data.temperature / 100.0f,
                              health_data.humidity / 100.0f,
                              health_data.body_temperature / 100.0f,
                              health_data.spo2 / 100.0f,
                              health_data.heart_rate,
                              health_data.noise_level / 10.0f);
                 }
                 health_counter = 0;
             }
        } else if (ble_state == BLUETOOTH_STATE_ADVERTISING) {
            ESP_LOGI(TAG, "BLE: Advertising for connections...");
        }
        
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

// Main function
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32C6 NimBLE Health Sensor Server");
    ESP_LOGI(TAG, "BLE Device Name: %s", BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "Test Service UUID: 0x%04x", TEST_SERVICE_UUID);
    ESP_LOGI(TAG, "Test Char UUID: 0x%04x", TEST_CHAR_UUID);
    ESP_LOGI(TAG, "Health Sensor Service UUID: 0x%04x", HEALTH_SENSOR_SERVICE_UUID);
    ESP_LOGI(TAG, "Temperature Char UUID: 0x%04x", TEMPERATURE_CHAR_UUID);
    ESP_LOGI(TAG, "Humidity Char UUID: 0x%04x", HUMIDITY_CHAR_UUID);
    ESP_LOGI(TAG, "Body Temp Char UUID: 0x%04x", BODY_TEMP_CHAR_UUID);
    ESP_LOGI(TAG, "SpO2 Char UUID: 0x%04x", SPO2_CHAR_UUID);
    ESP_LOGI(TAG, "Heart Rate Char UUID: 0x%04x", HEART_RATE_CHAR_UUID);
    ESP_LOGI(TAG, "Noise Sensor Service UUID: 0x%04x", NOISE_SENSOR_SERVICE_UUID);
    ESP_LOGI(TAG, "Noise Level Char UUID: 0x%04x", NOISE_LEVEL_CHAR_UUID);
    ESP_LOGI(TAG, "Control Service UUID: 0x%04x", CONTROL_SERVICE_UUID);
    ESP_LOGI(TAG, "Command Char UUID: 0x%04x", COMMAND_CHAR_UUID);
    
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Bluetooth 초기화
    ESP_ERROR_CHECK(bluetooth_init());
    
    // Test 태스크 생성
    xTaskCreate(test_task, "test_task", 4096, NULL, 5, NULL);
    
    // Noise measurement 태스크 생성
    xTaskCreate(noise_measurement_task, "noise_task", 4096, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "NimBLE Health Sensor Server initialized successfully");
    ESP_LOGI(TAG, "BLE advertising started - Device name: %s", BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "Use nRF Connect or similar BLE scanner to connect");
    ESP_LOGI(TAG, "Health sensor data can be written to the characteristics");
} 
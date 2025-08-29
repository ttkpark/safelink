#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "i2c.h"
#include "bluetooth.h"
#include "sensor.h"
#include "data_manager.h"
#include "test_simulator.h"
#include "mic_i2s.h"

static const char *TAG = "MAIN";

// Task handles
static TaskHandle_t gpio_task_handle = NULL;
static TaskHandle_t bluetooth_task_handle = NULL;
static TaskHandle_t terminal_task_handle = NULL;

// Event group for task synchronization
static EventGroupHandle_t sensor_event_group = NULL;
#define BLUETOOTH_READY_BIT      BIT1

// GPIO task
static void gpio_task(void *arg)
{
    ESP_LOGI(TAG, "GPIO task started");
    
    while(1) {
        // GPIO 상태 확인 및 처리
        // (필요한 경우 GPIO 이벤트 처리)
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}



// Bluetooth task
static void bluetooth_task(void *arg)
{
    ESP_LOGI(TAG, "Bluetooth task started");
    
    // Bluetooth 초기화
    esp_err_t ret = bluetooth_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth");
        vTaskDelete(NULL);
        return;
    }
    
    // Bluetooth 준비 완료 이벤트 설정
    xEventGroupSetBits(sensor_event_group, BLUETOOTH_READY_BIT);
    
    ESP_LOGI(TAG, "Bluetooth initialized successfully - Device: %s", BLE_DEVICE_NAME);
    
    while(1) {
        bluetooth_state_t state = bluetooth_get_state();
        
        switch(state) {
            case BLUETOOTH_STATE_ADVERTISING:
                // 광고 상태에서는 로그를 줄임 (10초마다)
                static uint32_t adv_log_counter = 0;
                if (++adv_log_counter >= 10) {
                    ESP_LOGI(TAG, "Bluetooth advertising - waiting for connection");
                    adv_log_counter = 0;
                }
                break;
                
            case BLUETOOTH_STATE_CONNECTED:
                // 연결된 상태에서 허브 데이터 특성 업데이트 (1초마다)
                esp_err_t update_ret = bluetooth_update_hub_data_characteristics();
                if (update_ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to update hub data characteristics");
                }
                break;
                
            case BLUETOOTH_STATE_DISCONNECTED:
                ESP_LOGI(TAG, "Bluetooth disconnected - restarting advertising");
                break;
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Terminal output task
static void terminal_task(void *arg)
{
    ESP_LOGI(TAG, "Terminal task started");
    
    while(1) {
        // 데이터 매니저에서 모든 데이터를 터미널에 출력
        data_manager_print_all_data();
        
        vTaskDelay(5000 / portTICK_PERIOD_MS); // 5초마다 출력
    }
}



void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32C3 SafeLink Wearable Hub Application");
    ESP_LOGI(TAG, "BLE Device Name: %s", BLE_DEVICE_NAME);
    
    // NVS 초기화
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // I2C 초기화
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return;
    }
    
    // 이벤트 그룹 생성
    sensor_event_group = xEventGroupCreate();
    if (sensor_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }
    
    // 센서 모듈 초기화
    ret = sensor_init();
    // I2S 마이크 초기화
    esp_err_t mic_ret = mic_i2s_init();
    if (mic_ret == ESP_OK) {
        mic_i2s_start();
        ESP_LOGI(TAG, "I2S microphone initialized");
    } else {
        ESP_LOGW(TAG, "I2S microphone init failed: %s", esp_err_to_name(mic_ret));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor module");
        return;
    }
    
    // 데이터 매니저 초기화
    ret = data_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize data manager");
        return;
    }
    
    // 센서 태스크들 생성
    ret = sensor_create_tasks(sensor_event_group);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create sensor tasks");
        return;
    }
    
    // GPIO 태스크 생성
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 5, &gpio_task_handle);
    
    // Bluetooth 태스크 생성
    xTaskCreate(bluetooth_task, "bluetooth_task", 8192, NULL, 5, &bluetooth_task_handle);
    
    // Terminal 태스크 생성
    xTaskCreate(terminal_task, "terminal_task", 4096, NULL, 3, &terminal_task_handle);
    
    // 테스트 시뮬레이터 비활성화 (실센서 사용)
    // test_simulator_start_periodic_data();
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "Use nRF Connect app to scan and connect to '%s'", BLE_DEVICE_NAME);
    
    // 메인 태스크는 여기서 종료
    vTaskDelete(NULL);
}

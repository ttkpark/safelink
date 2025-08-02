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

static const char *TAG = "SAFELINK_BAND";

// Event group bits
#define SENSOR_DATA_READY_BIT      BIT0
#define BLE_CONNECTED_BIT          BIT1
#define BLE_DISCONNECTED_BIT       BIT2
#define HUB_FOUND_BIT              BIT3

// Global variables
static EventGroupHandle_t sensor_event_group = NULL;
static TaskHandle_t sensor_monitor_task_handle = NULL;
static TaskHandle_t data_sender_task_handle = NULL;

// Sensor data
static sensor_data_t current_sensor_data = {0};
static heart_rate_data_t current_heart_rate_data = {0};

// Task priorities
#define SENSOR_MONITOR_TASK_PRIORITY    4
#define DATA_SENDER_TASK_PRIORITY       3
#define SENSOR_MONITOR_TASK_STACK_SIZE  4096
#define DATA_SENDER_TASK_STACK_SIZE     4096

// Sensor reading intervals
#define TEMP_HUMIDITY_READ_INTERVAL_MS  5000  // 5초마다 온습도 읽기
#define HEART_RATE_READ_INTERVAL_MS     1000  // 1초마다 심박수 읽기
#define DATA_SEND_INTERVAL_MS           2000  // 2초마다 데이터 전송

// Sensor monitor task
static void sensor_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor monitor task started");
    
    TickType_t last_temp_humidity_read = 0;
    TickType_t last_heart_rate_read = 0;
    TickType_t last_data_send = 0;
    
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        // 온습도 센서 읽기 (5초마다)
        if ((current_time - last_temp_humidity_read) >= pdMS_TO_TICKS(TEMP_HUMIDITY_READ_INTERVAL_MS)) {
            uint16_t temperature, humidity;
            esp_err_t ret = AM2320_read(&temperature, &humidity);
            
            if (ret == ESP_OK) {
                current_sensor_data.temperature = temperature;
                current_sensor_data.humidity = humidity;
                current_sensor_data.timestamp = (uint32_t)(current_time * portTICK_PERIOD_MS);
                
                ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%", 
                         temperature / 10.0f, humidity / 10.0f);
                
                // 건강 상태 분석
                current_sensor_data.health_status = analyze_health_status(
                    current_sensor_data.heart_rate,
                    current_sensor_data.temperature,
                    current_sensor_data.humidity
                );
                
                xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
            } else {
                ESP_LOGE(TAG, "Failed to read temperature/humidity sensor");
            }
            
            last_temp_humidity_read = current_time;
        }
        
        // 심박수 읽기 (1초마다)
        if ((current_time - last_heart_rate_read) >= pdMS_TO_TICKS(HEART_RATE_READ_INTERVAL_MS)) {
            esp_err_t ret = get_heart_rate(&current_heart_rate_data);
            
            if (ret == ESP_OK && current_heart_rate_data.is_valid) {
                current_sensor_data.heart_rate = current_heart_rate_data.heart_rate;
                current_sensor_data.timestamp = current_heart_rate_data.timestamp;
                
                ESP_LOGI(TAG, "Heart Rate: %d BPM (Quality: %d%%)", 
                         current_heart_rate_data.heart_rate, 
                         current_heart_rate_data.signal_quality);
                
                // 건강 상태 분석
                current_sensor_data.health_status = analyze_health_status(
                    current_sensor_data.heart_rate,
                    current_sensor_data.temperature,
                    current_sensor_data.humidity
                );
                
                xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
            } else {
                ESP_LOGW(TAG, "No valid heart rate data available");
            }
            
            last_heart_rate_read = current_time;
        }
        
        // 데이터 전송 (2초마다)
        if ((current_time - last_data_send) >= pdMS_TO_TICKS(DATA_SEND_INTERVAL_MS)) {
            if (bluetooth_is_hub_connected()) {
                xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
            }
            last_data_send = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 간격으로 체크
    }
}

// Data sender task
static void data_sender_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Data sender task started");
    
    while (1) {
        // 센서 데이터 준비 이벤트 대기
        EventBits_t bits = xEventGroupWaitBits(
            sensor_event_group,
            SENSOR_DATA_READY_BIT,
            pdFALSE,  // Clear bits on exit
            pdFALSE,  // Wait for any bit
            pdMS_TO_TICKS(1000)  // 1초 타임아웃
        );
        
        if (bits & SENSOR_DATA_READY_BIT) {
            // Hub에 연결되어 있는지 확인
            if (bluetooth_is_hub_connected()) {
                // 센서 데이터를 Hub로 전송
                esp_err_t ret = bluetooth_send_sensor_data(&current_sensor_data);
                
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "=== SENSOR DATA SENT TO HUB ===");
                    ESP_LOGI(TAG, "Heart Rate: %d BPM", current_sensor_data.heart_rate);
                    ESP_LOGI(TAG, "Temperature: %.1f°C", current_sensor_data.temperature / 10.0f);
                    ESP_LOGI(TAG, "Humidity: %.1f%%", current_sensor_data.humidity / 10.0f);
                    ESP_LOGI(TAG, "Health Status: %s", get_health_status_string(current_sensor_data.health_status));
                    ESP_LOGI(TAG, "Timestamp: %lu ms", current_sensor_data.timestamp);
                    ESP_LOGI(TAG, "=== END SENSOR DATA ===");
                } else {
                    ESP_LOGE(TAG, "Failed to send sensor data to hub");
                }
            } else {
                ESP_LOGW(TAG, "Not connected to hub, cannot send data");
                
                // Hub 재검색 시작
                bluetooth_discover_hubs();
            }
        }
    }
}

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
    
    // 심박수 측정 시작
    ESP_LOGI(TAG, "Starting heart rate measurement...");
    heart_rate_measurement_start();
    
    // 태스크 생성
    ESP_LOGI(TAG, "Creating tasks...");
    
    // 센서 모니터 태스크
    BaseType_t task_ret = xTaskCreate(
        sensor_monitor_task,
        "sensor_monitor",
        SENSOR_MONITOR_TASK_STACK_SIZE,
        NULL,
        SENSOR_MONITOR_TASK_PRIORITY,
        &sensor_monitor_task_handle
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor monitor task");
        return;
    }
    
    // 데이터 전송 태스크
    task_ret = xTaskCreate(
        data_sender_task,
        "data_sender",
        DATA_SENDER_TASK_STACK_SIZE,
        NULL,
        DATA_SENDER_TASK_PRIORITY,
        &data_sender_task_handle
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create data sender task");
        return;
    }
    
    // GPIO 태스크 (LED 상태 표시)
    task_ret = xTaskCreate(
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

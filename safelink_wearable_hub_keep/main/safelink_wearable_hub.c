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
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "i2c.h"
#include "analog.h"
#include "bluetooth.h"

static const char *TAG = "MAIN";

// Task handles
static TaskHandle_t gpio_task_handle = NULL;
static TaskHandle_t bluetooth_task_handle = NULL;

// Event group for task synchronization
static EventGroupHandle_t sensor_event_group = NULL;
#define SENSOR_DATA_READY_BIT    BIT0
#define BLUETOOTH_READY_BIT      BIT1

// Sensor data structure
static sensor_data_t current_sensor_data = {0};

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

// Heart rate sensor task
static void heart_rate_task(void *arg)
{
    ESP_LOGI(TAG, "Heart rate task started");
    
    // ADC 초기화
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    // ADC 채널 설정
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    
    while(1) {
        int adc_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw));
        
        // 심박수 계산 (간단한 예시 - 실제로는 더 복잡한 알고리즘 필요)
        uint16_t heart_rate = 60 + (adc_raw % 40); // 60-100 BPM 범위
        
        current_sensor_data.heart_rate = heart_rate;
        current_sensor_data.timestamp = esp_timer_get_time() / 1000; // ms 단위
        
        // 건강 상태 분석 (온도와 습도가 있는 경우에만)
        if (current_sensor_data.temperature > 0 && current_sensor_data.humidity > 0) {
            current_sensor_data.health_status = analyze_health_status(
                current_sensor_data.heart_rate,
                current_sensor_data.temperature,
                current_sensor_data.humidity
            );
        }
        
        ESP_LOGI(TAG, "Heart rate: %d BPM, Status: %s", 
                heart_rate, get_health_status_string(current_sensor_data.health_status));
        
        // 센서 데이터 준비 이벤트 설정
        xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
        
        vTaskDelay(2000 / portTICK_PERIOD_MS); // 2초마다 측정
    }
}

// 온도/습도 센서 태스크
static void temp_humidity_task(void *arg)
{
    ESP_LOGI(TAG, "Temperature/Humidity task started");
    
    while(1) {
        uint16_t temperature, humidity;
        
        // AM2320 센서에서 데이터 읽기
        esp_err_t ret = AM2320_read(&temperature, &humidity);
        if (ret == ESP_OK) {
            // 온도와 습도 값을 100으로 곱하여 0.01 단위 정밀도로 저장
            // (BLE 표준에 맞춰 0.01°C 단위로 저장)
            current_sensor_data.temperature = temperature; // 25.4°C -> 2540
            current_sensor_data.humidity = humidity;       // 45.2% -> 4520
            current_sensor_data.timestamp = esp_timer_get_time() / 1000; // ms 단위
            
            // 건강 상태 분석
            current_sensor_data.health_status = analyze_health_status(
                current_sensor_data.heart_rate,
                current_sensor_data.temperature,
                current_sensor_data.humidity
            );
            
            float temp_float = (float)temperature / 10.0f;
            float hum_float = (float)humidity / 10.0f;
            
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%, Status: %s", 
                    temp_float, hum_float, get_health_status_string(current_sensor_data.health_status));
            
            // 센서 데이터 준비 이벤트 설정
            xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
        } else {
            ESP_LOGW(TAG, "Failed to read temperature/humidity sensor: %s", esp_err_to_name(ret));
            // 실패 시 기본값 설정
            current_sensor_data.temperature = 2500; // 25.0°C
            current_sensor_data.humidity = 5000;    // 50.0%
            current_sensor_data.health_status = HEALTH_STATUS_NORMAL;
        }
        
        // 5초마다 측정 (watchdog 타이머보다 짧게)
        vTaskDelay(4000 / portTICK_PERIOD_MS);
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
                // 연결된 상태에서 센서 데이터 전송 (200ms마다)
                if (xEventGroupWaitBits(sensor_event_group, 
                                       SENSOR_DATA_READY_BIT, 
                                       pdTRUE, pdFALSE, 
                                       50 / portTICK_PERIOD_MS) & SENSOR_DATA_READY_BIT) {
                    
                    // 센서 데이터를 Bluetooth로 전송
                    esp_err_t send_ret = bluetooth_send_sensor_data(&current_sensor_data);
                    if (send_ret == ESP_OK) {
                        // 성공 로그는 간소화 (5초마다)
                        static uint32_t success_log_counter = 0;
                        if (++success_log_counter >= 25) { // 200ms * 25 = 5초
                            ESP_LOGI(TAG, "BLE: Data sent successfully");
                            success_log_counter = 0;
                        }
                    } else {
                        ESP_LOGW(TAG, "Failed to send sensor data via BLE");
                    }
                }
                break;
                
            case BLUETOOTH_STATE_DISCONNECTED:
                ESP_LOGI(TAG, "Bluetooth disconnected - restarting advertising");
                bluetooth_start_advertising();
                break;
        }
        
        vTaskDelay(200 / portTICK_PERIOD_MS); // 1초 → 200ms로 단축
    }
}

// Sensor data monitoring task
static void sensor_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor monitor task started");
    
    while(1) {
        // 모든 센서 데이터가 준비되었는지 확인 (타임아웃 단축)
        EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
                                              SENSOR_DATA_READY_BIT | BLUETOOTH_READY_BIT,
                                              pdTRUE, pdFALSE,
                                              1000 / portTICK_PERIOD_MS); // 5초 → 1초로 단축
        
        if (bits & SENSOR_DATA_READY_BIT) {
            float temp_float = (float)current_sensor_data.temperature / 100.0f;
            float hum_float = (float)current_sensor_data.humidity / 100.0f;
            
            // 로그 간소화 (5초마다만 출력)
            static uint32_t monitor_log_counter = 0;
            if (++monitor_log_counter >= 5) {
                ESP_LOGI(TAG, "Sensors: HR=%d, T=%.1f°C, H=%.1f%%, Status=%s", 
                        current_sensor_data.heart_rate,
                        temp_float, hum_float,
                        get_health_status_string(current_sensor_data.health_status));
                monitor_log_counter = 0;
            }
            
            // 건강 상태 경고는 즉시 출력
            switch (current_sensor_data.health_status) {
                case HEALTH_STATUS_ELEVATED_HR:
                    ESP_LOGW(TAG, "⚠️  WARNING: Elevated heart rate detected!");
                    break;
                case HEALTH_STATUS_HIGH_TEMP:
                    ESP_LOGW(TAG, "⚠️  WARNING: Temperature outside normal range!");
                    break;
                case HEALTH_STATUS_LOW_HUMIDITY:
                    ESP_LOGW(TAG, "⚠️  WARNING: Humidity outside normal range!");
                    break;
                case HEALTH_STATUS_WARNING:
                    ESP_LOGW(TAG, "⚠️  WARNING: Multiple health parameters abnormal!");
                    break;
                case HEALTH_STATUS_CRITICAL:
                    ESP_LOGE(TAG, "🚨 CRITICAL: Multiple critical health parameters!");
                    break;
                case HEALTH_STATUS_NORMAL:
                default:
                    // 정상 상태는 로그 출력 안함
                    break;
            }
        }
        
        vTaskDelay(500 / portTICK_PERIOD_MS); // 1초 → 500ms로 단축
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32C3 Sensor Application");
    ESP_LOGI(TAG, "BLE Device Name: %s", BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "Heart Rate Service UUID: 0x%04x", HEART_RATE_SERVICE_UUID);
    ESP_LOGI(TAG, "Heart Rate Char UUID: 0x%04x", HEART_RATE_CHAR_UUID);
    ESP_LOGI(TAG, "Temperature Service UUID: 0x%04x", TEMPERATURE_SERVICE_UUID);
    ESP_LOGI(TAG, "Temperature Char UUID: 0x%04x", TEMPERATURE_CHAR_UUID);
    ESP_LOGI(TAG, "Environmental Service UUID: 0x%04x", ENVIRONMENTAL_SERVICE_UUID);
    ESP_LOGI(TAG, "Humidity Char UUID: 0x%04x", HUMIDITY_CHAR_UUID);
    ESP_LOGI(TAG, "Pressure Char UUID: 0x%04x", PRESSURE_CHAR_UUID);
    
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
    
    // GPIO 태스크 생성
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 5, &gpio_task_handle);
    
    // 심박수 센서 태스크 생성
    xTaskCreate(heart_rate_task, "heart_rate_task", 4096, NULL, 5, NULL);
    
    // 온도/습도 센서 태스크 생성
    xTaskCreate(temp_humidity_task, "temp_humidity_task", 4096, NULL, 5, NULL);
    
    // Bluetooth 태스크 생성
    xTaskCreate(bluetooth_task, "bluetooth_task", 8192, NULL, 5, &bluetooth_task_handle);
    
    // 센서 모니터링 태스크 생성
    xTaskCreate(sensor_monitor_task, "sensor_monitor_task", 4096, NULL, 3, NULL);
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "Use nRF Connect app to scan and connect to '%s'", BLE_DEVICE_NAME);
    
    // 메인 태스크는 여기서 종료
    vTaskDelete(NULL);
}

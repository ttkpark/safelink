#include "sensor.h"
#include "data_manager.h"
#include "mic_i2s.h"
#include <math.h>

static const char *TAG = "SENSOR";

// Global variables
static sensor_data_t current_sensor_data = {0};
static extended_sensor_data_t current_extended_data = {0};
static EventGroupHandle_t sensor_event_group = NULL;
static adc_oneshot_unit_handle_t adc1_handle = NULL;

// Task handles
TaskHandle_t heart_rate_task_handle = NULL;
static TaskHandle_t temp_humidity_task_handle = NULL;
static TaskHandle_t sensor_monitor_task_handle = NULL;

// Heart rate sensor task
void heart_rate_task(void *arg)
{
    while(1) {
        vTaskDelay(HEART_RATE_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

// 온도/습도 센서 태스크
void temp_humidity_task(void *arg)
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
            
            // 확장된 센서 데이터 업데이트
            current_extended_data.external_temperature = temperature; // 외부 온도
            current_extended_data.external_humidity = humidity;       // 외부 습도
            current_extended_data.timestamp = esp_timer_get_time() / 1000; // ms 단위
            
            // 건강 상태 분석
            current_sensor_data.health_status = analyze_health_status(
                current_sensor_data.heart_rate,
                current_sensor_data.temperature,
                current_sensor_data.humidity
            );
            
            // 확장된 건강 상태 분석
            current_extended_data.health_status = analyze_extended_health_status(&current_extended_data);
            
            float temp_float = (float)temperature / 10.0f;
            float hum_float = (float)humidity / 10.0f;
            
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%, Status: %s", 
                    temp_float, hum_float, get_health_status_string(current_sensor_data.health_status));
            
            // 센서 데이터 준비 이벤트 설정
            if (sensor_event_group) {
                xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read temperature/humidity sensor: %s", esp_err_to_name(ret));
            // 실패 시 기본값 설정
            //current_sensor_data.temperature = 2500; // 25.0°C
            //current_sensor_data.humidity = 5000;    // 50.0%
            //current_sensor_data.health_status = HEALTH_STATUS_NORMAL;
        }
        
        // 5초마다 측정 (watchdog 타이머보다 짧게)
        vTaskDelay(TEMP_HUMIDITY_READ_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}
// WBGT 계산 함수
// temperature: 건구온도(℃), humidity: 상대습도(%)
float calc_wbgt(float temperature, float humidity) {
    float Td = temperature;
    float RH = humidity;

    // Stull(2011) 습구온도 근사 공식
    float Tw = Td * atanf(0.151977 * sqrtf(RH + 8.313659))
               + atanf(Td + RH)
               - atanf(RH - 1.676331)
               + 0.00391838 * powf(RH, 1.5) * atanf(0.023101 * RH)
               - 4.686035;

    // WBGT 근사 공식 (그늘 환경 기준)
    float WBGT = 0.7f * Tw + 0.3f * Td;

    return WBGT;
}

// Sensor data monitoring task
void sensor_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "Sensor monitor task started");
    
    // 허브 데이터 수집을 위한 변수들
    static uint32_t last_hub_update = 0;
    
    while(1) {
        // 모든 센서 데이터가 준비되었는지 확인
        EventBits_t bits = xEventGroupWaitBits(sensor_event_group,
                                              SENSOR_DATA_READY_BIT | BLUETOOTH_READY_BIT,
                                              pdTRUE, pdFALSE,
                                              1000 / portTICK_PERIOD_MS);
        
        if (bits & SENSOR_DATA_READY_BIT) {
            // I2S 마이크로부터 평균 dB SPL 읽기
            float avg_noise = mic_i2s_get_average_spl_db();
            ESP_LOGI(TAG, "Measured Noise: %.1fdB", avg_noise);
            // WBGT 계산 (간단한 추정치)
            float temp_float = (float)current_sensor_data.temperature / 100.0f;
            float hum_float = (float)current_sensor_data.humidity / 100.0f;
            float wbgt = calc_wbgt(temp_float, hum_float);
            
            // 경보 상태 결정
            uint8_t alarm_status = 0;
            
            // WBGT 경고 (30°C 이상)
            if (wbgt >= 30.0f) {
                alarm_status |= ALARM_WBGT_WARNING;
            }
            
            // 체온 경고 (밴드 데이터에서 확인)
            band_data_t band_data;
            if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
                if (band_data.skin_temp > 38.0f || band_data.skin_temp < 35.0f) {
                    alarm_status |= ALARM_TEMP_WARNING;
                }
                if (band_data.heart_rate > 120 || band_data.heart_rate < 50) {
                    alarm_status |= ALARM_HR_WARNING;
                }
            }
            
            // 허브 데이터 업데이트 (5초마다)
            uint32_t current_time = esp_timer_get_time() / 1000;
            if (current_time - last_hub_update >= 5000) {
                hub_data_t hub_data = {
                    .avg_noise = avg_noise,
                    .wbgt = wbgt,
                    .alarm_status = alarm_status,
                    .timestamp = current_time,
                    .is_valid = true
                };
                
                esp_err_t ret = data_manager_update_hub_data(&hub_data);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "Hub data updated: Noise=%.1fdB, WBGT=%.1f°C, Alarm=0x%02X",
                             avg_noise, wbgt, alarm_status);
                }
                
                last_hub_update = current_time;
            }
            
            // 로그 출력 (10초마다)
            static uint32_t monitor_log_counter = 0;
            if (++monitor_log_counter >= 10) {
                ESP_LOGI(TAG, "Sensors: HR=%d, T=%.1f°C, H=%.1f%%, Noise=%.1fdB, WBGT=%.1f°C", 
                        current_sensor_data.heart_rate,
                        temp_float, hum_float, avg_noise, wbgt);
                monitor_log_counter = 0;
            }
            
            // 경보 상태 로그
            if (alarm_status & ALARM_WBGT_WARNING) {
                ESP_LOGW(TAG, "⚠️  WBGT WARNING: %.1f°C", wbgt);
            }
            if (alarm_status & ALARM_TEMP_WARNING) {
                ESP_LOGW(TAG, "⚠️  TEMPERATURE WARNING");
            }
            if (alarm_status & ALARM_HR_WARNING) {
                ESP_LOGW(TAG, "⚠️  HEART RATE WARNING");
            }
        }
        
        vTaskDelay(MONITOR_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}

// Sensor initialization
esp_err_t sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing sensors");
    
    // 센서 데이터 초기화
    memset(&current_sensor_data, 0, sizeof(sensor_data_t));
    memset(&current_extended_data, 0, sizeof(extended_sensor_data_t));
    
    // 기본값 설정
    current_sensor_data.heart_rate = 00;
    current_sensor_data.temperature = 0000; // 25.0°C
    current_sensor_data.humidity = 0000;    // 50.0%
    current_sensor_data.health_status = HEALTH_STATUS_NORMAL;
    
    // 확장된 센서 데이터 기본값 설정
    current_extended_data.heart_rate = 00;
    current_extended_data.spo2 = 0000;           // 95.00%
    current_extended_data.body_temperature = 0000; // 36.50°C
    current_extended_data.external_temperature = 0000; // 25.00°C
    current_extended_data.external_humidity = 0000;    // 50.00%
    current_extended_data.noise_level = 000;     // 65.0dB
    current_extended_data.health_status = HEALTH_STATUS_NORMAL;
    
    ESP_LOGI(TAG, "Sensor initialization completed");
    return ESP_OK;
}

// Sensor deinitialization
esp_err_t sensor_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing sensors");
    
    // ADC 정리
    if (adc1_handle) {
        adc_oneshot_del_unit(adc1_handle);
        adc1_handle = NULL;
    }
    
    // 태스크 삭제
    if (heart_rate_task_handle) {
        vTaskDelete(heart_rate_task_handle);
        heart_rate_task_handle = NULL;
    }
    
    if (temp_humidity_task_handle) {
        vTaskDelete(temp_humidity_task_handle);
        temp_humidity_task_handle = NULL;
    }
    
    if (sensor_monitor_task_handle) {
        vTaskDelete(sensor_monitor_task_handle);
        sensor_monitor_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Sensor deinitialization completed");
    return ESP_OK;
}

// Get current sensor data
esp_err_t sensor_get_current_data(sensor_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(data, &current_sensor_data, sizeof(sensor_data_t));
    return ESP_OK;
}

// Get extended sensor data
esp_err_t sensor_get_extended_data(extended_sensor_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(data, &current_extended_data, sizeof(extended_sensor_data_t));
    return ESP_OK;
}

// Update health status
esp_err_t sensor_update_health_status(void)
{
    current_sensor_data.health_status = analyze_health_status(
        current_sensor_data.heart_rate,
        current_sensor_data.temperature,
        current_sensor_data.humidity
    );
    
    return ESP_OK;
}

// Create sensor tasks
esp_err_t sensor_create_tasks(EventGroupHandle_t event_group)
{
    if (!event_group) {
        ESP_LOGE(TAG, "Invalid event group");
        return ESP_ERR_INVALID_ARG;
    }
    
    sensor_event_group = event_group;
    
    // 심박수 센서 태스크 생성
    BaseType_t ret = xTaskCreate(heart_rate_task, 
                                 "heart_rate_task", 
                                 SENSOR_TASK_STACK_SIZE, 
                                 NULL, 
                                 SENSOR_TASK_PRIORITY, 
                                 &heart_rate_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heart rate task");
        return ESP_FAIL;
    }
    
    // 온도/습도 센서 태스크 생성
    ret = xTaskCreate(temp_humidity_task, 
                      "temp_humidity_task", 
                      SENSOR_TASK_STACK_SIZE, 
                      NULL, 
                      SENSOR_TASK_PRIORITY, 
                      &temp_humidity_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create temperature/humidity task");
        return ESP_FAIL;
    }
    
    // 센서 모니터링 태스크 생성
    ret = xTaskCreate(sensor_monitor_task, 
                      "sensor_monitor_task", 
                      MONITOR_TASK_STACK_SIZE, 
                      NULL, 
                      MONITOR_TASK_PRIORITY, 
                      &sensor_monitor_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor monitor task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "All sensor tasks created successfully");
    return ESP_OK;
}

// Health analysis functions
health_status_t analyze_health_status(uint16_t heart_rate, uint16_t temperature, uint16_t humidity)
{
    // 간단한 건강 상태 분석
    if (heart_rate > 120 || heart_rate < 50) {
        return HEALTH_STATUS_ELEVATED_HR;
    }
    
    float temp_float = (float)temperature / 100.0f;
    if (temp_float > 35.0f || temp_float < 15.0f) {
        return HEALTH_STATUS_HIGH_TEMP;
    }
    
    float hum_float = (float)humidity / 100.0f;
    if (hum_float < 20.0f || hum_float > 90.0f) {
        return HEALTH_STATUS_LOW_HUMIDITY;
    }
    
    return HEALTH_STATUS_NORMAL;
}

health_status_t analyze_extended_health_status(const extended_sensor_data_t *data)
{
    if (!data) {
        return HEALTH_STATUS_NORMAL;
    }
    
    // 확장된 건강 상태 분석
    if (data->heart_rate > 120 || data->heart_rate < 50) {
        return HEALTH_STATUS_ELEVATED_HR;
    }
    
    float body_temp = (float)data->body_temperature / 100.0f;
    if (body_temp > 38.0f || body_temp < 35.0f) {
        return HEALTH_STATUS_HIGH_TEMP;
    }
    
    return HEALTH_STATUS_NORMAL;
}

const char* get_health_status_string(health_status_t status)
{
    switch (status) {
        case HEALTH_STATUS_NORMAL:
            return "NORMAL";
        case HEALTH_STATUS_ELEVATED_HR:
            return "ELEVATED_HR";
        case HEALTH_STATUS_HIGH_TEMP:
            return "HIGH_TEMP";
        case HEALTH_STATUS_LOW_HUMIDITY:
            return "LOW_HUMIDITY";
        case HEALTH_STATUS_WARNING:
            return "WARNING";
        case HEALTH_STATUS_CRITICAL:
            return "CRITICAL";
        default:
            return "UNKNOWN";
    }
}

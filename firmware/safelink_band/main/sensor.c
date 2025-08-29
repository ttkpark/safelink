#include "sensor.h"

static const char *TAG = "SENSOR";

// Global variables
static sensor_data_t current_sensor_data = {0};
static heart_rate_data_t current_heart_rate_data = {0};
static EventGroupHandle_t sensor_event_group = NULL;

// Task handles
static TaskHandle_t sensor_monitor_task_handle = NULL;
static TaskHandle_t data_sender_task_handle = NULL;

// Sensor monitor task
void sensor_monitor_task(void *pvParameters)
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
                
                if (sensor_event_group) {
                    xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
                }
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
                
                if (sensor_event_group) {
                    xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
                }
            } else {
                ESP_LOGW(TAG, "No valid heart rate data available");
            }
            
            last_heart_rate_read = current_time;
        }
        
        // 데이터 전송 (2초마다)
        if ((current_time - last_data_send) >= pdMS_TO_TICKS(DATA_SEND_INTERVAL_MS)) {
            if (bluetooth_is_hub_connected()) {
                if (sensor_event_group) {
                    xEventGroupSetBits(sensor_event_group, SENSOR_DATA_READY_BIT);
                }
            }
            last_data_send = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 간격으로 체크
    }
}

// Data sender task
void data_sender_task(void *pvParameters)
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

// Sensor initialization
esp_err_t sensor_init(void)
{
    ESP_LOGI(TAG, "Initializing sensors");
    
    // 센서 데이터 초기화
    memset(&current_sensor_data, 0, sizeof(sensor_data_t));
    memset(&current_heart_rate_data, 0, sizeof(heart_rate_data_t));
    
    // 기본값 설정
    current_sensor_data.heart_rate = 70;
    current_sensor_data.temperature = 2500; // 25.0°C
    current_sensor_data.humidity = 5000;    // 50.0%
    current_sensor_data.health_status = HEALTH_STATUS_NORMAL;
    
    ESP_LOGI(TAG, "Sensor initialization completed");
    return ESP_OK;
}

// Sensor deinitialization
esp_err_t sensor_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing sensors");
    
    // 태스크 삭제
    if (sensor_monitor_task_handle) {
        vTaskDelete(sensor_monitor_task_handle);
        sensor_monitor_task_handle = NULL;
    }
    
    if (data_sender_task_handle) {
        vTaskDelete(data_sender_task_handle);
        data_sender_task_handle = NULL;
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

// Get heart rate data
esp_err_t sensor_get_heart_rate_data(heart_rate_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(data, &current_heart_rate_data, sizeof(heart_rate_data_t));
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
    
    // 센서 모니터링 태스크 생성
    BaseType_t ret = xTaskCreate(sensor_monitor_task, 
                                 "sensor_monitor_task", 
                                 SENSOR_MONITOR_TASK_STACK_SIZE, 
                                 NULL, 
                                 SENSOR_MONITOR_TASK_PRIORITY, 
                                 &sensor_monitor_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor monitor task");
        return ESP_FAIL;
    }
    
    // 데이터 전송 태스크 생성
    ret = xTaskCreate(data_sender_task, 
                      "data_sender_task", 
                      DATA_SENDER_TASK_STACK_SIZE, 
                      NULL, 
                      DATA_SENDER_TASK_PRIORITY, 
                      &data_sender_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create data sender task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "All sensor tasks created successfully");
    return ESP_OK;
} 
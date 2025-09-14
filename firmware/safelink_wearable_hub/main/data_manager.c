#include "data_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "DATA_MANAGER";

// 전역 변수들 (세마포어로 보호)
static band_data_t g_band_data = {0};
static hub_data_t g_hub_data = {0};
static SemaphoreHandle_t g_data_mutex = NULL;
static bool g_has_client_data = false;  // 클라이언트 데이터 존재 여부

// 데이터 출력 주기 제어
static uint32_t g_last_print_time = 0;
#define DATA_PRINT_INTERVAL_MS 5000  // 5초마다 출력

esp_err_t data_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing data manager");
    
    // 뮤텍스 생성
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 초기 데이터 설정
    memset(&g_band_data, 0, sizeof(band_data_t));
    memset(&g_hub_data, 0, sizeof(hub_data_t));
    
    g_band_data.is_valid = false;
    g_hub_data.is_valid = false;
    
    ESP_LOGI(TAG, "Data manager initialized successfully");
    return ESP_OK;
}

esp_err_t data_manager_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing data manager");
    
    if (g_data_mutex != NULL) {
        vSemaphoreDelete(g_data_mutex);
        g_data_mutex = NULL;
    }
    
    return ESP_OK;
}

esp_err_t data_manager_update_band_data(const band_data_t *data)
{
    
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!data_manager_validate_band_data(data)) {
        ESP_LOGW(TAG, "Invalid band data received");
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&g_band_data, data, sizeof(band_data_t));
        g_band_data.timestamp = esp_timer_get_time() / 1000; // ms 단위로 변환
        
        // 데이터 소스 설정 (클라이언트 데이터인 경우 플래그 설정)
        if (data->data_source == DATA_SOURCE_CLIENT) {
            g_has_client_data = true;
            ESP_LOGI(TAG, "Client data received - simulator will be paused");
        }
        
        xSemaphoreGive(g_data_mutex);
        
        ESP_LOGI(TAG, "Band data updated: T=%.1f°C, H=%.1f%%, Skin=%.1f°C, HR=%d, SpO2=%.1f%% (Source: %s)",
                 data->external_temp, data->external_humidity, data->skin_temp, 
                 data->heart_rate, data->spo2,
                 (data->data_source == DATA_SOURCE_CLIENT) ? "CLIENT" : "SIMULATOR");
        return ESP_OK;
    }else{
        ESP_LOGI(TAG, "Band MFG parse semaphore take failed");
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t data_manager_get_band_data(band_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &g_band_data, sizeof(band_data_t));
        xSemaphoreGive(g_data_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

bool data_manager_validate_band_data(const band_data_t *data)
{
    if (data == NULL) {
        return false;
    }
    
    // 피부온도 검증 (15.0~40.0°C)
    if (data->skin_temp < SKIN_TEMP_MIN || data->skin_temp > SKIN_TEMP_MAX) {
        ESP_LOGW(TAG, "Invalid skin temperature: %.1f°C (range: %.1f~%.1f°C)", 
                 data->skin_temp, SKIN_TEMP_MIN, SKIN_TEMP_MAX);
        return false;
    }
    
    // 심박수 검증 (20~160 BPM)
    if (data->heart_rate < HEART_RATE_MIN || data->heart_rate > HEART_RATE_MAX) {
        ESP_LOGW(TAG, "Invalid heart rate: %d BPM (range: %d~%d BPM)", 
                 data->heart_rate, HEART_RATE_MIN, HEART_RATE_MAX);
        return false;
    }
    
    return true;
}

esp_err_t data_manager_update_hub_data(const hub_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(&g_hub_data, data, sizeof(hub_data_t));
        g_hub_data.timestamp = esp_timer_get_time() / 1000; // ms 단위로 변환
        xSemaphoreGive(g_data_mutex);
        
        ESP_LOGI(TAG, "Hub data updated: Noise=%.1fdB, WBGT=%.1f°C, Alarm=0x%02X",
                 data->avg_noise, data->wbgt, data->alarm_status);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t data_manager_get_hub_data(hub_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &g_hub_data, sizeof(hub_data_t));
        xSemaphoreGive(g_data_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t data_manager_update_alarm_status(uint8_t alarm_status)
{
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_hub_data.alarm_status = alarm_status;
        g_hub_data.timestamp = esp_timer_get_time() / 1000;
        xSemaphoreGive(g_data_mutex);
        
        ESP_LOGI(TAG, "Alarm status updated: 0x%02X", alarm_status);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t data_manager_set_wbgt_alarm(bool enabled)
{
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (enabled) {
            g_hub_data.alarm_status |= ALARM_WBGT_WARNING;
        } else {
            g_hub_data.alarm_status &= ~ALARM_WBGT_WARNING;
        }
        g_hub_data.timestamp = esp_timer_get_time() / 1000;
        xSemaphoreGive(g_data_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t data_manager_set_temp_alarm(bool enabled)
{
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (enabled) {
            g_hub_data.alarm_status |= ALARM_TEMP_WARNING;
        } else {
            g_hub_data.alarm_status &= ~ALARM_TEMP_WARNING;
        }
        g_hub_data.timestamp = esp_timer_get_time() / 1000;
        xSemaphoreGive(g_data_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t data_manager_set_hr_alarm(bool enabled)
{
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (enabled) {
            g_hub_data.alarm_status |= ALARM_HR_WARNING;
        } else {
            g_hub_data.alarm_status &= ~ALARM_HR_WARNING;
        }
        g_hub_data.timestamp = esp_timer_get_time() / 1000;
        xSemaphoreGive(g_data_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

uint8_t data_manager_get_alarm_status(void)
{
    uint8_t alarm_status = 0;
    
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        alarm_status = g_hub_data.alarm_status;
        xSemaphoreGive(g_data_mutex);
    }
    
    return alarm_status;
}

void data_manager_print_all_data(void)
{
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // 출력 주기 제어
    if (current_time - g_last_print_time < DATA_PRINT_INTERVAL_MS) {
        return;
    }
    
    g_last_print_time = current_time;
    
    band_data_t band_data;
    hub_data_t hub_data;
    
    if (data_manager_get_band_data(&band_data) == ESP_OK &&
        data_manager_get_hub_data(&hub_data) == ESP_OK) {
        
        printf("\n=== SafeLink Wearable Hub Data ===\n");
        printf("Timestamp: %lu ms\n", current_time);
        
        // 밴드 데이터 출력
        printf("\n[Band Data] %s\n", band_data.is_valid ? "VALID" : "INVALID");
        if (band_data.is_valid) {
            printf("  External Temp: %.1f°C\n", band_data.external_temp);
            printf("  External Humidity: %.1f%%\n", band_data.external_humidity);
            printf("  Skin Temperature: %.1f°C\n", band_data.skin_temp);
            printf("  Heart Rate: %d BPM\n", band_data.heart_rate);
            printf("  SpO2: %.1f%%\n", band_data.spo2);
            printf("  Band Timestamp: %lu ms\n", band_data.timestamp);
            printf("  Data Source: %s\n", 
                   (band_data.data_source == DATA_SOURCE_CLIENT) ? "CLIENT" : "SIMULATOR");
        }
        
        // 허브 데이터 출력
        printf("\n[Hub Data] %s\n", hub_data.is_valid ? "VALID" : "INVALID");
        if (hub_data.is_valid) {
            printf("  Average Noise: %.1f dB\n", hub_data.avg_noise);
            printf("  WBGT: %.1f°C\n", hub_data.wbgt);
            printf("  Alarm Status: 0x%02X", hub_data.alarm_status);
            
            // 경보 상태 상세 출력
            if (hub_data.alarm_status & ALARM_WBGT_WARNING) printf(" (WBGT)");
            if (hub_data.alarm_status & ALARM_TEMP_WARNING) printf(" (TEMP)");
            if (hub_data.alarm_status & ALARM_HR_WARNING) printf(" (HR)");
            printf("\n");
            
            printf("  Hub Timestamp: %lu ms\n", hub_data.timestamp);
        }
        
        // 클라이언트 데이터 상태 출력
        printf("\n[System Status]\n");
        printf("  Client Data Active: %s\n", g_has_client_data ? "YES" : "NO");
        
        printf("================================\n\n");
    }
}

// 클라이언트 데이터 존재 여부 확인
bool data_manager_has_client_data(void)
{
    return g_has_client_data;
}

// 클라이언트 데이터 플래그 초기화 (시뮬레이터 재시작용)
void data_manager_clear_client_data(void)
{
    if (xSemaphoreTake(g_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_has_client_data = false;
        xSemaphoreGive(g_data_mutex);
        ESP_LOGI(TAG, "Client data flag cleared - simulator can resume");
    }
}

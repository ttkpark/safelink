#include "test_simulator.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <time.h>

static const char *TAG = "TEST_SIMULATOR";

// 시뮬레이션 상태
static bool g_simulator_enabled = false;
static TaskHandle_t g_simulator_task_handle = NULL;

// 시뮬레이션 데이터 생성 태스크
static void simulator_task(void *arg)
{
    ESP_LOGI(TAG, "Test simulator task started");
    
    // 랜덤 시드 초기화
    srand(esp_timer_get_time());
    
    while (g_simulator_enabled) {
        // 밴드 데이터 생성
        test_simulator_generate_band_data();
        
        // 허브 데이터 생성
        test_simulator_generate_hub_data();
        
        // 3초마다 데이터 생성
        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
    
    ESP_LOGI(TAG, "Test simulator task stopped");
    vTaskDelete(NULL);
}

esp_err_t test_simulator_generate_band_data(void)
{
    if (!g_simulator_enabled) {
        return ESP_OK;
    }
    
    // 클라이언트 데이터가 있으면 시뮬레이터 데이터 생성 중단
    if (data_manager_has_client_data()) {
        ESP_LOGD(TAG, "Client data active - skipping simulator data generation");
        return ESP_OK;
    }
    
    // 밴드 데이터 생성 (현실적인 범위 내에서)
    band_data_t band_data = {0};
    
    // 외기 온도 (15~35°C)
    band_data.external_temp = 15.0f + (rand() % 200) / 10.0f;
    
    // 외기 습도 (30~80%)
    band_data.external_humidity = 30.0f + (rand() % 500) / 10.0f;
    
    // 피부온도 (35.5~37.5°C)
    band_data.skin_temp = 35.5f + (rand() % 200) / 10.0f;
    
    // 심박수 (60~100 BPM)
    band_data.heart_rate = 60 + (rand() % 40);
    
    // 혈중산소포화농도 (95~99%)
    band_data.spo2 = 95.0f + (rand() % 40) / 10.0f;
    
    band_data.timestamp = esp_timer_get_time() / 1000;
    band_data.is_valid = true;
    band_data.data_source = DATA_SOURCE_SIMULATOR;  // 시뮬레이터 데이터로 표시
    
    // 데이터 매니저에 업데이트
    esp_err_t ret = data_manager_update_band_data(&band_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Simulated band data: T=%.1f°C, H=%.1f%%, Skin=%.1f°C, HR=%d, SpO2=%.1f%%",
                 band_data.external_temp, band_data.external_humidity, band_data.skin_temp,
                 band_data.heart_rate, band_data.spo2);
    }
    
    return ret;
}

esp_err_t test_simulator_generate_hub_data(void)
{
    if (!g_simulator_enabled) {
        return ESP_OK;
    }
    
    // 허브 데이터 생성
    hub_data_t hub_data = {0};
    
    // 평균소음 (50~80 dB)
    hub_data.avg_noise = 50.0f + (rand() % 300) / 10.0f;
    
    // WBGT (20~35°C)
    hub_data.wbgt = 20.0f + (rand() % 150) / 10.0f;
    
    // 경보상태 (랜덤하게 설정)
    hub_data.alarm_status = rand() % 8; // 0~7 (3비트)
    
    hub_data.timestamp = esp_timer_get_time() / 1000;
    hub_data.is_valid = true;
    
    // 데이터 매니저에 업데이트
    esp_err_t ret = data_manager_update_hub_data(&hub_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Simulated hub data: Noise=%.1fdB, WBGT=%.1f°C, Alarm=0x%02X",
                 hub_data.avg_noise, hub_data.wbgt, hub_data.alarm_status);
    }
    
    return ret;
}

esp_err_t test_simulator_start_periodic_data(void)
{
    if (g_simulator_enabled) {
        ESP_LOGW(TAG, "Simulator already running");
        return ESP_OK;
    }
    
    g_simulator_enabled = true;
    
    // 시뮬레이션 태스크 생성
    BaseType_t ret = xTaskCreate(simulator_task, "simulator_task", 4096, NULL, 3, &g_simulator_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create simulator task");
        g_simulator_enabled = false;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Test simulator started - generating data every 3 seconds");
    return ESP_OK;
}

esp_err_t test_simulator_stop_periodic_data(void)
{
    if (!g_simulator_enabled) {
        ESP_LOGW(TAG, "Simulator not running");
        return ESP_OK;
    }
    
    g_simulator_enabled = false;
    
    // 태스크가 종료될 때까지 대기
    if (g_simulator_task_handle) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        g_simulator_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Test simulator stopped");
    return ESP_OK;
}

void test_simulator_set_enabled(bool enabled)
{
    g_simulator_enabled = enabled;
}

bool test_simulator_is_enabled(void)
{
    return g_simulator_enabled;
}

// 클라이언트 데이터 플래그 초기화 (시뮬레이터 재시작)
void test_simulator_reset_client_data(void)
{
    data_manager_clear_client_data();
    ESP_LOGI(TAG, "Client data flag reset - simulator can resume normal operation");
}

#include "sensor.h"
#include "i2c.h"
#include "data_manager.h"
#include "dfplayer_mini.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "mic_i2s.h"
#include <string.h>
#include <math.h>

static const char *TAG = "SENSOR";

// 경고 시스템 전역 변수들
static warning_info_t warning_list[] = {
    {ALARM_WBGT_TYPE, WARNING_LEVEL_CAUTION, VOICE_WBGT_CAUTION,    "소음 주의: 귀마개를 착용하세요"                          , true, 0},
    {ALARM_WBGT_TYPE, WARNING_LEVEL_CRITICAL, VOICE_WBGT_CRITICAL,  "소음 경고: 즉시 귀마개 착용 후 소음원에서 벗어나세요"       , true, 0},
    {ALARM_TEMP_TYPE, WARNING_LEVEL_CAUTION, VOICE_TEMP_CAUTION,    "소음 위험: 즉시 작업을 중단하고 안전지대로 이동하세요"      , true, 0},
    {ALARM_TEMP_TYPE, WARNING_LEVEL_CRITICAL, VOICE_TEMP_CRITICAL,  "기온 주의: 1시간 후 15분 휴식을 권고합니다"               , true, 0},
    {ALARM_HR_TYPE, WARNING_LEVEL_CAUTION, VOICE_HR_CAUTION,        "기온 위험: 즉시 30분 휴식을 취하세요"                    , true, 0},
    {ALARM_HR_TYPE, WARNING_LEVEL_DANGER, VOICE_HR_DANGER,          "체온 주의: 휴식을 권고합니다"                            , true, 0},
    {ALARM_HR_TYPE, WARNING_LEVEL_CRITICAL, VOICE_HR_CRITICAL,      "체온 위험: 즉시 작업 중단 및 의료 확인이 필요합니다"        , true, 0},
    {ALARM_NOISE_TYPE, WARNING_LEVEL_CAUTION, VOICE_NOISE_CAUTION,  "심박수 주의: 심박수가 높습니다"                           , true, 0},
    {ALARM_NOISE_TYPE, WARNING_LEVEL_DANGER, VOICE_NOISE_DANGER,    "심박수 경고: 휴식을 취하세요"                            , true, 0},
    {ALARM_NOISE_TYPE, WARNING_LEVEL_CRITICAL, VOICE_NOISE_CRITICAL,"심박수 위험: 즉시 중단하고 관리자에게 연락하세요"           , true, 0},
};

#define WARNING_LIST_SIZE (sizeof(warning_list) / sizeof(warning_info_t))
static bool warning_system_initialized = false;
static uint32_t last_warning_check = 0;
static const uint32_t WARNING_CHECK_INTERVAL_MS = 5000; // 5초마다 경고 체크
static const uint32_t WARNING_COOLDOWN_MS = 30000; // 30초 쿨다운

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
    float Tw = Td * atanf(0.151977f * sqrtf(RH + 8.313659f))
               + atanf(Td + RH)
               - atanf(RH - 1.676331f)
               + 0.00391838f * powf(RH, 1.5f) * atanf(0.023101f * RH)
               - 4.686035f;

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
            float temp_float = (float)current_sensor_data.temperature / 10.0f;
            float hum_float = (float)current_sensor_data.humidity / 10.0f;
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%", temp_float, hum_float);
            float wbgt = calc_wbgt(temp_float, hum_float);
            
            // 경보 상태 결정
            uint8_t alarm_status = 0;
            
            // WBGT 경고 (30°C 이상)
            if (wbgt >= 30.0f) {
                alarm_status |= (3<<ALARM_WBGT_WARNING_POS);
            }else if (wbgt >= 28.0f) {
                alarm_status |= (1<<ALARM_WBGT_WARNING_POS);
            }

            if(avg_noise >= 120.0f){
                alarm_status |= (3<<ALARM_NOISE_WARNING_POS);
            }else if(avg_noise >= 110.0f){
                alarm_status |= (2<<ALARM_NOISE_WARNING_POS);
            }else if(avg_noise >= 95.0f){
                alarm_status |= (1<<ALARM_NOISE_WARNING_POS);
            }
            
            // 체온 경고 (밴드 데이터에서 확인)
            band_data_t band_data;
            if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
                if (band_data.skin_temp > 38.0f || band_data.skin_temp < 27.0f) {
                    alarm_status |= (3<<ALARM_TEMP_WARNING_POS);
                }else if (band_data.skin_temp > 37.5f) {
                    alarm_status |= (1<<ALARM_TEMP_WARNING_POS);
                }
                if(band_data.heart_rate != HEART_RATE_IGNORE){
                    if (band_data.heart_rate > 120 || band_data.heart_rate < 50) {
                        alarm_status |= (3<<ALARM_HR_WARNING_POS);
                    }else if (band_data.heart_rate > 100) {
                        alarm_status |= (1<<ALARM_HR_WARNING_POS);
                    }
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
            if (alarm_status & ALARM_WBGT_WARNING_FLAG) {
                ESP_LOGW(TAG, "⚠️  WBGT WARNING: %.1f°C", wbgt);
            }
            if (alarm_status & ALARM_TEMP_WARNING_FLAG) {
                ESP_LOGW(TAG, "⚠️  TEMPERATURE WARNING");
            }
            if (alarm_status & ALARM_HR_WARNING_FLAG) {
                ESP_LOGW(TAG, "⚠️  HEART RATE WARNING");
            }
            if (alarm_status & ALARM_NOISE_WARNING_FLAG) {
                ESP_LOGW(TAG, "⚠️  NOISE WARNING");
            }
            
            // 경고 시스템 체크 및 트리거
            warning_system_check_and_trigger(alarm_status);
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
    
    // 경고 시스템 초기화
    esp_err_t warning_ret = warning_system_init();
    if (warning_ret != ESP_OK) {
        ESP_LOGW(TAG, "Warning system initialization failed: %s", esp_err_to_name(warning_ret));
    }
    
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

// 경고 시스템 초기화
esp_err_t warning_system_init(void)
{
    ESP_LOGI(TAG, "Initializing warning system");
    /*
    // 진동 모터 GPIO 설정 (GPIO 2번 핀 사용)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << VIBRATION_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(VIBRATION_GPIO, 0); // 초기 상태: 꺼짐
    
    // DFPlayer 초기화 (UART1, TX: GPIO 17, RX: GPIO 16)
    esp_err_t ret = dfplayer_init(UART_NUM_1, 17, 16, 9600);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "DFPlayer initialization failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "DFPlayer initialized successfully");
    }
        */
    
    warning_system_initialized = true;
    last_warning_check = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "Warning system initialized");
    return ESP_OK;
}
bool music_playing = false;
// 음성 재생
esp_err_t warning_system_play_voice(uint8_t voice_file_num)
{
    if (!warning_system_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if(music_playing){
        if(voice_file_num >= 10 && voice_file_num <= 19){
            ESP_LOGI(TAG, "Music is playing, skipping voice file: %d", voice_file_num);
            return ESP_OK;
        }
        ESP_LOGI(TAG, "Music is playing, stopping music");
        esp_err_t ret = dfplayer_play_folder(0, 2); //ding, music stop
        music_playing = false;
        return ESP_OK;
    }else{
        ESP_LOGI(TAG, "Playing voice file: %d", voice_file_num);
        esp_err_t ret = dfplayer_play_folder(0, voice_file_num);
    
        if(!(voice_file_num >= 10 && voice_file_num <= 19)){
            music_playing = true;
        }
        return ret;
    }

}

// 진동 알림
esp_err_t warning_system_vibrate(uint32_t duration_ms)
{
    if (!warning_system_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Vibrating for %lu ms", duration_ms);
    
    // 진동 시작
    gpio_set_level(VIBRATION_GPIO, 1);
    
    // 지정된 시간 후 진동 중지
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    gpio_set_level(VIBRATION_GPIO, 0);
    
    return ESP_OK;
}

// 경고 시스템 체크 및 트리거
esp_err_t warning_system_check_and_trigger(uint8_t alarm_status)
{
    if (!warning_system_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t current_time = esp_timer_get_time() / 1000;
    
    // 경고 체크 간격 확인
    if (current_time - last_warning_check < WARNING_CHECK_INTERVAL_MS) {
        return ESP_OK;
    }
    
    last_warning_check = current_time;
    
    // 현재 센서 데이터 가져오기
    band_data_t band_data;
    hub_data_t hub_data;
    bool has_band_data = (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid);
    bool has_hub_data = (data_manager_get_hub_data(&hub_data) == ESP_OK && hub_data.is_valid);
    
    bool warning_triggered = false;
    
    // 각 경고 조건 체크
    for (int i = 0; i < WARNING_LIST_SIZE; i++) {
        warning_info_t* warning = &warning_list[i];
        
        // 쿨다운 체크
        if (warning->is_active && (current_time - warning->last_triggered) < WARNING_COOLDOWN_MS) {
            continue;
        }
        
        bool should_trigger = false;
        int level = 0;
        switch (warning->warning_type) {
            case ALARM_WBGT_TYPE:
                level = (alarm_status & ALARM_WBGT_WARNING_FLAG) >> ALARM_WBGT_WARNING_POS;
                if(level == warning->warning_level){
                    should_trigger = true;
                }
                break;
                
            case ALARM_TEMP_TYPE:
                level = (alarm_status & ALARM_TEMP_WARNING_FLAG) >> ALARM_TEMP_WARNING_POS;
                if(level == warning->warning_level){
                    should_trigger = true;
                }
                break;
                
            case ALARM_HR_TYPE:
                level = (alarm_status & ALARM_HR_WARNING_FLAG) >> ALARM_HR_WARNING_POS;
                if(level == warning->warning_level){
                    should_trigger = true;
                }
                break;
                
            case ALARM_NOISE_TYPE:
                level = (alarm_status & ALARM_NOISE_WARNING_FLAG) >> ALARM_NOISE_WARNING_POS;
                if(level == warning->warning_level){
                    should_trigger = true;
                }
                break;
        }
        
        if (should_trigger) {
            ESP_LOGW(TAG, "⚠️ WARNING TRIGGERED: %s", warning->message);
            
            
            // 진동 알림 (1초)
            warning_system_vibrate(1000);
            // 음성 알림 재생
            warning_system_play_voice(warning->voice_file_num);
            
            // 경고 상태 업데이트
            warning->is_active = true;
            warning->last_triggered = current_time;
            warning_triggered = true;
        } else {
            // 조건이 해제되면 비활성화
            warning->is_active = false;
        }
    }
    
    if (warning_triggered) {
        ESP_LOGI(TAG, "Warning system triggered - voice and vibration alerts sent");
    }
    
    return ESP_OK;
}

// 모든 경고 리셋
void warning_system_reset_all(void)
{
    for (int i = 0; i < WARNING_LIST_SIZE; i++) {
        warning_list[i].is_active = false;
        warning_list[i].last_triggered = 0;
    }
    ESP_LOGI(TAG, "All warnings reset");
}

#include "analog.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ANALOG";

// 전역 변수 정의
QueueHandle_t heart_rate_queue = NULL;
TaskHandle_t heart_rate_task_handle = NULL;
bool heart_rate_measurement_active = false;

// ADC 관련 변수
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle = NULL;
static bool do_calibration1 = false;

// 심박수 측정 관련 변수
static uint16_t adc_buffer[BUFFER_SIZE];
static uint32_t peak_timestamps[100];  // 최대 100개의 피크 시간 저장
static uint16_t peak_count = 0;
static uint32_t last_measurement_time = 0;

// ADC 캘리브레이션 함수
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "ADC Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

// ADC 초기화 함수
static esp_err_t adc_init(void)
{
    esp_err_t ret = ESP_OK;

    // ADC1 초기화
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 unit init failed");
        return ret;
    }

    // ADC 채널 설정 (더 민감한 설정)
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,  // 12비트 해상도로 최대 민감도
        .atten = ADC_ATTEN_DB_12,     // 최대 입력 범위 (0-3.3V)
    };
    ret = adc_oneshot_config_channel(adc1_handle, HEART_RATE_SENSOR_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed");
        return ret;
    }

    // ADC 캘리브레이션 초기화
    do_calibration1 = adc_calibration_init(ADC_UNIT_1, HEART_RATE_SENSOR_CHANNEL, ADC_ATTEN_DB_12, &adc1_cali_handle);

    return ESP_OK;
}

// 아날로그 값 읽기 함수
static esp_err_t read_analog_value(int *adc_raw, int *voltage)
{
    esp_err_t ret = ESP_OK;

    // ADC 원시값 읽기
    ret = adc_oneshot_read(adc1_handle, HEART_RATE_SENSOR_CHANNEL, adc_raw);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed");
        return ret;
    }

    // 캘리브레이션된 전압값 계산
    if (do_calibration1) {
        ret = adc_cali_raw_to_voltage(adc1_cali_handle, *adc_raw, voltage);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ADC calibration failed");
            return ret;
        }
    } else {
        *voltage = *adc_raw;
    }

    return ESP_OK;
}

// 신호 품질 평가 함수
static uint16_t calculate_signal_quality(uint16_t *buffer, uint16_t size)
{
    if (size == 0) return 0;

    // 1. 신호 범위 계산 (최대값 - 최소값)
    uint16_t max_val = 0;
    uint16_t min_val = 65535;
    
    for (int i = 0; i < size; i++) {
        if (buffer[i] > max_val) max_val = buffer[i];
        if (buffer[i] < min_val) min_val = buffer[i];
    }
    
    uint16_t signal_range = max_val - min_val;
    
    // 2. 신호의 표준편차 계산
    uint32_t sum = 0;
    uint32_t sum_sq = 0;
    
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
        sum_sq += (uint32_t)buffer[i] * buffer[i];
    }
    
    uint32_t mean = sum / size;
    uint32_t variance = (sum_sq / size) - (mean * mean);
    uint32_t std_dev = (uint32_t)sqrt(variance);
    
    // 3. 피크 개수 기반 품질 (심박수 측정 가능성)
    uint16_t peak_quality = 0;
    if (peak_count >= 2) {
        // 최소 2개 이상의 피크가 있어야 심박수 측정 가능
        peak_quality = (peak_count > 10) ? 100 : (peak_count * 10);
    }
    
    // 4. 종합 품질 계산
    // - 신호 범위: 30% (충분한 변동 필요)
    // - 표준편차: 30% (적절한 노이즈)
    // - 피크 개수: 40% (심박수 측정 가능성)
    
    uint16_t range_quality = (signal_range * 30) / 4095;
    uint16_t std_quality = (std_dev * 30) / 4095;
    
    uint16_t total_quality = range_quality + std_quality + peak_quality;
    
    // 5. 품질 범위 제한 (0-100)
    if (total_quality > 100) total_quality = 100;
    
    return total_quality;
}

// 피크 감지 함수 (개선된 버전)
static void detect_peaks(uint16_t *buffer, uint16_t size, uint32_t sample_rate)
{
    if (size < 5) return;

    peak_count = 0;
    uint16_t max_val = 0;
    uint16_t min_val = 65535;

    // 최대값과 최소값 찾기
    for (int i = 0; i < size; i++) {
        if (buffer[i] > max_val) max_val = buffer[i];
        if (buffer[i] < min_val) min_val = buffer[i];
    }

    uint16_t signal_range = max_val - min_val;
    
    // 신호가 너무 작으면 증폭
    if (signal_range < 100) {
        // 신호 증폭 (디지털 증폭)
        for (int i = 0; i < size; i++) {
            uint32_t amplified = (uint32_t)(buffer[i] - min_val) * SIGNAL_AMPLIFICATION + min_val;
            buffer[i] = (amplified > 65535) ? 65535 : (uint16_t)amplified;
        }
        // 증폭 후 다시 범위 계산
        max_val = 0;
        min_val = 65535;
        for (int i = 0; i < size; i++) {
            if (buffer[i] > max_val) max_val = buffer[i];
            if (buffer[i] < min_val) min_val = buffer[i];
        }
        signal_range = max_val - min_val;
    }

    // 적응형 임계값 설정
    uint16_t threshold;
    if (signal_range < 50) {
        // 신호가 매우 작을 때 더 낮은 임계값 사용
        threshold = min_val + (uint16_t)(signal_range * 0.2);
    } else if (signal_range < 200) {
        // 중간 신호 강도
        threshold = min_val + (uint16_t)(signal_range * PEAK_THRESHOLD);
    } else {
        // 강한 신호
        threshold = min_val + (uint16_t)(signal_range * 0.4);
    }

    uint32_t min_interval_samples = (uint32_t)(MIN_PEAK_INTERVAL * sample_rate);
    uint32_t last_peak_time = 0;

    // 개선된 피크 감지 (더 민감한 알고리즘)
    for (int i = 2; i < size - 2; i++) {
        // 현재 값이 임계값보다 크고, 주변 값들보다 큰지 확인
        if (buffer[i] > threshold && 
            buffer[i] > buffer[i-1] && buffer[i] > buffer[i-2] &&
            buffer[i] > buffer[i+1] && buffer[i] > buffer[i+2]) {
            
            uint32_t current_time = (uint32_t)(i * 1000 / sample_rate); // ms 단위
            
            if (current_time - last_peak_time >= min_interval_samples) {
                if (peak_count < 100) {
                    peak_timestamps[peak_count++] = current_time;
                    last_peak_time = current_time;
                }
            }
        }
    }

    // 피크가 너무 적으면 임계값을 더 낮춰서 재시도
    if (peak_count < 2 && signal_range > 20) {
        threshold = min_val + (uint16_t)(signal_range * 0.15); // 더 낮은 임계값
        
        for (int i = 2; i < size - 2; i++) {
            if (buffer[i] > threshold && 
                buffer[i] > buffer[i-1] && buffer[i] > buffer[i-2] &&
                buffer[i] > buffer[i+1] && buffer[i] > buffer[i+2]) {
                
                uint32_t current_time = (uint32_t)(i * 1000 / sample_rate);
                
                if (current_time - last_peak_time >= min_interval_samples) {
                    if (peak_count < 100) {
                        peak_timestamps[peak_count++] = current_time;
                        last_peak_time = current_time;
                    }
                }
            }
        }
    }
}

// 심박수 계산 함수
static uint16_t calculate_heart_rate(void)
{
    if (peak_count < 2) return 0;

    // 피크 간격들의 평균 계산
    uint32_t total_interval = 0;
    uint16_t valid_intervals = 0;

    for (int i = 1; i < peak_count; i++) {
        uint32_t interval = peak_timestamps[i] - peak_timestamps[i-1];
        if (interval > 0) {
            total_interval += interval;
            valid_intervals++;
        }
    }

    if (valid_intervals == 0) return 0;

    uint32_t avg_interval_ms = total_interval / valid_intervals;
    uint16_t heart_rate = (uint16_t)(60000 / avg_interval_ms); // BPM 계산

    // 유효한 심박수 범위 확인
    if (heart_rate >= MIN_HEART_RATE && heart_rate <= MAX_HEART_RATE) {
        return heart_rate;
    }

    return 0;
}

// 심박수 측정 태스크
static void heart_rate_measurement_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Heart rate measurement task started");
    
    uint16_t buffer_index = 0;
    const uint32_t sample_interval_ms = 1000 / ADC_SAMPLE_RATE;
    const TickType_t sample_interval_ticks = pdMS_TO_TICKS(sample_interval_ms);
    uint32_t sample_count = 0;
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "Sample interval: %lu ms (%lu ticks)", sample_interval_ms, sample_interval_ticks);

    // 최소 1 tick 이상인지 확인
    if (sample_interval_ticks < 1) {
        ESP_LOGE(TAG, "Sample interval too small: %lu ms (%lu ticks)", sample_interval_ms, (unsigned long)sample_interval_ticks);
        vTaskDelete(NULL);
        return;
    }

    while (heart_rate_measurement_active) {
        // 정확한 타이밍을 위해 vTaskDelayUntil 사용
        vTaskDelayUntil(&last_wake_time, sample_interval_ticks);
        
        int adc_raw, voltage;
        
        if (read_analog_value(&adc_raw, &voltage) == ESP_OK) {
            adc_buffer[buffer_index] = (uint16_t)adc_raw;
            buffer_index++;
            sample_count++;

            // 버퍼가 가득 찼을 때 심박수 계산
            if (buffer_index >= BUFFER_SIZE) {
                // 피크 감지
                detect_peaks(adc_buffer, BUFFER_SIZE, ADC_SAMPLE_RATE);
                
                // 심박수 계산
                uint16_t heart_rate = calculate_heart_rate();
                uint16_t signal_quality = calculate_signal_quality(adc_buffer, BUFFER_SIZE);
                
                // 디버깅 정보 출력 (신호 상태 모니터링)
                uint16_t max_val = 0, min_val = 65535;
                for (int i = 0; i < BUFFER_SIZE; i++) {
                    if (adc_buffer[i] > max_val) max_val = adc_buffer[i];
                    if (adc_buffer[i] < min_val) min_val = adc_buffer[i];
                }
                uint16_t signal_range = max_val - min_val;
                ESP_LOGI(TAG, "Signal: min=%d, max=%d, range=%d, peaks=%d, quality=%d%%", 
                        min_val, max_val, signal_range, peak_count, signal_quality);
                
                // 결과 구조체 생성 (타임스탬프는 샘플 카운트 기반으로 계산)
                heart_rate_data_t heart_rate_data = {
                    .heart_rate = heart_rate,
                    .signal_quality = signal_quality,
                    .is_valid = (heart_rate > 0),
                    .timestamp = (uint32_t)(sample_count * sample_interval_ms)
                };

                // 큐에 결과 전송
                if (heart_rate_queue != NULL) {
                    xQueueSend(heart_rate_queue, &heart_rate_data, 0);
                }

                // 버퍼 초기화
                buffer_index = 0;
                memset(adc_buffer, 0, sizeof(adc_buffer));
            }
        }
    }

    ESP_LOGI(TAG, "Heart rate measurement task stopped");
    vTaskDelete(NULL);
}

// 공개 함수 구현

esp_err_t analog_init(void)
{
    ESP_LOGI(TAG, "Initializing analog module");

    // ADC 초기화
    esp_err_t ret = adc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC initialization failed");
        return ret;
    }

    // 심박수 측정 큐 생성
    heart_rate_queue = xQueueCreate(10, sizeof(heart_rate_data_t));
    if (heart_rate_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create heart rate queue");
        return ESP_ERR_NO_MEM;
    }

    // 버퍼 초기화
    memset(adc_buffer, 0, sizeof(adc_buffer));
    memset(peak_timestamps, 0, sizeof(peak_timestamps));
    peak_count = 0;

    ESP_LOGI(TAG, "Analog module initialized successfully");
    return ESP_OK;
}

void heart_rate_measurement_start(void)
{
    if (heart_rate_measurement_active) {
        ESP_LOGW(TAG, "Heart rate measurement already active");
        return;
    }

    heart_rate_measurement_active = true;
    
    // 심박수 측정 태스크 생성
    BaseType_t ret = xTaskCreate(
        heart_rate_measurement_task,
        "heart_rate_task",
        HEART_RATE_TASK_STACK_SIZE,
        NULL,
        HEART_RATE_TASK_PRIORITY,
        &heart_rate_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heart rate measurement task");
        heart_rate_measurement_active = false;
        return;
    }

    ESP_LOGI(TAG, "Heart rate measurement started");
}

void heart_rate_measurement_stop(void)
{
    if (!heart_rate_measurement_active) {
        ESP_LOGW(TAG, "Heart rate measurement not active");
        return;
    }

    heart_rate_measurement_active = false;

    if (heart_rate_task_handle != NULL) {
        vTaskDelete(heart_rate_task_handle);
        heart_rate_task_handle = NULL;
    }

    ESP_LOGI(TAG, "Heart rate measurement stopped");
}

esp_err_t get_heart_rate(heart_rate_data_t *heart_rate_data)
{
    if (heart_rate_queue == NULL || heart_rate_data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 큐에서 최신 심박수 데이터 가져오기
    if (xQueueReceive(heart_rate_queue, heart_rate_data, pdMS_TO_TICKS(100)) == pdTRUE) {
        return ESP_OK;
    }

    // 큐가 비어있으면 기본값 반환
    heart_rate_data->heart_rate = 0;
    heart_rate_data->signal_quality = 0;
    heart_rate_data->is_valid = false;
    heart_rate_data->timestamp = 0;

    return ESP_ERR_NOT_FOUND;
} 
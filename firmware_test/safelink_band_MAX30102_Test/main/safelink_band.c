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
#include <math.h>
#include "dsps_fft2r.h"

// Custom includes
#include "i2c_driver.h"
#include "max3020x.h"

static const char *TAG = "SAFELINK_BAND";

// 심박수 피크 검출을 위한 상수
#define PEAK_DETECTION_BUFFER_SIZE 500  // 50Hz * 10초 = 500샘플
#define MIN_PEAK_INTERVAL 150           // 최소 피크 간격 (200 BPM 상한 = 300ms)
#define REFRACTORY_PERIOD 250           // 불응기 (250ms)
#define SMOOTHING_WINDOW 5              // 스무딩을 위한 윈도우 크기
#define MAX_BPM_JUMP 15                 // 최대 BPM 점프 제한

// Global variables
static EventGroupHandle_t sensor_event_group = NULL;
void buffer_task(void *arg);

void print_buffer_ready(fifo_sample_2slot_t* printbuffer, int n_data);
void analyze_buffer_ready(fifo_sample_2slot_t* analyzebuffer, int focus, int n_data);

// 경량 시간영역 피크 검출 함수들
void remove_dc_component(float* buffer, int size);
void bandpass_filter(float* buffer, float* output, int size);
void downsample_buffer(float* input, float* output, int input_size, int output_size);
float calculate_rms_threshold(float* buffer, int size, int window_size);
int detect_peaks(float* buffer, int size, float threshold, int* peak_indices, int max_peaks);
float calculate_bpm_from_peaks(int* peak_indices, int peak_count, float sample_rate);
float smooth_bpm(float new_bpm, float* bpm_history, int* history_index, int history_size);
bool is_realistic_bpm_jump(float old_bpm, float new_bpm);

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
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return;
    }
    ret = i2c_max3020x_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C MAX3020X initialization failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return;
    }
    if(!is_Part_ID_valid()) {
        ESP_LOGE(TAG, "I2C Part ID is not valid");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return;
    }
    else ESP_LOGI(TAG, "Addr 0x%02X is MAX30102(0x%02X)", MAX30102_I2C_ADDR, read_Part_ID());

    if (start_sensor_task() != pdPASS) {
        ESP_LOGE(TAG, "Failed to create sensor task");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return;
    }

    TaskHandle_t buffer_task_handle = NULL;
    BaseType_t task_created = xTaskCreate(buffer_task, "BUFFER_TASK", 8192, NULL, 3, &buffer_task_handle);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create buffer task");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
        return;
    }



}
void analyze_buffer_task(void *arg);

void buffer_task(void *arg){
    ESP_LOGI(TAG, "Buffer task started");
    
    TaskHandle_t analyze_buffer_task_handle = NULL;
    BaseType_t task_created = xTaskCreate(analyze_buffer_task, "analyze_buffer_task", 8192, NULL, 3, &analyze_buffer_task_handle);
    if (task_created != pdPASS) {
        ESP_LOGE(TAG, "Failed to create analyze buffer task");
        vTaskDelay(pdMS_TO_TICKS(5000));
        esp_restart();
        return;
    }

    while(1){
        read_printData(print_buffer_ready, analyze_buffer_ready);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void print_buffer_ready(fifo_sample_2slot_t* printbuffer, int n_data){
}

float pulse_buffer[DATA_SIZE];

// 경량 시간영역 피크 검출을 위한 새로운 배열들
float pulse_downsampled_buffer[DATA_SIZE];  // 다운샘플된 데이터 (50Hz)
float pulse_filtered_buffer[DATA_SIZE];  // 다운샘플된 데이터 (50Hz)
int peak_indices[20];  // 피크 인덱스 저장 (최대 20개)
float bpm_history[SMOOTHING_WINDOW];  // BPM 스무딩을 위한 히스토리
int bpm_history_index = 0;  // BPM 히스토리 인덱스
float current_bpm = 0.0f;  // 현재 BPM
float last_peak_time = 0.0f;  // 마지막 피크 시간
bool isAnalysing = false;
void analyze_buffer_ready(fifo_sample_2slot_t* analyzebuffer, int focus, int n_data){  

    static int ndata_count = 0;
    static int ndata_prev_focus = 0;

    ndata_count += n_data;
    if(ndata_count < DATA_SIZE){
        return;
        // data collecting
    }
    if(true/*ndata_prev_focus > focus*/){

        if(!isAnalysing)
        {
            // convert sample to float array
            int idx = 0;
            for(int i = focus; i < DATA_SIZE; i++,idx++){
                pulse_buffer[idx] = (analyzebuffer[i].slot1[2] | (analyzebuffer[i].slot1[1] << 8) | (analyzebuffer[i].slot1[0] << 16)) / 10.f;
            }
            for(int i = 0; i < focus; i++,idx++){
                pulse_buffer[idx] = (analyzebuffer[i].slot1[2] | (analyzebuffer[i].slot1[1] << 8) | (analyzebuffer[i].slot1[0] << 16)) / 10.f;
            }
            
    
            isAnalysing = true;
        }
    }
    ndata_prev_focus = focus;


}

//float pulse_movingavg_buffer[DATA_SIZE];
float pulse_fft_buffer[DATA_SIZE];
float pulse_fft_magnitude[DATA_SIZE];

// ===== 경량 시간영역 피크 검출 함수들 =====

float pulse_filtered_buffer_2[DATA_SIZE];  // 다운샘플된 데이터 (50Hz)
// DC 성분 제거 (평균값을 빼기)
void remove_dc_component(float* buffer, int size) {
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    float mean = sum / size;
    for (int i = 0; i < size; i++) {
        buffer[i] -= mean;
    }


    // 2차 IIR Butterworth 필터 계수 (0.5-5 Hz, 50Hz 샘플링)
    static const float b0 = 0.0201f;
    static const float b1 = 0.0f;
    static const float b2 = -0.0402f;
    static const float a1 = -1.5615f;
    static const float a2 = 0.6397f;
    
    static float x1 = 0.0f, x2 = 0.0f;
    static float y1 = 0.0f, y2 = 0.0f;
    
    for (int i = 0; i < size; i++) {
        float x0 = buffer[i];
        float y0 = b0 * x0 + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        buffer[i] = y0;
        pulse_filtered_buffer_2[i] = y0;
        
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
    }

    
}

void bandpass_filter(float* buffer, float* output, int size) {
    // 이동평균 필터
    // 160Hz => 1/160s = k/data_rate sec , k = data_rate/f 
    int window_size = 60;
    float sum = 0;
    for(int i = 0; i < window_size; i++){
        sum += buffer[i];
    }

    for (int i = 0; i < size; i++) {
        if(i>=window_size){
            sum += buffer[i] - buffer[i-window_size];
        }
        output[i] = sum / window_size;
    }
    
}

// 다운샘플링 (200Hz -> 50Hz, 4:1 비율)
void downsample_buffer(float* input, float* output, int input_size, int output_size) {
    for (int i = 0; i < output_size; i++) {
        output[i] = input[i * 4];  // 4개 샘플마다 하나씩 선택
    }
}

// RMS 기반 적응 임계치 계산
float calculate_rms_threshold(float* buffer, int size, int window_size) {
    float sum_squares = 0.0f;
    int start_idx = (size > window_size) ? (size - window_size) : 0;
    
    for (int i = start_idx; i < size; i++) {
        sum_squares += buffer[i] * buffer[i];
    }
    
    float rms = sqrtf(sum_squares / (size - start_idx));
    return rms * 1.5f;  // RMS의 1.5배를 임계치로 사용
}

// 피크 검출 (적응 임계치 + 불응기)
int detect_peaks(float* buffer, int size, float threshold, int* peak_indices, int max_peaks) {
    int peak_count = 0;
    int last_peak_idx = -1;
    
    for (int i = 1; i < size - 1; i++) {
        // 피크 조건: 이전과 다음 샘플보다 큰 값
        if (buffer[i] > buffer[i-1] && buffer[i] > buffer[i+1] && buffer[i] > threshold) {
            // 불응기 체크 (최소 간격)
            if (last_peak_idx == -1 || (i - last_peak_idx) >= MIN_PEAK_INTERVAL) {
                if (peak_count < max_peaks) {
                    peak_indices[peak_count] = i;
                    peak_count++;
                    last_peak_idx = i;
                }
            }
        }
    }
    
    return peak_count;
}

// 피크 간격으로부터 BPM 계산
float calculate_bpm_from_peaks(int* peak_indices, int peak_count, float sample_rate) {
    if (peak_count < 2) return 0.0f;
    
    float total_ibi = 0.0f;
    int valid_intervals = 0;
    
    for (int i = 1; i < peak_count; i++) {
        int interval = peak_indices[i] - peak_indices[i-1];
        float ibi_seconds = interval / sample_rate;
        
        // 0.3초 ~ 2초 (20-200 BPM 범위) 체크
        if (ibi_seconds >= 0.3f && ibi_seconds <= 2.0f) {
            total_ibi += ibi_seconds;
            valid_intervals++;
        }
    }
    
    if (valid_intervals == 0) return 0.0f;
    
    float avg_ibi = total_ibi / valid_intervals;
    return 60.0f / avg_ibi;  // BPM = 60/IBI
}

// BPM 스무딩 (이동평균)
float smooth_bpm(float new_bpm, float* bpm_history, int* history_index, int history_size) {
    if (new_bpm <= 0.0f) return 0.0f;
    
    bpm_history[*history_index] = new_bpm;
    *history_index = (*history_index + 1) % history_size;
    
    float sum = 0.0f;
    int valid_count = 0;
    
    for (int i = 0; i < history_size; i++) {
        if (bpm_history[i] > 0.0f) {
            sum += bpm_history[i];
            valid_count++;
        }
    }
    
    return (valid_count > 0) ? (sum / valid_count) : 0.0f;
}

// 비현실적인 BPM 점프 체크
bool is_realistic_bpm_jump(float old_bpm, float new_bpm) {
    if (old_bpm <= 0.0f) return true;  // 첫 번째 값은 항상 허용
    return fabsf(new_bpm - old_bpm) <= MAX_BPM_JUMP;
}

// ESP-DSP를 사용한 FFT 계산 함수 (기존 유지)
void esp_dsp_fft(float* input, float* magnitude,int count) {
    
    // ESP-DSP FFT 초기화
    esp_err_t ret = dsps_fft2r_init_fc32(NULL, count);
    if (ret != ESP_OK) {
        ESP_LOGE("FFT", "Failed to initialize FFT. Error = %i", ret);
        return;
    }
    
    // 복소수 배열 준비 (실수부만 사용, 허수부는 0)
    float* complex_input = (float*)malloc(count * 2 * sizeof(float));
    if (complex_input == NULL) {
        ESP_LOGE("FFT", "Failed to allocate memory for FFT");
        return;
    }
    
    // 입력 데이터를 복소수 형태로 변환
    for (int i = 0; i < count; i++) {
        complex_input[i * 2] = input[i];     // 실수부
        complex_input[i * 2 + 1] = 0.0f;    // 허수부
    }
    
    // FFT 계산
    dsps_fft2r_fc32(complex_input, count);
    
    // 비트 리버스
    dsps_bit_rev_fc32(complex_input, count);
    
    // 복소수에서 실수부로 변환
    dsps_cplx2reC_fc32(complex_input, count);
    
    // 크기 계산
    for (int k = 0; k < count/2; k++) {
        float real = complex_input[k * 2];
        float imag = complex_input[k * 2 + 1];
        magnitude[k] = sqrtf(real * real + imag * imag);
    }
    
    // 메모리 해제
    free(complex_input);
}


void analyze_buffer_task(void *arg)
{
    // BPM 히스토리 초기화
    memset(bpm_history, 0, sizeof(bpm_history));
    
    while(true){
        if(!isAnalysing) vTaskDelay(pdMS_TO_TICKS(10));
        else{
            ESP_LOGI(TAG, "Processing %d samples with peak detection", DATA_SIZE);
            
            memcpy(pulse_downsampled_buffer, pulse_buffer, sizeof(pulse_downsampled_buffer));

            // 1. DC 성분 제거
            remove_dc_component(pulse_downsampled_buffer, DATA_SIZE);
            
            // 2. 밴드패스 필터 적용 (0.5-5 Hz)
            bandpass_filter(pulse_downsampled_buffer, pulse_filtered_buffer, DATA_SIZE);
            
            
            // 4. 적응 임계치 계산 (최근 2-3초 RMS)
            float threshold = calculate_rms_threshold(pulse_filtered_buffer, DATA_SIZE, 150); // 3초
            
            // 5. 피크 검출
            int peak_count = detect_peaks(pulse_filtered_buffer, DATA_SIZE, threshold, peak_indices, 20);
            
            // 6. BPM 계산
            float new_bpm = 0.0f;
            if (peak_count >= 2) {
                new_bpm = calculate_bpm_from_peaks(peak_indices, peak_count, get_data_rate()); // 50Hz 샘플링
                
                // 7. 비현실적인 점프 체크
                if (is_realistic_bpm_jump(current_bpm, new_bpm)) {
                    // 8. BPM 스무딩
                    float smoothed_bpm = smooth_bpm(new_bpm, bpm_history, &bpm_history_index, SMOOTHING_WINDOW);
                    current_bpm = smoothed_bpm;
                }
            }
            
            // 결과 출력
            ESP_LOGI(TAG, "Peak Detection Results:");
            ESP_LOGI(TAG, "  Threshold: %.2f", threshold);
            ESP_LOGI(TAG, "  Peaks detected: %d", peak_count);
            ESP_LOGI(TAG, "  Current BPM: %.1f", current_bpm);
            ESP_LOGI(TAG, "  Data Rate: %.1f", get_data_rate());
            
            // 피크 위치 출력 (디버깅용)
            for (int i = 0; i < peak_count; i++) {
                ESP_DRAM_LOGI(TAG, "  Peak %d at index: %d, value: %5ld", 
                    i, peak_indices[i], (long)pulse_filtered_buffer[peak_indices[i]]);
            }
            
            // 원본 데이터와 필터링된 데이터 비교 출력 (일부만)
            ESP_DRAM_LOGI("buffer start", "`");
            for (int i = 0; i < DATA_SIZE; i++) {
                ESP_DRAM_LOGI("safelink_band","FLASH 버퍼 ;  %5d, %5ld, %5ld\\", 
                    i*2,
                    (long)pulse_filtered_buffer_2[i], 
                    (long)pulse_filtered_buffer[i]);
            }
            ESP_DRAM_LOGI("buffer end", "~");
            
            isAnalysing = false;
        }
    }
}
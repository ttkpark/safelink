#ifndef ANALOG_H
#define ANALOG_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// 심박 센서 설정
#define HEART_RATE_SENSOR_GPIO ADC_CHANNEL_4  // GPIO4
#define HEART_RATE_SENSOR_CHANNEL ADC_CHANNEL_4
#define HEART_RATE_SENSOR_UNIT ADC_UNIT_1

// 아날로그 샘플링 설정
#define ADC_SAMPLE_RATE 100  // 100Hz 샘플링 (10ms 간격)
#define BUFFER_SIZE 100       // 1초간의 데이터
#define HEART_RATE_TASK_STACK_SIZE 4096
#define HEART_RATE_TASK_PRIORITY 5

// 심박수 측정 설정
#define MIN_HEART_RATE 40     // 최소 심박수 (BPM)
#define MAX_HEART_RATE 200    // 최대 심박수 (BPM)
#define PEAK_THRESHOLD 0.3    // 피크 감지 임계값 (전체 신호 대비) - 감도 향상
#define MIN_PEAK_INTERVAL 0.2 // 최소 피크 간격 (초) - 더 빠른 심박수 감지
#define SIGNAL_AMPLIFICATION 2 // 신호 증폭 배수

// 심박수 측정 결과 구조체
typedef struct {
    uint16_t heart_rate;      // 심박수 (BPM)
    uint16_t signal_quality;  // 신호 품질 (0-100)
    bool is_valid;           // 유효한 측정값 여부
    uint32_t timestamp;      // 측정 시간
} heart_rate_data_t;

// 전역 변수 선언
extern QueueHandle_t heart_rate_queue;
extern TaskHandle_t heart_rate_task_handle;
extern bool heart_rate_measurement_active;

// 공개 함수 선언
esp_err_t analog_init(void);
void heart_rate_measurement_start(void);
void heart_rate_measurement_stop(void);
esp_err_t get_heart_rate(heart_rate_data_t *heart_rate_data);

#endif // ANALOG_H 
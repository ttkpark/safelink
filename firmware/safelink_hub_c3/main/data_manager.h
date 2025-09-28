#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <stdbool.h>

// 밴드로부터 받는 데이터 (GATT Subscribe)
typedef struct {
    float external_temp;      // 외기 온도 (°C)
    float external_humidity;  // 외기 습도 (%)
    float skin_temp;          // 피부온도 (15.0~40.0°C)
    uint16_t heart_rate;      // 심박수 (20~160 BPM)
    float spo2;              // 혈중산소포화농도 (80.0~99.9%)
    uint32_t timestamp;      // 타임스탬프
    bool is_valid;           // 데이터 유효성
    uint8_t data_source;     // 데이터 소스 (DATA_SOURCE_*)
} band_data_t;

// 허브에서 자체 수집하는 데이터 (GATT Publishing)
typedef struct {
    float avg_noise;         // 평균소음 (dB)
    float wbgt;              // WBGT(건구습구온도 추정치) (°C)
    uint8_t alarm_status;    // 경보상태 (비트마스크)
    uint32_t timestamp;      // 타임스탬프
    bool is_valid;           // 데이터 유효성
} hub_data_t;

// 경보상태 비트 정의
/*#define ALARM_WBGT_WARNING_FLAG    0x03    // WBGT 경고
#define ALARM_TEMP_WARNING_FLAG    0x0C    // 체온 경고
#define ALARM_HR_WARNING_FLAG      0x30    // 심박수 경고
#define ALARM_NOISE_WARNING_FLAG   0xC0    // 소음 경고
#define ALARM_WBGT_WARNING_POS     0
#define ALARM_TEMP_WARNING_POS     2
#define ALARM_HR_WARNING_POS       4
#define ALARM_NOISE_WARNING_POS    6
#define ALARM_WBGT_WARNING(x)      (x << ALARM_WBGT_WARNING_POS)
#define ALARM_TEMP_WARNING(x)      (x << ALARM_TEMP_WARNING_POS)
#define ALARM_HR_WARNING(x)        (x << ALARM_HR_WARNING_POS)
#define ALARM_NOISE_WARNING(x)     (x << ALARM_NOISE_WARNING_POS)*/

#define ALARM_WBGT_WARNING_FLAG    0x03    // WBGT 경고
#define ALARM_TEMP_WARNING_FLAG    0x0C    // 체온 경고
#define ALARM_HR_WARNING_FLAG      0x30    // 심박수 경고
#define ALARM_NOISE_WARNING_FLAG   0xC0    // 소음 경고
#define ALARM_WBGT_WARNING_POS     0
#define ALARM_TEMP_WARNING_POS     2
#define ALARM_HR_WARNING_POS       4
#define ALARM_NOISE_WARNING_POS    6

// 데이터 범위 검증 상수
#define SKIN_TEMP_MIN         15.0f
#define SKIN_TEMP_MAX         40.0f
#define HEART_RATE_IGNORE     0
#define HEART_RATE_MIN        20
#define HEART_RATE_MAX        160
#define SPO2_MIN              80.0f
#define SPO2_MAX              99.9f

// 데이터 소스 플래그
#define DATA_SOURCE_SIMULATOR    0x01
#define DATA_SOURCE_CLIENT       0x02

// 함수 선언
esp_err_t data_manager_init(void);
esp_err_t data_manager_deinit(void);

// 밴드 데이터 관리 (GATT Subscribe)
esp_err_t data_manager_update_band_data(const band_data_t *data);
esp_err_t data_manager_get_band_data(band_data_t *data);
bool data_manager_validate_band_data(const band_data_t *data);

// 허브 데이터 관리 (GATT Publishing)
esp_err_t data_manager_update_hub_data(const hub_data_t *data);
esp_err_t data_manager_get_hub_data(hub_data_t *data);

// 데이터 출력 함수
void data_manager_print_all_data(void);

// 데이터 소스 관리 함수
bool data_manager_has_client_data(void);
void data_manager_clear_client_data(void);

#endif // DATA_MANAGER_H

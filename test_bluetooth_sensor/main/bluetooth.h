#ifndef BLUETOOTH_H
#define BLUETOOTH_H

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

// NimBLE includes for ESP-IDF 5.4.2
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// 표준 BLE 서비스 UUID 사용 (nRF Connect에서 인식 가능)
#define HEART_RATE_SERVICE_UUID    0x180D  // Heart Rate Service
#define HEART_RATE_CHAR_UUID       0x2A37  // Heart Rate Measurement
#define TEMPERATURE_SERVICE_UUID   0x1809  // Health Thermometer Service
#define TEMPERATURE_CHAR_UUID      0x2A6E  // Temperature Measurement
#define TEMPERATURE_TYPE_CHAR_UUID 0x2A1D  // Temperature Type
#define ENVIRONMENTAL_SERVICE_UUID 0x181A  // Environmental Sensing Service
#define HUMIDITY_CHAR_UUID         0x2A6F  // Humidity
#define PRESSURE_CHAR_UUID         0x2A6D  // Pressure

// 커스텀 통합 센서 서비스 (온습도 포함)
#define CUSTOM_SENSOR_SERVICE_UUID 0x1810  // Custom Service
#define SENSOR_DATA_CHAR_UUID      0x2A6F  // Custom Sensor Data
#define HEALTH_STATUS_CHAR_UUID     0x2A70  // Custom Health Status

// BLE connection parameters
#define BLE_DEVICE_NAME            "ESP32C6_Sensor"
#define BLE_ADV_INTERVAL_MIN       0x20  // 20ms
#define BLE_ADV_INTERVAL_MAX       0x40  // 40ms
#define BLE_CONN_INTERVAL_MIN      0x10  // 20ms
#define BLE_CONN_INTERVAL_MAX      0x20  // 40ms

// Health status definitions
typedef enum {
    HEALTH_STATUS_NORMAL,
    HEALTH_STATUS_ELEVATED_HR,
    HEALTH_STATUS_HIGH_TEMP,
    HEALTH_STATUS_LOW_HUMIDITY,
    HEALTH_STATUS_WARNING,
    HEALTH_STATUS_CRITICAL
} health_status_t;

// MIT App Inventor 경보 상태 정의
typedef enum {
    WBGT_NORMAL = 0,
    WBGT_CAUTION = 1,
    WBGT_WARNING = 2,
    WBGT_DANGER = 3,
    WBGT_EXTREME = 4
} wbgt_warning_t;

typedef enum {
    TEMP_NORMAL = 0,
    TEMP_LOW = 1,
    TEMP_HIGH = 2,
    TEMP_DANGER = 3,
    TEMP_EXTREME = 4
} temp_warning_t;

typedef enum {
    HR_NORMAL = 0,
    HR_BRADYCARDIA = 1,
    HR_TACHYCARDIA = 2,
    HR_DANGER = 3
} hr_warning_t;

// Sensor data structure
typedef struct {
    uint16_t heart_rate;
    uint16_t temperature;  // Actual value × 100 (for 0.01°C precision)
    uint16_t humidity;     // Actual value × 100 (for 0.01% precision)
    uint32_t timestamp;
    health_status_t health_status;
} sensor_data_t;

// Bluetooth state
typedef enum {
    BLUETOOTH_STATE_DISCONNECTED,
    BLUETOOTH_STATE_ADVERTISING,
    BLUETOOTH_STATE_CONNECTED
} bluetooth_state_t;

// Event bits
#define SENSOR_DATA_READY_BIT      BIT0
#define BLE_CONNECTED_BIT          BIT1
#define BLE_DISCONNECTED_BIT       BIT2

// Function declarations
esp_err_t bluetooth_init(void);
esp_err_t bluetooth_deinit(void);
esp_err_t bluetooth_start_advertising(void);
esp_err_t bluetooth_stop_advertising(void);
esp_err_t bluetooth_send_sensor_data(const sensor_data_t *data);
esp_err_t bluetooth_set_custom_advertising_data(void);  // Custom advertising data 설정
esp_err_t bluetooth_update_advertising_with_sensor_data(uint16_t temperature, uint16_t humidity, uint16_t heart_rate, uint8_t battery_level);  // 기본 센서 데이터 업데이트
esp_err_t bluetooth_set_extended_advertising_data(uint16_t temperature, uint16_t humidity, uint16_t heart_rate, uint8_t battery_level, uint16_t noise_level, uint16_t spo2, uint16_t body_temp);  // 확장 센서 데이터 설정
esp_err_t bluetooth_update_mit_app_inventor_data(float skin_temp, uint16_t heart_rate, float spo2, float noise_level, wbgt_warning_t wbgt_warning, temp_warning_t temp_warning, hr_warning_t hr_warning);  // MIT App Inventor 데이터 업데이트
esp_err_t bluetooth_set_mit_app_inventor_advertising(void);  // MIT App Inventor 전용 Advertising 설정
esp_err_t bluetooth_start_advertising_update(void);  // Advertising 업데이트 태스크 시작
bluetooth_state_t bluetooth_get_state(void);
uint16_t bluetooth_get_connected_device(uint8_t *addr);

// Health monitoring functions
health_status_t analyze_health_status(uint16_t heart_rate, uint16_t temperature, uint16_t humidity);
const char* get_health_status_string(health_status_t status);

// GATT service functions
void gatt_sensor_service_init(void);

#endif // BLUETOOTH_H 
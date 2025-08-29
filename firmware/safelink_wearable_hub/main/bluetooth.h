#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "data_manager.h"

// NimBLE includes for ESP-IDF 5.4
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// BLE Device Name
#define BLE_DEVICE_NAME "SafeLink_Test"

// App Inventor 통합용 총합 Service/Characteristic (16-bit custom)
#define HUB_AGG_SERVICE_UUID     0xFFF0
#define HUB_AGG_CHAR_UUID        0xFFF1

// Test Service UUID (Custom)
#define TEST_SERVICE_UUID        0x1800  // Generic Access Service
#define TEST_CHAR_UUID           0x2A00  // Device Name Characteristic

// Health Sensor Service UUID (Custom)
#define HEALTH_SENSOR_SERVICE_UUID    0x1810  // Custom Health Sensor Service
#define TEMPERATURE_CHAR_UUID         0x2A6E  // Temperature Measurement
#define HUMIDITY_CHAR_UUID            0x2A6F  // Humidity
#define BODY_TEMP_CHAR_UUID           0x2A73  // Body Temperature
#define SPO2_CHAR_UUID                0x2A72  // SpO2 (산소포화도)
#define HEART_RATE_CHAR_UUID          0x2A37  // Heart Rate Measurement

// Noise Sensor Service UUID (Custom)
#define NOISE_SENSOR_SERVICE_UUID     0x1811  // Custom Noise Sensor Service
#define NOISE_LEVEL_CHAR_UUID         0x2A71  // Noise Level

// Control Service UUID (Custom)
#define CONTROL_SERVICE_UUID          0x1812  // Custom Control Service
#define COMMAND_CHAR_UUID             0x2A76  // Command Characteristic
// Debug command write-only characteristic (custom)
#define DEBUG_CMD_CHAR_UUID           0xFFF2

// Bluetooth states
typedef enum {
    BLUETOOTH_STATE_DISCONNECTED,
    BLUETOOTH_STATE_ADVERTISING,
    BLUETOOTH_STATE_CONNECTED
} bluetooth_state_t;

// Event group bits
#define BLUETOOTH_READY_BIT       BIT0

// Health sensor data structure
typedef struct {
    uint16_t temperature;      // 온도 (0.01°C 단위, 2500 = 25.00°C)
    uint16_t humidity;         // 습도 (0.01% 단위, 5000 = 50.00%)
    uint16_t body_temperature; // 체온 (0.01°C 단위, 3650 = 36.50°C)
    uint16_t spo2;            // 산소포화도 (0.01% 단위, 9500 = 95.00%)
    uint16_t heart_rate;      // 심박수 (BPM)
    uint16_t noise_level;     // 소음 레벨 (0.1dB 단위, 650 = 65.0dB)
    uint32_t timestamp;       // 타임스탬프
} health_sensor_data_t;

// Function declarations
esp_err_t bluetooth_init(void);
esp_err_t bluetooth_deinit(void);
bluetooth_state_t bluetooth_get_state(void);
esp_err_t bluetooth_send_test_data(const char *data);
esp_err_t bluetooth_get_health_data(health_sensor_data_t *data);
void measure_noise_level(void);
void check_dfplayer_status(void);

// MIT App Inventor 데이터 전송 함수
esp_err_t bluetooth_update_mit_app_inventor_data(float skin_temp, uint16_t heart_rate, 
                                                float spo2, float external_temp, 
                                                float external_humidity, float avg_noise, 
                                                float wbgt, uint8_t alarm_status);

// GATT Subscribe (밴드로부터 데이터 수신) 함수
esp_err_t bluetooth_handle_band_data_write(const uint8_t *data, size_t len);
esp_err_t bluetooth_handle_command_write(const uint8_t *data, size_t len);

// GATT Publishing (허브 데이터 전송) 함수
esp_err_t bluetooth_update_hub_data_characteristics(void);

#endif // BLUETOOTH_H 
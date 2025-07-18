/*
 * BLE Server Header
 * 
 * 모바일 앱과의 BLE GATT 서버 통신
 */

#pragma once

#include "esp_err.h"
#include "ble_scanner.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* BLE service UUIDs */
#define SAFELINK_HUB_SERVICE_UUID       0x1900
#define HUB_STATUS_CHAR_UUID            0x1A01
#define WEARABLE_LIST_CHAR_UUID         0x1A02
#define SAFETY_ALERT_CHAR_UUID          0x1A03
#define HUB_CONFIG_CHAR_UUID            0x1A04
#define NOISE_DATA_CHAR_UUID            0x1A05

/* BLE connection status */
typedef enum {
    BLE_SERVER_DISCONNECTED,
    BLE_SERVER_CONNECTED,
    BLE_SERVER_ADVERTISING
} ble_server_status_t;

/* Hub status data structure */
typedef struct {
    uint8_t connected_devices;      // Number of connected wearable devices
    float current_noise_level;      // Current noise level in dB
    uint8_t overall_safety_status;  // 0=Normal, 1=Warning, 2=Critical
    uint8_t battery_percentage;     // Hub battery percentage
    bool is_charging;               // Hub charging status
    uint64_t uptime_seconds;        // Hub uptime in seconds
    uint32_t total_alerts;          // Total alerts generated
    uint64_t timestamp;             // Current timestamp
} hub_status_t;

/* Hub configuration structure */
typedef struct {
    float noise_warning_threshold;      // dB level for noise warning
    float noise_critical_threshold;     // dB level for noise critical
    uint16_t scan_interval_ms;          // BLE scan interval
    uint8_t alert_volume;               // Audio alert volume (0-100)
    uint8_t vibration_intensity;        // Vibration intensity (0-100)
    bool enable_voice_alerts;           // Enable voice alerts
    bool enable_led_alerts;             // Enable LED alerts
    bool enable_vibration_alerts;       // Enable vibration alerts
} hub_config_t;

/* Alert types for mobile app */
typedef enum {
    ALERT_TYPE_SAFETY = 0,
    ALERT_TYPE_NOISE = 1,
    ALERT_TYPE_DEVICE_DISCONNECTED = 2,
    ALERT_TYPE_LOW_BATTERY = 3,
    ALERT_TYPE_SYSTEM_ERROR = 4
} alert_type_t;

/* Mobile alert structure */
typedef struct {
    alert_type_t type;
    uint8_t severity;               // 0=Info, 1=Warning, 2=Critical
    char message[64];               // Alert message
    uint64_t timestamp;             // Alert timestamp
    char device_id[13];             // Related device ID (if applicable)
} mobile_alert_t;

/* Function declarations */
esp_err_t ble_server_init(void);
esp_err_t ble_server_start_advertising(void);
esp_err_t ble_server_stop_advertising(void);
esp_err_t ble_server_send_hub_data(const hub_status_t *status);
esp_err_t ble_server_send_wearable_list(const wearable_device_t *devices, uint8_t count);
esp_err_t ble_server_send_alert(const mobile_alert_t *alert);
esp_err_t ble_server_send_noise_data(float noise_level, uint64_t timestamp);
esp_err_t ble_server_send_battery_alert(uint8_t battery_percentage);
ble_server_status_t ble_server_get_status(void);
bool ble_server_is_connected(void);
esp_err_t ble_server_get_config(hub_config_t *config);

#ifdef __cplusplus
}
#endif 
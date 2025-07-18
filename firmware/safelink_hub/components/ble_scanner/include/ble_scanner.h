/*
 * BLE Scanner Header
 * 
 * 웨어러블 밴드의 BLE Advertising 데이터 스캔 및 수신
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_WEARABLE_DEVICES    10
#define DEVICE_NAME_MAX_LEN     32
#define DEVICE_ID_LEN          6   // MAC address

/* Wearable device sensor data structure */
typedef struct {
    float heart_rate;        // BPM
    float skin_temperature;  // Celsius
    float ambient_temp;      // Celsius
    float humidity;          // Percentage
    float wbgt;             // WBGT Index
    uint64_t timestamp;      // Unix timestamp
    int8_t rssi;            // Signal strength
} wearable_data_t;

/* Wearable device information */
typedef struct {
    uint8_t device_id[DEVICE_ID_LEN];   // MAC address
    char name[DEVICE_NAME_MAX_LEN];     // Device name
    wearable_data_t sensor_data;        // Latest sensor data
    bool is_connected;                  // Connection status
    bool data_updated;                  // New data flag
    uint64_t last_seen;                 // Last communication time
    uint32_t packet_count;              // Total packets received
    uint32_t error_count;               // Error count
} wearable_device_t;

/* Scanner status */
typedef enum {
    SCANNER_STATUS_IDLE,
    SCANNER_STATUS_SCANNING,
    SCANNER_STATUS_ERROR
} scanner_status_t;

/* Data callback function type */
typedef void (*wearable_data_callback_t)(const wearable_device_t *device, const wearable_data_t *data);

/* Function declarations */
esp_err_t ble_scanner_init(void);
esp_err_t ble_scanner_start(void);
esp_err_t ble_scanner_stop(void);
esp_err_t ble_scanner_set_data_callback(wearable_data_callback_t callback);
scanner_status_t ble_scanner_get_status(void);
uint8_t ble_scanner_get_device_count(void);
esp_err_t ble_scanner_get_devices(wearable_device_t *devices, uint8_t max_count, uint8_t *actual_count);
esp_err_t ble_scanner_add_whitelist(const uint8_t *device_id);
esp_err_t ble_scanner_remove_whitelist(const uint8_t *device_id);
esp_err_t ble_scanner_clear_whitelist(void);

#ifdef __cplusplus
}
#endif 
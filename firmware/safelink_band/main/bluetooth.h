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

// NimBLE includes for ESP-IDF 5.5
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// Target Hub device name
#define TARGET_HUB_NAME "ESP32C3_Sensor"

// BLE connection parameters
#define BLE_DEVICE_NAME            "ESP32C3_Band"
#define BLE_SCAN_INTERVAL_MIN      0x10  // 10ms
#define BLE_SCAN_INTERVAL_MAX      0x20  // 20ms
#define BLE_SCAN_WINDOW            0x10  // 10ms
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
    BLUETOOTH_STATE_SCANNING,
    BLUETOOTH_STATE_CONNECTING,
    BLUETOOTH_STATE_CONNECTED
} bluetooth_state_t;

// Event bits
#define SENSOR_DATA_READY_BIT      BIT0
#define BLE_CONNECTED_BIT          BIT1
#define BLE_DISCONNECTED_BIT       BIT2
#define HUB_FOUND_BIT              BIT3

// Function declarations
esp_err_t bluetooth_init(void);
esp_err_t bluetooth_deinit(void);
esp_err_t bluetooth_start_scanning(void);
esp_err_t bluetooth_stop_scanning(void);
esp_err_t bluetooth_connect_to_hub(const uint8_t *addr, uint8_t addr_type);
esp_err_t bluetooth_disconnect(void);
esp_err_t bluetooth_send_sensor_data(const sensor_data_t *data);
bluetooth_state_t bluetooth_get_state(void);
uint16_t bluetooth_get_connected_device(uint8_t *addr);

// Health monitoring functions
health_status_t analyze_health_status(uint16_t heart_rate, uint16_t temperature, uint16_t humidity);
const char* get_health_status_string(health_status_t status);

// Hub discovery functions
esp_err_t bluetooth_discover_hubs(void);
bool bluetooth_is_hub_connected(void);

#endif // BLUETOOTH_H 
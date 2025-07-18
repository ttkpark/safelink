/*
 * Device Manager Header
 * 
 * 다중 웨어러블 디바이스 관리
 */

#pragma once

#include "esp_err.h"
#include "ble_scanner.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Device connection timeout */
#define DEVICE_TIMEOUT_MS       30000   // 30 seconds

/* Device status */
typedef enum {
    DEVICE_STATUS_DISCONNECTED = 0,
    DEVICE_STATUS_CONNECTING = 1,
    DEVICE_STATUS_CONNECTED = 2,
    DEVICE_STATUS_TIMEOUT = 3,
    DEVICE_STATUS_ERROR = 4
} device_status_t;

/* Device health status */
typedef enum {
    DEVICE_HEALTH_UNKNOWN = 0,
    DEVICE_HEALTH_GOOD = 1,
    DEVICE_HEALTH_WARNING = 2,
    DEVICE_HEALTH_CRITICAL = 3
} device_health_t;

/* Extended device information */
typedef struct {
    wearable_device_t device_info;      // Basic device information from scanner
    device_status_t status;             // Current device status
    device_health_t health;             // Device health assessment
    uint64_t connection_time;           // When device was first connected
    uint64_t last_data_time;           // Last time data was received
    uint32_t data_loss_count;          // Number of missed data packets
    float signal_quality;               // Signal quality (0-100%)
    bool low_battery_alert;             // Device reported low battery
} managed_device_t;

/* Device manager statistics */
typedef struct {
    uint8_t total_devices;              // Total devices ever seen
    uint8_t connected_devices;          // Currently connected devices
    uint8_t active_devices;             // Devices with recent data
    uint32_t total_data_packets;        // Total packets received
    uint32_t total_errors;              // Total errors
    uint64_t uptime_seconds;            // Manager uptime
} device_manager_stats_t;

/* Function declarations */
esp_err_t device_manager_init(void);
esp_err_t device_manager_start(void);
esp_err_t device_manager_stop(void);
esp_err_t device_manager_add_device(const uint8_t *device_id, const char *name);
esp_err_t device_manager_remove_device(const uint8_t *device_id);
esp_err_t device_manager_update_device_data(const uint8_t *device_id, const wearable_data_t *data);
esp_err_t device_manager_mark_data_processed(const uint8_t *device_id);
esp_err_t device_manager_get_device(const uint8_t *device_id, managed_device_t *device);
esp_err_t device_manager_get_all_devices(managed_device_t *devices, uint8_t max_count, uint8_t *actual_count);
esp_err_t device_manager_get_active_devices(wearable_device_t *devices, uint8_t *count);
uint8_t device_manager_get_connected_count(void);
uint8_t device_manager_get_active_count(void);
esp_err_t device_manager_get_statistics(device_manager_stats_t *stats);
esp_err_t device_manager_cleanup_inactive_devices(void);
bool device_manager_is_device_active(const uint8_t *device_id);

#ifdef __cplusplus
}
#endif 
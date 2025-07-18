/*
 * BLE Service Header
 * 
 * SafeLink 웨어러블 디바이스의 BLE GATT 서비스
 */

#pragma once

#include "esp_err.h"
#include "sensor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* BLE service UUIDs */
#define SAFELINK_SERVICE_UUID           0x1800
#define SENSOR_DATA_CHAR_UUID           0x2A01
#define ALERT_CHAR_UUID                 0x2A02
#define CONFIG_CHAR_UUID                0x2A03

/* BLE connection status */
typedef enum {
    BLE_DISCONNECTED,
    BLE_CONNECTED,
    BLE_ADVERTISING
} ble_connection_status_t;

/* Alert levels */
typedef enum {
    ALERT_NONE = 0,
    ALERT_WARNING = 1,
    ALERT_DANGER = 2,
    ALERT_CRITICAL = 3
} alert_level_t;

/* Function declarations */
esp_err_t ble_service_init(void);
esp_err_t ble_service_start_advertising(void);
esp_err_t ble_service_stop_advertising(void);
esp_err_t ble_service_send_sensor_data(const sensor_data_t *data);
esp_err_t ble_service_send_alert(alert_level_t level, const char *message);
ble_connection_status_t ble_service_get_status(void);

#ifdef __cplusplus
}
#endif 
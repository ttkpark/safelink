/*
 * Safety Monitor Header
 * 
 * 작업자 안전 상태 모니터링 및 위험 감지
 */

#pragma once

#include "esp_err.h"
#include "sensor_manager.h"
#include "ble_service.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Safety thresholds */
#define BODY_TEMP_WARNING_MIN       37.5f  // °C
#define BODY_TEMP_WARNING_MAX       38.0f  // °C
#define BODY_TEMP_CRITICAL          38.0f  // °C

#define HEART_RATE_WARNING_MIN      100.0f // BPM
#define HEART_RATE_WARNING_MAX      120.0f // BPM
#define HEART_RATE_CRITICAL_MIN     120.0f // BPM
#define HEART_RATE_CRITICAL_MAX     130.0f // BPM

#define WBGT_WARNING_MIN            28.0f  // °C
#define WBGT_WARNING_MAX            30.0f  // °C
#define WBGT_CRITICAL               30.0f  // °C

/* Safety status */
typedef enum {
    SAFETY_STATUS_NORMAL,
    SAFETY_STATUS_WARNING,
    SAFETY_STATUS_CRITICAL
} safety_status_t;

/* Alert types */
typedef enum {
    ALERT_TYPE_BODY_TEMP,
    ALERT_TYPE_HEART_RATE,
    ALERT_TYPE_WBGT,
    ALERT_TYPE_GENERAL
} alert_type_t;

/* Safety event structure */
typedef struct {
    alert_type_t type;
    safety_status_t status;
    float value;
    uint64_t timestamp;
    char message[64];
} safety_event_t;

/* Function declarations */
esp_err_t safety_monitor_init(void);
esp_err_t safety_monitor_start(void);
esp_err_t safety_monitor_stop(void);
esp_err_t safety_monitor_check(const sensor_data_t *data);
safety_status_t safety_monitor_get_status(void);

#ifdef __cplusplus
}
#endif 
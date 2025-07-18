/*
 * Safety Analyzer Header
 */

#pragma once

#include "esp_err.h"
#include "ble_scanner.h"
#include "audio_manager.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Safety status levels */
typedef enum {
    SAFETY_STATUS_NORMAL = 0,
    SAFETY_STATUS_WARNING = 1,
    SAFETY_STATUS_CRITICAL = 2
} safety_status_t;

/* Risk types */
typedef enum {
    RISK_TYPE_NONE = 0,
    RISK_TYPE_NOISE = 1,
    RISK_TYPE_TEMPERATURE = 2,
    RISK_TYPE_HEART_RATE = 3,
    RISK_TYPE_HEAT_STRESS = 4,
    RISK_TYPE_DEVICE_DISCONNECTED = 5,
    RISK_TYPE_MULTIPLE = 6
} risk_type_t;

/* Safety analysis result */
typedef struct {
    safety_status_t overall_status;     // Overall safety status
    risk_type_t primary_risk;           // Primary risk factor
    char message[128];                  // Human-readable message
    uint8_t affected_devices;           // Number of devices with issues
    float risk_score;                   // Calculated risk score (0-100)
    uint64_t timestamp;                 // Analysis timestamp
} safety_result_t;

/* Function declarations */
esp_err_t safety_analyzer_init(void);
esp_err_t safety_analyzer_analyze(const wearable_device_t *devices, uint8_t device_count,
                                 const noise_data_t *noise_data, safety_result_t *result);

#ifdef __cplusplus
}
#endif 
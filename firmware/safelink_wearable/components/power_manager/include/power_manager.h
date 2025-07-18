/*
 * Power Manager Header
 * 
 * 배터리 및 저전력 모드 관리
 */

#pragma once

#include "esp_err.h"
#include "esp_sleep.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Power states */
typedef enum {
    POWER_STATE_ACTIVE,
    POWER_STATE_LIGHT_SLEEP,
    POWER_STATE_DEEP_SLEEP,
    POWER_STATE_CHARGING
} power_state_t;

/* Battery status */
typedef struct {
    float voltage;          // Battery voltage in V
    float percentage;       // Battery percentage (0-100%)
    bool is_charging;       // Charging status
    bool is_low_battery;    // Low battery warning
} battery_status_t;

/* Power configuration */
typedef struct {
    uint32_t sleep_timeout_ms;      // Timeout before entering sleep
    float low_battery_threshold;    // Low battery warning threshold (%)
    float critical_battery_threshold; // Critical battery shutdown threshold (%)
} power_config_t;

/* Function declarations */
esp_err_t power_manager_init(void);
esp_err_t power_manager_set_config(const power_config_t *config);
esp_err_t power_manager_get_battery_status(battery_status_t *status);
esp_err_t power_manager_enter_light_sleep(uint32_t duration_ms);
esp_err_t power_manager_enter_deep_sleep(uint32_t duration_ms);
esp_err_t power_manager_wake_up(void);
power_state_t power_manager_get_state(void);
bool power_manager_is_battery_critical(void);

#ifdef __cplusplus
}
#endif 
/*
 * Power Manager Header (Hub Version)
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Power states */
typedef enum {
    POWER_STATE_ACTIVE = 0,
    POWER_STATE_SAVING = 1,
    POWER_STATE_STANDBY = 2,
    POWER_STATE_CHARGING = 3
} power_state_t;

/* Battery status */
typedef struct {
    uint8_t percentage;         // Battery percentage (0-100)
    float voltage;              // Battery voltage
    float current;              // Current consumption
    bool is_charging;           // Charging status
    uint32_t time_remaining;    // Estimated time remaining (minutes)
    uint32_t charge_cycles;     // Total charge cycles
} battery_status_t;

/* Function declarations */
esp_err_t power_manager_init(void);
esp_err_t power_manager_get_battery_status(battery_status_t *status);
power_state_t power_manager_get_state(void);
esp_err_t power_manager_set_power_mode(power_state_t mode);
bool power_manager_is_low_battery(void);

#ifdef __cplusplus
}
#endif 
/*
 * Data Manager Header (Hub Version)
 */

#pragma once

#include "esp_err.h"
#include "ble_scanner.h"
#include "ble_server.h"
#include "safety_analyzer.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function declarations */
esp_err_t data_manager_init(void);
esp_err_t data_manager_start(void);
esp_err_t data_manager_store_wearable_data(const wearable_device_t *device);
esp_err_t data_manager_store_safety_result(const safety_result_t *result);
esp_err_t data_manager_get_hub_status(hub_status_t *status);

#ifdef __cplusplus
}
#endif 
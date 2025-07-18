/*
 * Sensor Manager Header
 * 
 * 심박센서, 피부온도센서, 환경온습도센서 관리
 */

#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Sensor data structure */
typedef struct {
    float heart_rate;        // BPM
    float skin_temperature;  // Celsius
    float ambient_temp;      // Celsius
    float humidity;          // Percentage
    float wbgt;             // WBGT Index
    uint64_t timestamp;      // Unix timestamp
} sensor_data_t;

/* Sensor status */
typedef enum {
    SENSOR_STATUS_OK,
    SENSOR_STATUS_ERROR,
    SENSOR_STATUS_DISCONNECTED
} sensor_status_t;

/* Function declarations */
esp_err_t sensor_manager_init(void);
esp_err_t sensor_manager_start(void);
esp_err_t sensor_manager_stop(void);
esp_err_t sensor_manager_get_data(sensor_data_t *data);
sensor_status_t sensor_manager_get_status(void);
float calculate_wbgt(float temp, float humidity);

#ifdef __cplusplus
}
#endif 
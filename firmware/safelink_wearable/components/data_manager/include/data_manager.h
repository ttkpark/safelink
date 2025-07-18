/*
 * Data Manager Header
 * 
 * 센서 데이터 버퍼링, 저장 및 동기화 관리
 */

#pragma once

#include "esp_err.h"
#include "sensor_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DATA_BUFFER_SIZE        3600    // 1 hour of data at 1Hz
#define DATA_SYNC_INTERVAL_MS   10000   // 10 seconds

/* Data storage status */
typedef enum {
    DATA_STATUS_OK,
    DATA_STATUS_BUFFER_FULL,
    DATA_STATUS_SYNC_PENDING,
    DATA_STATUS_ERROR
} data_status_t;

/* Data statistics */
typedef struct {
    uint32_t total_samples;
    uint32_t buffer_count;
    uint32_t sync_count;
    uint32_t error_count;
    uint64_t last_sync_time;
} data_stats_t;

/* Function declarations */
esp_err_t data_manager_init(void);
esp_err_t data_manager_start(void);
esp_err_t data_manager_stop(void);
esp_err_t data_manager_store_data(const sensor_data_t *data);
esp_err_t data_manager_get_buffer_data(sensor_data_t *buffer, uint32_t max_count, uint32_t *actual_count);
esp_err_t data_manager_clear_buffer(void);
esp_err_t data_manager_force_sync(void);
data_status_t data_manager_get_status(void);
esp_err_t data_manager_get_stats(data_stats_t *stats);

#ifdef __cplusplus
}
#endif 
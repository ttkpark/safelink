#ifndef SENSOR_H
#define SENSOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "i2c.h"
#include "bluetooth.h"
#include "data_manager.h"

// Event group bits
#define SENSOR_DATA_READY_BIT    BIT0
// BLUETOOTH_READY_BIT는 bluetooth.h 정의 사용

// Task priorities
#define SENSOR_TASK_PRIORITY     5
#define MONITOR_TASK_PRIORITY    3

// Task stack sizes
#define SENSOR_TASK_STACK_SIZE   4096
#define MONITOR_TASK_STACK_SIZE  4096

// Sensor reading intervals (in milliseconds)
#define HEART_RATE_READ_INTERVAL_MS    2000
#define TEMP_HUMIDITY_READ_INTERVAL_MS 4000
#define MONITOR_INTERVAL_MS            500

// Health status enumeration
typedef enum {
    HEALTH_STATUS_NORMAL = 0,
    HEALTH_STATUS_ELEVATED_HR,
    HEALTH_STATUS_HIGH_TEMP,
    HEALTH_STATUS_LOW_HUMIDITY,
    HEALTH_STATUS_WARNING,
    HEALTH_STATUS_CRITICAL
} health_status_t;

// Basic sensor data structure
typedef struct {
    uint16_t heart_rate;      // 심박수 (BPM)
    uint16_t temperature;     // 온도 (0.01°C 단위)
    uint16_t humidity;        // 습도 (0.01% 단위)
    health_status_t health_status; // 건강 상태
    uint32_t timestamp;       // 타임스탬프
} sensor_data_t;

// Extended sensor data structure
typedef struct {
    uint16_t heart_rate;      // 심박수 (BPM)
    uint16_t spo2;           // 산소포화도 (0.01% 단위)
    uint16_t body_temperature; // 체온 (0.01°C 단위)
    uint16_t external_temperature; // 외부 온도 (0.01°C 단위)
    uint16_t external_humidity;    // 외부 습도 (0.01% 단위)
    uint16_t noise_level;     // 소음 레벨 (0.1dB 단위)
    health_status_t health_status; // 건강 상태
    uint32_t timestamp;       // 타임스탬프
} extended_sensor_data_t;

// Function declarations
esp_err_t sensor_init(void);
esp_err_t sensor_deinit(void);

// Health analysis functions
health_status_t analyze_health_status(uint16_t heart_rate, uint16_t temperature, uint16_t humidity);
health_status_t analyze_extended_health_status(const extended_sensor_data_t *data);
const char* get_health_status_string(health_status_t status);

// AM2320 sensor functions
esp_err_t AM2320_read(uint16_t *temperature, uint16_t *humidity);

// Task functions
void heart_rate_task(void *arg);
void temp_humidity_task(void *arg);
void sensor_monitor_task(void *arg);

// Sensor data management
esp_err_t sensor_get_current_data(sensor_data_t *data);
esp_err_t sensor_get_extended_data(extended_sensor_data_t *data);
esp_err_t sensor_update_health_status(void);

// Task creation functions
esp_err_t sensor_create_tasks(EventGroupHandle_t event_group);

#endif // SENSOR_H 
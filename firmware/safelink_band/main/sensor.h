#ifndef SENSOR_H
#define SENSOR_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "bluetooth.h"
#include "i2c.h"
#include "analog.h"

// Event group bits
#define SENSOR_DATA_READY_BIT      BIT0
#define BLE_CONNECTED_BIT          BIT1
#define BLE_DISCONNECTED_BIT       BIT2
#define HUB_FOUND_BIT              BIT3

// Task priorities
#define SENSOR_MONITOR_TASK_PRIORITY    4
#define DATA_SENDER_TASK_PRIORITY       3
#define SENSOR_MONITOR_TASK_STACK_SIZE  4096
#define DATA_SENDER_TASK_STACK_SIZE     4096

// Sensor reading intervals
#define TEMP_HUMIDITY_READ_INTERVAL_MS  5000  // 5초마다 온습도 읽기
#define HEART_RATE_READ_INTERVAL_MS     1000  // 1초마다 심박수 읽기
#define DATA_SEND_INTERVAL_MS           2000  // 2초마다 데이터 전송

// Function declarations
esp_err_t sensor_init(void);
esp_err_t sensor_deinit(void);

// Task functions
void sensor_monitor_task(void *pvParameters);
void data_sender_task(void *pvParameters);

// Sensor data management
esp_err_t sensor_get_current_data(sensor_data_t *data);
esp_err_t sensor_get_heart_rate_data(heart_rate_data_t *data);
esp_err_t sensor_update_health_status(void);

// Task creation functions
esp_err_t sensor_create_tasks(EventGroupHandle_t event_group);

#endif // SENSOR_H 
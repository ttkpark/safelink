/*
 * SafeLink Wearable Main Application
 * 
 * 작업자 안전 모니터링을 위한 웨어러블 디바이스 펌웨어
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_sleep.h"
#include "esp_pm.h"

#include "sensor_manager.h"
#include "ble_service.h"
#include "power_manager.h"
#include "safety_monitor.h"
#include "data_manager.h"

static const char *TAG = "SAFELINK_MAIN";

#define MAIN_LOOP_PERIOD_MS     1000    // 1 second main loop
#define BLE_DATA_SEND_PERIOD_MS 10000   // 10 seconds BLE data transmission

static TaskHandle_t main_task_handle = NULL;
static uint64_t last_ble_send_time = 0;

/* Private function declarations */
static void main_task(void *param);
static void process_sensor_data(void);
static void handle_system_status(void);
static esp_err_t initialize_all_components(void);

/* Application main task */
void app_main(void)
{
    ESP_LOGI(TAG, "SafeLink Wearable Device Starting...");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // Initialize NVS for storing configuration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize all components
    ret = initialize_all_components();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Component initialization failed, restarting...");
        esp_restart();
    }

    // Create main application task
    BaseType_t task_ret = xTaskCreate(main_task, "main_task", 8192, NULL, 5, &main_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create main task");
        esp_restart();
    }

    ESP_LOGI(TAG, "SafeLink Wearable Device Ready!");
}

static esp_err_t initialize_all_components(void)
{
    esp_err_t ret;

    // Initialize power management first
    ESP_LOGI(TAG, "Initializing power management...");
    ret = power_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize data manager
    ESP_LOGI(TAG, "Initializing data manager...");
    ret = data_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Data manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize sensor manager
    ESP_LOGI(TAG, "Initializing sensors...");
    ret = sensor_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize BLE service
    ESP_LOGI(TAG, "Initializing BLE service...");
    ret = ble_service_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE service init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize safety monitoring
    ESP_LOGI(TAG, "Initializing safety monitor...");
    ret = safety_monitor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Safety monitor init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start all services
    ESP_LOGI(TAG, "Starting all services...");
    
    data_manager_start();
    sensor_manager_start();
    safety_monitor_start();
    ble_service_start_advertising();

    return ESP_OK;
}

static void main_task(void *param)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Process sensor data and safety monitoring
        process_sensor_data();
        
        // Handle system status and power management
        handle_system_status();
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS));
    }
}

static void process_sensor_data(void)
{
    sensor_data_t sensor_data;
    uint64_t current_time = esp_timer_get_time() / 1000; // Convert to ms
    
    // Get latest sensor data
    esp_err_t ret = sensor_manager_get_data(&sensor_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to get sensor data: %s", esp_err_to_name(ret));
        return;
    }
    
    // Store data in buffer
    ret = data_manager_store_data(&sensor_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to store sensor data: %s", esp_err_to_name(ret));
    }
    
    // Check safety conditions
    ret = safety_monitor_check(&sensor_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Safety monitor check failed: %s", esp_err_to_name(ret));
    }
    
    // Send data via BLE periodically
    if (ble_service_get_status() == BLE_CONNECTED && 
        (current_time - last_ble_send_time) >= BLE_DATA_SEND_PERIOD_MS) {
        
        ret = ble_service_send_sensor_data(&sensor_data);
        if (ret == ESP_OK) {
            last_ble_send_time = current_time;
            ESP_LOGD(TAG, "Sensor data sent via BLE");
        } else {
            ESP_LOGW(TAG, "Failed to send BLE data: %s", esp_err_to_name(ret));
        }
    }
    
    // Log sensor data periodically
    ESP_LOGI(TAG, "Sensors - HR:%.1f BPM, SkinT:%.1f°C, Temp:%.1f°C, Hum:%.1f%%, WBGT:%.1f°C",
             sensor_data.heart_rate, sensor_data.skin_temperature, 
             sensor_data.ambient_temp, sensor_data.humidity, sensor_data.wbgt);
}

static void handle_system_status(void)
{
    // Check battery status
    battery_status_t battery_status;
    esp_err_t ret = power_manager_get_battery_status(&battery_status);
    if (ret == ESP_OK) {
        // Log battery status periodically
        static uint32_t battery_log_counter = 0;
        if (++battery_log_counter >= 30) { // Log every 30 seconds
            ESP_LOGI(TAG, "Battery: %.2fV (%.1f%%) %s%s",
                     battery_status.voltage,
                     battery_status.percentage,
                     battery_status.is_charging ? "Charging " : "",
                     battery_status.is_low_battery ? "LOW" : "");
            battery_log_counter = 0;
        }
        
        // Handle low battery
        if (battery_status.is_low_battery && !battery_status.is_charging) {
            ESP_LOGW(TAG, "Low battery detected: %.1f%%", battery_status.percentage);
            // Send low battery alert via BLE
            if (ble_service_get_status() == BLE_CONNECTED) {
                ble_service_send_alert(ALERT_WARNING, "배터리 부족");
            }
        }
    }
    
    // Check data manager status
    data_status_t data_status = data_manager_get_status();
    if (data_status == DATA_STATUS_ERROR) {
        ESP_LOGW(TAG, "Data manager error detected");
    } else if (data_status == DATA_STATUS_BUFFER_FULL) {
        ESP_LOGW(TAG, "Data buffer is full, forcing sync");
        data_manager_force_sync();
    }
    
    // Check safety status
    safety_status_t safety_status = safety_monitor_get_status();
    static safety_status_t last_safety_status = SAFETY_STATUS_NORMAL;
    
    if (safety_status != last_safety_status) {
        const char* status_str[] = {"NORMAL", "WARNING", "CRITICAL"};
        ESP_LOGI(TAG, "Safety status changed: %s", status_str[safety_status]);
        last_safety_status = safety_status;
    }
    
    // Check sensor status
    sensor_status_t sensor_status = sensor_manager_get_status();
    if (sensor_status != SENSOR_STATUS_OK) {
        ESP_LOGW(TAG, "Sensor status error: %d", sensor_status);
    }
    
    // Log system statistics periodically
    static uint32_t stats_log_counter = 0;
    if (++stats_log_counter >= 60) { // Log every 60 seconds
        data_stats_t data_stats;
        if (data_manager_get_stats(&data_stats) == ESP_OK) {
            ESP_LOGI(TAG, "Data Stats - Total:%lu, Buffer:%lu, Syncs:%lu, Errors:%lu",
                     data_stats.total_samples, data_stats.buffer_count,
                     data_stats.sync_count, data_stats.error_count);
        }
        
        ESP_LOGI(TAG, "System Status - Power:%d, BLE:%d, Safety:%d, Sensor:%d",
                 power_manager_get_state(), ble_service_get_status(),
                 safety_status, sensor_status);
        
        stats_log_counter = 0;
    }
} 
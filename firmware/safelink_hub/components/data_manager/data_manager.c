/*
 * Data Manager Implementation (Hub Version)
 */

#include "data_manager.h"
#include "power_manager.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "DATA_MGR"

static uint64_t system_start_time = 0;
static uint32_t total_alerts = 0;

esp_err_t data_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing data manager...");
    
    // Initialize SPIFFS for data storage
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/storage",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = true
    };
    
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    system_start_time = esp_timer_get_time() / 1000000; // Convert to seconds
    
    ESP_LOGI(TAG, "Data manager initialized");
    return ESP_OK;
}

esp_err_t data_manager_start(void)
{
    ESP_LOGI(TAG, "Data manager started");
    return ESP_OK;
}

esp_err_t data_manager_store_wearable_data(const wearable_device_t *device)
{
    if (device == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // In a real implementation, this would store data to SPIFFS or external storage
    ESP_LOGD(TAG, "Storing wearable data from device: %s", device->name);
    
    return ESP_OK;
}

esp_err_t data_manager_store_safety_result(const safety_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (result->overall_status != SAFETY_STATUS_NORMAL) {
        total_alerts++;
    }
    
    ESP_LOGD(TAG, "Storing safety result: Status=%d, Risk=%d", 
             result->overall_status, result->primary_risk);
    
    return ESP_OK;
}

esp_err_t data_manager_get_hub_status(hub_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Get battery status
    battery_status_t battery_status;
    power_manager_get_battery_status(&battery_status);
    
    // Fill hub status
    status->connected_devices = 0; // Will be updated by device manager
    status->current_noise_level = 0.0f; // Will be updated by audio manager
    status->overall_safety_status = 0; // Normal
    status->battery_percentage = battery_status.percentage;
    status->is_charging = battery_status.is_charging;
    status->uptime_seconds = (esp_timer_get_time() / 1000000) - system_start_time;
    status->total_alerts = total_alerts;
    status->timestamp = esp_timer_get_time() / 1000;
    
    return ESP_OK;
} 
/*
 * Device Manager Implementation
 */

#include "device_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#define TAG "DEVICE_MGR"

static managed_device_t managed_devices[MAX_WEARABLE_DEVICES];
static uint8_t device_count = 0;
static SemaphoreHandle_t device_mutex = NULL;
static device_manager_stats_t stats = {0};
static TaskHandle_t cleanup_task_handle = NULL;

/* Private function declarations */
static managed_device_t* find_device(const uint8_t *device_id);
static void cleanup_task(void *param);
static void update_device_health(managed_device_t *device);

esp_err_t device_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing device manager...");
    
    device_mutex = xSemaphoreCreateMutex();
    if (device_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create device mutex");
        return ESP_FAIL;
    }
    
    // Create cleanup task
    BaseType_t task_ret = xTaskCreate(cleanup_task, "device_cleanup", 2048, NULL, 3, &cleanup_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create cleanup task");
        return ESP_FAIL;
    }
    
    stats.uptime_seconds = 0;
    
    ESP_LOGI(TAG, "Device manager initialized");
    return ESP_OK;
}

esp_err_t device_manager_update_device_data(const uint8_t *device_id, const wearable_data_t *data)
{
    if (device_id == NULL || data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(device_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        managed_device_t *device = find_device(device_id);
        
        if (device != NULL) {
            device->device_info.sensor_data = *data;
            device->device_info.data_updated = true;
            device->last_data_time = esp_timer_get_time() / 1000;
            device->status = DEVICE_STATUS_CONNECTED;
            
            update_device_health(device);
            stats.total_data_packets++;
        }
        
        xSemaphoreGive(device_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

uint8_t device_manager_get_connected_count(void)
{
    return stats.connected_devices;
}

esp_err_t device_manager_get_active_devices(wearable_device_t *devices, uint8_t *count)
{
    if (devices == NULL || count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *count = 0;
    
    if (xSemaphoreTake(device_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (uint8_t i = 0; i < device_count && *count < MAX_WEARABLE_DEVICES; i++) {
            if (managed_devices[i].status == DEVICE_STATUS_CONNECTED) {
                devices[*count] = managed_devices[i].device_info;
                (*count)++;
            }
        }
        
        xSemaphoreGive(device_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t device_manager_mark_data_processed(const uint8_t *device_id)
{
    if (device_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(device_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        managed_device_t *device = find_device(device_id);
        
        if (device != NULL) {
            device->device_info.data_updated = false;
        }
        
        xSemaphoreGive(device_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/* Private functions */
static managed_device_t* find_device(const uint8_t *device_id)
{
    for (uint8_t i = 0; i < device_count; i++) {
        if (memcmp(managed_devices[i].device_info.device_id, device_id, DEVICE_ID_LEN) == 0) {
            return &managed_devices[i];
        }
    }
    return NULL;
}

static void cleanup_task(void *param)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000)); // Run every 10 seconds
        
        uint64_t current_time = esp_timer_get_time() / 1000;
        
        if (xSemaphoreTake(device_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
            stats.connected_devices = 0;
            stats.active_devices = 0;
            
            for (uint8_t i = 0; i < device_count; i++) {
                managed_device_t *device = &managed_devices[i];
                
                // Check for timeout
                if ((current_time - device->last_data_time) > DEVICE_TIMEOUT_MS) {
                    if (device->status == DEVICE_STATUS_CONNECTED) {
                        device->status = DEVICE_STATUS_TIMEOUT;
                        ESP_LOGW(TAG, "Device timeout: %02x:%02x:%02x:%02x:%02x:%02x",
                                device->device_info.device_id[0], device->device_info.device_id[1],
                                device->device_info.device_id[2], device->device_info.device_id[3],
                                device->device_info.device_id[4], device->device_info.device_id[5]);
                    }
                } else {
                    if (device->status == DEVICE_STATUS_CONNECTED) {
                        stats.connected_devices++;
                        
                        if ((current_time - device->last_data_time) < 5000) { // Active in last 5 seconds
                            stats.active_devices++;
                        }
                    }
                }
            }
            
            xSemaphoreGive(device_mutex);
        }
        
        stats.uptime_seconds += 10;
    }
}

static void update_device_health(managed_device_t *device)
{
    // Simple health assessment based on signal quality and data freshness
    if (device->device_info.sensor_data.rssi > -60) {
        device->signal_quality = 100.0f;
    } else if (device->device_info.sensor_data.rssi > -80) {
        device->signal_quality = 70.0f;
    } else {
        device->signal_quality = 30.0f;
    }
    
    // Assess overall health
    if (device->signal_quality > 80 && device->data_loss_count < 5) {
        device->health = DEVICE_HEALTH_GOOD;
    } else if (device->signal_quality > 50 && device->data_loss_count < 20) {
        device->health = DEVICE_HEALTH_WARNING;
    } else {
        device->health = DEVICE_HEALTH_CRITICAL;
    }
} 
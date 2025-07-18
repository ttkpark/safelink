/*
 * SafeLink Vest Hub Main Application
 * 
 * 웨어러블 밴드 데이터 수집 및 종합 안전 모니터링 허브
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

#include "ble_scanner.h"
#include "ble_server.h"
#include "audio_manager.h"
#include "device_manager.h"
#include "safety_analyzer.h"
#include "alert_manager.h"
#include "power_manager.h"
#include "data_manager.h"

static const char *TAG = "SAFELINK_HUB";

#define MAIN_LOOP_PERIOD_MS     100     // 100ms main loop
#define APP_DATA_SEND_PERIOD_MS 1000    // 1 second app data transmission
#define SAFETY_CHECK_PERIOD_MS  500     // 500ms safety check

static TaskHandle_t main_task_handle = NULL;
static uint64_t last_app_send_time = 0;
static uint64_t last_safety_check_time = 0;

/* System event bits */
#define WEARABLE_DATA_RECEIVED_BIT  BIT0
#define NOISE_ALERT_BIT            BIT1
#define SAFETY_ALERT_BIT           BIT2
#define LOW_BATTERY_BIT            BIT3

static EventGroupHandle_t system_events = NULL;

/* Private function declarations */
static void main_task(void *param);
static void process_wearable_data(void);
static void perform_safety_analysis(void);
static void handle_system_events(void);
static esp_err_t initialize_all_components(void);
static void wearable_data_callback(const wearable_device_t *device, const wearable_data_t *data);
static void noise_alert_callback(noise_level_t level, float db_value);

/* Application main */
void app_main(void)
{
    ESP_LOGI(TAG, "SafeLink Vest Hub Starting...");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    
    // Initialize NVS for storing configuration
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Create system event group
    system_events = xEventGroupCreate();
    if (system_events == NULL) {
        ESP_LOGE(TAG, "Failed to create system event group");
        esp_restart();
    }

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

    ESP_LOGI(TAG, "SafeLink Vest Hub Ready!");
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

    // Initialize audio manager
    ESP_LOGI(TAG, "Initializing audio manager...");
    ret = audio_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize alert manager
    ESP_LOGI(TAG, "Initializing alert manager...");
    ret = alert_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Alert manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize device manager
    ESP_LOGI(TAG, "Initializing device manager...");
    ret = device_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device manager init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize BLE scanner
    ESP_LOGI(TAG, "Initializing BLE scanner...");
    ret = ble_scanner_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE scanner init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize BLE server
    ESP_LOGI(TAG, "Initializing BLE server...");
    ret = ble_server_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BLE server init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize safety analyzer
    ESP_LOGI(TAG, "Initializing safety analyzer...");
    ret = safety_analyzer_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Safety analyzer init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Start all services
    ESP_LOGI(TAG, "Starting all services...");
    
    data_manager_start();
    audio_manager_start();
    alert_manager_start();
    
    // Set callbacks
    ble_scanner_set_data_callback(wearable_data_callback);
    audio_manager_set_noise_callback(noise_alert_callback);
    
    // Start scanning and server
    ble_scanner_start();
    ble_server_start_advertising();

    return ESP_OK;
}

static void main_task(void *param)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        uint64_t current_time = esp_timer_get_time() / 1000; // Convert to ms
        
        // Process wearable data
        process_wearable_data();
        
        // Perform safety analysis periodically
        if ((current_time - last_safety_check_time) >= SAFETY_CHECK_PERIOD_MS) {
            perform_safety_analysis();
            last_safety_check_time = current_time;
        }
        
        // Send data to app periodically
        if (ble_server_is_connected() && 
            (current_time - last_app_send_time) >= APP_DATA_SEND_PERIOD_MS) {
            
            hub_status_t hub_status;
            data_manager_get_hub_status(&hub_status);
            ble_server_send_hub_data(&hub_status);
            last_app_send_time = current_time;
        }
        
        // Handle system events
        handle_system_events();
        
        // Wait for next cycle
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(MAIN_LOOP_PERIOD_MS));
    }
}

static void process_wearable_data(void)
{
    // Get latest wearable data from device manager
    wearable_device_t devices[MAX_WEARABLE_DEVICES];
    uint8_t device_count = 0;
    
    device_manager_get_active_devices(devices, &device_count);
    
    for (uint8_t i = 0; i < device_count; i++) {
        if (devices[i].is_connected && devices[i].data_updated) {
            // Store data
            data_manager_store_wearable_data(&devices[i]);
            
            // Mark as processed
            device_manager_mark_data_processed(devices[i].device_id);
            
            ESP_LOGD(TAG, "Device %s: HR=%.1f, Temp=%.1f, WBGT=%.1f", 
                     devices[i].name, 
                     devices[i].sensor_data.heart_rate,
                     devices[i].sensor_data.skin_temperature,
                     devices[i].sensor_data.wbgt);
        }
    }
}

static void perform_safety_analysis(void)
{
    // Get current noise level
    noise_data_t noise_data;
    audio_manager_get_noise_data(&noise_data);
    
    // Get wearable devices data
    wearable_device_t devices[MAX_WEARABLE_DEVICES];
    uint8_t device_count = 0;
    device_manager_get_active_devices(devices, &device_count);
    
    // Perform comprehensive safety analysis
    safety_result_t safety_result;
    safety_analyzer_analyze(devices, device_count, &noise_data, &safety_result);
    
    // Handle safety alerts
    if (safety_result.overall_status != SAFETY_STATUS_NORMAL) {
        alert_level_t alert_level;
        
        switch (safety_result.overall_status) {
            case SAFETY_STATUS_WARNING:
                alert_level = ALERT_LEVEL_WARNING;
                break;
            case SAFETY_STATUS_CRITICAL:
                alert_level = ALERT_LEVEL_CRITICAL;
                break;
            default:
                alert_level = ALERT_LEVEL_NORMAL;
                break;
        }
        
        // Trigger appropriate alerts
        alert_manager_trigger_alert(alert_level, safety_result.primary_risk, safety_result.message);
        
        // Set system event
        xEventGroupSetBits(system_events, SAFETY_ALERT_BIT);
        
        ESP_LOGW(TAG, "Safety Alert: %s (Level: %d)", safety_result.message, alert_level);
    }
    
    // Store safety analysis result
    data_manager_store_safety_result(&safety_result);
}

static void handle_system_events(void)
{
    EventBits_t events = xEventGroupGetBits(system_events);
    
    // Handle noise alerts
    if (events & NOISE_ALERT_BIT) {
        ESP_LOGW(TAG, "Noise alert detected");
        xEventGroupClearBits(system_events, NOISE_ALERT_BIT);
    }
    
    // Handle safety alerts
    if (events & SAFETY_ALERT_BIT) {
        // Safety alert already processed in perform_safety_analysis
        xEventGroupClearBits(system_events, SAFETY_ALERT_BIT);
    }
    
    // Handle low battery
    if (events & LOW_BATTERY_BIT) {
        ESP_LOGW(TAG, "Low battery detected");
        
        battery_status_t battery_status;
        power_manager_get_battery_status(&battery_status);
        
        if (ble_server_is_connected()) {
            ble_server_send_battery_alert(battery_status.percentage);
        }
        
        xEventGroupClearBits(system_events, LOW_BATTERY_BIT);
    }
    
    // Check system status periodically
    static uint32_t status_counter = 0;
    if (++status_counter >= 50) { // Every 5 seconds (100ms * 50)
        // Log system status
        uint8_t connected_devices = device_manager_get_connected_count();
        noise_level_t noise_level = audio_manager_get_current_noise_level();
        power_state_t power_state = power_manager_get_state();
        
        ESP_LOGI(TAG, "System Status - Devices:%d, Noise:%d, Power:%d", 
                 connected_devices, noise_level, power_state);
        
        status_counter = 0;
    }
}

/* Callback functions */
static void wearable_data_callback(const wearable_device_t *device, const wearable_data_t *data)
{
    if (device == NULL || data == NULL) {
        return;
    }
    
    ESP_LOGD(TAG, "Received data from device: %s", device->name);
    
    // Update device in device manager
    device_manager_update_device_data(device->device_id, data);
    
    // Set system event
    xEventGroupSetBits(system_events, WEARABLE_DATA_RECEIVED_BIT);
}

static void noise_alert_callback(noise_level_t level, float db_value)
{
    ESP_LOGW(TAG, "Noise alert: Level %d, %.1f dB", level, db_value);
    
    // Trigger immediate noise alert
    alert_manager_trigger_noise_alert(level, db_value);
    
    // Set system event
    xEventGroupSetBits(system_events, NOISE_ALERT_BIT);
} 
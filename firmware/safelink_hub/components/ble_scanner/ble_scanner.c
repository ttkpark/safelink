/*
 * BLE Scanner Implementation
 */

#include "ble_scanner.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#define TAG "BLE_SCANNER"

#define SCAN_DURATION       0    // Continuous scan
#define SCAN_INTERVAL       0x50 // 50ms
#define SCAN_WINDOW         0x30 // 30ms
#define SAFELINK_MANUFACTURER_ID 0xFFFF

static scanner_status_t scanner_status = SCANNER_STATUS_IDLE;
static wearable_device_t discovered_devices[MAX_WEARABLE_DEVICES];
static uint8_t device_count = 0;
static SemaphoreHandle_t device_mutex = NULL;
static wearable_data_callback_t data_callback = NULL;

/* Whitelist for trusted devices */
static uint8_t whitelist[MAX_WEARABLE_DEVICES][DEVICE_ID_LEN];
static uint8_t whitelist_count = 0;
static bool use_whitelist = false;

/* BLE scan parameters */
static esp_ble_scan_params_t scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = SCAN_INTERVAL,
    .scan_window = SCAN_WINDOW,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
};

/* Private function declarations */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static esp_err_t parse_advertising_data(const uint8_t *adv_data, uint8_t adv_len, 
                                       wearable_data_t *sensor_data);
static wearable_device_t* find_or_create_device(const uint8_t *device_id, const char *name);
static bool is_device_whitelisted(const uint8_t *device_id);
static void update_device_data(wearable_device_t *device, const wearable_data_t *data, int8_t rssi);

esp_err_t ble_scanner_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE scanner...");
    
    // Create mutex for device list protection
    device_mutex = xSemaphoreCreateMutex();
    if (device_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create device mutex");
        return ESP_FAIL;
    }
    
    // Initialize BT controller if not already done
    if (!esp_bt_controller_get_status()) {
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Initialize Bluedroid if not already done
    if (!esp_bluedroid_get_status()) {
        esp_err_t ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        
        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    
    // Register GAP callback
    esp_err_t ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE scanner initialized successfully");
    return ESP_OK;
}

esp_err_t ble_scanner_start(void)
{
    ESP_LOGI(TAG, "Starting BLE scanner...");
    
    esp_err_t ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret) {
        ESP_LOGE(TAG, "Set scan params failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_ble_gap_start_scanning(SCAN_DURATION);
    if (ret) {
        ESP_LOGE(TAG, "Start scanning failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    scanner_status = SCANNER_STATUS_SCANNING;
    ESP_LOGI(TAG, "BLE scanner started");
    return ESP_OK;
}

esp_err_t ble_scanner_stop(void)
{
    ESP_LOGI(TAG, "Stopping BLE scanner...");
    
    esp_err_t ret = esp_ble_gap_stop_scanning();
    if (ret) {
        ESP_LOGE(TAG, "Stop scanning failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    scanner_status = SCANNER_STATUS_IDLE;
    ESP_LOGI(TAG, "BLE scanner stopped");
    return ESP_OK;
}

esp_err_t ble_scanner_set_data_callback(wearable_data_callback_t callback)
{
    data_callback = callback;
    return ESP_OK;
}

scanner_status_t ble_scanner_get_status(void)
{
    return scanner_status;
}

uint8_t ble_scanner_get_device_count(void)
{
    return device_count;
}

esp_err_t ble_scanner_get_devices(wearable_device_t *devices, uint8_t max_count, uint8_t *actual_count)
{
    if (devices == NULL || actual_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *actual_count = 0;
    
    if (xSemaphoreTake(device_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t count = (device_count < max_count) ? device_count : max_count;
        memcpy(devices, discovered_devices, count * sizeof(wearable_device_t));
        *actual_count = count;
        
        xSemaphoreGive(device_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t ble_scanner_add_whitelist(const uint8_t *device_id)
{
    if (device_id == NULL || whitelist_count >= MAX_WEARABLE_DEVICES) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check if already in whitelist
    for (uint8_t i = 0; i < whitelist_count; i++) {
        if (memcmp(whitelist[i], device_id, DEVICE_ID_LEN) == 0) {
            return ESP_OK; // Already exists
        }
    }
    
    memcpy(whitelist[whitelist_count], device_id, DEVICE_ID_LEN);
    whitelist_count++;
    use_whitelist = true;
    
    ESP_LOGI(TAG, "Added device to whitelist: %02x:%02x:%02x:%02x:%02x:%02x",
             device_id[0], device_id[1], device_id[2], 
             device_id[3], device_id[4], device_id[5]);
    
    return ESP_OK;
}

esp_err_t ble_scanner_remove_whitelist(const uint8_t *device_id)
{
    if (device_id == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    for (uint8_t i = 0; i < whitelist_count; i++) {
        if (memcmp(whitelist[i], device_id, DEVICE_ID_LEN) == 0) {
            // Move last element to this position
            if (i < whitelist_count - 1) {
                memcpy(whitelist[i], whitelist[whitelist_count - 1], DEVICE_ID_LEN);
            }
            whitelist_count--;
            
            if (whitelist_count == 0) {
                use_whitelist = false;
            }
            
            ESP_LOGI(TAG, "Removed device from whitelist");
            return ESP_OK;
        }
    }
    
    return ESP_ERR_NOT_FOUND;
}

esp_err_t ble_scanner_clear_whitelist(void)
{
    whitelist_count = 0;
    use_whitelist = false;
    ESP_LOGI(TAG, "Whitelist cleared");
    return ESP_OK;
}

/* Private function implementations */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Scan parameters set complete");
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Scan started successfully");
                scanner_status = SCANNER_STATUS_SCANNING;
            } else {
                ESP_LOGE(TAG, "Scan start failed");
                scanner_status = SCANNER_STATUS_ERROR;
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            esp_ble_gap_cb_param_t *scan_result = param;
            
            switch (scan_result->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT: {
                    // Check if device is whitelisted (if whitelist is enabled)
                    if (use_whitelist && !is_device_whitelisted(scan_result->scan_rst.bda)) {
                        break;
                    }
                    
                    // Parse advertising data
                    wearable_data_t sensor_data;
                    if (parse_advertising_data(scan_result->scan_rst.ble_adv, 
                                             scan_result->scan_rst.adv_data_len, 
                                             &sensor_data) == ESP_OK) {
                        
                        // Find or create device entry
                        wearable_device_t *device = find_or_create_device(scan_result->scan_rst.bda, 
                                                                         "SafeLink-Wearable");
                        
                        if (device != NULL) {
                            // Update device data
                            update_device_data(device, &sensor_data, scan_result->scan_rst.rssi);
                            
                            // Call callback if registered
                            if (data_callback != NULL) {
                                data_callback(device, &sensor_data);
                            }
                            
                            ESP_LOGD(TAG, "Received data from device %s: HR=%.1f, Temp=%.1f",
                                     device->name, sensor_data.heart_rate, sensor_data.skin_temperature);
                        }
                    }
                    break;
                }
                
                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                    ESP_LOGD(TAG, "Scan complete");
                    break;
                    
                default:
                    break;
            }
            break;
        }
        
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Scan stopped successfully");
                scanner_status = SCANNER_STATUS_IDLE;
            } else {
                ESP_LOGE(TAG, "Scan stop failed");
            }
            break;
            
        default:
            break;
    }
}

static esp_err_t parse_advertising_data(const uint8_t *adv_data, uint8_t adv_len, 
                                       wearable_data_t *sensor_data)
{
    if (adv_data == NULL || sensor_data == NULL || adv_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uint8_t pos = 0;
    
    while (pos < adv_len) {
        uint8_t length = adv_data[pos];
        if (length == 0 || pos + length >= adv_len) {
            break;
        }
        
        uint8_t type = adv_data[pos + 1];
        
        // Look for manufacturer specific data
        if (type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE && length >= 15) {
            uint16_t company_id = (adv_data[pos + 3] << 8) | adv_data[pos + 2];
            
            if (company_id == SAFELINK_MANUFACTURER_ID) {
                // Parse SafeLink sensor data
                const uint8_t *data = &adv_data[pos + 4];
                
                // Extract sensor values (assuming little-endian format)
                uint16_t hr_raw = (data[1] << 8) | data[0];
                uint16_t temp_raw = (data[3] << 8) | data[2];
                uint16_t ambient_raw = (data[5] << 8) | data[4];
                uint16_t humidity_raw = (data[7] << 8) | data[6];
                uint16_t wbgt_raw = (data[9] << 8) | data[8];
                
                // Convert to actual values (scale factors may need adjustment)
                sensor_data->heart_rate = hr_raw / 10.0f;
                sensor_data->skin_temperature = temp_raw / 100.0f;
                sensor_data->ambient_temp = ambient_raw / 100.0f;
                sensor_data->humidity = humidity_raw / 100.0f;
                sensor_data->wbgt = wbgt_raw / 100.0f;
                sensor_data->timestamp = esp_timer_get_time() / 1000;
                
                return ESP_OK;
            }
        }
        
        pos += length + 1;
    }
    
    return ESP_ERR_NOT_FOUND;
}

static wearable_device_t* find_or_create_device(const uint8_t *device_id, const char *name)
{
    if (xSemaphoreTake(device_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return NULL;
    }
    
    // Search for existing device
    for (uint8_t i = 0; i < device_count; i++) {
        if (memcmp(discovered_devices[i].device_id, device_id, DEVICE_ID_LEN) == 0) {
            xSemaphoreGive(device_mutex);
            return &discovered_devices[i];
        }
    }
    
    // Create new device if space available
    if (device_count < MAX_WEARABLE_DEVICES) {
        wearable_device_t *device = &discovered_devices[device_count];
        
        memcpy(device->device_id, device_id, DEVICE_ID_LEN);
        strncpy(device->name, name, DEVICE_NAME_MAX_LEN - 1);
        device->name[DEVICE_NAME_MAX_LEN - 1] = '\0';
        device->is_connected = true;
        device->data_updated = false;
        device->last_seen = esp_timer_get_time() / 1000;
        device->packet_count = 0;
        device->error_count = 0;
        
        device_count++;
        
        ESP_LOGI(TAG, "New device discovered: %02x:%02x:%02x:%02x:%02x:%02x",
                 device_id[0], device_id[1], device_id[2],
                 device_id[3], device_id[4], device_id[5]);
        
        xSemaphoreGive(device_mutex);
        return device;
    }
    
    xSemaphoreGive(device_mutex);
    ESP_LOGW(TAG, "Device list full, cannot add new device");
    return NULL;
}

static bool is_device_whitelisted(const uint8_t *device_id)
{
    for (uint8_t i = 0; i < whitelist_count; i++) {
        if (memcmp(whitelist[i], device_id, DEVICE_ID_LEN) == 0) {
            return true;
        }
    }
    return false;
}

static void update_device_data(wearable_device_t *device, const wearable_data_t *data, int8_t rssi)
{
    if (device == NULL || data == NULL) {
        return;
    }
    
    device->sensor_data = *data;
    device->sensor_data.rssi = rssi;
    device->is_connected = true;
    device->data_updated = true;
    device->last_seen = esp_timer_get_time() / 1000;
    device->packet_count++;
} 
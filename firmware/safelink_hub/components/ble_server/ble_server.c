/*
 * BLE Server Implementation
 */

#include "ble_server.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

#define TAG "BLE_SERVER"

#define GATTS_APP_ID            1
#define GATTS_NUM_HANDLE        20

static ble_server_status_t server_status = BLE_SERVER_DISCONNECTED;
static uint16_t gatts_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static uint16_t service_handle = 0;

/* Characteristic handles */
static uint16_t hub_status_handle = 0;
static uint16_t wearable_list_handle = 0;
static uint16_t safety_alert_handle = 0;
static uint16_t hub_config_handle = 0;
static uint16_t noise_data_handle = 0;

/* Hub configuration */
static hub_config_t hub_config = {
    .noise_warning_threshold = 90.0f,
    .noise_critical_threshold = 105.0f,
    .scan_interval_ms = 100,
    .alert_volume = 70,
    .vibration_intensity = 80,
    .enable_voice_alerts = true,
    .enable_led_alerts = true,
    .enable_vibration_alerts = true
};

static SemaphoreHandle_t config_mutex = NULL;

/* BLE advertising data */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(uint16_t),
    .p_service_uuid = (uint8_t*)&SAFELINK_HUB_SERVICE_UUID,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* Private function declarations */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void create_service(void);
static void handle_write_event(esp_ble_gatts_cb_param_t *param);

esp_err_t ble_server_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE server...");
    
    // Create config mutex
    config_mutex = xSemaphoreCreateMutex();
    if (config_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create config mutex");
        return ESP_FAIL;
    }
    
    esp_err_t ret;
    
    // Initialize BT controller if not already done
    if (!esp_bt_controller_get_status()) {
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
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
        ret = esp_bluedroid_init();
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
    
    // Register GATT server callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register GAP callbacks
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register GATT server app
    ret = esp_ble_gatts_app_register(GATTS_APP_ID);
    if (ret) {
        ESP_LOGE(TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Set device name
    ret = esp_ble_gap_set_device_name("SafeLink-Hub");
    if (ret) {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE server initialized successfully");
    return ESP_OK;
}

esp_err_t ble_server_start_advertising(void)
{
    ESP_LOGI(TAG, "Starting BLE advertising...");
    
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    server_status = BLE_SERVER_ADVERTISING;
    return ESP_OK;
}

esp_err_t ble_server_stop_advertising(void)
{
    ESP_LOGI(TAG, "Stopping BLE advertising...");
    
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret) {
        ESP_LOGE(TAG, "Stop advertising failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    server_status = BLE_SERVER_DISCONNECTED;
    return ESP_OK;
}

esp_err_t ble_server_send_hub_data(const hub_status_t *status)
{
    if (server_status != BLE_SERVER_CONNECTED || status == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, hub_status_handle,
                                               sizeof(hub_status_t), (uint8_t*)status, false);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send hub data failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t ble_server_send_wearable_list(const wearable_device_t *devices, uint8_t count)
{
    if (server_status != BLE_SERVER_CONNECTED || devices == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Send device count first, then individual devices
    uint8_t data[sizeof(uint8_t) + MAX_WEARABLE_DEVICES * sizeof(wearable_device_t)];
    data[0] = count;
    
    if (count > 0) {
        memcpy(&data[1], devices, count * sizeof(wearable_device_t));
    }
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, wearable_list_handle,
                                               sizeof(uint8_t) + count * sizeof(wearable_device_t), 
                                               data, false);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send wearable list failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t ble_server_send_alert(const mobile_alert_t *alert)
{
    if (server_status != BLE_SERVER_CONNECTED || alert == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, safety_alert_handle,
                                               sizeof(mobile_alert_t), (uint8_t*)alert, true);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send alert failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Alert sent to app: %s", alert->message);
    }
    
    return ret;
}

esp_err_t ble_server_send_noise_data(float noise_level, uint64_t timestamp)
{
    if (server_status != BLE_SERVER_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    struct {
        float noise_level;
        uint64_t timestamp;
    } noise_data = {
        .noise_level = noise_level,
        .timestamp = timestamp
    };
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, noise_data_handle,
                                               sizeof(noise_data), (uint8_t*)&noise_data, false);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send noise data failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t ble_server_send_battery_alert(uint8_t battery_percentage)
{
    mobile_alert_t alert = {
        .type = ALERT_TYPE_LOW_BATTERY,
        .severity = (battery_percentage < 10) ? 2 : 1,
        .timestamp = esp_timer_get_time() / 1000
    };
    
    snprintf(alert.message, sizeof(alert.message), "허브 배터리 부족: %d%%", battery_percentage);
    strcpy(alert.device_id, "Hub");
    
    return ble_server_send_alert(&alert);
}

ble_server_status_t ble_server_get_status(void)
{
    return server_status;
}

bool ble_server_is_connected(void)
{
    return (server_status == BLE_SERVER_CONNECTED);
}

esp_err_t ble_server_get_config(hub_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *config = hub_config;
        xSemaphoreGive(config_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

/* Private function implementations */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if_param, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "GATT server registered, app_id: %d", param->reg.app_id);
            gatts_if = gatts_if_param;
            create_service();
            break;
            
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG, "Service created, service_handle: %d", param->create.service_handle);
            service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(service_handle);
            break;
            
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "Client connected, conn_id: %d", param->connect.conn_id);
            conn_id = param->connect.conn_id;
            server_status = BLE_SERVER_CONNECTED;
            esp_ble_gap_stop_advertising();
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected");
            server_status = BLE_SERVER_DISCONNECTED;
            ble_server_start_advertising();
            break;
            
        case ESP_GATTS_WRITE_EVT:
            handle_write_event(param);
            break;
            
        default:
            break;
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "Advertising data set complete");
            esp_ble_gap_start_advertising(&adv_params);
            break;
            
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Advertising started successfully");
            } else {
                ESP_LOGE(TAG, "Advertising start failed");
            }
            break;
            
        default:
            break;
    }
}

static void create_service(void)
{
    esp_gatt_srvc_id_t service_id = {
        .is_primary = true,
        .id.inst_id = 0x00,
        .id.uuid.len = ESP_UUID_LEN_16,
        .id.uuid.uuid.uuid16 = SAFELINK_HUB_SERVICE_UUID,
    };
    
    esp_err_t ret = esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
    if (ret) {
        ESP_LOGE(TAG, "Create service failed: %s", esp_err_to_name(ret));
    }
}

static void handle_write_event(esp_ble_gatts_cb_param_t *param)
{
    if (param->write.handle == hub_config_handle) {
        // Handle configuration update from app
        if (param->write.len == sizeof(hub_config_t)) {
            if (xSemaphoreTake(config_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                memcpy(&hub_config, param->write.value, sizeof(hub_config_t));
                xSemaphoreGive(config_mutex);
                
                ESP_LOGI(TAG, "Hub configuration updated from app");
                
                // Send response
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, 
                                          ESP_GATT_OK, NULL);
            }
        }
    }
} 
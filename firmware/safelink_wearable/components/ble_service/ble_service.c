/*
 * BLE Service Implementation
 */

#include "ble_service.h"
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

#define TAG "BLE_SERVICE"

#define GATTS_APP_ID            0
#define GATTS_NUM_HANDLE        10

static ble_connection_status_t connection_status = BLE_DISCONNECTED;
static uint16_t gatts_if = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static uint16_t service_handle = 0;
static uint16_t sensor_char_handle = 0;
static uint16_t alert_char_handle = 0;
static uint16_t config_char_handle = 0;

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
    .p_service_uuid = (uint8_t*)&SAFELINK_SERVICE_UUID,
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

esp_err_t ble_service_init(void)
{
    ESP_LOGI(TAG, "Initializing BLE service...");
    
    esp_err_t ret;
    
    // Initialize BT controller
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
    
    // Initialize Bluedroid
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
    ret = esp_ble_gap_set_device_name("SafeLink-Wearable");
    if (ret) {
        ESP_LOGE(TAG, "Set device name failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BLE service initialized successfully");
    return ESP_OK;
}

esp_err_t ble_service_start_advertising(void)
{
    ESP_LOGI(TAG, "Starting BLE advertising...");
    
    esp_err_t ret = esp_ble_gap_start_advertising(&adv_params);
    if (ret) {
        ESP_LOGE(TAG, "Start advertising failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    connection_status = BLE_ADVERTISING;
    return ESP_OK;
}

esp_err_t ble_service_stop_advertising(void)
{
    ESP_LOGI(TAG, "Stopping BLE advertising...");
    
    esp_err_t ret = esp_ble_gap_stop_advertising();
    if (ret) {
        ESP_LOGE(TAG, "Stop advertising failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    connection_status = BLE_DISCONNECTED;
    return ESP_OK;
}

esp_err_t ble_service_send_sensor_data(const sensor_data_t *data)
{
    if (connection_status != BLE_CONNECTED || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create JSON-like data string
    char json_data[256];
    snprintf(json_data, sizeof(json_data),
             "{\"hr\":%.1f,\"st\":%.1f,\"at\":%.1f,\"hum\":%.1f,\"wbgt\":%.1f,\"ts\":%llu}",
             data->heart_rate, data->skin_temperature, data->ambient_temp,
             data->humidity, data->wbgt, data->timestamp);
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, sensor_char_handle,
                                               strlen(json_data), (uint8_t*)json_data, false);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send sensor data failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

esp_err_t ble_service_send_alert(alert_level_t level, const char *message)
{
    if (connection_status != BLE_CONNECTED || message == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Create alert data
    char alert_data[128];
    snprintf(alert_data, sizeof(alert_data), "{\"level\":%d,\"msg\":\"%s\"}", level, message);
    
    esp_err_t ret = esp_ble_gatts_send_indicate(gatts_if, conn_id, alert_char_handle,
                                               strlen(alert_data), (uint8_t*)alert_data, true);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send alert failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

ble_connection_status_t ble_service_get_status(void)
{
    return connection_status;
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
            connection_status = BLE_CONNECTED;
            esp_ble_gap_stop_advertising();
            break;
            
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "Client disconnected");
            connection_status = BLE_DISCONNECTED;
            ble_service_start_advertising();
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
        .id.uuid.uuid.uuid16 = SAFELINK_SERVICE_UUID,
    };
    
    esp_err_t ret = esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
    if (ret) {
        ESP_LOGE(TAG, "Create service failed: %s", esp_err_to_name(ret));
    }
} 
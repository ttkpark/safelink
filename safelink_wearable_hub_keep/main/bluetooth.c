#include "bluetooth.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <string.h>

static const char *TAG = "NIMBLE_BLE";

// Global variables
static bluetooth_state_t current_state = BLUETOOTH_STATE_DISCONNECTED;
static uint16_t conn_handle = 0xFFFF;
static EventGroupHandle_t ble_event_group = NULL;
static uint8_t own_addr_type;

// GATT service handles
static uint16_t heart_rate_char_handle = 0;
static uint16_t temperature_char_handle = 0;
static uint16_t temperature_type_char_handle = 0;
static uint16_t humidity_char_handle = 0;
static uint16_t pressure_char_handle = 0;
static uint16_t sensor_data_char_handle = 0;
static uint16_t health_status_char_handle = 0;

// Static data buffers for characteristics
static uint16_t heart_rate_value = 75;
static uint32_t temp_humidity_value = 0;
static uint8_t control_value = 0;

// Health monitoring functions
health_status_t analyze_health_status(uint16_t heart_rate, uint16_t temperature, uint16_t humidity)
{
    // Convert to actual values (temperature and humidity are stored × 10)
    float temp_celsius = (float)temperature / 10.0f;
    float hum_percent = (float)humidity / 10.0f;
    
    int warning_count = 0;
    
    // Heart rate analysis (BPM)
    if (heart_rate > 100) {
        warning_count++;
    } else if (heart_rate < 60) {
        warning_count++;
    }
    
    // Temperature analysis (°C)
    if (temp_celsius > 37.5) {
        warning_count++;
    } else if (temp_celsius < 35.0) {
        warning_count++;
    }
    
    // Humidity analysis (%)
    if (hum_percent < 30.0) {
        warning_count++;
    } else if (hum_percent > 70.0) {
        warning_count++;
    }
    
    // Determine health status
    if (warning_count == 0) {
        return HEALTH_STATUS_NORMAL;
    } else if (warning_count == 1) {
        if (heart_rate > 100 || heart_rate < 60) {
            return HEALTH_STATUS_ELEVATED_HR;
        } else if (temp_celsius > 37.5 || temp_celsius < 35.0) {
            return HEALTH_STATUS_HIGH_TEMP;
        } else {
            return HEALTH_STATUS_LOW_HUMIDITY;
        }
    } else if (warning_count == 2) {
        return HEALTH_STATUS_WARNING;
    } else {
        return HEALTH_STATUS_CRITICAL;
    }
}

const char* get_health_status_string(health_status_t status)
{
    switch (status) {
        case HEALTH_STATUS_NORMAL:
            return "Normal";
        case HEALTH_STATUS_ELEVATED_HR:
            return "Elevated Heart Rate";
        case HEALTH_STATUS_HIGH_TEMP:
            return "Temperature Alert";
        case HEALTH_STATUS_LOW_HUMIDITY:
            return "Humidity Alert";
        case HEALTH_STATUS_WARNING:
            return "Warning";
        case HEALTH_STATUS_CRITICAL:
            return "Critical";
        default:
            return "Unknown";
    }
}

// GATT access callback implementation
static int gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const struct ble_gatt_chr_def *chr = ctxt->chr;
    const ble_uuid_t *uuid;
    
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            uuid = chr->uuid;
            if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HEART_RATE_CHAR_UUID)) == 0) {
                // Heart Rate Measurement characteristic format
                uint8_t heart_rate_data[2];
                heart_rate_data[0] = 0x00; // Flags: no energy expended, no RR intervals
                heart_rate_data[1] = heart_rate_value & 0xFF;
                os_mbuf_append(ctxt->om, heart_rate_data, sizeof(heart_rate_data));
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(TEMPERATURE_CHAR_UUID)) == 0) {
                // Temperature Measurement characteristic format (IEEE 11073-20601)
                // Flags (1 byte) + Temperature (4 bytes) + Time Stamp (7 bytes) + Temperature Type (1 byte)
                uint8_t temp_data[13];
                temp_data[0] = 0x00; // Flags: no time stamp, no temperature type
                
                // Temperature value (4 bytes, IEEE 11073-20601 format)
                int32_t temp_value = temp_humidity_value >> 16; // Temperature in 0.01 degrees Celsius
                temp_data[1] = temp_value & 0xFF;
                temp_data[2] = (temp_value >> 8) & 0xFF;
                temp_data[3] = (temp_value >> 16) & 0xFF;
                temp_data[4] = (temp_value >> 24) & 0xFF;
                
                // Time Stamp (7 bytes) - 현재 시간
                uint32_t timestamp = esp_timer_get_time() / 1000000; // 초 단위
                temp_data[5] = timestamp & 0xFF;
                temp_data[6] = (timestamp >> 8) & 0xFF;
                temp_data[7] = (timestamp >> 16) & 0xFF;
                temp_data[8] = (timestamp >> 24) & 0xFF;
                temp_data[9] = 0x00; // Year (1900년부터)
                temp_data[10] = 0x00; // Month
                temp_data[11] = 0x00; // Day
                
                // Temperature Type (1 byte)
                temp_data[12] = 0x02; // Body (general)
                
                os_mbuf_append(ctxt->om, temp_data, sizeof(temp_data));
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(TEMPERATURE_TYPE_CHAR_UUID)) == 0) {
                // Temperature Type: 1 = Armpit, 2 = Body (general), 3 = Ear, 4 = Finger, 5 = Gastro-intestinal, 6 = Mouth, 7 = Rectum, 8 = Toe, 9 = Tympanum
                uint8_t temp_type = 2; // Body (general)
                os_mbuf_append(ctxt->om, &temp_type, sizeof(temp_type));
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HUMIDITY_CHAR_UUID)) == 0) {
                // Humidity characteristic format (IEEE 11073-20601)
                // Flags (1 byte) + Humidity (4 bytes)
                uint8_t humidity_data[5];
                humidity_data[0] = 0x00; // Flags: no time stamp
                
                // Humidity value (4 bytes, IEEE 11073-20601 format)
                int32_t humidity_value = temp_humidity_value & 0xFFFF; // Humidity in 0.01% units
                humidity_data[1] = humidity_value & 0xFF;
                humidity_data[2] = (humidity_value >> 8) & 0xFF;
                humidity_data[3] = (humidity_value >> 16) & 0xFF;
                humidity_data[4] = (humidity_value >> 24) & 0xFF;
                
                os_mbuf_append(ctxt->om, humidity_data, sizeof(humidity_data));
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(PRESSURE_CHAR_UUID)) == 0) {
                // Pressure characteristic format (IEEE 11073-20601)
                // Flags (1 byte) + Pressure (4 bytes)
                uint8_t pressure_data[5];
                pressure_data[0] = 0x00; // Flags: no time stamp
                
                // Pressure value (4 bytes, IEEE 11073-20601 format)
                // 표준 대기압을 기본값으로 설정 (1013.25 hPa)
                int32_t pressure_value = 101325; // Pressure in 0.1 Pa units
                pressure_data[1] = pressure_value & 0xFF;
                pressure_data[2] = (pressure_value >> 8) & 0xFF;
                pressure_data[3] = (pressure_value >> 16) & 0xFF;
                pressure_data[4] = (pressure_value >> 24) & 0xFF;
                
                os_mbuf_append(ctxt->om, pressure_data, sizeof(pressure_data));
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(SENSOR_DATA_CHAR_UUID)) == 0) {
                // 통합 센서 데이터 (심박수 + 온도 + 습도)
                uint8_t sensor_data[8];
                sensor_data[0] = heart_rate_value & 0xFF;           // 심박수 (1바이트)
                sensor_data[1] = (heart_rate_value >> 8) & 0xFF;    // 심박수 (상위 바이트)
                sensor_data[2] = (temp_humidity_value >> 16) & 0xFF; // 온도 하위 바이트
                sensor_data[3] = (temp_humidity_value >> 24) & 0xFF; // 온도 상위 바이트
                sensor_data[4] = temp_humidity_value & 0xFF;         // 습도 하위 바이트
                sensor_data[5] = (temp_humidity_value >> 8) & 0xFF;  // 습도 상위 바이트
                sensor_data[6] = 0x00; // 예약
                sensor_data[7] = 0x00; // 예약
                os_mbuf_append(ctxt->om, sensor_data, sizeof(sensor_data));
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HEALTH_STATUS_CHAR_UUID)) == 0) {
                // 건강 상태 문자열 전송
                const char* status_str = get_health_status_string(analyze_health_status(
                    heart_rate_value, 
                    temp_humidity_value >> 16, 
                    temp_humidity_value & 0xFFFF
                ));
                os_mbuf_append(ctxt->om, status_str, strlen(status_str));
                return 0;
            }
            break;
            
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            // 표준 Heart Rate Service는 쓰기 권한이 없음
            break;
            
        default:
            break;
    }
    
    return BLE_ATT_ERR_UNLIKELY;
}

// GATT service definition
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(HEART_RATE_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(HEART_RATE_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &heart_rate_char_handle,
            },
            {0} // end
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(TEMPERATURE_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(TEMPERATURE_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &temperature_char_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(TEMPERATURE_TYPE_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
                .val_handle = &temperature_type_char_handle,
            },
            {0} // end
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(ENVIRONMENTAL_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(HUMIDITY_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &humidity_char_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(PRESSURE_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &pressure_char_handle,
            },
            {0} // end
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(CUSTOM_SENSOR_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(SENSOR_DATA_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &sensor_data_char_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(HEALTH_STATUS_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &health_status_char_handle,
            },
            {0} // end
        }
    },
    {0} // end
};

// GAP event handler
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
                current_state = BLUETOOTH_STATE_CONNECTED;
                ESP_LOGI(TAG, "=== BLE CONNECTION ESTABLISHED ===");
                ESP_LOGI(TAG, "Connection handle: %d", conn_handle);
                ESP_LOGI(TAG, "=== END CONNECTION INFO ===");
                if (ble_event_group) {
                    xEventGroupSetBits(ble_event_group, BLE_CONNECTED_BIT);
                }
            } else {
                ESP_LOGE(TAG, "Connection failed; restarting advertising");
                ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, 
                                 &(struct ble_gap_adv_params){0}, ble_gap_event_cb, NULL);
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "=== BLE CONNECTION LOST ===");
            ESP_LOGI(TAG, "Reason: %d", event->disconnect.reason);
            ESP_LOGI(TAG, "=== END DISCONNECT INFO ===");
            conn_handle = 0xFFFF;
            current_state = BLUETOOTH_STATE_DISCONNECTED;
            if (ble_event_group) {
                xEventGroupSetBits(ble_event_group, BLE_DISCONNECTED_BIT);
            }
            // Restart advertising after disconnect
            ESP_LOGI(TAG, "BLE advertising restarted after disconnect");
            ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, 
                             &(struct ble_gap_adv_params){0}, ble_gap_event_cb, NULL);
            break;
            
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete");
            break;
            
        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG, "Subscribe event; cur_notify=%d", event->subscribe.cur_notify);
            break;
            
        default:
            break;
    }
    
    return 0;
}

// BLE host task
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// BLE sync callback
static void ble_on_sync(void)
{
    int rc;
    
    // Set device name
    rc = ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name; rc=%d", rc);
        return;
    }
    
    // Configure the address
    ble_hs_id_infer_auto(0, &own_addr_type);
    
    // Create advertising data with device name
    struct ble_hs_adv_fields adv_fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .tx_pwr_lvl = 0,  // 0 dBm
        .tx_pwr_lvl_is_present = 1,
        .name = (uint8_t *)BLE_DEVICE_NAME,
        .name_len = strlen(BLE_DEVICE_NAME),
        .name_is_complete = 1,
    };
    
    // Set advertising data
    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data; rc=%d", rc);
        return;
    }
    
    // Begin advertising
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = BLE_GAP_ADV_ITVL_MS(100),
        .itvl_max = BLE_GAP_ADV_ITVL_MS(200),
    };
    
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising; rc=%d", rc);
        return;
    }
    
    ESP_LOGI(TAG, "=== BLE ADVERTISING SUCCESS ===");
    ESP_LOGI(TAG, "Device should now be visible in nRF Connect");
    ESP_LOGI(TAG, "Look for device named: %s", BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "=== END SUCCESS INFO ===");
    current_state = BLUETOOTH_STATE_ADVERTISING;
}

// Initialize BLE
esp_err_t bluetooth_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing NimBLE...");
    
    // Create event group
    ble_event_group = xEventGroupCreate();
    if (!ble_event_group) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize NimBLE
    nimble_port_init();
    
    // Initialize GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Configure GATT services
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);
    
    // Set sync callback
    ble_hs_cfg.sync_cb = ble_on_sync;
    
    // Create host task
    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "NimBLE initialization completed");
    return ESP_OK;
}

// Deinitialize BLE
esp_err_t bluetooth_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing NimBLE...");
    
    if (ble_event_group) {
        vEventGroupDelete(ble_event_group);
        ble_event_group = NULL;
    }
    
    current_state = BLUETOOTH_STATE_DISCONNECTED;
    conn_handle = 0xFFFF;
    
    nimble_port_stop();
    nimble_port_deinit();
    
    ESP_LOGI(TAG, "NimBLE deinitialization completed");
    return ESP_OK;
}

// Start advertising
esp_err_t bluetooth_start_advertising(void)
{
    if (current_state == BLUETOOTH_STATE_ADVERTISING) {
        ESP_LOGW(TAG, "Already advertising");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting BLE advertising...");
    current_state = BLUETOOTH_STATE_ADVERTISING;
    
    // Create advertising data with device name
    struct ble_hs_adv_fields adv_fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .tx_pwr_lvl = 0,  // 0 dBm
        .tx_pwr_lvl_is_present = 1,
        .name = (uint8_t *)BLE_DEVICE_NAME,
        .name_len = strlen(BLE_DEVICE_NAME),
        .name_is_complete = 1,
    };
    
    // Set advertising data
    int rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data; rc=%d", rc);
        return ESP_FAIL;
    }
    
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = BLE_GAP_ADV_ITVL_MS(100),
        .itvl_max = BLE_GAP_ADV_ITVL_MS(200),
    };
    
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Start advertising failed; rc=%d", rc);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Stop advertising
esp_err_t bluetooth_stop_advertising(void)
{
    ESP_LOGI(TAG, "Stopping BLE advertising...");
    int rc = ble_gap_adv_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "Stop advertising failed; rc=%d", rc);
        return ESP_FAIL;
    }
    current_state = BLUETOOTH_STATE_DISCONNECTED;
    return ESP_OK;
}

// Send sensor data via BLE notification
esp_err_t bluetooth_send_sensor_data(const sensor_data_t *data)
{
    if (conn_handle == 0xFFFF) {
        ESP_LOGW(TAG, "No BLE connection available");
        return ESP_FAIL;
    }
    
    if (!data) {
        ESP_LOGE(TAG, "Invalid sensor data");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Sending sensor data - HR: %d, Temp: %.1f°C, Humidity: %.1f%%",
             data->heart_rate, data->temperature / 10.0f, data->humidity / 10.0f);
    
    // Send heart rate data
    if (heart_rate_char_handle != 0) {
        heart_rate_value = data->heart_rate;
        int rc = ble_gatts_notify(conn_handle, heart_rate_char_handle);
        if (rc != 0) {
            ESP_LOGE(TAG, "Send heart rate data failed; rc=%d", rc);
        }
    }
    
    // Send temperature data (Health Thermometer Service)
    if (temperature_char_handle != 0) {
        temp_humidity_value = (data->temperature << 16) | data->humidity;
        int rc = ble_gatts_notify(conn_handle, temperature_char_handle);
        if (rc != 0) {
            ESP_LOGE(TAG, "Send temperature data failed; rc=%d", rc);
        }
    }
    
    // Send humidity data (Environmental Sensing Service)
    if (humidity_char_handle != 0) {
        int rc = ble_gatts_notify(conn_handle, humidity_char_handle);
        if (rc != 0) {
            ESP_LOGE(TAG, "Send humidity data failed; rc=%d", rc);
        }
    }
    
    // Send pressure data (Environmental Sensing Service)
    if (pressure_char_handle != 0) {
        int rc = ble_gatts_notify(conn_handle, pressure_char_handle);
        if (rc != 0) {
            ESP_LOGE(TAG, "Send pressure data failed; rc=%d", rc);
        }
    }
    
    // Send custom sensor data (통합 데이터)
    if (sensor_data_char_handle != 0) {
        int rc = ble_gatts_notify(conn_handle, sensor_data_char_handle);
        if (rc != 0) {
            ESP_LOGE(TAG, "Send custom sensor data failed; rc=%d", rc);
        }
    }
    
    // Send health status
    if (health_status_char_handle != 0) {
        int rc = ble_gatts_notify(conn_handle, health_status_char_handle);
        if (rc != 0) {
            ESP_LOGE(TAG, "Send health status failed; rc=%d", rc);
        }
    }
    
    return ESP_OK;
}

// Get current BLE state
bluetooth_state_t bluetooth_get_state(void)
{
    return current_state;
}

// Get connected device address
uint16_t bluetooth_get_connected_device(uint8_t *addr)
{
    if (conn_handle == 0xFFFF) {
        return 0;
    }
    
    return conn_handle;
}

// Initialize GATT sensor service
void gatt_sensor_service_init(void)
{
    ESP_LOGI(TAG, "GATT sensor service initialization");
    ESP_LOGI(TAG, "Heart Rate Service UUID: 0x%04x", HEART_RATE_SERVICE_UUID);
    ESP_LOGI(TAG, "Heart Rate Char UUID: 0x%04x", HEART_RATE_CHAR_UUID);
    ESP_LOGI(TAG, "Temperature Service UUID: 0x%04x", TEMPERATURE_SERVICE_UUID);
    ESP_LOGI(TAG, "Temperature Char UUID: 0x%04x", TEMPERATURE_CHAR_UUID);
    ESP_LOGI(TAG, "Environmental Service UUID: 0x%04x", ENVIRONMENTAL_SERVICE_UUID);
    ESP_LOGI(TAG, "Humidity Char UUID: 0x%04x", HUMIDITY_CHAR_UUID);
    ESP_LOGI(TAG, "Pressure Char UUID: 0x%04x", PRESSURE_CHAR_UUID);
    
    ESP_LOGI(TAG, "GATT service will be created automatically when BLE is initialized");
} 
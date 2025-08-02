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

static const char *TAG = "NIMBLE_CLIENT";

// Global variables
static bluetooth_state_t current_state = BLUETOOTH_STATE_DISCONNECTED;
static uint16_t conn_handle = 0xFFFF;
static EventGroupHandle_t ble_event_group = NULL;
static uint8_t own_addr_type;

// Hub discovery variables
static uint8_t target_hub_addr[6];
static uint8_t target_hub_addr_type;
static bool hub_found = false;
static int8_t best_rssi = -100;

// GATT service handles for Hub
static uint16_t heart_rate_svc_handle = 0;
static uint16_t temperature_svc_handle = 0;
static uint16_t environmental_svc_handle = 0;
static uint16_t custom_sensor_svc_handle = 0;

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

// GAP event handler
static int ble_gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_SCAN_COMPLETE:
            ESP_LOGI(TAG, "Scan complete");
            if (!hub_found) {
                ESP_LOGI(TAG, "No hub found, restarting scan");
                bluetooth_start_scanning();
            }
            break;
            
        case BLE_GAP_EVENT_DISC:
            // Device discovered during scan
            if (event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_ADV_IND ||
                event->disc.event_type == BLE_HCI_ADV_RPT_EVTYPE_DIR_IND) {
                
                // Check if this is our target hub
                if (event->disc.length_data > 0) {
                    // Parse advertising data to find device name
                    uint8_t *data = event->disc.data;
                    uint8_t len = event->disc.length_data;
                    
                    for (int i = 0; i < len; i++) {
                        if (i + 1 < len) {
                            uint8_t field_len = data[i];
                            uint8_t field_type = data[i + 1];
                            
                            if (field_type == BLE_HS_ADV_TYPE_COMP_NAME ||
                                field_type == BLE_HS_ADV_TYPE_INCOMP_NAME) {
                                
                                if (field_len > 2 && i + field_len <= len) {
                                    char name[32];
                                    uint8_t name_len = field_len - 1;
                                    if (name_len > sizeof(name) - 1) {
                                        name_len = sizeof(name) - 1;
                                    }
                                    memcpy(name, &data[i + 2], name_len);
                                    name[name_len] = '\0';
                                    
                                    ESP_LOGI(TAG, "Found device: %s (RSSI: %d)", name, event->disc.rssi);
                                    
                                    if (strcmp(name, TARGET_HUB_NAME) == 0) {
                                        // Found our target hub
                                        if (event->disc.rssi > best_rssi) {
                                            best_rssi = event->disc.rssi;
                                            memcpy(target_hub_addr, event->disc.addr.val, 6);
                                            target_hub_addr_type = event->disc.addr.type;
                                            hub_found = true;
                                            
                                            ESP_LOGI(TAG, "=== TARGET HUB FOUND ===");
                                            ESP_LOGI(TAG, "Name: %s", name);
                                            ESP_LOGI(TAG, "Address: %02x:%02x:%02x:%02x:%02x:%02x",
                                                     target_hub_addr[5], target_hub_addr[4], target_hub_addr[3],
                                                     target_hub_addr[2], target_hub_addr[1], target_hub_addr[0]);
                                            ESP_LOGI(TAG, "RSSI: %d", event->disc.rssi);
                                            ESP_LOGI(TAG, "=== END HUB INFO ===");
                                            
                                            // Stop scanning and connect
                                            bluetooth_stop_scanning();
                                            bluetooth_connect_to_hub(target_hub_addr, target_hub_addr_type);
                                        }
                                    }
                                }
                            }
                            
                            i += field_len;
                        }
                    }
                }
            }
            break;
            
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                conn_handle = event->connect.conn_handle;
                current_state = BLUETOOTH_STATE_CONNECTED;
                ESP_LOGI(TAG, "=== CONNECTED TO HUB ===");
                ESP_LOGI(TAG, "Connection handle: %d", conn_handle);
                ESP_LOGI(TAG, "=== END CONNECTION INFO ===");
                if (ble_event_group) {
                    xEventGroupSetBits(ble_event_group, BLE_CONNECTED_BIT);
                }
                
                // Start GATT service discovery
                ble_gattc_disc_all_svcs(conn_handle, NULL, NULL);
            } else {
                ESP_LOGE(TAG, "Connection failed; restarting scan");
                current_state = BLUETOOTH_STATE_DISCONNECTED;
                bluetooth_start_scanning();
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "=== DISCONNECTED FROM HUB ===");
            ESP_LOGI(TAG, "Reason: %d", event->disconnect.reason);
            ESP_LOGI(TAG, "=== END DISCONNECT INFO ===");
            conn_handle = 0xFFFF;
            current_state = BLUETOOTH_STATE_DISCONNECTED;
            hub_found = false;
            best_rssi = -100;
            if (ble_event_group) {
                xEventGroupSetBits(ble_event_group, BLE_DISCONNECTED_BIT);
            }
            // Restart scanning after disconnect
            ESP_LOGI(TAG, "Restarting scan after disconnect");
            bluetooth_start_scanning();
            break;
            
        default:
            break;
    }
    
    return 0;
}

// GATT service discovery callback
static int ble_gattc_svc_disc_cb(uint16_t conn_handle, const struct ble_gatt_error *error,
                                 const struct ble_gatt_svc *service, void *arg)
{
    if (error != NULL) {
        ESP_LOGE(TAG, "Service discovery failed; status=%d", error->status);
        return error->status;
    }
    
    if (service != NULL) {
        ESP_LOGI(TAG, "Service discovered: UUID=0x%04x, start_handle=%d, end_handle=%d",
                 service->uuid.u16.value, service->start_handle, service->end_handle);
        
        // Store service handles for later use
        if (service->uuid.u16.value == 0x180D) { // Heart Rate Service
            heart_rate_svc_handle = service->start_handle;
        } else if (service->uuid.u16.value == 0x1809) { // Health Thermometer Service
            temperature_svc_handle = service->start_handle;
        } else if (service->uuid.u16.value == 0x181A) { // Environmental Sensing Service
            environmental_svc_handle = service->start_handle;
        } else if (service->uuid.u16.value == 0x1810) { // Custom Sensor Service
            custom_sensor_svc_handle = service->start_handle;
        }
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
    
    ESP_LOGI(TAG, "=== BLE CLIENT INITIALIZED ===");
    ESP_LOGI(TAG, "Device name: %s", BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "Target hub: %s", TARGET_HUB_NAME);
    ESP_LOGI(TAG, "=== END INIT INFO ===");
    
    // Start scanning for hubs
    bluetooth_start_scanning();
}

// Initialize BLE
esp_err_t bluetooth_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Initializing NimBLE Client...");
    
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
    
    // Set sync callback
    ble_hs_cfg.sync_cb = ble_on_sync;
    
    // Create host task
    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "NimBLE Client initialization completed");
    return ESP_OK;
}

// Deinitialize BLE
esp_err_t bluetooth_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing NimBLE Client...");
    
    if (ble_event_group) {
        vEventGroupDelete(ble_event_group);
        ble_event_group = NULL;
    }
    
    current_state = BLUETOOTH_STATE_DISCONNECTED;
    conn_handle = 0xFFFF;
    hub_found = false;
    best_rssi = -100;
    
    nimble_port_stop();
    nimble_port_deinit();
    
    ESP_LOGI(TAG, "NimBLE Client deinitialization completed");
    return ESP_OK;
}

// Start scanning for hubs
esp_err_t bluetooth_start_scanning(void)
{
    if (current_state == BLUETOOTH_STATE_SCANNING) {
        ESP_LOGW(TAG, "Already scanning");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting BLE scan for hubs...");
    current_state = BLUETOOTH_STATE_SCANNING;
    
    struct ble_gap_scan_params scan_params = {
        .filter_duplicates = 1,
        .passive = 0,
        .itvl = BLE_GAP_SCAN_ITVL_MS(BLE_SCAN_INTERVAL_MIN),
        .window = BLE_GAP_SCAN_WIN_MS(BLE_SCAN_WINDOW),
        .filter_policy = 0,
        .limited = 0,
    };
    
    int rc = ble_gap_scan(own_addr_type, BLE_HS_FOREVER, &scan_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Start scanning failed; rc=%d", rc);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Stop scanning
esp_err_t bluetooth_stop_scanning(void)
{
    ESP_LOGI(TAG, "Stopping BLE scan...");
    int rc = ble_gap_scan_stop();
    if (rc != 0) {
        ESP_LOGE(TAG, "Stop scanning failed; rc=%d", rc);
        return ESP_FAIL;
    }
    return ESP_OK;
}

// Connect to hub
esp_err_t bluetooth_connect_to_hub(const uint8_t *addr, uint8_t addr_type)
{
    if (current_state == BLUETOOTH_STATE_CONNECTING || current_state == BLUETOOTH_STATE_CONNECTED) {
        ESP_LOGW(TAG, "Already connecting or connected");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Connecting to hub...");
    current_state = BLUETOOTH_STATE_CONNECTING;
    
    struct ble_gap_conn_params conn_params = {
        .scan_itvl = BLE_GAP_SCAN_ITVL_MS(BLE_CONN_INTERVAL_MIN),
        .scan_window = BLE_GAP_SCAN_WIN_MS(BLE_SCAN_WINDOW),
        .itvl_min = BLE_GAP_CONN_ITVL_MS(BLE_CONN_INTERVAL_MIN),
        .itvl_max = BLE_GAP_CONN_ITVL_MS(BLE_CONN_INTERVAL_MAX),
        .latency = 0,
        .supervision_timeout = BLE_GAP_SUPERVISION_TIMEOUT_MS(4000),
        .min_ce_len = 0,
        .max_ce_len = 0,
    };
    
    int rc = ble_gap_connect(own_addr_type, (ble_addr_t *)addr, BLE_HS_FOREVER, &conn_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Connect failed; rc=%d", rc);
        current_state = BLUETOOTH_STATE_DISCONNECTED;
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Disconnect from hub
esp_err_t bluetooth_disconnect(void)
{
    if (conn_handle == 0xFFFF) {
        ESP_LOGW(TAG, "Not connected");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Disconnecting from hub...");
    int rc = ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    if (rc != 0) {
        ESP_LOGE(TAG, "Disconnect failed; rc=%d", rc);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// Send sensor data to hub
esp_err_t bluetooth_send_sensor_data(const sensor_data_t *data)
{
    if (conn_handle == 0xFFFF) {
        ESP_LOGW(TAG, "Not connected to hub");
        return ESP_FAIL;
    }
    
    if (!data) {
        ESP_LOGE(TAG, "Invalid sensor data");
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Sending sensor data to hub - HR: %d, Temp: %.1f°C, Humidity: %.1f%%",
             data->heart_rate, data->temperature / 10.0f, data->humidity / 10.0f);
    
    // For now, just log the data. In a real implementation, you would write to Hub's characteristics
    // This would require discovering and storing characteristic handles during service discovery
    
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

// Discover hubs
esp_err_t bluetooth_discover_hubs(void)
{
    return bluetooth_start_scanning();
}

// Check if hub is connected
bool bluetooth_is_hub_connected(void)
{
    return (conn_handle != 0xFFFF);
} 
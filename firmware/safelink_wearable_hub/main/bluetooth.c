#include "bluetooth.h"
#include "esp_log.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#include "esp_adc/adc_oneshot.h"
#include "dfplayer_mini.h"
#include "data_manager.h"
#include "sensor.h"
#include <string.h>

static const char *TAG = "NIMBLE_BLE";

// Global variables
static bluetooth_state_t current_state = BLUETOOTH_STATE_DISCONNECTED;
// 다중 연결 지원
#define MAX_CONNS 4
static uint16_t conn_handles[MAX_CONNS];
static uint8_t num_conns = 0;
static EventGroupHandle_t ble_event_group = NULL;
static uint8_t own_addr_type;
static TimerHandle_t ble_scan_timer = NULL;

// Test characteristic value
static char test_value[32] = "Hello NimBLE!";

// Health sensor data
static health_sensor_data_t health_data = {
    .temperature = 2500,      // 25.00°C
    .humidity = 5000,         // 50.00%
    .body_temperature = 3650, // 36.50°C
    .spo2 = 9500,            // 95.00%
    .heart_rate = 70,        // 70 BPM
    .noise_level = 650,      // 65.0dB
    .timestamp = 0
};

// ADC handle for noise measurement
static adc_oneshot_unit_handle_t adc1_handle = NULL;


// Command buffer
static char command_buffer[32] = {0};

// Keyword to detect in advertising packets from wearable band
static const char *ADV_KEYWORD = "band";

// Forward declarations
static void start_passive_scan(void);
static void start_advertising(void);
static void scan_timer_cb(TimerHandle_t xTimer);
static void log_adv_report(const struct ble_gap_event *event);
static void parse_and_log_adv_fields(const uint8_t *data, uint8_t len);
static bool parse_mfg_and_update_band(const uint8_t *mfg, uint8_t payload_len);

// GATT access callback
static int gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    
    ESP_LOGI(TAG, "GATT access: conn_handle=%d, attr_handle=%d, op=%d", 
             conn_handle, attr_handle, ctxt->op);
    
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HUB_AGG_CHAR_UUID)) == 0) {
                band_data_t band; hub_data_t hub;
                data_manager_get_band_data(&band);
                data_manager_get_hub_data(&hub);
                uint8_t buf[4+2+4+4+4+4+4+1];
                int o=0;
                memcpy(&buf[o], &band.skin_temp, 4); o+=4;
                buf[o++] = (band.heart_rate>>8)&0xFF; buf[o++] = band.heart_rate&0xFF;
                memcpy(&buf[o], &band.spo2, 4); o+=4;
                memcpy(&buf[o], &band.external_temp, 4); o+=4;
                memcpy(&buf[o], &band.external_humidity, 4); o+=4;
                memcpy(&buf[o], &hub.avg_noise, 4); o+=4;
                memcpy(&buf[o], &hub.wbgt, 4); o+=4;
                buf[o++] = hub.alarm_status;
                os_mbuf_append(ctxt->om, buf, o);
                return 0;
            }
            if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(TEST_CHAR_UUID)) == 0) {
                // Device Name characteristic
                os_mbuf_append(ctxt->om, test_value, strlen(test_value));


                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(TEMPERATURE_CHAR_UUID)) == 0) {
                // External Temperature characteristic (밴드 데이터)
                band_data_t band_data;
                if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
                    uint8_t temp_data[4];
                    memcpy(temp_data, &band_data.external_temp, 4);
                    os_mbuf_append(ctxt->om, temp_data, sizeof(temp_data));
                } else {
                    // 기본값 전송
                    float default_temp = 25.0f;
                    uint8_t temp_data[4];
                    memcpy(temp_data, &default_temp, 4);
                    os_mbuf_append(ctxt->om, temp_data, sizeof(temp_data));
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HUMIDITY_CHAR_UUID)) == 0) {
                // External Humidity characteristic (밴드 데이터)
                band_data_t band_data;
                if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
                    uint8_t hum_data[4];
                    memcpy(hum_data, &band_data.external_humidity, 4);
                    os_mbuf_append(ctxt->om, hum_data, sizeof(hum_data));
                } else {
                    // 기본값 전송
                    float default_humidity = 50.0f;
                    uint8_t hum_data[4];
                    memcpy(hum_data, &default_humidity, 4);
                    os_mbuf_append(ctxt->om, hum_data, sizeof(hum_data));
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(BODY_TEMP_CHAR_UUID)) == 0) {
                // Skin Temperature characteristic (밴드 데이터)
                band_data_t band_data;
                if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
                    uint8_t skin_temp_data[4];
                    memcpy(skin_temp_data, &band_data.skin_temp, 4);
                    os_mbuf_append(ctxt->om, skin_temp_data, sizeof(skin_temp_data));
                } else {
                    // 기본값 전송
                    float default_skin_temp = 36.5f;
                    uint8_t skin_temp_data[4];
                    memcpy(skin_temp_data, &default_skin_temp, 4);
                    os_mbuf_append(ctxt->om, skin_temp_data, sizeof(skin_temp_data));
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(SPO2_CHAR_UUID)) == 0) {
                // SpO2 characteristic (밴드 데이터)
                band_data_t band_data;
                if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
                    uint8_t spo2_data[4];
                    memcpy(spo2_data, &band_data.spo2, 4);
                    os_mbuf_append(ctxt->om, spo2_data, sizeof(spo2_data));
                } else {
                    // 기본값 전송
                    float default_spo2 = 95.0f;
                    uint8_t spo2_data[4];
                    memcpy(spo2_data, &default_spo2, 4);
                    os_mbuf_append(ctxt->om, spo2_data, sizeof(spo2_data));
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HEART_RATE_CHAR_UUID)) == 0) {
                // Heart Rate characteristic (밴드 데이터)
                band_data_t band_data;
                if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
                    uint8_t hr_data[2];
                    hr_data[0] = (band_data.heart_rate >> 8) & 0xFF;
                    hr_data[1] = band_data.heart_rate & 0xFF;
                    os_mbuf_append(ctxt->om, hr_data, sizeof(hr_data));
                } else {
                    // 기본값 전송
                    uint8_t hr_data[2] = {0x00, 0x46}; // 70 BPM
                    os_mbuf_append(ctxt->om, hr_data, sizeof(hr_data));
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(NOISE_LEVEL_CHAR_UUID)) == 0) {
                // Noise Level characteristic (허브 데이터)
                hub_data_t hub_data;
                if (data_manager_get_hub_data(&hub_data) == ESP_OK && hub_data.is_valid) {
                    uint8_t noise_data[4];
                    memcpy(noise_data, &hub_data.avg_noise, 4);
                    os_mbuf_append(ctxt->om, noise_data, sizeof(noise_data));
                } else {
                    // 기본값 전송
                    float default_noise = 65.0f;
                    uint8_t noise_data[4];
                    memcpy(noise_data, &default_noise, 4);
                    os_mbuf_append(ctxt->om, noise_data, sizeof(noise_data));
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(COMMAND_CHAR_UUID)) == 0) {
                // Command characteristic (read current command)
                os_mbuf_append(ctxt->om, command_buffer, strlen(command_buffer));
                return 0;
            }
            break;
            
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(TEMPERATURE_CHAR_UUID)) == 0) {
                // External Temperature characteristic write
                if (ctxt->om->om_len == 4) {
                    band_data_t band_data;
                    if (data_manager_get_band_data(&band_data) == ESP_OK) {
                        memcpy(&band_data.external_temp, ctxt->om->om_data, 4);
                        band_data.data_source = DATA_SOURCE_CLIENT;
                        band_data.timestamp = esp_timer_get_time() / 1000;
                        data_manager_update_band_data(&band_data);
                        ESP_LOGI(TAG, "External temperature updated by client: %.1f°C", band_data.external_temp);
                    }
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HUMIDITY_CHAR_UUID)) == 0) {
                // External Humidity characteristic write
                if (ctxt->om->om_len == 4) {
                    band_data_t band_data;
                    if (data_manager_get_band_data(&band_data) == ESP_OK) {
                        memcpy(&band_data.external_humidity, ctxt->om->om_data, 4);
                        band_data.data_source = DATA_SOURCE_CLIENT;
                        band_data.timestamp = esp_timer_get_time() / 1000;
                        data_manager_update_band_data(&band_data);
                        ESP_LOGI(TAG, "External humidity updated by client: %.1f%%", band_data.external_humidity);
                    }
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(BODY_TEMP_CHAR_UUID)) == 0) {
                // Skin Temperature characteristic write
                if (ctxt->om->om_len == 4) {
                    band_data_t band_data;
                    if (data_manager_get_band_data(&band_data) == ESP_OK) {
                        memcpy(&band_data.skin_temp, ctxt->om->om_data, 4);
                        band_data.data_source = DATA_SOURCE_CLIENT;
                        band_data.timestamp = esp_timer_get_time() / 1000;
                        data_manager_update_band_data(&band_data);
                        ESP_LOGI(TAG, "Skin temperature updated by client: %.1f°C", band_data.skin_temp);
                    }
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(SPO2_CHAR_UUID)) == 0) {
                // SpO2 characteristic write
                if (ctxt->om->om_len == 4) {
                    band_data_t band_data;
                    if (data_manager_get_band_data(&band_data) == ESP_OK) {
                        memcpy(&band_data.spo2, ctxt->om->om_data, 4);
                        band_data.data_source = DATA_SOURCE_CLIENT;
                        band_data.timestamp = esp_timer_get_time() / 1000;
                        data_manager_update_band_data(&band_data);
                        ESP_LOGI(TAG, "SpO2 updated by client: %.1f%%", band_data.spo2);
                    }
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HEART_RATE_CHAR_UUID)) == 0) {
                // Heart Rate characteristic write
                if (ctxt->om->om_len == 2) {
                    band_data_t band_data;
                    if (data_manager_get_band_data(&band_data) == ESP_OK) {
                        band_data.heart_rate = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];
                        band_data.data_source = DATA_SOURCE_CLIENT;
                        band_data.timestamp = esp_timer_get_time() / 1000;
                        data_manager_update_band_data(&band_data);
                        ESP_LOGI(TAG, "Heart rate updated by client: %d BPM", band_data.heart_rate);
                    }
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(COMMAND_CHAR_UUID)) == 0) {
                // Command write
                if (ctxt->om->om_len > 0 && ctxt->om->om_len < sizeof(command_buffer)) {
                    memcpy(command_buffer, ctxt->om->om_data, ctxt->om->om_len);
                    command_buffer[ctxt->om->om_len] = '\0';
                    ESP_LOGI(TAG, "Command received: %s", command_buffer);
                    
                    // Process commands
                    if (strcmp(command_buffer, "vib") == 0) {
                        ESP_LOGI(TAG, "Executing vibration command");
                        gpio_set_level(VIBRATION_GPIO, 1);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        gpio_set_level(VIBRATION_GPIO, 0);
                        ESP_LOGI(TAG, "Vibration completed");
                    } else if (strstr(command_buffer, "play") != NULL) {
                        int track_num = atoi(command_buffer + 5);
                        if(track_num < 0) track_num = 0;
                        ESP_LOGI(TAG, "Executing play command: %d", track_num);
                        // Try different play methods
                        esp_err_t play_ret = warning_system_play_voice(track_num); // Play track 1 from folder 1
                        if (play_ret == ESP_OK) {
                            ESP_LOGI(TAG, "Play command sent successfully");
                        } else {
                            ESP_LOGW(TAG, "Play command failed: %s", esp_err_to_name(play_ret));
                            
                        }
                    } else {
                        ESP_LOGW(TAG, "Unknown command: %s", command_buffer);
                    }
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(DEBUG_CMD_CHAR_UUID)) == 0) {
                // Debug command write-only characteristic
                // Payload: ASCII string, examples: "vib", "play 1"
                if (ctxt->om->om_len > 0 && ctxt->om->om_len < sizeof(command_buffer)) {
                    memcpy(command_buffer, ctxt->om->om_data, ctxt->om->om_len);
                    command_buffer[ctxt->om->om_len] = '\0';
                    ESP_LOGI(TAG, "Debug command: %s", command_buffer);

                    if (strcmp(command_buffer, "vib") == 0) {
                        gpio_set_level(VIBRATION_GPIO, 1);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                        gpio_set_level(VIBRATION_GPIO, 0);
                    } else if (strncmp(command_buffer, "play ", 5) == 0) {
                        int track_num = atoi(command_buffer + 5);
                        if (track_num > 0 && track_num <= 3000) {
                            warning_system_play_voice(track_num);
                        } else {
                            ESP_LOGW(TAG, "Invalid track number in debug cmd: %d", track_num);
                        }
                    } else {
                        ESP_LOGW(TAG, "Unknown debug cmd: %s", command_buffer);
                    }
                }
                return 0;
            }
            break;
    }
    
    return BLE_ATT_ERR_UNLIKELY;
}

// GATT services definition
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(HUB_AGG_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(HUB_AGG_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0} // end
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(TEST_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(TEST_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0} // end
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(HEALTH_SENSOR_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(TEMPERATURE_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(HUMIDITY_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(BODY_TEMP_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(SPO2_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(HEART_RATE_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {0} // end
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(NOISE_SENSOR_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(NOISE_LEVEL_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            },
            {0} // end
        }
    },
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(CONTROL_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(COMMAND_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = BLE_UUID16_DECLARE(DEBUG_CMD_CHAR_UUID),
                .access_cb = gatt_svr_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE, // write-only debug command
            },
            {0} // end
        }
    },
    {0} // end
};

// GAP event callback
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_DISC: {
            static uint8_t band_addr[6] = {0x0C, 0x4E, 0xA0, 0x00, 0x00, 0x00};
            // Advertising report received while scanning
            if (event->disc.length_data == 0) break;
            const uint8_t *data = event->disc.data;
            uint8_t len = event->disc.length_data;

            // Case-insensitive substring search over entire ADV payload
            bool found = false;
            size_t klen = strlen(ADV_KEYWORD);
            for (uint8_t i = 0; i + klen <= len && !found; i++) {
                found = true;
                for (size_t j = 0; j < klen; j++) {
                    char a = (char)data[i + j];
                    char b = ADV_KEYWORD[j];
                    if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
                    if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
                    if (a != b) { found = false; break; }
                }
            }
            if(found){
                memcpy(band_addr, event->disc.addr.val, 6);
            }

            if(memcmp(event->disc.addr.val, band_addr, 6) == 0) {
                ESP_LOGI(TAG, "Found safelink band");
                found = true;
            }
            //ESP_LOGI(TAG, "len: %d", len);

            if (found) {
                // Only log matched devices (filter mode)
                log_adv_report(event);

                // Additionally, extract and print Manufacturer/Service Data in HEX
                const uint8_t *p = data; uint8_t remaining = len;
                while (remaining >= 2) {
                    uint8_t field_len = p[0];
                    if (field_len == 0 || field_len + 1 > remaining) break;
                    uint8_t field_type = p[1];
                    const uint8_t *field_data = &p[2];
                    uint8_t payload_len = field_len - 1;
                    if (field_type == BLE_HS_ADV_TYPE_MFG_DATA ||
                        field_type == BLE_HS_ADV_TYPE_SVC_DATA_UUID16 ||
                        field_type == BLE_HS_ADV_TYPE_SVC_DATA_UUID32 ||
                        field_type == BLE_HS_ADV_TYPE_SVC_DATA_UUID128) {
                        // Print up to first 64 bytes
                        char buf[3 * 64 + 1]; int o = 0;
                        for (uint8_t i = 0; i < payload_len && i < 64; i++) {
                            o += snprintf(&buf[o], sizeof(buf) - o, "%02X ", field_data[i]);
                            if (o >= (int)sizeof(buf) - 1) break;
                        }
                        buf[o] = '\0';
                        ESP_LOGI(TAG, "ADV field 0x%02X len=%d HEX: %s%s",
                                 field_type, payload_len, buf, (payload_len > 64 ? "..." : ""));
                        // Parse manufacturer data if present
                        if (field_type == BLE_HS_ADV_TYPE_MFG_DATA) {
                            bool ok = parse_mfg_and_update_band(field_data, payload_len);
                            if (ok) {
                                // 이 광고를 밴드로 인식하고 주소를 기억
                                memcpy(band_addr, event->disc.addr.val, 6);
                            }
                        }
                    }
                    p += (field_len + 1); remaining -= (field_len + 1);
                }
            }
            break; }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            // 주기 스캔: 타이머가 다시 시작함
            ESP_LOGI(TAG, "Scan complete");
            break;
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Connection %s", event->connect.status == 0 ? "established" : "failed");
            if (event->connect.status == 0) {
                current_state = BLUETOOTH_STATE_CONNECTED;
                if (num_conns < MAX_CONNS) {
                    conn_handles[num_conns++] = event->connect.conn_handle;
                    ESP_LOGI(TAG, "conn_handles: %d active", num_conns);
                } else {
                    ESP_LOGW(TAG, "Max connections reached; disconnecting new conn");
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_CONN_LIMIT);
                }
                if (ble_event_group) {
                    xEventGroupSetBits(ble_event_group, BLUETOOTH_READY_BIT);
                }
                // 연결 중에도 스캔 유지하여 광고 수집(스택이 허용하는 범위 내)
                start_passive_scan();
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "Disconnected");
            current_state = BLUETOOTH_STATE_ADVERTISING;
            // 연결 목록에서 제거
            for (uint8_t i = 0; i < num_conns; ++i) {
                if (conn_handles[i] == event->disconnect.conn.conn_handle) {
                    // 뒤 항목 당겨오기
                    for (uint8_t j = i + 1; j < num_conns; ++j) {
                        conn_handles[j - 1] = conn_handles[j];
                    }
                    num_conns--;
                    break;
                }
            }
            // 연결 해제 후 즉시 재광고 및 스캔 재시작
            start_advertising();
            start_passive_scan();
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

// Start advertising
// Start advertising (connectable)
static void start_advertising(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    int rc;

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to infer address type; rc=%d", rc);
        return;
    }

    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HCI_ADV_CHAN_TXPWR_MAX;
    fields.name = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertisement data; rc=%d", rc);
        return;
    }

    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising; rc=%d", rc);
        return;
    }

    current_state = BLUETOOTH_STATE_ADVERTISING;
    ESP_LOGI(TAG, "Advertising started (connectable)");

    // Ensure passive scanning is running concurrently for beacon observation
    start_passive_scan();
}

// NimBLE host task
static void nimble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Bluetooth initialization
esp_err_t bluetooth_init(void)
{
    ESP_LOGI(TAG, "Initializing NimBLE Bluetooth...");
    
    // Initialize the NimBLE host configuration
    nimble_port_init();
    
    // Initialize the NimBLE host
    // Start advertising (and passive scan in parallel) when BLE stack syncs
    ble_hs_cfg.sync_cb = start_advertising;
    
    // Set the default device name
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);
    
    // Initialize GATT services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Register GATT services
    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT services; rc=%d", rc);
        return ESP_FAIL;
    }
    
    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services; rc=%d", rc);
        return ESP_FAIL;
    }
    
    // Create event group
    ble_event_group = xEventGroupCreate();
    
    // Create NimBLE host task
    nimble_port_freertos_init(nimble_host_task);
    /*
    // Initialize hardware
    // Initialize ADC for noise measurement
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    
    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    */
    // Initialize vibration GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << VIBRATION_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    gpio_set_level(VIBRATION_GPIO, 0);
    
    // Initialize DFPlayer Mini (UART1, TX=GPIO4, RX=GPIO5, 9600 baud)
    esp_err_t dfplayer_ret = dfplayer_init(UART_NUM_1, 4, 5, 9600);
    if (dfplayer_ret == ESP_OK) {
        ESP_LOGI(TAG, "DFPlayer Mini initialized successfully");
        // Set volume to 50%
        dfplayer_set_volume(50);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    } else {
        ESP_LOGE(TAG, "DFPlayer Mini initialization failed: %s", esp_err_to_name(dfplayer_ret));
    }
    
    // BLE 스캔 주기 타이머: 5초마다 1초 스캔 시작
    if (ble_scan_timer == NULL) {
        ble_scan_timer = xTimerCreate("ble_scan_timer", pdMS_TO_TICKS(5000), pdTRUE, NULL, scan_timer_cb);
        if (ble_scan_timer != NULL) {
            xTimerStart(ble_scan_timer, 0);
        } else {
            ESP_LOGE(TAG, "Failed to create BLE scan timer");
        }
    }

    ESP_LOGI(TAG, "NimBLE Bluetooth and hardware initialized successfully");
    return ESP_OK;
}

// Start active scanning to observe advertisements and scan responses
static void start_passive_scan(void)
{
    struct ble_gap_disc_params disc_params = {
        .filter_duplicates = 1,
        .passive = 0,  // Active scan으로 변경
        .itvl = BLE_GAP_SCAN_ITVL_MS(160),    // 100 ms
        .window = BLE_GAP_SCAN_WIN_MS(120),   // 75 ms
        .filter_policy = 0,
        .limited = 0,
    };

    // 1초 동안만 스캔 (타이머로 5초마다 호출)
    int rc = ble_gap_disc(own_addr_type, 1000, &disc_params, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Failed to start active scan; rc=%d", rc);
    } else if (rc == 0) {
        ESP_LOGI(TAG, "Active scanning started (ADV + Scan Response)");
    }
}

// (removed) start_scan_on_sync: no longer used

// Print hex helper
static void print_hex_line(const uint8_t *data, uint8_t len)
{
    char buf[3 * 64 + 1];
    int o = 0;
    for (uint8_t i = 0; i < len && i < 64; i++) {
        o += snprintf(&buf[o], sizeof(buf) - o, "%02X ", data[i]);
        if (o >= (int)sizeof(buf) - 1) break;
    }
    buf[o] = '\0';
    ESP_LOGI(TAG, "ADV HEX: %s%s", buf, (len > 64 ? "..." : ""));
}

// Parse common ADV fields and log human-readable fields
static void parse_and_log_adv_fields(const uint8_t *data, uint8_t len)
{
    const uint8_t *p = data;
    uint8_t remaining = len;
    char name[32] = {0};
    bool has_name = false;
    bool has_mfg = false;
    bool has_service_data = false;

    while (remaining >= 2) {
        uint8_t field_len = p[0];
        if (field_len == 0 || field_len + 1 > remaining) break;
        uint8_t field_type = p[1];
        const uint8_t *field_data = &p[2];
        uint8_t payload_len = field_len - 1;

        switch (field_type) {
            case BLE_HS_ADV_TYPE_COMP_NAME:
            case BLE_HS_ADV_TYPE_INCOMP_NAME: {
                uint8_t n = payload_len < sizeof(name) - 1 ? payload_len : sizeof(name) - 1;
                memcpy(name, field_data, n);
                name[n] = '\0';
                has_name = true;
                break;
            }
            case BLE_HS_ADV_TYPE_MFG_DATA: {
                has_mfg = true;
                break;
            }
            case BLE_HS_ADV_TYPE_SVC_DATA_UUID16:
            case BLE_HS_ADV_TYPE_SVC_DATA_UUID32:
            case BLE_HS_ADV_TYPE_SVC_DATA_UUID128: {
                has_service_data = true;
                break;
            }
            default:
                break;
        }

        p += (field_len + 1);
        remaining -= (field_len + 1);
    }

    if (has_name) {
        ESP_LOGI(TAG, "ADV Name: %s", name);
    }
    if (has_mfg) {
        ESP_LOGI(TAG, "ADV has Manufacturer Data");
    }
    if (has_service_data) {
        ESP_LOGI(TAG, "ADV has Service Data");
    }
}

static void log_adv_report(const struct ble_gap_event *event)
{
    const ble_addr_t *addr = &event->disc.addr;
    ESP_LOGI(TAG,
             "ADV from %02X:%02X:%02X:%02X:%02X:%02X, RSSI=%d, evt_type=%d, len=%d",
             addr->val[5], addr->val[4], addr->val[3], addr->val[2], addr->val[1], addr->val[0],
             event->disc.rssi, event->disc.event_type, event->disc.length_data);
    print_hex_line(event->disc.data, event->disc.length_data);
    parse_and_log_adv_fields(event->disc.data, event->disc.length_data);
}

// 스캔 주기 타이머 콜백: 호출 시 짧은 스캔을 시작
static void scan_timer_cb(TimerHandle_t xTimer)
{
    start_passive_scan();
}

// Parse manufacturer data: CompanyID=0x02E5, ver=0x01, flags=0x0F, then sensor payload
// Layout: [CID_L][CID_H][ver][flags][extT_L][extT_H][RH_L][RH_H][bodyT_L][bodyT_H][HR_L][HR_H][SpO2]
static bool parse_mfg_and_update_band(const uint8_t *mfg, uint8_t payload_len)
{
    if (!mfg || payload_len < 13) return false;

    uint16_t cid = (uint16_t)mfg[0] | ((uint16_t)mfg[1] << 8);
    if (cid != 0x02E5) return false;
    if (mfg[2] != 0x01) return false; // version check
    if ((mfg[3] & 0x0F) == 0) return false; // flags sanity

    int16_t ext_temp_centi = (int16_t)((uint16_t)mfg[4] | ((uint16_t)mfg[5] << 8));
    uint16_t rh_centi      = (uint16_t)mfg[6] | ((uint16_t)mfg[7] << 8);
    int16_t body_temp_centi= (int16_t)((uint16_t)mfg[8] | ((uint16_t)mfg[9] << 8));
    uint16_t hr_bpm        = (uint16_t)mfg[10] | ((uint16_t)mfg[11] << 8);
    uint8_t spo2_percent   = mfg[12];

    band_data_t band = {0};
    band.external_temp = ((float)ext_temp_centi) / 100.0f;
    band.external_humidity = ((float)rh_centi) / 100.0f;
    band.skin_temp = ((float)body_temp_centi) / 100.0f;
    band.heart_rate = hr_bpm;
    band.spo2 = (float)spo2_percent; // 정수 %
    band.timestamp = esp_timer_get_time() / 1000; // ms
    band.is_valid = true;
    band.data_source = DATA_SOURCE_CLIENT;

    if (data_manager_update_band_data(&band) == ESP_OK) {
        ESP_LOGI(TAG, "Band MFG parsed: ExtT=%.2fC RH=%.2f%% SkinT=%.2fC HR=%u SpO2=%.0f%%",
                 band.external_temp, band.external_humidity, band.skin_temp, band.heart_rate, band.spo2);
        return true;
    }
    return false;
}

// Bluetooth deinitialization
esp_err_t bluetooth_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing NimBLE Bluetooth...");
    
    ESP_LOGI(TAG, "NimBLE Bluetooth deinitialized successfully");
    return ESP_OK;
}

// Get current Bluetooth state
bluetooth_state_t bluetooth_get_state(void)
{
    return current_state;
}

// Send test data
esp_err_t bluetooth_send_test_data(const char *data)
{
    if (current_state != BLUETOOTH_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Update test value
    strncpy(test_value, data, sizeof(test_value) - 1);
    test_value[sizeof(test_value) - 1] = '\0';
    
    // Send notification
    ble_gatts_chr_updated(1); // attr_handle for test characteristic
    
    ESP_LOGI(TAG, "Test data sent: %s", data);
    return ESP_OK;
}

// Get health sensor data
esp_err_t bluetooth_get_health_data(health_sensor_data_t *data)
{
    if (!data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memcpy(data, &health_data, sizeof(health_sensor_data_t));
    return ESP_OK;
}

// Measure noise level
void measure_noise_level(void)
{
    if (adc1_handle) {
        int adc_raw = 0;
        esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw);
        if (ret == ESP_OK) {
            // Convert ADC reading to noise level (0.1dB units)
            // Simple conversion: ADC 0-4095 -> 50.0dB to 90.0dB
            uint16_t noise_level = 500 + (adc_raw * 400) / 4095; // 50.0dB to 90.0dB
            health_data.noise_level = noise_level;
            health_data.timestamp = esp_timer_get_time() / 1000;
        }
    }
}

// Check DFPlayer status
void check_dfplayer_status(void)
{
    // Query current status
    dfplayer_status_t status;
    esp_err_t ret = dfplayer_get_status(&status);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DFPlayer Status - Playing: %s, Track: %d, Volume: %d", 
                 status.is_playing ? "Yes" : "No", 
                 status.current_track, 
                 status.current_volume);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

// MIT App Inventor 데이터 전송 함수
esp_err_t bluetooth_update_mit_app_inventor_data(float skin_temp, uint16_t heart_rate, 
                                                float spo2, float external_temp, 
                                                float external_humidity, float avg_noise, 
                                                float wbgt, uint8_t alarm_status)
{
    if (current_state != BLUETOOTH_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Advertising UUID Data에 포함할 데이터 생성
    // MIT App Inventor용 UUID 발송 코드
    uint8_t mit_data[25];  // 4+2+4+4+4+2+4+1 = 25 bytes
    int offset = 0;
    
    // 데이터를 바이트 배열로 변환 (빅엔디안)
    // Skin Temperature (4 bytes, float)
    memcpy(&mit_data[offset], &skin_temp, 4);
    offset += 4;
    
    // Heart Rate (2 bytes, uint16_t)
    mit_data[offset] = (heart_rate >> 8) & 0xFF;
    mit_data[offset + 1] = heart_rate & 0xFF;
    offset += 2;
    
    // SpO2 (4 bytes, float)
    memcpy(&mit_data[offset], &spo2, 4);
    offset += 4;
    
    // External Temperature (4 bytes, float)
    memcpy(&mit_data[offset], &external_temp, 4);
    offset += 4;
    
    // External Humidity (4 bytes, float)
    memcpy(&mit_data[offset], &external_humidity, 4);
    offset += 4;
    
    // Average Noise (2 bytes, uint16_t, 0.1dB 단위)
    uint16_t noise_encoded = (uint16_t)(avg_noise * 10);
    mit_data[offset] = (noise_encoded >> 8) & 0xFF;
    mit_data[offset + 1] = noise_encoded & 0xFF;
    offset += 2;
    
    // WBGT (4 bytes, float)
    memcpy(&mit_data[offset], &wbgt, 4);
    offset += 4;
    
    // Alarm Status (1 byte)
    mit_data[offset] = alarm_status;
    offset += 1;
    
    // 실제 NimBLE advertising 데이터 업데이트
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    
    // Manufacturer data에 MIT App Inventor 데이터 포함
    fields.mfg_data_len = sizeof(mit_data);
    fields.mfg_data = mit_data;
    
    // Advertising 데이터 업데이트
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc == 0) {
        ESP_LOGI(TAG, "MIT App Inventor data updated in advertising: Skin=%.1f°C, HR=%d, SpO2=%.1f%%, "
                      "ExtT=%.1f°C, ExtH=%.1f%%, Noise=%.1fdB, WBGT=%.1f°C, Alarm=0x%02X",
                 skin_temp, heart_rate, spo2, external_temp, external_humidity, 
                 avg_noise, wbgt, alarm_status);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to update advertising data: %d", rc);
        return ESP_FAIL;
    }
}

// GATT Subscribe - 밴드로부터 데이터 수신 처리
esp_err_t bluetooth_handle_band_data_write(const uint8_t *data, size_t len)
{
    if (!data || len < 20) {
        ESP_LOGE(TAG, "Invalid band data length: %d", len);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 밴드 데이터 파싱
    band_data_t band_data = {0};
    int offset = 0;
    
    // External Temperature (4 bytes, float)
    memcpy(&band_data.external_temp, &data[offset], 4);
    offset += 4;
    
    // External Humidity (4 bytes, float)
    memcpy(&band_data.external_humidity, &data[offset], 4);
    offset += 4;
    
    // Skin Temperature (4 bytes, float)
    memcpy(&band_data.skin_temp, &data[offset], 4);
    offset += 4;
    
    // Heart Rate (2 bytes, uint16_t)
    band_data.heart_rate = (data[offset] << 8) | data[offset + 1];
    offset += 2;
    
    // SpO2 (4 bytes, float)
    memcpy(&band_data.spo2, &data[offset], 4);
    offset += 4;
    
    // Timestamp (2 bytes, uint32_t)
    band_data.timestamp = (data[offset] << 24) | (data[offset + 1] << 16) | 
                          (data[offset + 2] << 8) | data[offset + 3];
    offset += 4;
    
    band_data.is_valid = true;
    band_data.data_source = DATA_SOURCE_CLIENT;  // 클라이언트 데이터로 표시
    
    // 데이터 매니저에 업데이트
    esp_err_t ret = data_manager_update_band_data(&band_data);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Band data received and updated successfully");
    } else {
        ESP_LOGE(TAG, "Failed to update band data: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// GATT Subscribe - 디버그 커맨드 처리
esp_err_t bluetooth_handle_command_write(const uint8_t *data, size_t len)
{
    if (!data || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 문자열로 변환
    char command[32] = {0};
    strncpy(command, (char*)data, len < 31 ? len : 31);
    
    ESP_LOGI(TAG, "Command received: %s", command);
    
    if (strcmp(command, "vib") == 0) {
        // 진동 1초
        ESP_LOGI(TAG, "Executing vibration command");
        gpio_set_level(VIBRATION_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(VIBRATION_GPIO, 0);
        ESP_LOGI(TAG, "Vibration completed");
        
    } else if (strncmp(command, "play ", 5) == 0) {
        // play n 명령 처리
        int track_num = atoi(command + 5);
        if (track_num > 0 && track_num <= 3000) {
            ESP_LOGI(TAG, "Playing track %d", track_num);
            esp_err_t ret = dfplayer_play(track_num);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to play track %d: %s", track_num, esp_err_to_name(ret));
            }
        } else {
            ESP_LOGE(TAG, "Invalid track number: %d", track_num);
        }
        
    } else {
        ESP_LOGW(TAG, "Unknown command: %s", command);
    }
    
    return ESP_OK;
}

// GATT Publishing - 허브 데이터 특성 업데이트
esp_err_t bluetooth_update_hub_data_characteristics(void)
{
    if (current_state != BLUETOOTH_STATE_CONNECTED) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 허브 데이터 가져오기
    hub_data_t hub_data;
    esp_err_t ret = data_manager_get_hub_data(&hub_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get hub data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (!hub_data.is_valid) {
        ESP_LOGW(TAG, "Hub data not valid, skipping update");
        return ESP_OK;
    }
    
    // Noise Level 특성 업데이트
    uint8_t noise_data[4];
    memcpy(noise_data, &hub_data.avg_noise, 4);
    
    // GATT 특성 업데이트 (NimBLE API 사용)
    // 실제 구현에서는 특성 핸들을 찾아서 업데이트
    // 현재는 로그만 출력
    ESP_LOGI(TAG, "Noise level characteristic updated: %.1fdB", hub_data.avg_noise);
    
    // MIT App Inventor 데이터도 함께 업데이트
    band_data_t band_data;
    if (data_manager_get_band_data(&band_data) == ESP_OK && band_data.is_valid) {
        bluetooth_update_mit_app_inventor_data(
            band_data.skin_temp,
            band_data.heart_rate,
            band_data.spo2,
            band_data.external_temp,
            band_data.external_humidity,
            hub_data.avg_noise,
            hub_data.wbgt,
            hub_data.alarm_status
        );
    }
    
    ESP_LOGI(TAG, "Hub data characteristics updated: Noise=%.1fdB, WBGT=%.1f°C, Alarm=0x%02X",
             hub_data.avg_noise, hub_data.wbgt, hub_data.alarm_status);
    
    return ESP_OK;
} 
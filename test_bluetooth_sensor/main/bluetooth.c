#include "bluetooth.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
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
            uuid = chr->uuid;
            if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HEART_RATE_CHAR_UUID)) == 0) {
                // 심박수 데이터 쓰기 (테스트용)
                if (ctxt->om->om_len >= 1) {
                    heart_rate_value = ctxt->om->om_data[0];
                    ESP_LOGI(TAG, "Heart rate written: %d BPM", heart_rate_value);
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(TEMPERATURE_CHAR_UUID)) == 0) {
                // 온도 데이터 쓰기 (테스트용)
                if (ctxt->om->om_len >= 2) {
                    uint16_t temp_value = ctxt->om->om_data[0] | (ctxt->om->om_data[1] << 8);
                    temp_humidity_value = (temp_humidity_value & 0xFFFF) | (temp_value << 16);
                    ESP_LOGI(TAG, "Temperature written: %.1f°C", temp_value / 10.0f);
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HUMIDITY_CHAR_UUID)) == 0) {
                // 습도 데이터 쓰기 (테스트용)
                if (ctxt->om->om_len >= 2) {
                    uint16_t hum_value = ctxt->om->om_data[0] | (ctxt->om->om_data[1] << 8);
                    temp_humidity_value = (temp_humidity_value & 0xFFFF0000) | hum_value;
                    ESP_LOGI(TAG, "Humidity written: %.1f%%", hum_value / 10.0f);
                }
                return 0;
            } else if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(SENSOR_DATA_CHAR_UUID)) == 0) {
                // 통합 센서 데이터 쓰기 (테스트용)
                if (ctxt->om->om_len >= 6) {
                    heart_rate_value = ctxt->om->om_data[0] | (ctxt->om->om_data[1] << 8);
                    uint16_t temp_value = ctxt->om->om_data[2] | (ctxt->om->om_data[3] << 8);
                    uint16_t hum_value = ctxt->om->om_data[4] | (ctxt->om->om_data[5] << 8);
                    temp_humidity_value = (temp_value << 16) | hum_value;
                    ESP_LOGI(TAG, "Sensor data written - HR: %d, Temp: %.1f°C, Hum: %.1f%%", 
                            heart_rate_value, temp_value / 10.0f, hum_value / 10.0f);
                }
                return 0;
            }
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
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
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
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
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
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
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
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY,
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

// MIT App Inventor 전용 11바이트 Advertising 데이터
static uint8_t mit_app_inventor_adv_data[] = {
    // 피부온도 (2바이트, 0.1°C 단위, 15.0~40.0°C = 150~400)
    0x96, 0x00,  // 150 = 36.0°C (기본값)
    
    // 심박수 (2바이트, 20~160 BPM)
    0x4B, 0x00,  // 75 BPM (기본값)
    
    // 혈중포화농도 (2바이트, 0.1% 단위, 80.0~99.9% = 800~999)
    0xE8, 0x03,  // 1000 = 95.0% (기본값)
    
    // 평균소음 (2바이트, 0.1dB 단위, 40~150dB = 400~1500)
    0x90, 0x01,  // 400 = 40.0dB (기본값)
    
    // 경보상태 플래그 (1바이트)
    // 비트 7-5: WBGT 경고 (0=정상, 1=주의, 2=경고, 3=위험, 4=매우위험)
    // 비트 4-2: 체온경고 (0=정상, 1=저체온, 2=고체온, 3=위험, 4=매우위험)
    // 비트 1-0: 심박수경고 (0=정상, 1=서맥, 2=빈맥, 3=위험)
    0x00,        // 모든 경고 정상 (기본값)
    
    // 예약 (2바이트)
    0x00, 0x00   // 향후 확장용
};

// MIT App Inventor 전용 Advertising 데이터 업데이트 함수
esp_err_t bluetooth_update_mit_app_inventor_data(float skin_temp, uint16_t heart_rate, 
                                                float spo2, float noise_level,
                                                wbgt_warning_t wbgt_warning,
                                                temp_warning_t temp_warning,
                                                hr_warning_t hr_warning)
{
    // 입력값 검증
    if (skin_temp < 15.0f || skin_temp > 40.0f) {
        ESP_LOGE(TAG, "Invalid skin temperature: %.1f°C (15.0~40.0°C)", skin_temp);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (heart_rate < 20 || heart_rate > 160) {
        ESP_LOGE(TAG, "Invalid heart rate: %d BPM (20~160)", heart_rate);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (spo2 < 80.0f || spo2 > 99.9f) {
        ESP_LOGE(TAG, "Invalid SpO2: %.1f%% (80.0~99.9%%)", spo2);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (noise_level < 40.0f || noise_level > 150.0f) {
        ESP_LOGE(TAG, "Invalid noise level: %.1fdB (40~150dB)", noise_level);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 피부온도 (0.1°C 단위로 변환)
    uint16_t temp_raw = (uint16_t)(skin_temp * 10.0f);
    mit_app_inventor_adv_data[0] = temp_raw & 0xFF;
    mit_app_inventor_adv_data[1] = (temp_raw >> 8) & 0xFF;
    
    // 심박수
    mit_app_inventor_adv_data[2] = heart_rate & 0xFF;
    mit_app_inventor_adv_data[3] = (heart_rate >> 8) & 0xFF;
    
    // 혈중포화농도 (0.1% 단위로 변환)
    uint16_t spo2_raw = (uint16_t)(spo2 * 10.0f);
    mit_app_inventor_adv_data[4] = spo2_raw & 0xFF;
    mit_app_inventor_adv_data[5] = (spo2_raw >> 8) & 0xFF;
    
    // 평균소음 (0.1dB 단위로 변환)
    uint16_t noise_raw = (uint16_t)(noise_level * 10.0f);
    mit_app_inventor_adv_data[6] = noise_raw & 0xFF;
    mit_app_inventor_adv_data[7] = (noise_raw >> 8) & 0xFF;
    
    // 경보상태 플래그 조합
    uint8_t warning_flags = 0;
    warning_flags |= (wbgt_warning & 0x07) << 5;  // 비트 7-5: WBGT 경고
    warning_flags |= (temp_warning & 0x07) << 2;  // 비트 4-2: 체온경고
    warning_flags |= (hr_warning & 0x03);         // 비트 1-0: 심박수경고
    mit_app_inventor_adv_data[8] = warning_flags;
    
    // 예약 바이트는 0으로 유지
    mit_app_inventor_adv_data[9] = 0x00;
    mit_app_inventor_adv_data[10] = 0x00;
    
    ESP_LOGI(TAG, "MIT App Inventor data updated:");
    ESP_LOGI(TAG, "  Skin Temp: %.1f°C (%d)", skin_temp, temp_raw);
    ESP_LOGI(TAG, "  Heart Rate: %d BPM", heart_rate);
    ESP_LOGI(TAG, "  SpO2: %.1f%% (%d)", spo2, spo2_raw);
    ESP_LOGI(TAG, "  Noise: %.1fdB (%d)", noise_level, noise_raw);
    ESP_LOGI(TAG, "  Warnings: WBGT=%d, Temp=%d, HR=%d", wbgt_warning, temp_warning, hr_warning);
    
    return ESP_OK;
}

// 정상적인 GATT Server + 커스텀 Beacon Advertising 설정
esp_err_t bluetooth_set_mit_app_inventor_advertising(void)
{
    // Advertising을 중지하고 안전하게 필드를 갱신
    ble_gap_adv_stop();

    // 29바이트 Advertising Data (Device Name 포함)
    uint8_t adv_data[29] = {
        // 1. Flags (3 bytes)
        0x02, 0x01, 0x06,  // Length=2, Type=Flags, Value=LE General Discoverable + BR/EDR Not Supported
        
        // 2. Device Name (13 bytes) - "Safelink_HR"
        0x0C, 0x09, 'S', 'a', 'f', 'e', 'l', 'i', 'n', 'k', '_', 'H', 'R',
        
        // 3. Service UUID (Heart Rate Service) (5 bytes)
        0x05, 0x16, 0x0D, 0x18, 0x00, 0x00,  // Length=5, Type=Service UUID, UUID=0x180D
        
        // 4. Manufacturer Data (8 bytes) - Company ID + 6바이트 센서 데이터
        0x09, 0xFF,  // Length=9, Type=Manufacturer Specific Data
        0x4E, 0x00,  // Company ID (0x004E = Nordic Semiconductor)
        
        // 6바이트 센서 데이터 (압축된 버전)
        0x96, 0x00, 0x4B, 0x00, 0xE8, 0x03
    };
    
    // 최신 센서 데이터로 업데이트 (6바이트만 사용)
    memcpy(&adv_data[21], mit_app_inventor_adv_data, 6);
    
    // Advertising data 설정
    int rc = ble_gap_adv_set_data(adv_data, sizeof(adv_data));
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set advertising data; rc=%d", rc);
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "=== 29-byte Advertising Data Set ===");
    ESP_LOGI(TAG, "Device Name: Safelink_HR (included in ADV)");
    ESP_LOGI(TAG, "Service UUID: 0x180D (Heart Rate Service)");
    ESP_LOGI(TAG, "Manufacturer Data: 6-byte sensor data");
    ESP_LOGI(TAG, "Total Advertising Data: %d bytes", sizeof(adv_data));
    
    // Advertising 데이터 내용 출력 (디버깅용)
    ESP_LOGI(TAG, "Advertising data dump:");
    for (int i = 0; i < sizeof(adv_data); i++) {
        ESP_LOGI(TAG, "  [%02d]: 0x%02X", i, adv_data[i]);
    }
    ESP_LOGI(TAG, "=== END ADVERTISING DATA ===");
    
    // 재시작
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = BLE_GAP_ADV_ITVL_MS(500),
        .itvl_max = BLE_GAP_ADV_ITVL_MS(2000),
    };
    
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to restart advertising; rc=%d", rc);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// 5초마다 Advertising 데이터 업데이트 태스크
static void advertising_update_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Advertising update task started");
    
    while (1) {
        // 현재 센서 데이터로 MIT App Inventor 데이터 업데이트
        float skin_temp = 25.0f + (esp_random() % 100) / 10.0f;  // 25.0~35.0°C
        uint16_t heart_rate = 60 + (esp_random() % 100);         // 60~160 BPM
        float spo2 = 90.0f + (esp_random() % 100) / 10.0f;      // 90.0~100.0%
        float noise_level = 40.0f + (esp_random() % 1100) / 10.0f; // 40.0~150.0dB
        
        // 경보 상태 결정
        wbgt_warning_t wbgt_warning = WBGT_NORMAL;
        temp_warning_t temp_warning = TEMP_NORMAL;
        hr_warning_t hr_warning = HR_NORMAL;
        
        // 온도 경보
        if (skin_temp > 38.0f) {
            temp_warning = TEMP_HIGH;
        } else if (skin_temp < 35.0f) {
            temp_warning = TEMP_LOW;
        }
        
        // 심박수 경보
        if (heart_rate > 120) {
            hr_warning = HR_TACHYCARDIA;
        } else if (heart_rate < 60) {
            hr_warning = HR_BRADYCARDIA;
        }
        
        // MIT App Inventor 데이터 업데이트
        esp_err_t ret = bluetooth_update_mit_app_inventor_data(
            skin_temp, heart_rate, spo2, noise_level,
            wbgt_warning, temp_warning, hr_warning
        );
        
        if (ret == ESP_OK) {
            // GATT Server 데이터도 동기화
            temp_humidity_value = ((uint16_t)(skin_temp * 10.0f) << 16) | (uint16_t)(50.0f * 10.0f); // 온도 + 습도
            heart_rate_value = heart_rate;
            
            // Advertising 데이터 다시 설정 (실시간 업데이트)
            bluetooth_set_mit_app_inventor_advertising();
            
            ESP_LOGI(TAG, "Advertising updated - Temp: %.1f°C, HR: %d, SpO2: %.1f%%, Noise: %.1fdB", 
                    skin_temp, heart_rate, spo2, noise_level);
        }
        
        // 5초 대기
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Advertising 업데이트 태스크 시작
esp_err_t bluetooth_start_advertising_update(void)
{
    xTaskCreate(advertising_update_task, "adv_update", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Advertising update task created");
    return ESP_OK;
}

// BLE sync callback
static void ble_on_sync(void)
{
    int rc;
    
    // Set device name to "Safelink_HR" (nRF Connect에서 표시될 이름)
    rc = ble_svc_gap_device_name_set("Safelink_HR");
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to set device name; rc=%d", rc);
        return;
    }
    
    // Configure the address
    ble_hs_id_infer_auto(0, &own_addr_type);
    
    // 정상적인 GATT Server + 커스텀 Beacon Advertising 설정
    rc = bluetooth_set_mit_app_inventor_advertising();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set advertising data");
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
    
    ESP_LOGI(TAG, "=== BLE GATT SERVER + BEACON ADVERTISING SUCCESS ===");
    ESP_LOGI(TAG, "Device Name: Safelink_HR");
    ESP_LOGI(TAG, "Service: Heart Rate (0x180D)");
    ESP_LOGI(TAG, "Custom Beacon: 8-byte sensor data");
    ESP_LOGI(TAG, "nRF Connect에서 'Safelink_HR'로 표시됨");
    ESP_LOGI(TAG, "=== END SUCCESS INFO ===");
    current_state = BLUETOOTH_STATE_ADVERTISING;
    
    // 초기 센서 데이터 설정
    temp_humidity_value = (250 << 16) | 500;  // 25.0°C, 50.0%
    heart_rate_value = 75;
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
    
    // Advertising 업데이트 태스크 시작
    bluetooth_start_advertising_update();
    
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
    
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
        .itvl_min = BLE_GAP_ADV_ITVL_MS(100),
        .itvl_max = BLE_GAP_ADV_ITVL_MS(200),
    };
    
    int rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event_cb, NULL);
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
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

// 연결 체크 시스템 변수들
static bool check_system_active = false;
static TimerHandle_t check_timeout_timer = NULL;
static uint16_t check_conn_handle = 0;

// Test characteristic value
static char test_value[32] = "Hello NimBLE!";

// Command buffer
static char command_buffer[32] = {0};

// Keyword to detect in advertising packets from wearable band
static const char *ADV_KEYWORD = "band";

// GATT 응답 캐시 시스템
static struct {
    uint8_t cached_data[32];
    uint32_t cache_timestamp;
    bool is_valid;
} gatt_cache = {0};

// Forward declarations
static void start_passive_scan(void);
static void start_advertising(void);
static void scan_timer_cb(TimerHandle_t xTimer);
static void check_timeout_cb(TimerHandle_t xTimer);
static void log_adv_report(const struct ble_gap_event *event);
static void parse_and_log_adv_fields(const uint8_t *data, uint8_t len);
static bool parse_mfg_and_update_band(const uint8_t *mfg, uint8_t payload_len);

// GATT access callback with optimized response time
static int gatt_svr_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid = ctxt->chr->uuid;
    
    // 로그 레벨을 줄여서 응답 시간 단축
    ESP_LOGD(TAG, "GATT access: conn_handle=%d, attr_handle=%d, op=%d", 
             conn_handle, attr_handle, ctxt->op);
    
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            if (ble_uuid_cmp(uuid, BLE_UUID16_DECLARE(HUB_AGG_CHAR_UUID)) == 0) {
                // 캐시된 데이터가 500ms 이내라면 재사용
                uint32_t current_time = esp_timer_get_time() / 1000;
                if (gatt_cache.is_valid && (current_time - gatt_cache.cache_timestamp) < 500) {
                    os_mbuf_append(ctxt->om, gatt_cache.cached_data, 29);
                    return 0;
                }
                
                // 캐시가 없거나 오래된 경우 새로 생성
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
                
                // 캐시 업데이트
                memcpy(gatt_cache.cached_data, buf, o);
                gatt_cache.cache_timestamp = current_time;
                gatt_cache.is_valid = true;
                
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
                    } else if (strncmp(command_buffer, "check start", 11) == 0) {
                        
                    } else if (strncmp(command_buffer, "check", 6) == 0) {
                        // 체크 수신. 기기는 check start를 했다면 이후 최소 5초 내로 check\0 명령어를 보내야함.
                        if (check_system_active && conn_handle == check_conn_handle) {
                            // 타이머 리셋
                            if (check_timeout_timer != NULL) {
                                xTimerReset(check_timeout_timer, 0);
                            }
                            ESP_LOGI(TAG, "Check received, timer reset for conn_handle: %d", conn_handle);
                        }
                        
                    } else if (strncmp(command_buffer, "check disconnect", 16) == 0) {
                        // 연결을 자동 해제하는 명령어. (응답 없을 때의 트리거)
                        if (check_system_active && conn_handle == check_conn_handle) {
                            ESP_LOGI(TAG, "Check disconnect command received for conn_handle: %d", conn_handle);
                            
                            check_timeout_cb(check_timeout_timer);
                        }
                    } else if (strcmp(command_buffer, "scan status") == 0) {
                        // 스캔 상태 확인
                        bluetooth_get_scan_status();
                    } else if (strcmp(command_buffer, "force scan") == 0) {
                        // 강제 스캔 시작
                        bluetooth_force_scan_start();
                    } else if (strcmp(command_buffer, "scan stats") == 0) {
                        // 스캔 통계 출력
                        bluetooth_print_scan_statistics();
                    } else {
                        ESP_LOGW(TAG, "Unknown command: %s", command_buffer);
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
            static uint32_t last_band_update = 0;
            static uint32_t last_keyword_band_update = 0;
            static uint8_t duplicate_count = 0;
            static uint32_t total_adv_count = 0;
            
            // 스캔 이벤트 타입 로그 (모든 이벤트 타입 로깅)
            const char* event_type_str;
            switch (event->disc.event_type) {
                case 0: event_type_str = "Connectable Undirected"; break;
                case 1: event_type_str = "Connectable Directed"; break;
                case 2: event_type_str = "Non-connectable Undirected"; break;
                case 3: event_type_str = "Scan Response"; break;
                case 4: event_type_str = "Scan Response (Alt)"; break;
                case 5: event_type_str = "Non-connectable Directed"; break;
                default: event_type_str = "Unknown"; break;
            }
            total_adv_count++;
            
            // 주기적으로 스캔 상태 로그
            if (total_adv_count % 1000 == 0) {
                ESP_LOGI(TAG, "Scan active: %u advertisements received, RSSI=%d", 
                         (unsigned int)total_adv_count, event->disc.rssi);
            }
            bool found = false;

            // 알려진 밴드 주소와 비교
            if(memcmp(event->disc.addr.val, band_addr, 6) == 0) {
                found = true;
                duplicate_count++;
                // 중복 방지: 1초 내 동일한 밴드 데이터는 무시
                uint32_t current_time = esp_timer_get_time() / 1000;
                if ((current_time - last_band_update) < 1000) {
                    //ESP_LOGI(TAG, "Band device ignored - within 1 second cooldown period, now = %lu", current_time);
                    found = false;
                    return 0;
                }
                ESP_LOGD(TAG, "Band device processing allowed - cooldown period expired");
            }
            if(found)
                ESP_LOGD(TAG, "Scan event: %s (type=%d), found = %d,RSSI=%d, len=%d, MAC : %02X:%02X:%02X:%02X:%02X:%02X", 
                    event_type_str, event->disc.event_type, found, event->disc.rssi, event->disc.length_data,
                    event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
                    event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);
                
            
            // Advertising report received while scanning
            if (event->disc.length_data == 0) break;
            const uint8_t *data = event->disc.data;
            uint8_t len = event->disc.length_data;

            // 빠른 키워드 검색 (첫 번째 바이트부터 시작)
            size_t klen = strlen(ADV_KEYWORD);
            
            // 최적화된 문자열 검색
            
            bool found_keyword = false;
            for (uint8_t i = 0; i + klen <= len && !found; i++) {
                if (data[i] == ADV_KEYWORD[0] || 
                    (data[i] >= 'A' && data[i] <= 'Z' && data[i] == ADV_KEYWORD[0] - 'A' + 'a') ||
                    (data[i] >= 'a' && data[i] <= 'z' && data[i] == ADV_KEYWORD[0] - 'a' + 'A')) {
                    found_keyword = true;
                    for (size_t j = 1; j < klen; j++) {
                        char a = (char)data[i + j];
                        char b = ADV_KEYWORD[j];
                        if (a >= 'A' && a <= 'Z') a = (char)(a - 'A' + 'a');
                        if (b >= 'A' && b <= 'Z') b = (char)(b - 'A' + 'a');
                        if (a != b) { found_keyword = false; break; }
                    }
                    
                }
            }
            found |= found_keyword;
            if(found_keyword){
                // 이 광고를 밴드로 인식하고 주소를 기억
                memcpy(band_addr, event->disc.addr.val, 6);
            }
            // 키워드로 찾은 밴드 디바이스도 1초 쿨다운 적용
            if (found) {
                uint32_t current_time = esp_timer_get_time() / 1000;
                if ((current_time - last_keyword_band_update) < 1000) {
                    //ESP_LOGI(TAG, "Keyword band device ignored - within 1 second cooldown period, now = %lu", current_time);
                    found = false;
                    return 0;
                } else {
                    ESP_LOGD(TAG, "Keyword band device processing allowed - cooldown period expired");
                }
            }
            
            if (found) {
                ESP_LOGD(TAG, "*** BAND DEVICE FOUND *** RSSI=%d, Addr=%02X:%02X:%02X:%02X:%02X:%02X", 
                         event->disc.rssi,
                         event->disc.addr.val[5], event->disc.addr.val[4], event->disc.addr.val[3],
                         event->disc.addr.val[2], event->disc.addr.val[1], event->disc.addr.val[0]);
                
                // Only log matched devices (filter mode)
                
                //log_adv_report(event);

                // Additionally, extract and print Manufacturer/Service Data in HEX
                const uint8_t *p = data; uint8_t remaining = len;
                bool found_mfg_data = false;
                
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
                        ESP_LOGD(TAG, "ADV field 0x%02X len=%d HEX: %s%s",
                                 field_type, payload_len, buf, (payload_len > 64 ? "..." : ""));
                        
                        // Parse manufacturer data if present
                        if (field_type == BLE_HS_ADV_TYPE_MFG_DATA) {
                            found_mfg_data = true;
                            bool ok = parse_mfg_and_update_band(field_data, payload_len);
                            if (ok) {
                            }
                            // 성공하든 안 하든 타이머 업데이트(BPM에 의한 실패도 간주.)
                            uint32_t current_time = esp_timer_get_time() / 1000;
                            last_band_update = current_time;
                            last_keyword_band_update = current_time;
                        }
                    }
                    
                    p += (field_len + 1); remaining -= (field_len + 1);
                }
            }
            break; }

        case BLE_GAP_EVENT_DISC_COMPLETE:
            ESP_LOGI(TAG, "Scan complete - will restart automatically");
            // 스캔 완료 후 자동으로 재시작 (타이머가 관리)
            break;
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "Connection %s", event->connect.status == 0 ? "established" : "failed");
            if (event->connect.status == 0) {
                current_state = BLUETOOTH_STATE_CONNECTED;
                if (num_conns < MAX_CONNS) {
                    conn_handles[num_conns++] = event->connect.conn_handle;
                    ESP_LOGI(TAG, "conn_handles: %d active", num_conns);
                    
                    // 연결 파라미터 업데이트 요청 (더 빠른 응답을 위해)
                    struct ble_gap_upd_params conn_params = {
                        .itvl_min = BLE_GAP_CONN_ITVL_MS(7.5),   // 7.5ms
                        .itvl_max = BLE_GAP_CONN_ITVL_MS(15),    // 15ms
                        .latency = 0,                            // 지연 없음
                        .supervision_timeout = BLE_GAP_SUPERVISION_TIMEOUT_MS(4000), // 4초
                    };
                    ble_gap_update_params(event->connect.conn_handle, &conn_params);
                    
                } else {
                    ESP_LOGW(TAG, "Max connections reached; disconnecting new conn");
                    ble_gap_terminate(event->connect.conn_handle, BLE_ERR_CONN_LIMIT);
                }
                if (ble_event_group) {
                    xEventGroupSetBits(ble_event_group, BLUETOOTH_READY_BIT);
                }
                
                dfplayer_play_folder(0, 23);

                // 이 연결은 체크를 확인함. 5초동안 응답 없으면 연결 해제
                check_system_active = true;
                check_conn_handle = event->connect.conn_handle;
                
                // 체크 타이머 생성 (5초)
                if (check_timeout_timer == NULL) {
                    check_timeout_timer = xTimerCreate("check_timeout", 
                                                      pdMS_TO_TICKS(5000), 
                                                      pdTRUE, // auto-reload
                                                      0, 
                                                      check_timeout_cb);
                }
                
                if (check_timeout_timer != NULL) {
                    xTimerStart(check_timeout_timer, 0);
                }
                
                ESP_LOGI(TAG, "Check system activated for conn_handle: %d", event->connect.conn_handle);
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGE(TAG, "Disconnected");
            dfplayer_play_folder(0, 24);
            current_state = BLUETOOTH_STATE_ADVERTISING;
            
            // 체크 시스템이 활성화된 연결이 해제되면 체크 시스템 정리
            if (check_system_active && check_conn_handle == event->disconnect.conn.conn_handle) {
                ESP_LOGI(TAG, "Cleaning up check system for disconnected conn_handle: %d", check_conn_handle);
                check_system_active = false;
                check_conn_handle = 0;
                
                if (check_timeout_timer != NULL) {
                    xTimerStop(check_timeout_timer, 0);
                }
            }
            
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
    // 연결 간격 최적화 (빠른 연결을 위해)
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(20);  // 20ms
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(30);  // 30ms
    // 채널 호핑 최적화
    adv_params.channel_map = 0x07;  // 모든 채널 사용

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_cb, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to start advertising; rc=%d", rc);
        return;
    }

    current_state = BLUETOOTH_STATE_ADVERTISING;
    ESP_LOGI(TAG, "Advertising started (connectable)");

    // BLE 스택이 준비된 후 스캔 타이머 시작
    if (ble_scan_timer != NULL) {
        BaseType_t timer_result = xTimerStart(ble_scan_timer, 0);
        ESP_LOGI(TAG, "BLE scan timer started, result: %s", timer_result == pdPASS ? "SUCCESS" : "FAILED");
        ESP_LOGI(TAG, "Timer handle: %p", ble_scan_timer);
    } else {
        ESP_LOGE(TAG, "BLE scan timer is NULL - cannot start");
    }

    // 즉시 첫 번째 스캔 시작
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
    

    
    // BLE 스캔 주기 타이머 생성 (BLE 스택 준비 후 시작)
    if (ble_scan_timer == NULL) {
        ESP_LOGI(TAG, "Creating BLE scan timer...");
        ble_scan_timer = xTimerCreate("ble_scan_timer", pdMS_TO_TICKS(3000), pdTRUE, NULL, scan_timer_cb);
        if (ble_scan_timer == NULL) {
            ESP_LOGE(TAG, "Failed to create BLE scan timer");
            return ESP_FAIL;
        }
        ESP_LOGI(TAG, "BLE scan timer created successfully, handle: %p", ble_scan_timer);
        ESP_LOGI(TAG, "Timer period: 3000ms, Auto-reload: TRUE");
    }


    // Create event group
    ble_event_group = xEventGroupCreate();
    
    // Create NimBLE host task
    nimble_port_freertos_init(nimble_host_task);
    
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
    esp_err_t dfplayer_ret = dfplayer_init(UART_NUM_1, 2, -1, 9600);
    if (dfplayer_ret == ESP_OK) {
        ESP_LOGI(TAG, "DFPlayer Mini initialized successfully");
        // Set volume to 20%
        dfplayer_set_volume(30);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        dfplayer_play_folder(0, 25);
    } else {
        ESP_LOGE(TAG, "DFPlayer Mini initialization failed: %s", esp_err_to_name(dfplayer_ret));
    }

    ESP_LOGI(TAG, "NimBLE Bluetooth and hardware initialized successfully");
    return ESP_OK;
}

// Start optimized scanning for crowded environments
static void start_passive_scan(void)
{
    struct ble_gap_disc_params disc_params = {
        .filter_duplicates = 0,  // 중복 필터링 비활성화 (더 많은 데이터 수집)
        .passive = 0,  // Active scan으로 변경 (스캔 응답도 수신)
        .itvl = BLE_GAP_SCAN_ITVL_MS(100),    // 100ms 간격 (더 빠른 스캔)
        .window = BLE_GAP_SCAN_WIN_MS(100),   // 100ms 윈도우 (더 긴 수신 시간)
        .filter_policy = 0,  // 필터 정책 없음 (모든 디바이스 스캔)
        .limited = 0,
    };

    // 3초 동안 스캔 (scan response를 위한 충분한 시간)
    int rc = ble_gap_disc(own_addr_type, 3000, &disc_params, gap_event_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGE(TAG, "Failed to start scan; rc=%d", rc);
    } else if (rc == 0) {
        ESP_LOGI(TAG, "BLE scanning started - looking for band devices");
    } else {
        ESP_LOGI(TAG, "BLE scan already running");
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
                ESP_LOGI(TAG, "Manufacturer Data found: %d bytes", payload_len);
                if (payload_len >= 2) {
                    uint16_t company_id = (uint16_t)field_data[0] | ((uint16_t)field_data[1] << 8);
                    ESP_LOGI(TAG, "Company ID: 0x%04X", company_id);
                    if (payload_len > 2) {
                        ESP_LOGI(TAG, "MFG Payload: %d bytes", payload_len - 2);
                        char mfg_hex[64] = {0};
                        int hex_len = 0;
                        for (int i = 2; i < payload_len && hex_len < 60; i++) {
                            hex_len += snprintf(&mfg_hex[hex_len], sizeof(mfg_hex) - hex_len, "%02X ", field_data[i]);
                        }
                        ESP_LOGI(TAG, "MFG Data: %s", mfg_hex);
                    }
                }
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
    ESP_LOGI(TAG, "=== SCAN TIMER CALLBACK TRIGGERED ===");
    ESP_LOGI(TAG, "Timer handle: %p", xTimer);
    ESP_LOGI(TAG, "Current tick count: %lu", xTaskGetTickCount());
    ESP_LOGI(TAG, "Scan timer triggered - starting BLE scan");
    start_passive_scan();
    ESP_LOGI(TAG, "=== SCAN TIMER CALLBACK COMPLETED ===");
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
    band.skin_temp = ((float)body_temp_centi) / 100.0f + 2.5f; // 체온센서로부터 보정값 일괄 2.5도 상승 처리
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

// 체크 타이머 콜백 함수 - 5초 타임아웃 시 연결 해제
static void check_timeout_cb(TimerHandle_t xTimer)
{
    if (check_system_active) {
        ESP_LOGW(TAG, "Check timeout! Disconnecting conn_handle: %d", check_conn_handle);
        
        // 체크 시스템 비활성화
        check_system_active = false;
        uint16_t timeout_conn_handle = check_conn_handle;
        check_conn_handle = 0;
        
        // 타이머 정지
        xTimerStop(check_timeout_timer, 0);
        
        // 연결 해제
        ble_gap_terminate(timeout_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    }
}

// BLE 스캔 디버깅 함수들
esp_err_t bluetooth_force_scan_start(void)
{
    ESP_LOGI(TAG, "Force starting BLE scan...");
    
    // 현재 스캔 중지
    ble_gap_disc_cancel();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    // 즉시 스캔 시작
    start_passive_scan();
    
    ESP_LOGI(TAG, "BLE scan force started");
    return ESP_OK;
}

esp_err_t bluetooth_get_scan_status(void)
{
    ESP_LOGI(TAG, "=== BLE Scan Status ===");
    ESP_LOGI(TAG, "Current state: %d", current_state);
    ESP_LOGI(TAG, "Active connections: %d", num_conns);
    ESP_LOGI(TAG, "Scan timer: %s", (ble_scan_timer != NULL) ? "Created" : "NULL");
    
    if (ble_scan_timer != NULL) {
        ESP_LOGI(TAG, "Scan timer active: %s", 
                 (xTimerIsTimerActive(ble_scan_timer) == pdTRUE) ? "Yes" : "No");
    }
    
    ESP_LOGI(TAG, "========================");
    return ESP_OK;
}

void bluetooth_print_scan_statistics(void)
{
    ESP_LOGI(TAG, "=== BLE Scan Statistics ===");
    ESP_LOGI(TAG, "Use bluetooth_get_scan_status() for detailed status");
    ESP_LOGI(TAG, "Check logs for 'Scan active' and 'BAND DEVICE FOUND' messages");
    ESP_LOGI(TAG, "=============================");
}

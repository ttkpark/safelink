

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include <inttypes.h>

// WiFi and Network
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "lwip/sockets.h"

// Sensors
#include "driver/i2c.h"
#include "driver/adc.h"

// I2C 설정
#define I2C_MASTER_SCL_IO           7
#define I2C_MASTER_SDA_IO           6
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// AHT20
#define AHT20_I2C_ADDR              0x38
#define AHT20_INIT_CMD              0xBE
#define AHT20_MEASURE_CMD           0xAC

// MCP9808
#define MCP9808_I2C_ADDR            0x18
#define MCP9808_REG_AMBIENT_TEMP    0x05
#define MCP9808_REG_MANUF_ID        0x06
#define MCP9808_REG_DEVICE_ID       0x07
#define MCP9808_REG_CONFIG          0x01

// ADC4 심박수 측정 설정
#define HEART_RATE_ADC_CHANNEL      ADC1_CHANNEL_3  // GPIO4
#define HEART_RATE_BUFFER_SIZE      390
#define HEART_RATE_SAMPLE_RATE_MS   10  // 10ms마다 샘플링 (50Hz)
#define HEART_RATE_MIN_BPM          40
#define HEART_RATE_MAX_BPM          200
#define ADC_THRESHOLD_PERCENT       50  // 50% 임계값

// 광고 디바이스 이름
#define ADV_DEVICE_NAME             "link_band"

// WiFi 설정
#define WIFI_SSID                   "Smart Meeting"        // WiFi SSID를 여기에 입력하세요
#define WIFI_PASS                   "12345678"    // WiFi 비밀번호를 여기에 입력하세요
#define WIFI_MAXIMUM_RETRY          5
#define WIFI_SSID1                  "junespark-zone2"        // WiFi SSID를 여기에 입력하세요
#define WIFI_PASS1                  "moon6412"    // WiFi 비밀번호를 여기에 입력하세요
#define WIFI_MAXIMUM_RETRY2         5

// UDP 서버 설정
#define UDP_SERVER_IP               "211.221.184.17"
#define UDP_SERVER_PORT             8888
#define UDP_BUFFER_SIZE             1024

static const char *TAG = "NIMBLE_GATTC";
static const char *WIFI_TAG = "WIFI";
static const char *UDP_TAG = "UDP";

// NimBLE GATT Client 관련 변수
static bool gatt_client_ready = false;
static uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC; // ble_hs_id_infer_auto로 설정 예정
// (스캔/연결 상태 제거)

// 센서 가용성 플래그
static bool aht20_available = false;
static bool mcp9808_available = false;

// ADC4 심박수 측정 변수
static uint16_t heart_rate_buffer[HEART_RATE_BUFFER_SIZE];
static uint16_t heart_rate_1step_buffer[HEART_RATE_BUFFER_SIZE];
static float heart_rate_movingavg_buffer[HEART_RATE_BUFFER_SIZE];
static uint16_t heart_rate_buffer_index = 0;
static bool heart_rate_buffer_full = false;
static uint16_t current_heart_rate = 0;
static bool heart_rate_available = false;
static SemaphoreHandle_t heart_rate_mutex = NULL;

uint16_t heart_rate_print_buffer[HEART_RATE_BUFFER_SIZE];
uint16_t heart_rate_movingavg_print_buffer[HEART_RATE_BUFFER_SIZE];
uint16_t heart_rate_print_buffer_index = 0;

// ADC 관련 변수
static uint32_t adc_max_value = 4095;  // 12-bit ADC
static uint32_t adc_threshold_value;

// 심박수 이동 평균 버퍼 (1분 = 30개 데이터, 2초 간격)
#define HEART_RATE_MOVAVG_BUFFER_SIZE 30
static uint16_t heart_rate_movavg_buffer[HEART_RATE_MOVAVG_BUFFER_SIZE];
static uint16_t movavg_buffer_index = 0;
static bool movavg_buffer_full = false;

// 심박센서 펄스 감지 변수
static uint32_t last_pulse_time = 0;           // 마지막 펄스 시간

// WiFi 및 UDP 관련 변수
static int s_retry_num = 0;
static int udp_socket = -1;
static struct sockaddr_in server_addr;
static bool wifi_connected = false;
static SemaphoreHandle_t wifi_semaphore = NULL;


// I2C 및 센서 함수 원형
static esp_err_t i2c_master_init(void);
static void i2c_scan(void);
static esp_err_t aht20_init(void);
static esp_err_t aht20_measure(void);
static esp_err_t aht20_read_data(float *temperature, float *humidity);
static esp_err_t mcp9808_init(void);
static esp_err_t mcp9808_read_temperature(float *temperature);

// ADC4 심박수 측정 함수
static esp_err_t heart_rate_adc_init(void);
static void heart_rate_sampling_task(void *param);
static uint16_t calculate_heart_rate_from_buffer(void);
static void start_heart_rate_sampling(void);
static uint16_t calculate_moving_average_heart_rate(void);
static void print_heart_rate_buffer(void);  // 디버깅용 함수

// 광고만 사용
static void start_sensor_beacon(void);
static void sensor_beacon_task(void *param);
static void ble_app_on_sync(void);
static void ble_host_task(void *param);

// WiFi 및 UDP 함수
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static esp_err_t wifi_init_sta(void);
static esp_err_t udp_client_init(void);
static void udp_send_heart_rate_data(uint16_t heart_rate);
static void udp_send_heart_rate_buffer(void);
static void wifi_udp_task(void *param);
static void udp_send_heart_peak_data(uint16_t *peak_positions, uint16_t peak_count, uint16_t *valid_intervals_array, uint16_t valid_intervals, uint16_t calculated_hr, uint16_t moving_avg_hr);


// NimBLE 초기화
static esp_err_t nimble_init(void)
{
    esp_err_t ret;
    
    ESP_LOGI(TAG, "NimBLE 초기화 시작");
    
    // NVS 초기화
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // NimBLE 초기화
    ret = nimble_port_init();
    if (ret != 0) {
        ESP_LOGE(TAG, "nimble_port_init failed: %ld", (long)ret);
        return ESP_FAIL;
    }
    

    
    // GAP/GATT 서비스 직접 초기화 호출 없이 광고 필드에서 이름 설정 예정
    
    // NimBLE 호스트 스택 초기화
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    
    // NimBLE 포트 FreeRTOS 태스크 시작
    nimble_port_freertos_init(ble_host_task);
    
    ESP_LOGI(TAG, "NimBLE 초기화 완료");
    return ESP_OK;
}

// NimBLE 동기화 콜백
static void ble_app_on_sync(void)
{
    ESP_LOGI(TAG, "NimBLE 동기화 완료");
    gatt_client_ready = true;
    
    ESP_LOGI(TAG, "NimBLE 스택이 성공적으로 초기화되었습니다!");
    
    // own address type 자동 추론 (공개/랜덤/정체성)
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "own_addr_type 결정 실패: %ld", (long)rc);
        own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }
    ESP_LOGI(TAG, "own_addr_type=%ld", (long)own_addr_type);
    
    // I2C + 센서 초기화
    if (i2c_master_init() == ESP_OK) {
        i2c_scan();
        if (aht20_init() == ESP_OK) {
            aht20_available = true;
            ESP_LOGI(TAG, "AHT20 센서 초기화 성공");
        } else {
            ESP_LOGW(TAG, "AHT20 센서 초기화 실패");
        }
        if (mcp9808_init() == ESP_OK) {
            mcp9808_available = true;
            ESP_LOGI(TAG, "MCP9808 센서 초기화 성공");
        } else {
            ESP_LOGW(TAG, "MCP9808 센서 초기화 실패");
        }
    } else {
        ESP_LOGE(TAG, "I2C 초기화 실패");
    }

    // ADC4 심박수 측정 초기화
    if (heart_rate_adc_init() == ESP_OK) {
        heart_rate_available = true;
        start_heart_rate_sampling();
    } else {
        ESP_LOGE(TAG, "ADC4 심박수 측정 초기화 실패");
    }

    // 광고 비콘 시작
    start_sensor_beacon();
}

// NimBLE FreeRTOS 태스크 함수
static void ble_host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE 호스트 태스크 시작");
    nimble_port_run();
}

// 스캔 및 연결 함수
// (스캔 및 연결 함수 제거)

// 서비스 및 특성 탐색
// (서비스 및 특성 탐색 제거)

// 스캔 취소 후 폴백 연결 태스크 (DISC_COMPLETE 미수신 대비)
// (연결 폴백 태스크 제거)

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 Sensor Beacon 시작");
    
    // NimBLE 초기화
    esp_err_t ret = nimble_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NimBLE 초기화 실패: %s", esp_err_to_name(ret));
        return;
    }
    
    ESP_LOGI(TAG, "NimBLE 초기화 완료. Advertising 비콘을 시작합니다...");
    
    // WiFi UDP 태스크 시작
    xTaskCreate(wifi_udp_task, "wifi_udp_task", 8192, NULL, 5, NULL);
    ESP_LOGI(TAG, "WiFi UDP 태스크 시작됨");
    
    // 무한 루프로 상태 모니터링
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        //ESP_LOGI(TAG, "Sensor Beacon 실행 중... (광고명: %s)", ADV_DEVICE_NAME);
    }
}

// Manufacturer Specific Data 빌더 (회사 ID 0x02E5 = Espressif)
/*
 * Advertising Data Structure Report
 * =================================
 * 
 * Device Name: "Safelink_band" (in Scan Response)
 * Company ID: 0x02E5 (Espressif Systems)
 * 
 * Manufacturer Specific Data Format (13 bytes total):
 * --------------------------------------------------
 * Byte 0-1:   Company ID (2바이트 uint16) - 0x02E5 (Espressif)
 * Byte 2:     Version (1바이트 uint8) - 0x01
 * Byte 3:     Flags (1바이트 uint8) - 0x0F (모든 센서 데이터 존재 표시)
 * Byte 4-5:   External Temperature (2바이트 int16) - 센티도 단위 (AHT20)
 * Byte 6-7:   External Humidity (2바이트 uint16) - 센티퍼센트 단위 (AHT20)
 * Byte 8-9:   Body Temperature (2바이트 int16) - 센티도 단위 (MCP9808)
 * Byte 10-11: Heart Rate (2바이트 uint16) - BPM 단위 (MAX3010x)
 * Byte 12:    SpO2 (1바이트 uint8) - 퍼센트 단위 (MAX3010x)
 * 
 * Data Conversion:
 * - Temperature values: float * 100 (centi-degree)
 * - Humidity: float * 100 (centi-percent)
 * - Heart Rate: float to uint16 (BPM)
 * - SpO2: float to uint8 (0-100%)
 * 
 * Example:
 * - External Temp: 32.50°C → 3250 (0x0CAA)
 * - Humidity: 58.14% → 5814 (0x16B6)
 * - Body Temp: 32.69°C → 3269 (0x0CC5)
 * - Heart Rate: 130 BPM → 130 (0x0082)
 * - SpO2: 85% → 85 (0x55)
 */
static uint8_t build_mfg_payload(uint8_t *out, int16_t ext_temp_centi, uint16_t rh_centi,
                                 int16_t body_temp_centi, uint16_t heart_rate_bpm, uint8_t spo2_percent)
{
    const uint16_t company_id = 0x02E5;
    // [CompanyID(2)][ver(1)][flags(1)][extT(2)][RH(2)][bodyT(2)][HR(2)][SpO2(1)]
    out[0] = company_id & 0xFF;
    out[1] = (company_id >> 8) & 0xFF;
    out[2] = 0x01; // version
    out[3] = 0x0F; // flags: extT|RH|bodyT|HR|SpO2 존재 표시
    out[4] = (uint8_t)(ext_temp_centi & 0xFF);
    out[5] = (uint8_t)((ext_temp_centi >> 8) & 0xFF);
    out[6] = (uint8_t)(rh_centi & 0xFF);
    out[7] = (uint8_t)((rh_centi >> 8) & 0xFF);
    out[8] = (uint8_t)(body_temp_centi & 0xFF);
    out[9] = (uint8_t)((body_temp_centi >> 8) & 0xFF);
    out[10] = (uint8_t)(heart_rate_bpm & 0xFF);
    out[11] = (uint8_t)((heart_rate_bpm >> 8) & 0xFF);
    out[12] = spo2_percent;
    return 13;
}

static void start_sensor_beacon(void)
{
    // 디바이스 이름은 광고 필드에서 직접 설정
    // 주기적 광고 업데이트 태스크 시작
    xTaskCreate(sensor_beacon_task, "sensor_beacon", 4096, NULL, 5, NULL);
}

static void sensor_beacon_task(void *param)
{
    while (1) {
        // 센서 측정
        float aht_temp = NAN, aht_rh = NAN;
        float mcp_temp = NAN;
        uint16_t heart_rate_bpm = 0;
        uint8_t spo2_percent = 0;

        if (aht20_available) {
            if (aht20_measure() == ESP_OK) {
                if (aht20_read_data(&aht_temp, &aht_rh) == ESP_OK) {
                    ESP_LOGD(TAG, "AHT20 데이터 읽기 성공: T=%.2fC, RH=%.2f%%", aht_temp, aht_rh);
                } else {
                    ESP_LOGW(TAG, "AHT20 데이터 읽기 실패");
                }
            } else {
                ESP_LOGW(TAG, "AHT20 측정 명령 실패");
            }
        }
        if (mcp9808_available) {
            if (mcp9808_read_temperature(&mcp_temp) == ESP_OK) {
                ESP_LOGD(TAG, "MCP9808 데이터 읽기 성공: T=%.2fC", mcp_temp);
            } else {
                ESP_LOGW(TAG, "MCP9808 데이터 읽기 실패");
            }
        }
        // GPIO4 심박수 측정 사용
        if (heart_rate_available) {
            if (xSemaphoreTake(heart_rate_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                heart_rate_bpm = current_heart_rate;
                xSemaphoreGive(heart_rate_mutex);
            }
        }
        
        //ESP_LOGI(TAG, "heart_rate_bpm: %d", heart_rate_bpm);
        // 산소포화도는 0으로 고정 (MAX3010x 제거됨)
        spo2_percent = 0;
        

        // 값 변환(centi 단위)
        int16_t ext_temp_centi = (int16_t)lroundf((isnan(aht_temp) ? 0.0f : aht_temp) * 100.0f);
        uint16_t rh_centi = (uint16_t)lroundf((isnan(aht_rh) ? 0.0f : aht_rh) * 100.0f);
        int16_t body_temp_centi = (int16_t)lroundf((isnan(mcp_temp) ? 0.0f : mcp_temp) * 100.0f);

        // Advertising Fields 구성 (이름은 Scan Response로 이동하여 31바이트 제한 회피)
        struct ble_hs_adv_fields adv_fields;
        memset(&adv_fields, 0, sizeof(adv_fields));
        adv_fields.flags = BLE_HS_ADV_F_BREDR_UNSUP;  // BR/EDR 미지원만 설정 (연결 불가능)

        uint8_t mfg[31];
        uint8_t mfg_len = build_mfg_payload(mfg, ext_temp_centi, rh_centi, body_temp_centi, heart_rate_bpm, spo2_percent);
        adv_fields.mfg_data = mfg;
        adv_fields.mfg_data_len = mfg_len;

        struct ble_hs_adv_fields rsp_fields;
        memset(&rsp_fields, 0, sizeof(rsp_fields));
        rsp_fields.name = (uint8_t *)ADV_DEVICE_NAME;
        rsp_fields.name_len = strlen(ADV_DEVICE_NAME);
        rsp_fields.name_is_complete = 1;

        // 광고 중지 후 필드 설정 및 재시작 (stop은 비동기이므로 짧게 대기)
        ble_gap_adv_stop();
        vTaskDelay(20 / portTICK_PERIOD_MS);

        int rc = ble_gap_adv_set_fields(&adv_fields);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_set_fields 실패: %ld", (long)rc);
        }
        rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields 실패: %ld", (long)rc);
        }

        struct ble_gap_adv_params advp;
        memset(&advp, 0, sizeof(advp));
        advp.conn_mode = BLE_GAP_CONN_MODE_NON;  // 비연결형 (연결 불가능)
        advp.disc_mode = BLE_GAP_DISC_MODE_GEN;  // 일반 발견 가능
        advp.itvl_min = BLE_GAP_ADV_ITVL_MS(20);   // 광고 간격 최소값 (20ms)
        advp.itvl_max = BLE_GAP_ADV_ITVL_MS(20);   // 광고 간격 최대값 (20ms)

        rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &advp, NULL, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_start 실패: %ld", (long)rc);
        } else {
            //ESP_LOGI(TAG, "Advertising 업데이트 완료 (T=%.2fC, RH=%.2f%%, Body=%.2fC, HR=%lubpm, SpO2=%lu%%)",
            //         ext_temp_centi / 100.0f, rh_centi / 100.0f, body_temp_centi / 100.0f, (unsigned)heart_rate_bpm, (unsigned)spo2_percent);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // 2초 주기로 갱신
    }
}

// ===== I2C & Sensors =====
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "I2C 스캔 시작...");
    for (int addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C 응답: 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C 스캔 완료");
}

static esp_err_t aht20_init(void)
{
    uint8_t cmd[3] = {AHT20_INIT_CMD, 0x08, 0x00};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_I2C_ADDR, cmd, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "AHT20 센서 초기화 완료");
    return ESP_OK;
}

static esp_err_t aht20_measure(void)
{
    uint8_t cmd[3] = {AHT20_MEASURE_CMD, 0x33, 0x00};
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, AHT20_I2C_ADDR, cmd, 3, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    vTaskDelay(80 / portTICK_PERIOD_MS);
    return ESP_OK;
}

static esp_err_t aht20_read_data(float *temperature, float *humidity)
{
    uint8_t data[6];
    esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_NUM, AHT20_I2C_ADDR, data, 6, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;
    if (data[0] & 0x80) return ESP_ERR_INVALID_STATE;
    uint32_t humidity_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | ((data[3] & 0xF0) >> 4);
    *humidity = (float)humidity_raw * 100.0f / 1048576.0f;
    uint32_t temperature_raw = ((uint32_t)(data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
    *temperature = (float)temperature_raw * 200.0f / 1048576.0f - 50.0f;
    return ESP_OK;
}

static esp_err_t mcp9808_init(void)
{
    // 제조사 ID 확인
    uint8_t reg = MCP9808_REG_MANUF_ID; uint8_t d[2];
    if (i2c_master_write_to_device(I2C_MASTER_NUM, MCP9808_I2C_ADDR, &reg, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) return ESP_FAIL;
    if (i2c_master_read_from_device(I2C_MASTER_NUM, MCP9808_I2C_ADDR, d, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) return ESP_FAIL;
    uint16_t manuf = (d[0] << 8) | d[1];
    if (manuf != 0x0054) {
        ESP_LOGW(TAG, "MCP9808 제조사 ID 불일치: 0x%04X", manuf);
    }
    // 장치 ID 확인
    reg = MCP9808_REG_DEVICE_ID; 
    if (i2c_master_write_to_device(I2C_MASTER_NUM, MCP9808_I2C_ADDR, &reg, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) return ESP_FAIL;
    if (i2c_master_read_from_device(I2C_MASTER_NUM, MCP9808_I2C_ADDR, d, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) return ESP_FAIL;
    ESP_LOGI(TAG, "MCP9808 ID: 0x%02X%02X", d[0], d[1]);
    ESP_LOGI(TAG, "MCP9808 센서 초기화 완료");
    return ESP_OK;
}

static esp_err_t mcp9808_read_temperature(float *temperature)
{
    uint8_t reg = MCP9808_REG_AMBIENT_TEMP; uint8_t t[2];
    if (i2c_master_write_to_device(I2C_MASTER_NUM, MCP9808_I2C_ADDR, &reg, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) return ESP_FAIL;
    if (i2c_master_read_from_device(I2C_MASTER_NUM, MCP9808_I2C_ADDR, t, 2, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS) != ESP_OK) return ESP_FAIL;
    uint16_t raw = ((uint16_t)t[0] << 8) | t[1];
    uint16_t clean = raw & 0x1FFF; // 하위 13비트
    bool neg = (clean & 0x1000) != 0;
    uint8_t upper = (clean >> 4) & 0xFF; uint8_t lower = clean & 0x0F;
    int16_t temp_int = neg ? -(int16_t)((~clean + 1) & 0x1FFF) : (int16_t)upper;
    *temperature = (float)temp_int + (float)lower * 0.0625f;
    return ESP_OK;
}


// ===== ADC4 심박수 측정 함수들 =====

static esp_err_t heart_rate_adc_init(void)
{
    // ADC1 초기화
    esp_err_t ret = adc1_config_width(ADC_WIDTH_BIT_12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 width 설정 실패: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = adc1_config_channel_atten(HEART_RATE_ADC_CHANNEL, ADC_ATTEN_DB_12);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 channel attenuation 설정 실패: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // ADC 원시값 사용 (캘리브레이션 없이)
    ESP_LOGI(TAG, "ADC 원시값 사용 모드");
    
    // 임계값 계산 (50% of max value)
    adc_threshold_value = (adc_max_value * ADC_THRESHOLD_PERCENT) / 100;
    
    // 뮤텍스 생성
    heart_rate_mutex = xSemaphoreCreateMutex();
    if (heart_rate_mutex == NULL) {
        ESP_LOGE(TAG, "심박수 뮤텍스 생성 실패");
        return ESP_FAIL;
    }
    
    // 버퍼 초기화
    memset(heart_rate_buffer, 0, sizeof(heart_rate_buffer));
    heart_rate_buffer_index = 0;
    heart_rate_buffer_full = false;
    current_heart_rate = 0;
    
    // 이동 평균 버퍼 초기화
    memset(heart_rate_movavg_buffer, 0, sizeof(heart_rate_movavg_buffer));
    movavg_buffer_index = 0;
    movavg_buffer_full = false;
    
    // 펄스 감지 변수 초기화
    last_pulse_time = 0;
    
    ESP_LOGI(TAG, "ADC4 심박수 측정 초기화 완료 (임계값: %lu)", (unsigned long)adc_threshold_value);
    return ESP_OK;
}

uint16_t peak_positions[50]; // 최대 50개 피크 위치 저장
bool peak_positions_valid[50]; // 최대 50개 피크 위치 저장
uint16_t peak_count = 0;
uint16_t invalid_peak_count = 0;
uint16_t valid_intervals = 0;
uint16_t valid_intervals_array[50] = {0};
uint16_t outliers_count = 0;
uint16_t outliers_array[50] = {0};
static void heart_rate_sampling_task(void *param)
{
    ESP_LOGI(TAG, "심박수 샘플링 태스크 시작");
    
    uint32_t last_calculation_time = 0;
    const uint32_t calculation_interval_ms = 2000; // 2초마다 심박수 계산
    uint8_t last_gpio_level = 0;
    
    while (1) {
        // ADC4에서 아날로그 값 읽기
        uint32_t adc_value = adc1_get_raw(HEART_RATE_ADC_CHANNEL);
        
        // 50% 임계값으로 HIGH/LOW 판단
        uint8_t digital_level = (adc_value > adc_threshold_value) ? 1 : 0;
        
        // 펄스 감지 (상승 엣지)
        if (last_gpio_level == 0 && digital_level == 1) {
            last_pulse_time = esp_timer_get_time() / 1000; // ms 단위
        }
        last_gpio_level = digital_level;
        
        // 뮤텍스로 보호하여 버퍼에 데이터 저장 (아날로그 값 저장)
        if (xSemaphoreTake(heart_rate_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            // ADC 값을 0-65535 범위로 스케일링하여 저장 (16비트)
            uint16_t scaled_value = (uint16_t)((adc_value * 65535.0f) / adc_max_value);
            heart_rate_buffer[heart_rate_buffer_index] = scaled_value;
            heart_rate_buffer_index++;
            
            if (heart_rate_buffer_index >= HEART_RATE_BUFFER_SIZE) {
                heart_rate_buffer_index = 0;
                heart_rate_buffer_full = true;
            }
            
            xSemaphoreGive(heart_rate_mutex);
        }
        
        // 주기적으로 심박수 계산 및 이동 평균 업데이트
        uint32_t current_time = esp_timer_get_time() / 1000; // ms 단위
        if (heart_rate_buffer_full){//(current_time - last_calculation_time) >= calculation_interval_ms) {
            // 심박수 계산
            uint16_t heart_rate_buffer_index_val = heart_rate_buffer_index;
            static uint16_t heart_rate_buffer_index_pre;

            uint16_t buffer_size = heart_rate_buffer_full ? HEART_RATE_BUFFER_SIZE : heart_rate_buffer_index;
            uint16_t calculated_hr = calculate_heart_rate_from_buffer();
            
            // 이동 평균 버퍼에 저장 (유효한 값이든 아니든 모두 저장)
            heart_rate_movavg_buffer[movavg_buffer_index] = calculated_hr;
            movavg_buffer_index++;
            
            if (movavg_buffer_index >= HEART_RATE_MOVAVG_BUFFER_SIZE) {
                movavg_buffer_index = 0;
                movavg_buffer_full = true;
            }
            
            // 4분 데이터에서 이동 평균 계산
            uint16_t moving_avg_hr_val = calculate_moving_average_heart_rate();
            static uint16_t moving_avg_hr = 0;
            if(moving_avg_hr_val > 0)moving_avg_hr = moving_avg_hr_val;

            //moving_avg_hr = 75;
            // 현재 심박수 업데이트 (이동 평균값 사용)
            if (xSemaphoreTake(heart_rate_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                current_heart_rate = calculated_hr;//moving_avg_hr;
                xSemaphoreGive(heart_rate_mutex);
            }
            
            ESP_LOGI(TAG, "심박수: %ld BPM (현재), 이동평균: %ld BPM (1분)", (long)calculated_hr, (long)moving_avg_hr);
            
            
            for(uint16_t i = 0; i < buffer_size; i++) {
                heart_rate_print_buffer[i] = heart_rate_buffer[i];
                heart_rate_movingavg_print_buffer[i] = (uint16_t)heart_rate_movingavg_buffer[i];
            }
            heart_rate_print_buffer_index = buffer_size;
            
            ESP_LOGI(TAG, "이전 Idx : %d, 현재 Idx : %d, diff : %d", heart_rate_buffer_index_pre, heart_rate_buffer_index_val, heart_rate_buffer_index_val - heart_rate_buffer_index_pre);
            heart_rate_buffer_index_pre = heart_rate_buffer_index_val;
            //print_heart_rate_buffer();
            udp_send_heart_rate_buffer();
            
            udp_send_heart_peak_data(peak_positions,peak_count,valid_intervals_array,valid_intervals, calculated_hr, moving_avg_hr);
    

            last_calculation_time = current_time;

            heart_rate_buffer_full = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(HEART_RATE_SAMPLE_RATE_MS));
    }
}
float K_mean = 0;
float K_stddev = 0;
static uint16_t calculate_heart_rate_from_buffer(void)
{
    if (!heart_rate_buffer_full && heart_rate_buffer_index < 150) {
        return 0; // 최소 데이터 확보 필요
    }

    uint16_t buffer_size = heart_rate_buffer_full ? HEART_RATE_BUFFER_SIZE : heart_rate_buffer_index;

    memset(peak_positions, 0, sizeof(peak_positions));
    peak_count = 0;

    // -------------------------------
    // (1) Baseline 제거
    // -------------------------------
    uint16_t *filtered_buffer1 = heart_rate_1step_buffer;
    uint16_t baseline_win = 15; // 0.15초
    for (uint16_t i = 0; i < buffer_size; i++) {
        float sum = 0;
        uint16_t count = 0;
        for (int j = -(baseline_win/2); j <= (baseline_win/2); j++) {
            int idx = i + j;
            if (idx >= 0 && idx < buffer_size) {
                sum += heart_rate_buffer[idx];
                count++;
            }
        }
        float mean = sum / count;
        filtered_buffer1[i] = (float)(mean);
    }
    memcpy(heart_rate_buffer, heart_rate_1step_buffer, buffer_size * sizeof(uint16_t));

    // -------------------------------
    // (1) Baseline 제거
    // -------------------------------
    float *filtered_buffer = heart_rate_movingavg_buffer;
    baseline_win = 150; // 1.5초
    for (uint16_t i = 0; i < buffer_size; i++) {
        float sum = 0;
        uint16_t count = 0;
        for (int j = -(baseline_win/2); j <= (baseline_win/2); j++) {
            int idx = i + j;
            if (idx >= 0 && idx < buffer_size) {
                sum += filtered_buffer1[idx];
                count++;
            }
        }
        float mean = sum / count;
        filtered_buffer[i] = (float)(filtered_buffer1[i]-mean);
    }

    // -------------------------------
    // (2) 피크 검출
    // -------------------------------
    const uint16_t min_distance = 25; // 최소 간격 240BPM    
    const uint16_t window_size  = 10;
    invalid_peak_count = 0;

    for (uint16_t i = window_size; i < buffer_size - window_size && peak_count < 50; i++) {
        float current = filtered_buffer[i];

        // local max 조건
        bool is_peak = true;
        for (int j = -window_size; j <= window_size; j++) {
            if (j == 0) continue;
            if (filtered_buffer[i+j] > current || filtered_buffer[i+j] < 0.f) { is_peak = false; break; }
        }

        if (!is_peak) continue;
        // refractory check
        if (peak_count > 0 && (i - peak_positions[peak_count-1]) < min_distance)continue;


        
        peak_positions_valid[peak_count] = true;
       /*if(fabsf(current) > 32000.f){
            ESP_LOGI(TAG, "피크 높이 과다: idx=%d, 값=%f", i, current);
            peak_positions_valid[peak_count] = false;
            invalid_peak_count++;
        }else */if(fabsf(current) < 80.f){
            ESP_LOGI(TAG, "피크 높이 과소: idx=%d, 값=%f", i, current);
            peak_positions_valid[peak_count] = false;
            invalid_peak_count++;
        }
        peak_positions[peak_count++] = i;

        ESP_LOGI(TAG, "피크 발견: idx=%d, 값=%f", i, current);
    }

    if ((peak_count-invalid_peak_count) < 2) {
        ESP_LOGI(TAG, "피크 부족");
        return 0;
    }
    
    // -------------------------------
    // (3) interval 계산 (이번 측정만)
    // -------------------------------
    valid_intervals = 0;
    for (uint16_t i = 1; i < peak_count; i++) {
        if(!peak_positions_valid[i-1])continue;
        uint16_t interval = peak_positions[i] - peak_positions[i-1];
        if (interval >= 25 && interval <= 240) { // 25~120 = 25~120BPM
            valid_intervals_array[valid_intervals++] = interval;
        }
    }

    if (valid_intervals == 0) {
        ESP_LOGI(TAG, "이번 측정에서 유효한 interval 없음");
        return 0;
    }

    // -------------------------------
    // (4) Outlier 제거 (이번 측정 값 기준)
    // -------------------------------
    float sum = 0;
    for (uint16_t i = 0; i < valid_intervals; i++) sum += valid_intervals_array[i];
    K_mean = sum / valid_intervals;
    uint16_t K_availble_mean = 0;

    float var = 0;
    for (uint16_t i = 0; i < valid_intervals; i++) {
        float diff = valid_intervals_array[i] - K_mean;
        var += diff * diff;
    }
    K_stddev = sqrtf(var / valid_intervals);
    outliers_count=0;

                
    // -------------------------------
    // (5) 정상 interval만 history에 저장
    // -------------------------------
    for (uint16_t i = 0; i < valid_intervals; i++) {
        uint16_t iv = valid_intervals_array[i];
        if (fabsf(iv - K_mean) <= K_stddev * 1.5f) { // 평균 ±1.5σ 안에 있으면 정상
            K_availble_mean += iv;
        } else {
            outliers_array[outliers_count++] = i;
            ESP_LOGI(TAG, "Outlier interval=%d (mean=%.1f, std=%.1f)", iv, K_mean, K_stddev);
        }
    }

    
    float after_var = 0; int after_valid_intervals = 0;
    for (uint16_t i = 0; i < valid_intervals; i++) {
        bool is_outlier = false;
        for (uint16_t j = 0; j < outliers_count; j++) {
            if (i == outliers_array[j]) {
                is_outlier = true;
                break;
            }
        }
        if (is_outlier) continue;

        float diff = valid_intervals_array[i] - K_mean;
        after_var += diff * diff;
        after_valid_intervals++;
    }
    for (uint16_t j = 0; j < outliers_count; j++) {
        outliers_array[j] = valid_intervals_array[j];
    }
    K_stddev = sqrtf(after_var / after_valid_intervals);
    // -------------------------------
    // (7) BPM 계산
    // -------------------------------
    ESP_LOGI(TAG, "after_var: %f", after_var);
    K_availble_mean = K_availble_mean / (valid_intervals - outliers_count);
    uint16_t bpm = (uint16_t)(60.0f * 100.0f / K_availble_mean); // 100Hz 샘플링
    ESP_LOGI(TAG, "최종 BPM: %d", bpm);

    if (bpm < HEART_RATE_MIN_BPM || bpm > HEART_RATE_MAX_BPM) {
        ESP_LOGI(TAG, "BPM 범위 초과");
        return 0;
    }

    if (after_var < 0.1f) {
        ESP_LOGI(TAG, "var 너무 작음");
        return 0;
    }
    if (after_var > 1000.f) {
        ESP_LOGI(TAG, "var 너무 큼");
        return 0;
    }
    return bpm;
}

// 심박수 이동 평균 계산 함수 (4분 데이터에서 이상값 제외)
static uint16_t calculate_moving_average_heart_rate(void)
{
    uint16_t buffer_size = movavg_buffer_full ? HEART_RATE_MOVAVG_BUFFER_SIZE : movavg_buffer_index;
    
    if (buffer_size == 0) {
        return 0;
    }
    
    uint32_t sum = 0;
    uint16_t valid_count = 0;
    
    // 4분 데이터에서 BPM 0과 200 초과 값 제외하고 평균 계산
    for (uint16_t i = 0; i < buffer_size; i++) {
        uint16_t hr = heart_rate_movavg_buffer[i];
        
        // 유효한 심박수 범위 체크 (0 제외, 200 초과 제외)
        if (hr > 0 && hr <= 200) {
            sum += hr;
            valid_count++;
        }
    }
    
    if (valid_count == 0) {
        return 0;
    }
    
    return (uint16_t)(sum / valid_count);
}

// 심박수 버퍼 내용을 인덱스별로 출력하는 함수 (디버깅용)

static void print_heart_rate_buffer(void)
{
    if (xSemaphoreTake(heart_rate_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint16_t buffer_size = heart_rate_print_buffer_index;
        
        ESP_LOGI(TAG, "=== 심박수 버퍼 출력 (크기: %ld) ===", (long)buffer_size);
        
        // 한 열로 출력
        ESP_LOGI(TAG, "심박수 버퍼: `");
        for (uint16_t i = 0; i < buffer_size; i++) {
            ESP_LOGI(TAG, "IR 버퍼   %4d,   %5d, %5d", i, heart_rate_print_buffer[i],heart_rate_movingavg_print_buffer[i]);

            if((i%0x1F) == 0)
                vTaskDelay(pdMS_TO_TICKS(1));
        }
        
        ESP_LOGI(TAG, "end ~");
        xSemaphoreGive(heart_rate_mutex);
    }
}


static void start_heart_rate_sampling(void)
{
    xTaskCreate(heart_rate_sampling_task, "heart_rate_sampling", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "심박수 샘플링 태스크 시작됨");
}

// ===== WiFi 및 UDP 함수들 =====

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "WiFi 재연결 시도 %d/%d", s_retry_num, WIFI_MAXIMUM_RETRY);
        } else {
            ESP_LOGE(WIFI_TAG, "WiFi 연결 실패. 최대 재시도 횟수 초과");
        }
        wifi_connected = false;
        ESP_LOGI(WIFI_TAG, "WiFi 연결 끊어짐");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(WIFI_TAG, "IP 주소 획득:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        wifi_connected = true;
        if (wifi_semaphore) {
            xSemaphoreGive(wifi_semaphore);
        }
    }
}

static esp_err_t wifi_init_sta(void)
{
    wifi_semaphore = xSemaphoreCreateBinary();
    if (wifi_semaphore == NULL) {
        ESP_LOGE(WIFI_TAG, "WiFi 세마포어 생성 실패");
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(WIFI_TAG, "WiFi 초기화 완료");

    // WiFi 연결 대기
    if (xSemaphoreTake(wifi_semaphore, portMAX_DELAY) == pdTRUE) {
        ESP_LOGI(WIFI_TAG, "WiFi 연결 성공");
        return ESP_OK;
    } else {
        ESP_LOGE(WIFI_TAG, "WiFi 연결 실패");
        return ESP_FAIL;
    }
}

static esp_err_t udp_client_init(void)
{
    // UDP 소켓 생성
    udp_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_socket < 0) {
        ESP_LOGE(UDP_TAG, "UDP 소켓 생성 실패: errno %d", errno);
        return ESP_FAIL;
    }

    // 서버 주소 설정
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(UDP_SERVER_PORT);
    inet_pton(AF_INET, UDP_SERVER_IP, &server_addr.sin_addr);

    ESP_LOGI(UDP_TAG, "UDP 클라이언트 초기화 완료 - 서버: %s:%d", UDP_SERVER_IP, UDP_SERVER_PORT);
    return ESP_OK;
}
static void udp_send_heart_peak_data(uint16_t *peak_positions, uint16_t peak_count, uint16_t *valid_intervals_array, uint16_t valid_intervals, uint16_t calculated_hr, uint16_t moving_avg_hr)
{
    if (udp_socket < 0 || !wifi_connected) {
        return;
    }
    
    // JSON 형태로 심박 버퍼 데이터 구성
    char udp_data[1024];
    char temp_buffer[64];
    
    // JSON 시작
    snprintf(udp_data, sizeof(udp_data), "\np\npeak_count : %d,\ncalculated_hr : %d,\nmoving_avg_hr : %d\n",peak_count, calculated_hr, moving_avg_hr);
    int idx = strlen(udp_data);

    for(int i=0;i<peak_count;i++){
        snprintf(temp_buffer, sizeof(temp_buffer),"peak %d : idx=%d\n",i,peak_positions[i]);
        strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
        idx += strlen(temp_buffer);
    }
    snprintf(temp_buffer, sizeof(temp_buffer),"valid_intervals %d\n",valid_intervals);
    strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
    idx += strlen(temp_buffer);

    for(int i=0;i<valid_intervals;i++){
        snprintf(temp_buffer, sizeof(temp_buffer),"valid_intervals %d : idx=%d\n",i,valid_intervals_array[i]);
        strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
        idx += strlen(temp_buffer);
    }

    snprintf(temp_buffer, sizeof(temp_buffer),"outliers_count %d\n",outliers_count);
    strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
    idx += strlen(temp_buffer);

    for(int i=0;i<outliers_count;i++){
        snprintf(temp_buffer, sizeof(temp_buffer),"outliers_array %d : idx=%d\n",i,outliers_array[i]);
        strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
        idx += strlen(temp_buffer);
    }

    snprintf(temp_buffer, sizeof(temp_buffer),"interval_history_index %d\n",interval_history_index);
    strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
    idx += strlen(temp_buffer);
    
    snprintf(temp_buffer, sizeof(temp_buffer),"mean %f\n",K_mean);
    strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
    idx += strlen(temp_buffer);

    snprintf(temp_buffer, sizeof(temp_buffer),"stddev %f\n",K_stddev);
    strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
    idx += strlen(temp_buffer);


    int err = sendto(udp_socket, udp_data, idx, 0,
        (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err < 0) {
    ESP_LOGE(UDP_TAG, "심박 피크 버퍼 UDP 전송 실패: errno %d", errno);
    }else{
    //ESP_LOGI(UDP_TAG, "심박 피크 버퍼 데이터 전송 완료 (크기: %d, 전송 바이트: %d)", 
    //         buffer_size, idx);
    }
}

static void udp_send_heart_rate_data(uint16_t heart_rate)
{
    if (udp_socket < 0 || !wifi_connected) {
        return;
    }

    // JSON 형태로 심박 데이터 구성
    char json_data[256];
    snprintf(json_data, sizeof(json_data), 
             "{\"device_id\":\"%s\",\"heart_rate\":%d,\"timestamp\":%lld}",
             ADV_DEVICE_NAME, heart_rate, esp_timer_get_time() / 1000);

    // UDP로 데이터 전송
    int err = sendto(udp_socket, json_data, strlen(json_data), 0,
                     (struct sockaddr *)&server_addr, sizeof(server_addr));
    
    if (err < 0) {
        ESP_LOGE(UDP_TAG, "UDP 전송 실패: errno %d", errno);
    } else {
        ESP_LOGI(UDP_TAG, "심박 데이터 전송: %s", json_data);
    }
}

static void udp_send_heart_rate_buffer(void)
{
    if (udp_socket < 0 || !wifi_connected) {
        return;
    }
    
    uint16_t buffer_size = heart_rate_print_buffer_index;
        
    // JSON 형태로 심박 버퍼 데이터 구성
    char udp_data[1024];
    char temp_buffer[64];
    
    // JSON 시작
    snprintf(udp_data, sizeof(udp_data), 
             "{\"device_id\":\"%s\",\"timestamp\":%lld,\"buffer_size\":%d\n",
             ADV_DEVICE_NAME, esp_timer_get_time() / 1000, buffer_size);
    int idx = strlen(udp_data);
    // 버퍼 데이터 추가 (buffer_size/2까지만, 원본 함수와 동일)
    for (uint16_t i = 0; i < buffer_size; i++) {

        snprintf(temp_buffer, sizeof(temp_buffer), 
                 "%3d,%5d,%5d\n",
                 i, heart_rate_print_buffer[i], heart_rate_movingavg_print_buffer[i]);
        strncpy(udp_data+idx, temp_buffer, strlen(temp_buffer));
        idx += strlen(temp_buffer);
        
        if((i%0x1F) == 0)
            vTaskDelay(pdMS_TO_TICKS(1));


        if(idx >= sizeof(udp_data)-16 || i == buffer_size-1){
            vTaskDelay(pdMS_TO_TICKS(1));
            int err = sendto(udp_socket, udp_data, idx, 0,
                             (struct sockaddr *)&server_addr, sizeof(server_addr));
            if (err < 0) {
                ESP_LOGE(UDP_TAG, "심박 버퍼 UDP 전송 실패: errno %d", errno);
            }else{
                //ESP_LOGI(UDP_TAG, "심박 버퍼 데이터 전송 완료 (크기: %d, 전송 바이트: %d)", 
                //         buffer_size, idx);
            }

            idx = 0;
        }
    }
}

static void wifi_udp_task(void *param)
{
    ESP_LOGI(WIFI_TAG, "WiFi UDP 태스크 시작");
    
    // WiFi 초기화 및 연결
    if (wifi_init_sta() != ESP_OK) {
        ESP_LOGE(WIFI_TAG, "WiFi 초기화 실패");
        vTaskDelete(NULL);
        return;
    }

    // UDP 클라이언트 초기화
    if (udp_client_init() != ESP_OK) {
        ESP_LOGE(UDP_TAG, "UDP 클라이언트 초기화 실패");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(WIFI_TAG, "WiFi 및 UDP 초기화 완료");

    // 주기적으로 심박 데이터 전송
    while (1) {
        if (wifi_connected && heart_rate_available) {
            uint16_t current_hr = 0;
            if (xSemaphoreTake(heart_rate_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                current_hr = current_heart_rate;
                xSemaphoreGive(heart_rate_mutex);
            }
            
            if (current_hr > 0) {
                udp_send_heart_rate_data(current_hr);
                //udp_send_heart_rate_buffer();
                
            }
        }
        
        // 5초마다 전송
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}


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
// GATT 서비스 헤더는 사용하지 않음
#include <ctype.h>
#include <inttypes.h>

// Sensors
#include "driver/i2c.h"
#include "driver/gpio.h"

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

// GPIO4 심박수 측정 설정
#define HEART_RATE_GPIO             4
#define HEART_RATE_BUFFER_SIZE      1000
#define HEART_RATE_SAMPLE_RATE_MS   10  // 10ms마다 샘플링 (100Hz)
#define HEART_RATE_MIN_BPM          40
#define HEART_RATE_MAX_BPM          200

// 광고 디바이스 이름
#define ADV_DEVICE_NAME             "link_band"

static const char *TAG = "NIMBLE_GATTC";

// NimBLE GATT Client 관련 변수
static bool gatt_client_ready = false;
static uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC; // ble_hs_id_infer_auto로 설정 예정
// (스캔/연결 상태 제거)

// 센서 가용성 플래그
static bool aht20_available = false;
static bool mcp9808_available = false;

// GPIO4 심박수 측정 변수
static uint8_t heart_rate_buffer[HEART_RATE_BUFFER_SIZE];
static uint16_t heart_rate_buffer_index = 0;
static bool heart_rate_buffer_full = false;
static uint16_t current_heart_rate = 0;
static bool heart_rate_available = false;
static SemaphoreHandle_t heart_rate_mutex = NULL;

// I2C 및 센서 함수 원형
static esp_err_t i2c_master_init(void);
static void i2c_scan(void);
static esp_err_t aht20_init(void);
static esp_err_t aht20_measure(void);
static esp_err_t aht20_read_data(float *temperature, float *humidity);
static esp_err_t mcp9808_init(void);
static esp_err_t mcp9808_read_temperature(float *temperature);

// GPIO4 심박수 측정 함수
static esp_err_t heart_rate_gpio_init(void);
static void heart_rate_sampling_task(void *param);
static uint16_t calculate_heart_rate_from_buffer(void);
static void start_heart_rate_sampling(void);

// 광고만 사용
static void start_sensor_beacon(void);
static void sensor_beacon_task(void *param);
static void ble_app_on_sync(void);
static void ble_host_task(void *param);
// (GATT 콜백 제거)

// GATT 에러 상세 로깅 유틸
static void log_gatt_error(const char *phase, uint16_t conn_handle, const struct ble_gatt_error *error)
{
    if (error == NULL) return;
    ESP_LOGE(TAG, "%s: status=%d (0x%02x), conn_handle=%u, att_handle=%u",
             phase, error->status, error->status, conn_handle, error->att_handle);
}

// (GAP 이벤트 핸들러 제거)

// GATT 서비스 발견 콜백
static int gatt_svc_discovered(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_svc *service, void *arg)
{
    if (error != NULL && error->status != 0) {
        log_gatt_error("서비스 탐색 오류", conn_handle, error);
        return 0;
    }
    
    if (service == NULL) {
        ESP_LOGI(TAG, "=== 서비스 탐색 완료 ===");
        return 0;
    }
    
    // 서비스 UUID를 문자열로 변환
    char uuid_str[37];
    if (BLE_UUID16(&service->uuid)) {
        snprintf(uuid_str, sizeof(uuid_str), "0x%04X", BLE_UUID16(&service->uuid)->value);
    } else if (BLE_UUID32(&service->uuid)) {
        snprintf(uuid_str, sizeof(uuid_str), "0x%08" PRIX32, BLE_UUID32(&service->uuid)->value);
    } else {
        snprintf(uuid_str, sizeof(uuid_str), "128-bit UUID");
    }
    
    ESP_LOGI(TAG, "서비스 발견 - UUID: %s, 시작 핸들: %d, 끝 핸들: %d",
             uuid_str, service->start_handle, service->end_handle);
    
    // 알려진 서비스 UUID 매핑
    if (BLE_UUID16(&service->uuid)) {
        switch (BLE_UUID16(&service->uuid)->value) {
            case 0x1800: ESP_LOGI(TAG, "  -> Generic Access Service"); break;
            case 0x1801: ESP_LOGI(TAG, "  -> Generic Attribute Service"); break;
            case 0x180A: ESP_LOGI(TAG, "  -> Device Information Service"); break;
            case 0x180D: ESP_LOGI(TAG, "  -> Heart Rate Service"); break;
            case 0x180F: ESP_LOGI(TAG, "  -> Battery Service"); break;
            case 0x1810: ESP_LOGI(TAG, "  -> Health Thermometer Service"); break;
            case 0x1812: ESP_LOGI(TAG, "  -> HID Service"); break;
            case 0x1813: ESP_LOGI(TAG, "  -> Scan Parameters Service"); break;
            case 0x1814: ESP_LOGI(TAG, "  -> Running Speed and Cadence Service"); break;
            case 0x1815: ESP_LOGI(TAG, "  -> Automation IO Service"); break;
            case 0x1816: ESP_LOGI(TAG, "  -> Cycling Speed and Cadence Service"); break;
            case 0x1818: ESP_LOGI(TAG, "  -> Weight Scale Service"); break;
            case 0x1819: ESP_LOGI(TAG, "  -> Current Time Service"); break;
            case 0x181A: ESP_LOGI(TAG, "  -> Environmental Sensing Service"); break;
            case 0x181B: ESP_LOGI(TAG, "  -> Location and Navigation Service"); break;
            case 0x181C: ESP_LOGI(TAG, "  -> User Data Service"); break;
            case 0x181D: ESP_LOGI(TAG, "  -> Weight Scale Service"); break;
            case 0x181E: ESP_LOGI(TAG, "  -> Bond Management Service"); break;
            case 0x181F: ESP_LOGI(TAG, "  -> Continuous Glucose Monitoring Service"); break;
            case 0x1820: ESP_LOGI(TAG, "  -> Internet Protocol Support Service"); break;
            case 0x1821: ESP_LOGI(TAG, "  -> Indoor Positioning Service"); break;
            case 0x1822: ESP_LOGI(TAG, "  -> Pulse Oximeter Service"); break;
            case 0x1823: ESP_LOGI(TAG, "  -> HTTP Proxy Service"); break;
            case 0x1824: ESP_LOGI(TAG, "  -> Transport Discovery Service"); break;
            case 0x1825: ESP_LOGI(TAG, "  -> Object Transfer Service"); break;
            case 0x1826: ESP_LOGI(TAG, "  -> Fitness Machine Service"); break;
            case 0x1827: ESP_LOGI(TAG, "  -> Mesh Provisioning Service"); break;
            case 0x1828: ESP_LOGI(TAG, "  -> Mesh Proxy Service"); break;
            case 0x1829: ESP_LOGI(TAG, "  -> Reconnection Configuration Service"); break;
            case 0x182A: ESP_LOGI(TAG, "  -> Insulin Delivery Service"); break;
            case 0x182B: ESP_LOGI(TAG, "  -> Binary Sensor Service"); break;
            case 0x182C: ESP_LOGI(TAG, "  -> Emergency Configuration Service"); break;
            case 0x182D: ESP_LOGI(TAG, "  -> Physical Activity Monitor Service"); break;
            case 0x182E: ESP_LOGI(TAG, "  -> Audio Input Control Service"); break;
            case 0x182F: ESP_LOGI(TAG, "  -> Volume Control Service"); break;
            case 0x1830: ESP_LOGI(TAG, "  -> Volume Offset Control Service"); break;
            case 0x1831: ESP_LOGI(TAG, "  -> Coordinated Set Identification Service"); break;
            case 0x1832: ESP_LOGI(TAG, "  -> Device Time Service"); break;
            case 0x1833: ESP_LOGI(TAG, "  -> Media Control Service"); break;
            case 0x1834: ESP_LOGI(TAG, "  -> Generic Media Control Service"); break;
            case 0x1835: ESP_LOGI(TAG, "  -> Constant Tone Extension Service"); break;
            case 0x1836: ESP_LOGI(TAG, "  -> Telephone Bearer Service"); break;
            case 0x1837: ESP_LOGI(TAG, "  -> Generic Telephone Bearer Service"); break;
            case 0x1838: ESP_LOGI(TAG, "  -> Microphone Control Service"); break;
            case 0x1839: ESP_LOGI(TAG, "  -> Audio Stream Control Service"); break;
            case 0x183A: ESP_LOGI(TAG, "  -> Broadcast Audio Scan Service"); break;
            case 0x183B: ESP_LOGI(TAG, "  -> Published Audio Capabilities Service"); break;
            case 0x183C: ESP_LOGI(TAG, "  -> Audio Input Control Service"); break;
            case 0x183D: ESP_LOGI(TAG, "  -> Volume Control Service"); break;
            case 0x183E: ESP_LOGI(TAG, "  -> Volume Offset Control Service"); break;
            case 0x183F: ESP_LOGI(TAG, "  -> Coordinated Set Identification Service"); break;
            case 0x1840: ESP_LOGI(TAG, "  -> Device Time Service"); break;
            case 0x1841: ESP_LOGI(TAG, "  -> Media Control Service"); break;
            case 0x1842: ESP_LOGI(TAG, "  -> Generic Media Control Service"); break;
            case 0x1843: ESP_LOGI(TAG, "  -> Constant Tone Extension Service"); break;
            case 0x1844: ESP_LOGI(TAG, "  -> Telephone Bearer Service"); break;
            case 0x1845: ESP_LOGI(TAG, "  -> Generic Telephone Bearer Service"); break;
            case 0x1846: ESP_LOGI(TAG, "  -> Microphone Control Service"); break;
            case 0x1847: ESP_LOGI(TAG, "  -> Audio Stream Control Service"); break;
            case 0x1848: ESP_LOGI(TAG, "  -> Broadcast Audio Scan Service"); break;
            case 0x1849: ESP_LOGI(TAG, "  -> Published Audio Capabilities Service"); break;
            default: ESP_LOGI(TAG, "  -> 사용자 정의 서비스"); break;
        }
    }
    
    // 타겟 서비스 검사 제거 (GATT Client 비활성)
    (void)conn_handle; (void)error; (void)service; (void)arg;
    
    return 0;
}

// GATT 특성 발견 콜백
static int gatt_chr_discovered(uint16_t conn_handle, const struct ble_gatt_error *error, const struct ble_gatt_chr *chr, void *arg)
{
    if (error != NULL && error->status != 0) {
        log_gatt_error("특성 탐색 오류", conn_handle, error);
        return 0;
    }
    
    if (chr == NULL) {
        ESP_LOGI(TAG, "=== 특성 탐색 완료 ===");
        return 0;
    }
    
    // 특성 UUID를 문자열로 변환
    char uuid_str[37];
    if (BLE_UUID16(&chr->uuid)) {
        snprintf(uuid_str, sizeof(uuid_str), "0x%04X", BLE_UUID16(&chr->uuid)->value);
    } else if (BLE_UUID32(&chr->uuid)) {
        snprintf(uuid_str, sizeof(uuid_str), "0x%08" PRIX32, BLE_UUID32(&chr->uuid)->value);
    } else {
        snprintf(uuid_str, sizeof(uuid_str), "128-bit UUID");
    }
    
    ESP_LOGI(TAG, "특성 발견 - UUID: %s, 핸들: %d, 속성: 0x%02X",
             uuid_str, chr->val_handle, chr->properties);
    
    // 알려진 특성 UUID 매핑
    if (BLE_UUID16(&chr->uuid)) {
        switch (BLE_UUID16(&chr->uuid)->value) {
            case 0x2A00: ESP_LOGI(TAG, "  -> Device Name"); break;
            case 0x2A01: ESP_LOGI(TAG, "  -> Appearance"); break;
            case 0x2A02: ESP_LOGI(TAG, "  -> Peripheral Privacy Flag"); break;
            case 0x2A03: ESP_LOGI(TAG, "  -> Reconnection Address"); break;
            case 0x2A04: ESP_LOGI(TAG, "  -> Peripheral Preferred Connection Parameters"); break;
            case 0x2A05: ESP_LOGI(TAG, "  -> Service Changed"); break;
            case 0x2A06: ESP_LOGI(TAG, "  -> Alert Level"); break;
            case 0x2A07: ESP_LOGI(TAG, "  -> Tx Power Level"); break;
            case 0x2A08: ESP_LOGI(TAG, "  -> Date Time"); break;
            case 0x2A09: ESP_LOGI(TAG, "  -> Day of Week"); break;
            case 0x2A0A: ESP_LOGI(TAG, "  -> Day Date Time"); break;
            case 0x2A0B: ESP_LOGI(TAG, "  -> Exact Time 100"); break;
            case 0x2A0C: ESP_LOGI(TAG, "  -> Exact Time 256"); break;
            case 0x2A0D: ESP_LOGI(TAG, "  -> DST Offset"); break;
            case 0x2A0E: ESP_LOGI(TAG, "  -> Time Zone"); break;
            case 0x2A0F: ESP_LOGI(TAG, "  -> Local Time Information"); break;
            case 0x2A10: ESP_LOGI(TAG, "  -> Secondary Time Zone"); break;
            case 0x2A11: ESP_LOGI(TAG, "  -> Time with DST"); break;
            case 0x2A12: ESP_LOGI(TAG, "  -> Time Accuracy"); break;
            case 0x2A13: ESP_LOGI(TAG, "  -> Time Source"); break;
            case 0x2A14: ESP_LOGI(TAG, "  -> Reference Time Information"); break;
            case 0x2A15: ESP_LOGI(TAG, "  -> Time Update Control Point"); break;
            case 0x2A16: ESP_LOGI(TAG, "  -> Time Update State"); break;
            case 0x2A17: ESP_LOGI(TAG, "  -> Glucose Measurement"); break;
            case 0x2A18: ESP_LOGI(TAG, "  -> Battery Level"); break;
            case 0x2A19: ESP_LOGI(TAG, "  -> Temperature Measurement"); break;
            case 0x2A1A: ESP_LOGI(TAG, "  -> Temperature Type"); break;
            case 0x2A1B: ESP_LOGI(TAG, "  -> Intermediate Temperature"); break;
            case 0x2A1C: ESP_LOGI(TAG, "  -> Measurement Interval"); break;
            case 0x2A1D: ESP_LOGI(TAG, "  -> Boot Keyboard Input Report"); break;
            case 0x2A1E: ESP_LOGI(TAG, "  -> System ID"); break;
            case 0x2A1F: ESP_LOGI(TAG, "  -> Model Number String"); break;
            case 0x2A20: ESP_LOGI(TAG, "  -> Serial Number String"); break;
            case 0x2A21: ESP_LOGI(TAG, "  -> Firmware Revision String"); break;
            case 0x2A22: ESP_LOGI(TAG, "  -> Hardware Revision String"); break;
            case 0x2A23: ESP_LOGI(TAG, "  -> Software Revision String"); break;
            case 0x2A24: ESP_LOGI(TAG, "  -> Manufacturer Name String"); break;
            case 0x2A25: ESP_LOGI(TAG, "  -> IEEE 11073-20601 Regulatory Certification Data List"); break;
            case 0x2A26: ESP_LOGI(TAG, "  -> Current Time"); break;
            case 0x2A27: ESP_LOGI(TAG, "  -> Elevation"); break;
            case 0x2A28: ESP_LOGI(TAG, "  -> Latitude"); break;
            case 0x2A29: ESP_LOGI(TAG, "  -> Longitude"); break;
            case 0x2A2A: ESP_LOGI(TAG, "  -> LN Control Point"); break;
            case 0x2A2B: ESP_LOGI(TAG, "  -> LN Feature"); break;
            case 0x2A2C: ESP_LOGI(TAG, "  -> Local North Coordinate"); break;
            case 0x2A2D: ESP_LOGI(TAG, "  -> Local East Coordinate"); break;
            case 0x2A2E: ESP_LOGI(TAG, "  -> Local North Reference"); break;
            case 0x2A2F: ESP_LOGI(TAG, "  -> Local East Reference"); break;
            case 0x2A30: ESP_LOGI(TAG, "  -> Local North Reference"); break;
            case 0x2A31: ESP_LOGI(TAG, "  -> Local East Reference"); break;
            case 0x2A32: ESP_LOGI(TAG, "  -> Local North Reference"); break;
            case 0x2A33: ESP_LOGI(TAG, "  -> Local East Reference"); break;
            case 0x2A34: ESP_LOGI(TAG, "  -> Local North Reference"); break;
            case 0x2A35: ESP_LOGI(TAG, "  -> Local East Reference"); break;
            case 0x2A36: ESP_LOGI(TAG, "  -> Local North Reference"); break;
            case 0x2A37: ESP_LOGI(TAG, "  -> Heart Rate Measurement"); break;
            case 0x2A38: ESP_LOGI(TAG, "  -> Body Sensor Location"); break;
            case 0x2A39: ESP_LOGI(TAG, "  -> Heart Rate Control Point"); break;
            case 0x2A3A: ESP_LOGI(TAG, "  -> Alert Status"); break;
            case 0x2A3B: ESP_LOGI(TAG, "  -> Ringer Control point"); break;
            case 0x2A3C: ESP_LOGI(TAG, "  -> Ringer Setting"); break;
            case 0x2A3D: ESP_LOGI(TAG, "  -> Alert Category ID Bit Mask"); break;
            case 0x2A3E: ESP_LOGI(TAG, "  -> Alert Category ID"); break;
            case 0x2A3F: ESP_LOGI(TAG, "  -> Alert Notification Control Point"); break;
            case 0x2A40: ESP_LOGI(TAG, "  -> Unread Alert Status"); break;
            case 0x2A41: ESP_LOGI(TAG, "  -> New Alert"); break;
            case 0x2A42: ESP_LOGI(TAG, "  -> Supported New Alert Category"); break;
            case 0x2A43: ESP_LOGI(TAG, "  -> Supported Unread Alert Category"); break;
            case 0x2A44: ESP_LOGI(TAG, "  -> Blood Pressure Feature"); break;
            case 0x2A45: ESP_LOGI(TAG, "  -> Intermediate Cuff Pressure"); break;
            case 0x2A46: ESP_LOGI(TAG, "  -> Heart Rate Measurement"); break;
            case 0x2A47: ESP_LOGI(TAG, "  -> Heart Rate Max"); break;
            case 0x2A48: ESP_LOGI(TAG, "  -> Heart Rate Min"); break;
            default: ESP_LOGI(TAG, "  -> 사용자 정의 특성"); break;
        }
    }
    
    // GATT Client 비활성: 매개변수 미사용 처리
    (void)conn_handle; (void)error; (void)chr; (void)arg;
    return 0;
}

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
        ESP_LOGE(TAG, "nimble_port_init failed: %d", ret);
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
        ESP_LOGE(TAG, "own_addr_type 결정 실패: %d", rc);
        own_addr_type = BLE_OWN_ADDR_PUBLIC;
    }
    ESP_LOGI(TAG, "own_addr_type=%d", own_addr_type);
    
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

    // GPIO4 심박수 측정 초기화
    if (heart_rate_gpio_init() == ESP_OK) {
        heart_rate_available = true;
        start_heart_rate_sampling();
    } else {
        ESP_LOGE(TAG, "GPIO4 심박수 측정 초기화 실패");
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
    
    // 무한 루프로 상태 모니터링
    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Sensor Beacon 실행 중... (광고명: %s)", ADV_DEVICE_NAME);
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
            ESP_LOGE(TAG, "ble_gap_adv_set_fields 실패: %d", rc);
        }
        rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields 실패: %d", rc);
        }

        struct ble_gap_adv_params advp;
        memset(&advp, 0, sizeof(advp));
        advp.conn_mode = BLE_GAP_CONN_MODE_NON;  // 비연결형 (연결 불가능)
        advp.disc_mode = BLE_GAP_DISC_MODE_GEN;  // 일반 발견 가능
        advp.itvl_min = BLE_GAP_ADV_ITVL_MS(20);   // 광고 간격 최소값 (20ms)
        advp.itvl_max = BLE_GAP_ADV_ITVL_MS(20);   // 광고 간격 최대값 (20ms)

        rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &advp, NULL, NULL);
        if (rc != 0) {
            ESP_LOGE(TAG, "ble_gap_adv_start 실패: %d", rc);
        } else {
            ESP_LOGI(TAG, "Advertising 업데이트 완료 (T=%.2fC, RH=%.2f%%, Body=%.2fC, HR=%ubpm, SpO2=%u%%)",
                     ext_temp_centi / 100.0f, rh_centi / 100.0f, body_temp_centi / 100.0f, (unsigned)heart_rate_bpm, (unsigned)spo2_percent);
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


// ===== GPIO4 심박수 측정 함수들 =====

static esp_err_t heart_rate_gpio_init(void)
{
    // GPIO4를 입력으로 설정
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << HEART_RATE_GPIO),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO4 설정 실패: %s", esp_err_to_name(ret));
        return ret;
    }
    
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
    
    ESP_LOGI(TAG, "GPIO4 심박수 측정 초기화 완료");
    return ESP_OK;
}

static void heart_rate_sampling_task(void *param)
{
    ESP_LOGI(TAG, "심박수 샘플링 태스크 시작");
    
    uint32_t last_calculation_time = 0;
    const uint32_t calculation_interval_ms = 2000; // 2초마다 심박수 계산
    
    while (1) {
        // GPIO4 상태 읽기
        int gpio_level = gpio_get_level(HEART_RATE_GPIO);
        
        // 뮤텍스로 보호하여 버퍼에 데이터 저장
        if (xSemaphoreTake(heart_rate_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            heart_rate_buffer[heart_rate_buffer_index] = (uint8_t)gpio_level;
            heart_rate_buffer_index++;
            
            if (heart_rate_buffer_index >= HEART_RATE_BUFFER_SIZE) {
                heart_rate_buffer_index = 0;
                heart_rate_buffer_full = true;
            }
            
            xSemaphoreGive(heart_rate_mutex);
        }
        
        // 주기적으로 심박수 계산
        uint32_t current_time = esp_timer_get_time() / 1000; // ms 단위
        if (current_time - last_calculation_time >= calculation_interval_ms) {
            uint16_t calculated_hr = calculate_heart_rate_from_buffer();
            if (calculated_hr >= HEART_RATE_MIN_BPM && calculated_hr <= HEART_RATE_MAX_BPM) {
                if (xSemaphoreTake(heart_rate_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    current_heart_rate = calculated_hr;
                    xSemaphoreGive(heart_rate_mutex);
                }
                ESP_LOGI(TAG, "심박수: %d BPM", calculated_hr);
            }
            last_calculation_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(HEART_RATE_SAMPLE_RATE_MS));
    }
}

static uint16_t calculate_heart_rate_from_buffer(void)
{
    if (!heart_rate_buffer_full && heart_rate_buffer_index < 100) {
        return 0; // 충분한 데이터가 없음
    }
    
    uint16_t buffer_size = heart_rate_buffer_full ? HEART_RATE_BUFFER_SIZE : heart_rate_buffer_index;
    uint16_t peak_positions[50]; // 최대 50개 피크 위치 저장
    uint16_t peak_count = 0;
    uint8_t last_state = 0;
    uint8_t current_state = 0;
    
    // HIGH->LOW->HIGH 패턴을 찾아서 피크 위치 저장
    for (uint16_t i = 1; i < buffer_size && peak_count < 50; i++) {
        current_state = heart_rate_buffer[i];
        
        // 상승 엣지 감지 (LOW -> HIGH)
        if (last_state == 0 && current_state == 1) {
            peak_positions[peak_count] = i;
            peak_count++;
        }
        
        last_state = current_state;
    }
    
    if (peak_count < 2) {
        return 0; // 최소 2개 피크가 필요
    }
    
    // 피크 간 간격 계산
    uint32_t total_intervals = 0;
    uint16_t valid_intervals = 0;
    
    for (uint16_t i = 1; i < peak_count; i++) {
        uint16_t interval = peak_positions[i] - peak_positions[i-1];
        
        // 유효한 간격 범위 체크 (40-200 BPM에 해당하는 간격)
        // 100Hz 샘플링에서: 30-150 샘플 간격이 유효
        if (interval >= 30 && interval <= 150) {
            total_intervals += interval;
            valid_intervals++;
        }
    }
    
    if (valid_intervals == 0) {
        return 0;
    }
    
    // 평균 피크 간격 계산
    uint32_t avg_interval = total_intervals / valid_intervals;
    
    // BPM 계산: 100Hz 샘플링에서 60초(6000 샘플) / 평균간격
    uint16_t bpm = 6000 / avg_interval;
    
    // 유효 범위 체크
    if (bpm < HEART_RATE_MIN_BPM || bpm > HEART_RATE_MAX_BPM) {
        return 0;
    }
    
    return bpm;
}

static void start_heart_rate_sampling(void)
{
    xTaskCreate(heart_rate_sampling_task, "heart_rate_sampling", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "심박수 샘플링 태스크 시작됨");
}
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

static const char *TAG = "SENSOR_SPP_SERVER";

// AM2320 센서 설정
#define AM2320_I2C_ADDR        0x5C
#define AM2320_I2C_MASTER_SCL_IO    9     // GPIO9 for SCL (ESP32-C6)
#define AM2320_I2C_MASTER_SDA_IO    8     // GPIO8 for SDA (ESP32-C6)
#define AM2320_I2C_MASTER_NUM       I2C_NUM_0
#define AM2320_I2C_MASTER_FREQ_HZ   50000
#define AM2320_I2C_MASTER_TX_BUF_DISABLE   0
#define AM2320_I2C_MASTER_RX_BUF_DISABLE   0

// 아날로그 심박 센서 설정 (ESP32-C6용)
#define HEARTBEAT_ADC_UNIT         ADC_UNIT_1
#define HEARTBEAT_ADC_CHANNEL      ADC_CHANNEL_4  // GPIO4 (ESP32-C6)
#define HEARTBEAT_ADC_ATTEN        ADC_ATTEN_DB_12  // ESP32-C6는 DB_11 대신 DB_12 사용
#define HEARTBEAT_ADC_BITWIDTH     ADC_BITWIDTH_12

// SPP 설정
#define SPP_TAG "SPP_SERVER"
#define SPP_SERVER_NAME "SafeLink_Sensor_SPP"

// AM2320 명령어
#define AM2320_CMD_READ_REG        0x03
#define AM2320_CMD_WRITE_REG       0x10
#define AM2320_REG_HUMIDITY_H      0x00
#define AM2320_REG_TEMPERATURE_H   0x02

// 심박 측정 설정
#define HEARTBEAT_SAMPLE_RATE      100  // Hz
#define HEARTBEAT_BUFFER_SIZE      1000
#define HEARTBEAT_THRESHOLD        2000  // 임계값 (12비트 ADC 기준)

// 전역 변수
static adc_oneshot_unit_handle_t adc1_handle;
static uint32_t heartbeat_buffer[HEARTBEAT_BUFFER_SIZE];
static int buffer_index = 0;
static int heartbeat_count = 0;
static uint32_t last_heartbeat_time = 0;
static bool spp_connected = false;
static uint32_t spp_handle = 0;

// 센서 데이터 구조체
typedef struct {
    float temperature;
    float humidity;
    float heart_rate;
    uint32_t timestamp;
} sensor_data_t;

// 새로운 I2C 마스터 버스 설정
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = AM2320_I2C_MASTER_SCL_IO,
    .sda_io_num = AM2320_I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

// AM2320 디바이스 설정
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = AM2320_I2C_ADDR,
    .scl_speed_hz = AM2320_I2C_MASTER_FREQ_HZ,
};

// I2C 마스터 초기화 (새로운 API 사용)
static esp_err_t i2c_master_init(void)
{
    // GPIO 설정 - 내부 풀업 저항 활성화
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << AM2320_I2C_MASTER_SDA_IO) | (1ULL << AM2320_I2C_MASTER_SCL_IO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,  // Open-drain 모드
        .pull_up_en = GPIO_PULLUP_ENABLE,   // 내부 풀업 저항 활성화
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully with internal pull-up resistors");
    ESP_LOGI(TAG, "SDA: GPIO%d, SCL: GPIO%d", AM2320_I2C_MASTER_SDA_IO, AM2320_I2C_MASTER_SCL_IO);
    return ESP_OK;
}

// AM2320 센서 초기화 (새로운 I2C API 사용)
static esp_err_t am2320_init(void)
{    
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    i2c_master_dev_handle_t dev_handle;
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 대기
    
    // 리소스 정리
    i2c_master_bus_rm_device(dev_handle);
    i2c_del_master_bus(bus_handle);
    
    ESP_LOGI(TAG, "AM2320 sensor initialized");
    return ESP_OK;
}

// AM2320에서 온습도 읽기 (올바른 프로토콜 사용)
static esp_err_t am2320_read_temp_humidity(float *temperature, float *humidity)
{
    uint8_t data[8];
    uint16_t temp_raw, hum_raw;
    
    i2c_master_bus_handle_t bus_handle;
    esp_err_t ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    i2c_master_dev_handle_t dev_handle;
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(bus_handle);
        return ret;
    }
    
    // 온습도 읽기 명령 전송
    uint8_t read_cmd[3] = {
        AM2320_CMD_READ_REG,
        AM2320_REG_HUMIDITY_H,
        0x04  // 4바이트 읽기
    };
    
    ret = i2c_master_transmit(dev_handle, read_cmd, 3, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AM2320 read command failed: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev_handle);
        i2c_del_master_bus(bus_handle);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(2));  // 2ms 대기
    
    // 데이터 읽기 (7바이트: 1바이트 함수코드 + 1바이트 바이트수 + 4바이트 데이터 + 1바이트 CRC)
    ret = i2c_master_receive(dev_handle, data, 7, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AM2320 read data failed: %s", esp_err_to_name(ret));
        i2c_master_bus_rm_device(dev_handle);
        i2c_del_master_bus(bus_handle);
        return ret;
    }
    
    ESP_LOGI(TAG, "AM2320 raw data: %02X %02X %02X %02X %02X %02X %02X",
             data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
    
    if (data[0] != 0x03 || data[1] != 0x04) {
        ESP_LOGE(TAG, "AM2320 invalid response");
        i2c_master_bus_rm_device(dev_handle);
        i2c_del_master_bus(bus_handle);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    hum_raw = (data[2] << 8) | data[3];   // 습도 (상위바이트, 하위바이트)
    temp_raw = (data[4] << 8) | data[5];  // 온도 (상위바이트, 하위바이트)
    
    *humidity = (float)hum_raw / 10.0;
    *temperature = (float)temp_raw / 10.0;
    
    if (*temperature > 100) {
        *temperature = *temperature - 256;
    }
    
    i2c_master_bus_rm_device(dev_handle);
    i2c_del_master_bus(bus_handle);
    
    return ESP_OK;
}

// ADC 초기화
static esp_err_t adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 new unit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "ADC1 initialized successfully");
    return ESP_OK;
}

// 심박 측정
static void heartbeat_measure(void)
{
    int adc_reading = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL_4, &adc_reading);

    if (ret == ESP_OK) {
        heartbeat_buffer[buffer_index] = (uint32_t)adc_reading;
        buffer_index = (buffer_index + 1) % HEARTBEAT_BUFFER_SIZE;
        
        static uint32_t last_peak_time = 0;
        static bool was_above_threshold = false;
        
        if (adc_reading > HEARTBEAT_THRESHOLD) {
            if (!was_above_threshold) {
                uint32_t current_time = xTaskGetTickCount();
                if (current_time - last_peak_time > pdMS_TO_TICKS(300)) {
                    heartbeat_count++;
                    last_heartbeat_time = current_time;
                    last_peak_time = current_time;
                    ESP_LOGI(TAG, "Heartbeat detected! Count: %d", heartbeat_count);
                }
                was_above_threshold = true;
            }
        } else {
            was_above_threshold = false;
        }
    } else {
        ESP_LOGE(TAG, "ADC1 read failed: %s", esp_err_to_name(ret));
    }
}

// 심박수 계산
static float calculate_heart_rate(void)
{
    uint32_t current_time = xTaskGetTickCount();
    uint32_t ten_seconds_ago = current_time - pdMS_TO_TICKS(10000);
    
    if (last_heartbeat_time > ten_seconds_ago) {
        return (float)heartbeat_count * 6.0;
    }
    return 0.0;
}

// SPP를 통해 센서 데이터 전송
static void send_sensor_data_spp(sensor_data_t *data)
{
    if (!spp_connected) {
        ESP_LOGW(TAG, "SPP not connected, cannot send data");
        return;
    }
    
    char buffer[256];
    int len = snprintf(buffer, sizeof(buffer), 
                      "SENSOR_DATA:{\"temperature\":%.1f,\"humidity\":%.1f,\"heart_rate\":%.1f,\"timestamp\":%lu}\r\n",
                      data->temperature, data->humidity, data->heart_rate, data->timestamp);
    
    esp_err_t ret = esp_spp_write(spp_handle, len, (uint8_t*)buffer);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sent via SPP: %s", buffer);
    } else {
        ESP_LOGE(TAG, "SPP write failed: %s", esp_err_to_name(ret));
    }
}

// 블루투스 초기화
static esp_err_t bt_init(void)
{
    esp_err_t ret;
    
    // 블루투스 컨트롤러 초기화
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Bluetooth initialized successfully");
    return ESP_OK;
}

// SPP 콜백 함수
static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP initialized");
            // SPP 서버 시작
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SPP_SERVER_NAME);
            break;
            
        case ESP_SPP_START_EVT:
            ESP_LOGI(TAG, "SPP server started, waiting for client connection...");
            break;
            
        case ESP_SPP_SRV_OPEN_EVT:
            ESP_LOGI(TAG, "SPP client connected");
            spp_connected = true;
            spp_handle = param->srv_open.handle;
            break;
            
        case ESP_SPP_CLOSE_EVT:
            ESP_LOGI(TAG, "SPP client disconnected");
            spp_connected = false;
            spp_handle = 0;
            break;
            
        case ESP_SPP_DATA_IND_EVT:
            // 데이터 수신 처리 (필요시)
            ESP_LOGI(TAG, "SPP data received, len: %d", param->data_ind.len);
            break;
            
        case ESP_SPP_WRITE_EVT:
            // 데이터 전송 완료
            if (param->write.status != ESP_SPP_SUCCESS) {
                ESP_LOGE(TAG, "SPP write failed");
            }
            break;
            
        case ESP_SPP_CONG_EVT:
            // 연결 상태 변경
            if (param->cong.cong) {
                ESP_LOGW(TAG, "SPP connection congested");
            } else {
                ESP_LOGI(TAG, "SPP connection not congested");
            }
            break;
            
        default:
            break;
    }
}

// SPP 초기화
esp_err_t spp_init(void)
{
    esp_err_t ret = esp_spp_register_callback(esp_spp_cb);
    if (ret) {
        ESP_LOGE(TAG, "esp_spp_register_callback failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_spp_init(ESP_SPP_MODE_CB);
    if (ret) {
        ESP_LOGE(TAG, "esp_spp_init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "SPP initialized successfully");
    return ESP_OK;
}

// 센서 데이터 수집 및 전송 태스크
void sensor_data_task(void *pvParameters)
{
    sensor_data_t sensor_data;
    int error_count = 0;
    
    ESP_LOGI(TAG, "Sensor data task started");
    
    while (1) {
        esp_err_t ret = am2320_read_temp_humidity(&sensor_data.temperature, &sensor_data.humidity);
        
        if (ret == ESP_OK) {
            error_count = 0;
        } else {
            error_count++;
            ESP_LOGE(TAG, "AM2320 read failed (error count: %d): %s", error_count, esp_err_to_name(ret));
            
            if (error_count >= 5) {
                ESP_LOGE(TAG, "Too many AM2320 errors, reinitializing...");
                am2320_init();
                error_count = 0;
            }
        }
        
        sensor_data.heart_rate = calculate_heart_rate();
        sensor_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        if (spp_connected) {
            ESP_LOGI(TAG, "SPP Connected - Temperature: %.1f°C, Humidity: %.1f%%, Heart Rate: %.1f BPM", 
                     sensor_data.temperature, sensor_data.humidity, sensor_data.heart_rate);
            send_sensor_data_spp(&sensor_data);
        } else {
            ESP_LOGI(TAG, "Waiting for SPP client connection... Temperature: %.1f°C, Humidity: %.1f%%, Heart Rate: %.1f BPM", 
                     sensor_data.temperature, sensor_data.humidity, sensor_data.heart_rate);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10000));  // 10초마다 측정 및 전송
    }
}

// 심박 센서 모니터링 태스크
void heartbeat_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Heartbeat monitor task started");
    
    while (1) {
        heartbeat_measure();
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz 샘플링
    }
}

// 메인 함수
void app_main(void)
{
    ESP_LOGI(TAG, "SafeLink Sensor SPP Server Starting...");
    
    // I2C 초기화
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return;
    }
    
    // AM2320 센서 초기화
    ret = am2320_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AM2320 initialization failed");
        return;
    }
    
    // ADC 초기화
    ret = adc_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC initialization failed");
        return;
    }
    
    // 블루투스 초기화
    ret = bt_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bluetooth initialization failed");
        return;
    }
    
    // SPP 초기화
    ret = spp_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPP initialization failed");
        return;
    }
    
    // 태스크 생성
    xTaskCreate(sensor_data_task, "sensor_data_task", 4096, NULL, 5, NULL);
    xTaskCreate(heartbeat_monitor_task, "heartbeat_monitor_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "AM2320: SCL=GPIO%d, SDA=GPIO%d", AM2320_I2C_MASTER_SCL_IO, AM2320_I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "Heartbeat: ADC1_CH4 (GPIO4)");
    ESP_LOGI(TAG, "SPP Server: Waiting for client connection...");
} 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAG = "SENSOR_TEST";

// AM2320 센서 설정
#define AM2320_I2C_ADDR        0x5C
#define AM2320_I2C_MASTER_SCL_IO    8      // GPIO8 for SCL
#define AM2320_I2C_MASTER_SDA_IO    10     // GPIO10 for SDA
#define AM2320_I2C_MASTER_NUM       I2C_NUM_0
#define AM2320_I2C_MASTER_FREQ_HZ   100000
#define AM2320_I2C_MASTER_TX_BUF_DISABLE   0
#define AM2320_I2C_MASTER_RX_BUF_DISABLE   0

// 아날로그 심박 센서 설정
#define HEARTBEAT_ADC_CHANNEL      ADC1_CHANNEL_4  // GPIO4 (ESP32-S3)
#define HEARTBEAT_ADC_ATTEN        ADC_ATTEN_DB_11
#define HEARTBEAT_ADC_WIDTH        ADC_WIDTH_BIT_12
#define HEARTBEAT_ADC_UNIT         ADC_UNIT_1

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
static esp_adc_cal_characteristics_t adc_chars;
static uint32_t heartbeat_buffer[HEARTBEAT_BUFFER_SIZE];
static int buffer_index = 0;
static int heartbeat_count = 0;
static uint32_t last_heartbeat_time = 0;

// I2C 마스터 초기화
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = AM2320_I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = AM2320_I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = AM2320_I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t err = i2c_param_config(AM2320_I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter config failed: %s", esp_err_to_name(err));
        return err;
    }
    
    err = i2c_driver_install(AM2320_I2C_MASTER_NUM, conf.mode, 
                           AM2320_I2C_MASTER_RX_BUF_DISABLE, 
                           AM2320_I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

// AM2320 센서 초기화
static esp_err_t am2320_init(void)
{
    // AM2320는 특별한 초기화가 필요하지 않음
    // 단순히 I2C 통신이 가능한지 확인
    uint8_t dummy_data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, dummy_data, true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(AM2320_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "AM2320 sensor detected");
    } else {
        ESP_LOGE(TAG, "AM2320 sensor not found: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

// AM2320에서 온습도 읽기
static esp_err_t am2320_read_temp_humidity(float *temperature, float *humidity)
{
    uint8_t data[8];
    uint16_t temp_raw, hum_raw;
    
    // AM2320는 읽기 전에 웨이크업 명령이 필요
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);  // Wake up command
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(AM2320_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 대기
    
    // 온습도 읽기 명령 전송
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, AM2320_CMD_READ_REG, true);
    i2c_master_write_byte(cmd, AM2320_REG_HUMIDITY_H, true);
    i2c_master_write_byte(cmd, 0x04, true);  // 4바이트 읽기
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(AM2320_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AM2320 read command failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(2));  // 2ms 대기
    
    // 데이터 읽기
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 8, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(AM2320_I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "AM2320 read data failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // CRC 검증 (간단한 버전)
    if (data[0] != 0x03 || data[1] != 0x04) {
        ESP_LOGE(TAG, "AM2320 invalid response");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    // 데이터 파싱
    hum_raw = (data[2] << 8) | data[3];
    temp_raw = (data[4] << 8) | data[5];
    
    // 온도는 10배로 저장되어 있음, 습도는 10배로 저장되어 있음
    *humidity = (float)hum_raw / 10.0;
    *temperature = (float)temp_raw / 10.0;
    
    // 온도가 음수인 경우 처리
    if (*temperature > 100) {
        *temperature = *temperature - 256;
    }
    
    return ESP_OK;
}

// ADC 초기화
static esp_err_t adc_init(void)
{
    esp_err_t ret = esp_adc_cal_characterize(HEARTBEAT_ADC_UNIT, HEARTBEAT_ADC_ATTEN, 
                                           HEARTBEAT_ADC_WIDTH, 1100, &adc_chars);
    if (ret == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ADC calibration using eFuse Vref");
    } else if (ret == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ADC calibration using eFuse Two Point");
    } else {
        ESP_LOGI(TAG, "ADC calibration using Default Vref");
    }
    
    ret = adc1_config_width(HEARTBEAT_ADC_WIDTH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 config width failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = adc1_config_channel_atten(HEARTBEAT_ADC_CHANNEL, HEARTBEAT_ADC_ATTEN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC1 config channel atten failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ADC initialized successfully");
    return ESP_OK;
}

// 심박 측정
static void heartbeat_measure(void)
{
    uint32_t adc_reading = adc1_get_raw(HEARTBEAT_ADC_CHANNEL);
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    
    // 버퍼에 저장
    heartbeat_buffer[buffer_index] = adc_reading;
    buffer_index = (buffer_index + 1) % HEARTBEAT_BUFFER_SIZE;
    
    // 심박 감지 (간단한 임계값 기반)
    static uint32_t last_peak_time = 0;
    static bool was_above_threshold = false;
    
    if (adc_reading > HEARTBEAT_THRESHOLD) {
        if (!was_above_threshold) {
            uint32_t current_time = xTaskGetTickCount();
            if (current_time - last_peak_time > pdMS_TO_TICKS(300)) {  // 최소 300ms 간격
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
}

// AM2320 센서 테스트 태스크
static void am2320_task(void *pvParameters)
{
    float temperature, humidity;
    int error_count = 0;
    
    ESP_LOGI(TAG, "AM2320 sensor task started");
    
    while (1) {
        esp_err_t ret = am2320_read_temp_humidity(&temperature, &humidity);
        
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.1f%%", temperature, humidity);
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
        
        vTaskDelay(pdMS_TO_TICKS(2000));  // 2초마다 측정
    }
}

// 심박 센서 테스트 태스크
static void heartbeat_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Heartbeat sensor task started");
    
    while (1) {
        heartbeat_measure();
        
        // 1초마다 심박수 계산 및 출력
        static uint32_t last_report_time = 0;
        uint32_t current_time = xTaskGetTickCount();
        
        if (current_time - last_report_time >= pdMS_TO_TICKS(1000)) {
            // 최근 10초간의 심박수 계산
            uint32_t recent_heartbeats = 0;
            uint32_t ten_seconds_ago = current_time - pdMS_TO_TICKS(10000);
            
            if (last_heartbeat_time > ten_seconds_ago) {
                recent_heartbeats = heartbeat_count;
            }
            
            float bpm = (float)recent_heartbeats * 6.0;  // 10초 * 6 = 60초
            
            ESP_LOGI(TAG, "Heart Rate: %.1f BPM, Total Count: %d", bpm, heartbeat_count);
            last_report_time = current_time;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz 샘플링
    }
}

// 메인 함수
void app_main(void)
{
    ESP_LOGI(TAG, "ESP-PICO Sensor Test (AM2320 + Heartbeat) Starting...");
    
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
    
    // 태스크 생성
    xTaskCreate(am2320_task, "am2320_task", 4096, NULL, 5, NULL);
    xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "AM2320: SCL=GPIO8, SDA=GPIO10");
    ESP_LOGI(TAG, "Heartbeat: ADC1_CH4 (GPIO4)");
} 
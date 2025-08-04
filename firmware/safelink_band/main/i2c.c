#include "i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include <string.h>

static const char *TAG = "I2C";

esp_err_t i2c_master_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C master...");
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_master_deinit(void)
{
    esp_err_t ret = i2c_driver_delete(I2C_NUM_0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C master deinitialized");
    return ESP_OK;
}

// AM2320 센서 읽기 함수
esp_err_t AM2320_read(uint16_t *temperature, uint16_t *humidity)
{
    // 센서 깨우기
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_ADDR << 1) | I2C_MASTER_WRITE, false);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake sensor: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 읽기 명령 전송
    uint8_t read_cmd[3] = {0x03, 0x00, 0x04};
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, read_cmd, 3, true);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send read command: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 데이터 읽기
    uint8_t data[8];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 8, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 데이터 검증
    uint16_t checksum = (data[6] << 8) | data[7];
    uint16_t calculated_checksum = data[0] + data[1] + data[2] + data[3] + data[4] + data[5];
    
    if (checksum != calculated_checksum) {
        ESP_LOGE(TAG, "Checksum mismatch: expected 0x%04x, got 0x%04x", calculated_checksum, checksum);
        return ESP_ERR_INVALID_CRC;
    }
    
    // 온도 및 습도 계산
    *humidity = (data[2] << 8) | data[3];
    *temperature = (data[4] << 8) | data[5];
    
    ESP_LOGI(TAG, "Temperature: %d.%d°C, Humidity: %d.%d%%", 
             *temperature / 10, *temperature % 10,
             *humidity / 10, *humidity % 10);
    
    return ESP_OK;
} 
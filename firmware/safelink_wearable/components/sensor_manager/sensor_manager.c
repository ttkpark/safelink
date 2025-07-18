/*
 * Sensor Manager Implementation
 */

#include "sensor_manager.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "freertos/timers.h"
#include <math.h>

#define TAG "SENSOR_MGR"

#define I2C_MASTER_SCL_IO    22    // GPIO for I2C SCL
#define I2C_MASTER_SDA_IO    21    // GPIO for I2C SDA
#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

// Sensor sampling rate (1Hz)
#define SENSOR_SAMPLE_PERIOD_MS 1000

static sensor_data_t current_sensor_data = {0};
static sensor_status_t sensor_status = SENSOR_STATUS_OK;
static TaskHandle_t sensor_task_handle = NULL;
static TimerHandle_t sensor_timer = NULL;
static SemaphoreHandle_t data_mutex = NULL;

/* Private function declarations */
static void sensor_task(void *param);
static void sensor_timer_callback(TimerHandle_t timer);
static esp_err_t init_heart_rate_sensor(void);
static esp_err_t init_skin_temp_sensor(void);
static esp_err_t init_env_temp_humidity_sensor(void);
static float read_heart_rate(void);
static float read_skin_temperature(void);
static float read_ambient_temperature(void);
static float read_humidity(void);

esp_err_t sensor_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing sensor manager...");
    
    // Create mutex for data protection
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_FAIL;
    }

    // Initialize I2C for sensors
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Initialize individual sensors
    ret = init_heart_rate_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Heart rate sensor init failed");
        return ret;
    }

    ret = init_skin_temp_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Skin temperature sensor init failed");
        return ret;
    }

    ret = init_env_temp_humidity_sensor();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Environment sensor init failed");
        return ret;
    }

    // Create sensor sampling timer
    sensor_timer = xTimerCreate("sensor_timer", 
                               pdMS_TO_TICKS(SENSOR_SAMPLE_PERIOD_MS),
                               pdTRUE, // Auto-reload
                               NULL,
                               sensor_timer_callback);
    
    if (sensor_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor timer");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor manager initialized successfully");
    return ESP_OK;
}

esp_err_t sensor_manager_start(void)
{
    ESP_LOGI(TAG, "Starting sensor manager...");
    
    // Start sensor timer
    if (xTimerStart(sensor_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start sensor timer");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sensor manager started");
    return ESP_OK;
}

esp_err_t sensor_manager_stop(void)
{
    ESP_LOGI(TAG, "Stopping sensor manager...");
    
    // Stop sensor timer
    if (sensor_timer != NULL) {
        xTimerStop(sensor_timer, portMAX_DELAY);
    }

    ESP_LOGI(TAG, "Sensor manager stopped");
    return ESP_OK;
}

esp_err_t sensor_manager_get_data(sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        memcpy(data, &current_sensor_data, sizeof(sensor_data_t));
        xSemaphoreGive(data_mutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}

sensor_status_t sensor_manager_get_status(void)
{
    return sensor_status;
}

float calculate_wbgt(float temp, float humidity)
{
    // Simplified WBGT calculation for indoor use
    // WBGT ≈ 0.7 * Tw + 0.3 * Tg
    // Where Tw is wet bulb temperature approximated from temp and humidity
    
    // Approximate wet bulb temperature calculation
    float wet_bulb = temp * atan(0.151977 * sqrt(humidity + 8.313659)) +
                     atan(temp + humidity) - atan(humidity - 1.676331) +
                     0.00391838 * pow(humidity, 1.5) * atan(0.023101 * humidity) - 4.686035;
    
    // Simplified WBGT for indoor conditions (no globe temperature)
    float wbgt = 0.7 * wet_bulb + 0.3 * temp;
    
    return wbgt;
}

/* Private function implementations */
static void sensor_timer_callback(TimerHandle_t timer)
{
    // Read all sensors
    float heart_rate = read_heart_rate();
    float skin_temp = read_skin_temperature();
    float ambient_temp = read_ambient_temperature();
    float humidity = read_humidity();
    float wbgt = calculate_wbgt(ambient_temp, humidity);
    
    // Update sensor data with mutex protection
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        current_sensor_data.heart_rate = heart_rate;
        current_sensor_data.skin_temperature = skin_temp;
        current_sensor_data.ambient_temp = ambient_temp;
        current_sensor_data.humidity = humidity;
        current_sensor_data.wbgt = wbgt;
        current_sensor_data.timestamp = esp_timer_get_time() / 1000; // Convert to ms
        
        xSemaphoreGive(data_mutex);
    }
    
    ESP_LOGD(TAG, "Sensors: HR=%.1f, SkinT=%.1f, AmbT=%.1f, Hum=%.1f, WBGT=%.1f",
             heart_rate, skin_temp, ambient_temp, humidity, wbgt);
}

static esp_err_t init_heart_rate_sensor(void)
{
    // TODO: Initialize specific heart rate sensor (e.g., MAX30102)
    ESP_LOGI(TAG, "Heart rate sensor initialized (placeholder)");
    return ESP_OK;
}

static esp_err_t init_skin_temp_sensor(void)
{
    // TODO: Initialize skin temperature sensor (e.g., MLX90614)
    ESP_LOGI(TAG, "Skin temperature sensor initialized (placeholder)");
    return ESP_OK;
}

static esp_err_t init_env_temp_humidity_sensor(void)
{
    // TODO: Initialize environment sensor (e.g., SHT30, DHT22)
    ESP_LOGI(TAG, "Environment sensor initialized (placeholder)");
    return ESP_OK;
}

static float read_heart_rate(void)
{
    // TODO: Implement actual heart rate reading
    // For now, return simulated data
    return 75.0 + (esp_random() % 20); // 75-95 BPM
}

static float read_skin_temperature(void)
{
    // TODO: Implement actual skin temperature reading
    // For now, return simulated data
    return 36.0 + (float)(esp_random() % 200) / 100.0; // 36.0-38.0°C
}

static float read_ambient_temperature(void)
{
    // TODO: Implement actual ambient temperature reading
    // For now, return simulated data
    return 25.0 + (float)(esp_random() % 1000) / 100.0; // 25.0-35.0°C
}

static float read_humidity(void)
{
    // TODO: Implement actual humidity reading
    // For now, return simulated data
    return 40.0 + (float)(esp_random() % 4000) / 100.0; // 40.0-80.0%
} 
/*
 * Power Manager Implementation (Hub Version)
 */

#include "power_manager.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "esp_pm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "POWER_MGR"

#define BATTERY_ADC_CHANNEL ADC_CHANNEL_0
#define CHARGING_STATUS_PIN GPIO_NUM_12
#define LOW_BATTERY_THRESHOLD 20 // 20%

static battery_status_t current_battery_status = {0};
static power_state_t current_power_state = POWER_STATE_ACTIVE;
static adc_oneshot_unit_handle_t adc_handle = NULL;

esp_err_t power_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing power manager...");
    
    // Initialize ADC for battery monitoring
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };
    
    ret = adc_oneshot_config_channel(adc_handle, BATTERY_ADC_CHANNEL, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize charging status pin
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << CHARGING_STATUS_PIN),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);
    
    // Initialize battery status
    current_battery_status.percentage = 100;
    current_battery_status.voltage = 3.7f;
    current_battery_status.is_charging = false;
    current_battery_status.time_remaining = 2880; // 48 hours in minutes
    
    ESP_LOGI(TAG, "Power manager initialized");
    return ESP_OK;
}

esp_err_t power_manager_get_battery_status(battery_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read battery voltage from ADC
    int adc_raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, BATTERY_ADC_CHANNEL, &adc_raw);
    if (ret == ESP_OK) {
        // Convert ADC reading to voltage (assuming 2:1 voltage divider)
        float voltage = (adc_raw * 3.3f * 2.0f) / 4096.0f;
        current_battery_status.voltage = voltage;
        
        // Estimate battery percentage based on voltage
        if (voltage >= 4.1f) {
            current_battery_status.percentage = 100;
        } else if (voltage >= 3.9f) {
            current_battery_status.percentage = 80 + (uint8_t)((voltage - 3.9f) * 100.0f);
        } else if (voltage >= 3.7f) {
            current_battery_status.percentage = 40 + (uint8_t)((voltage - 3.7f) * 200.0f);
        } else if (voltage >= 3.4f) {
            current_battery_status.percentage = 10 + (uint8_t)((voltage - 3.4f) * 100.0f);
        } else {
            current_battery_status.percentage = 0;
        }
    }
    
    // Check charging status
    current_battery_status.is_charging = (gpio_get_level(CHARGING_STATUS_PIN) == 0);
    
    // Update power state based on charging
    if (current_battery_status.is_charging) {
        current_power_state = POWER_STATE_CHARGING;
    }
    
    // Estimate time remaining (simplified calculation)
    if (!current_battery_status.is_charging) {
        // Assume 3000mAh battery, ~150mA average consumption
        uint32_t remaining_capacity = (current_battery_status.percentage * 3000) / 100;
        current_battery_status.time_remaining = remaining_capacity / 150; // Hours
        current_battery_status.time_remaining *= 60; // Convert to minutes
    } else {
        current_battery_status.time_remaining = 0; // Charging
    }
    
    *status = current_battery_status;
    return ESP_OK;
}

power_state_t power_manager_get_state(void)
{
    return current_power_state;
}

esp_err_t power_manager_set_power_mode(power_state_t mode)
{
    ESP_LOGI(TAG, "Setting power mode: %d", mode);
    current_power_state = mode;
    
    switch (mode) {
        case POWER_STATE_ACTIVE:
            // Full performance mode
            break;
            
        case POWER_STATE_SAVING:
            // Reduce CPU frequency, scan intervals
            break;
            
        case POWER_STATE_STANDBY:
            // Minimal operation mode
            break;
            
        case POWER_STATE_CHARGING:
            // Charging mode optimization
            break;
    }
    
    return ESP_OK;
}

bool power_manager_is_low_battery(void)
{
    return (current_battery_status.percentage <= LOW_BATTERY_THRESHOLD);
} 
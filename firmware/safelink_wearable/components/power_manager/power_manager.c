/*
 * Power Manager Implementation
 */

#include "power_manager.h"
#include "esp_log.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define TAG "POWER_MGR"

#define BATTERY_ADC_CHANNEL     ADC1_CHANNEL_0
#define CHARGING_GPIO           GPIO_NUM_5
#define BATTERY_CHECK_PERIOD_MS 30000  // 30 seconds

// Battery voltage to percentage mapping (Li-Po 3.7V nominal)
#define BATTERY_MAX_VOLTAGE     4.2f   // Fully charged
#define BATTERY_MIN_VOLTAGE     3.0f   // Empty
#define BATTERY_NOMINAL_VOLTAGE 3.7f   // Nominal

static power_state_t current_state = POWER_STATE_ACTIVE;
static battery_status_t battery_status = {0};
static power_config_t power_config = {
    .sleep_timeout_ms = 60000,           // 1 minute
    .low_battery_threshold = 20.0f,      // 20%
    .critical_battery_threshold = 5.0f   // 5%
};

static TimerHandle_t battery_timer = NULL;
static TaskHandle_t power_task_handle = NULL;

/* Private function declarations */
static void battery_timer_callback(TimerHandle_t timer);
static void power_task(void *param);
static float read_battery_voltage(void);
static float voltage_to_percentage(float voltage);
static void check_battery_status(void);
static esp_err_t configure_wakeup_sources(void);

esp_err_t power_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing power manager...");
    
    // Initialize ADC for battery monitoring
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN_DB_11);
    
    // Initialize charging detection GPIO
    gpio_config_t gpio_conf = {
        .pin_bit_mask = (1ULL << CHARGING_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpio_conf);
    
    // Configure power management
    esp_pm_config_esp32c6_t pm_config = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 10,
        .light_sleep_enable = true
    };
    esp_err_t ret = esp_pm_configure(&pm_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power management config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Create battery monitoring timer
    battery_timer = xTimerCreate("battery_timer",
                                pdMS_TO_TICKS(BATTERY_CHECK_PERIOD_MS),
                                pdTRUE, // Auto-reload
                                NULL,
                                battery_timer_callback);
    
    if (battery_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create battery timer");
        return ESP_FAIL;
    }
    
    // Create power management task
    BaseType_t task_ret = xTaskCreate(power_task, "power_task", 2048, NULL, 3, &power_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create power task");
        return ESP_FAIL;
    }
    
    // Start battery monitoring
    if (xTimerStart(battery_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start battery timer");
        return ESP_FAIL;
    }
    
    // Configure wakeup sources
    configure_wakeup_sources();
    
    // Initial battery status check
    check_battery_status();
    
    ESP_LOGI(TAG, "Power manager initialized successfully");
    return ESP_OK;
}

esp_err_t power_manager_set_config(const power_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    power_config = *config;
    ESP_LOGI(TAG, "Power configuration updated");
    return ESP_OK;
}

esp_err_t power_manager_get_battery_status(battery_status_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *status = battery_status;
    return ESP_OK;
}

esp_err_t power_manager_enter_light_sleep(uint32_t duration_ms)
{
    ESP_LOGI(TAG, "Entering light sleep for %lu ms", duration_ms);
    
    current_state = POWER_STATE_LIGHT_SLEEP;
    
    if (duration_ms > 0) {
        esp_sleep_enable_timer_wakeup(duration_ms * 1000); // Convert to microseconds
    }
    
    esp_err_t ret = esp_light_sleep_start();
    
    current_state = POWER_STATE_ACTIVE;
    ESP_LOGI(TAG, "Woke up from light sleep");
    
    return ret;
}

esp_err_t power_manager_enter_deep_sleep(uint32_t duration_ms)
{
    ESP_LOGI(TAG, "Entering deep sleep for %lu ms", duration_ms);
    
    current_state = POWER_STATE_DEEP_SLEEP;
    
    if (duration_ms > 0) {
        esp_sleep_enable_timer_wakeup(duration_ms * 1000); // Convert to microseconds
    }
    
    esp_deep_sleep_start();
    
    // This line should never be reached
    return ESP_OK;
}

esp_err_t power_manager_wake_up(void)
{
    if (current_state == POWER_STATE_LIGHT_SLEEP) {
        current_state = POWER_STATE_ACTIVE;
        ESP_LOGI(TAG, "System woke up");
        return ESP_OK;
    }
    
    return ESP_ERR_INVALID_STATE;
}

power_state_t power_manager_get_state(void)
{
    return current_state;
}

bool power_manager_is_battery_critical(void)
{
    return battery_status.percentage <= power_config.critical_battery_threshold;
}

/* Private function implementations */
static void battery_timer_callback(TimerHandle_t timer)
{
    check_battery_status();
}

static void power_task(void *param)
{
    TickType_t last_activity = xTaskGetTickCount();
    
    while (1) {
        // Check for system activity timeout
        TickType_t current_time = xTaskGetTickCount();
        TickType_t inactive_time = (current_time - last_activity) * portTICK_PERIOD_MS;
        
        if (inactive_time >= power_config.sleep_timeout_ms && 
            current_state == POWER_STATE_ACTIVE && 
            !battery_status.is_charging) {
            
            ESP_LOGI(TAG, "System inactive for %lu ms, entering light sleep", inactive_time);
            power_manager_enter_light_sleep(1000); // 1 second sleep cycles
            last_activity = xTaskGetTickCount();
        }
        
        // Check for critical battery
        if (power_manager_is_battery_critical() && !battery_status.is_charging) {
            ESP_LOGW(TAG, "Critical battery level, entering deep sleep");
            power_manager_enter_deep_sleep(0); // Sleep until external wakeup
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Check every second
    }
}

static float read_battery_voltage(void)
{
    // Read ADC value
    int adc_reading = adc1_get_raw(BATTERY_ADC_CHANNEL);
    
    // Convert ADC reading to voltage
    // ESP32-C6 ADC reference voltage is 3.3V, 12-bit resolution
    float voltage = (float)adc_reading * 3.3f / 4095.0f;
    
    // Apply voltage divider correction if needed
    // Assuming 2:1 voltage divider (R1=R2)
    voltage *= 2.0f;
    
    return voltage;
}

static float voltage_to_percentage(float voltage)
{
    if (voltage >= BATTERY_MAX_VOLTAGE) {
        return 100.0f;
    } else if (voltage <= BATTERY_MIN_VOLTAGE) {
        return 0.0f;
    }
    
    // Linear interpolation between min and max voltage
    float percentage = ((voltage - BATTERY_MIN_VOLTAGE) / 
                       (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE)) * 100.0f;
    
    return percentage;
}

static void check_battery_status(void)
{
    // Read battery voltage
    battery_status.voltage = read_battery_voltage();
    
    // Calculate percentage
    battery_status.percentage = voltage_to_percentage(battery_status.voltage);
    
    // Check charging status
    battery_status.is_charging = (gpio_get_level(CHARGING_GPIO) == 0); // Active low
    
    // Check low battery warning
    battery_status.is_low_battery = (battery_status.percentage <= power_config.low_battery_threshold);
    
    // Update power state if charging
    if (battery_status.is_charging && current_state != POWER_STATE_CHARGING) {
        current_state = POWER_STATE_CHARGING;
        ESP_LOGI(TAG, "Charging detected");
    } else if (!battery_status.is_charging && current_state == POWER_STATE_CHARGING) {
        current_state = POWER_STATE_ACTIVE;
        ESP_LOGI(TAG, "Charging stopped");
    }
    
    ESP_LOGD(TAG, "Battery: %.2fV (%.1f%%) %s", 
             battery_status.voltage, 
             battery_status.percentage,
             battery_status.is_charging ? "Charging" : "");
    
    // Log battery warnings
    if (battery_status.is_low_battery && !battery_status.is_charging) {
        ESP_LOGW(TAG, "Low battery warning: %.1f%%", battery_status.percentage);
    }
}

static esp_err_t configure_wakeup_sources(void)
{
    // Configure timer wakeup
    esp_sleep_enable_timer_wakeup(1000000); // 1 second default
    
    // Configure GPIO wakeup (charging detection)
    esp_sleep_enable_ext1_wakeup((1ULL << CHARGING_GPIO), ESP_EXT1_WAKEUP_ANY_HIGH);
    
    // Configure ULP wakeup for sensor interrupts if needed
    // esp_sleep_enable_ulp_wakeup();
    
    ESP_LOGI(TAG, "Wakeup sources configured");
    return ESP_OK;
} 
/*
 * Alert Manager Implementation
 */

#include "alert_manager.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define TAG "ALERT_MGR"

/* GPIO pins */
#define LED_RED_PIN     GPIO_NUM_8
#define LED_GREEN_PIN   GPIO_NUM_9
#define LED_BLUE_PIN    GPIO_NUM_10
#define VIBRATION_PIN   GPIO_NUM_11

/* PWM configuration */
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE

static TimerHandle_t led_timer = NULL;
static TimerHandle_t vibration_timer = NULL;
static bool is_initialized = false;

/* Private function declarations */
static void led_timer_callback(TimerHandle_t timer);
static void vibration_timer_callback(TimerHandle_t timer);
static esp_err_t init_gpio(void);
static esp_err_t init_pwm(void);

esp_err_t alert_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing alert manager...");
    
    esp_err_t ret = init_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO init failed");
        return ret;
    }
    
    ret = init_pwm();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM init failed");
        return ret;
    }
    
    // Create timers
    led_timer = xTimerCreate("led_timer", pdMS_TO_TICKS(500), pdTRUE, NULL, led_timer_callback);
    vibration_timer = xTimerCreate("vib_timer", pdMS_TO_TICKS(100), pdFALSE, NULL, vibration_timer_callback);
    
    if (led_timer == NULL || vibration_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create timers");
        return ESP_FAIL;
    }
    
    is_initialized = true;
    ESP_LOGI(TAG, "Alert manager initialized");
    return ESP_OK;
}

esp_err_t alert_manager_start(void)
{
    ESP_LOGI(TAG, "Alert manager started");
    return ESP_OK;
}

esp_err_t alert_manager_trigger_alert(alert_level_t level, risk_type_t risk, const char *message)
{
    if (!is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGW(TAG, "Alert triggered: Level=%d, Risk=%d, Message=%s", level, risk, message);
    
    switch (level) {
        case ALERT_LEVEL_WARNING:
            alert_manager_set_led(LED_COLOR_YELLOW, true);
            alert_manager_set_vibration(50, 200);
            break;
            
        case ALERT_LEVEL_CRITICAL:
            alert_manager_set_led(LED_COLOR_RED, true);
            alert_manager_set_vibration(100, 1000);
            break;
            
        default:
            alert_manager_set_led(LED_COLOR_GREEN, false);
            break;
    }
    
    return ESP_OK;
}

esp_err_t alert_manager_trigger_noise_alert(noise_level_t level, float db_value)
{
    switch (level) {
        case NOISE_LEVEL_WARNING:
            alert_manager_set_led(LED_COLOR_YELLOW, false);
            break;
        case NOISE_LEVEL_CRITICAL:
        case NOISE_LEVEL_DANGEROUS:
            alert_manager_set_led(LED_COLOR_RED, true);
            alert_manager_set_vibration(80, 500);
            break;
        default:
            break;
    }
    
    return ESP_OK;
}

esp_err_t alert_manager_set_led(led_color_t color, bool blink)
{
    // Turn off all LEDs first
    gpio_set_level(LED_RED_PIN, 0);
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_BLUE_PIN, 0);
    
    // Stop blinking timer
    if (led_timer != NULL) {
        xTimerStop(led_timer, 0);
    }
    
    // Set LED color
    switch (color) {
        case LED_COLOR_GREEN:
            gpio_set_level(LED_GREEN_PIN, 1);
            break;
        case LED_COLOR_YELLOW:
            gpio_set_level(LED_RED_PIN, 1);
            gpio_set_level(LED_GREEN_PIN, 1);
            break;
        case LED_COLOR_RED:
            gpio_set_level(LED_RED_PIN, 1);
            break;
        case LED_COLOR_BLUE:
            gpio_set_level(LED_BLUE_PIN, 1);
            break;
        default:
            break;
    }
    
    // Start blinking if requested
    if (blink && color != LED_COLOR_OFF && led_timer != NULL) {
        xTimerStart(led_timer, 0);
    }
    
    return ESP_OK;
}

esp_err_t alert_manager_set_vibration(uint8_t intensity, uint16_t duration_ms)
{
    if (intensity > 100) {
        intensity = 100;
    }
    
    // Set PWM duty cycle for vibration intensity
    uint32_t duty = (intensity * 1023) / 100;
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
    
    // Start vibration timer
    if (vibration_timer != NULL) {
        xTimerChangePeriod(vibration_timer, pdMS_TO_TICKS(duration_ms), 0);
        xTimerStart(vibration_timer, 0);
    }
    
    return ESP_OK;
}

esp_err_t alert_manager_clear_all_alerts(void)
{
    alert_manager_set_led(LED_COLOR_OFF, false);
    alert_manager_set_vibration(0, 0);
    return ESP_OK;
}

/* Private function implementations */
static void led_timer_callback(TimerHandle_t timer)
{
    static bool led_state = false;
    led_state = !led_state;
    
    if (led_state) {
        gpio_set_level(LED_RED_PIN, 0);
        gpio_set_level(LED_GREEN_PIN, 0);
        gpio_set_level(LED_BLUE_PIN, 0);
    }
}

static void vibration_timer_callback(TimerHandle_t timer)
{
    // Stop vibration
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}

static esp_err_t init_gpio(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_RED_PIN) | (1ULL << LED_GREEN_PIN) | (1ULL << LED_BLUE_PIN),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    
    return gpio_config(&io_conf);
}

static esp_err_t init_pwm(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER
    };
    
    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = VIBRATION_PIN,
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER
    };
    
    return ledc_channel_config(&ledc_channel);
} 
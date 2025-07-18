/*
 * Safety Monitor Implementation
 */

#include "safety_monitor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include <string.h>

#define TAG "SAFETY_MON"

#define SAFETY_CHECK_PERIOD_MS      1000    // 1 second
#define ALERT_QUEUE_SIZE            10
#define WARNING_ALERT_INTERVAL_MS   60000   // 1 minute for warnings
#define CRITICAL_ALERT_INTERVAL_MS  1000    // 1 second for critical

static safety_status_t current_status = SAFETY_STATUS_NORMAL;
static TaskHandle_t safety_task_handle = NULL;
static QueueHandle_t alert_queue = NULL;
static TimerHandle_t safety_timer = NULL;

/* Alert state tracking */
static struct {
    bool body_temp_warning_active;
    bool body_temp_critical_active;
    bool heart_rate_warning_active;
    bool heart_rate_critical_active;
    bool wbgt_warning_active;
    bool wbgt_critical_active;
    
    uint64_t last_body_temp_alert;
    uint64_t last_heart_rate_alert;
    uint64_t last_wbgt_alert;
} alert_state = {0};

/* Private function declarations */
static void safety_task(void *param);
static void safety_timer_callback(TimerHandle_t timer);
static void check_body_temperature(float temp, uint64_t timestamp);
static void check_heart_rate(float hr, uint64_t timestamp);
static void check_wbgt(float wbgt, uint64_t timestamp);
static void send_alert(alert_type_t type, safety_status_t status, float value, const char *message);
static bool should_send_alert(alert_type_t type, safety_status_t status, uint64_t timestamp);

esp_err_t safety_monitor_init(void)
{
    ESP_LOGI(TAG, "Initializing safety monitor...");
    
    // Create alert queue
    alert_queue = xQueueCreate(ALERT_QUEUE_SIZE, sizeof(safety_event_t));
    if (alert_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create alert queue");
        return ESP_FAIL;
    }
    
    // Create safety monitoring task
    BaseType_t ret = xTaskCreate(safety_task, "safety_task", 4096, NULL, 5, &safety_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create safety task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Safety monitor initialized successfully");
    return ESP_OK;
}

esp_err_t safety_monitor_start(void)
{
    ESP_LOGI(TAG, "Starting safety monitor...");
    
    // Reset alert state
    memset(&alert_state, 0, sizeof(alert_state));
    current_status = SAFETY_STATUS_NORMAL;
    
    ESP_LOGI(TAG, "Safety monitor started");
    return ESP_OK;
}

esp_err_t safety_monitor_stop(void)
{
    ESP_LOGI(TAG, "Stopping safety monitor...");
    
    current_status = SAFETY_STATUS_NORMAL;
    
    ESP_LOGI(TAG, "Safety monitor stopped");
    return ESP_OK;
}

esp_err_t safety_monitor_check(const sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Check each safety parameter
    check_body_temperature(data->skin_temperature, data->timestamp);
    check_heart_rate(data->heart_rate, data->timestamp);
    check_wbgt(data->wbgt, data->timestamp);
    
    // Update overall safety status
    safety_status_t new_status = SAFETY_STATUS_NORMAL;
    
    if (alert_state.body_temp_critical_active || 
        alert_state.heart_rate_critical_active || 
        alert_state.wbgt_critical_active) {
        new_status = SAFETY_STATUS_CRITICAL;
    } else if (alert_state.body_temp_warning_active || 
               alert_state.heart_rate_warning_active || 
               alert_state.wbgt_warning_active) {
        new_status = SAFETY_STATUS_WARNING;
    }
    
    if (new_status != current_status) {
        current_status = new_status;
        ESP_LOGI(TAG, "Safety status changed to: %d", current_status);
    }
    
    return ESP_OK;
}

safety_status_t safety_monitor_get_status(void)
{
    return current_status;
}

/* Private function implementations */
static void safety_task(void *param)
{
    safety_event_t event;
    
    while (1) {
        // Wait for safety events
        if (xQueueReceive(alert_queue, &event, portMAX_DELAY) == pdTRUE) {
            // Send alert via BLE
            alert_level_t ble_level;
            switch (event.status) {
                case SAFETY_STATUS_WARNING:
                    ble_level = ALERT_WARNING;
                    break;
                case SAFETY_STATUS_CRITICAL:
                    ble_level = ALERT_CRITICAL;
                    break;
                default:
                    ble_level = ALERT_NONE;
                    break;
            }
            
            if (ble_level != ALERT_NONE) {
                ble_service_send_alert(ble_level, event.message);
                ESP_LOGW(TAG, "Safety Alert: %s (Value: %.1f)", event.message, event.value);
            }
        }
    }
}

static void check_body_temperature(float temp, uint64_t timestamp)
{
    bool was_warning = alert_state.body_temp_warning_active;
    bool was_critical = alert_state.body_temp_critical_active;
    
    // Check critical level
    if (temp >= BODY_TEMP_CRITICAL) {
        if (!alert_state.body_temp_critical_active) {
            alert_state.body_temp_critical_active = true;
            send_alert(ALERT_TYPE_BODY_TEMP, SAFETY_STATUS_CRITICAL, temp, "체온 위험 - 즉시 작업 중단 필요");
        } else if (should_send_alert(ALERT_TYPE_BODY_TEMP, SAFETY_STATUS_CRITICAL, timestamp)) {
            send_alert(ALERT_TYPE_BODY_TEMP, SAFETY_STATUS_CRITICAL, temp, "체온 위험 지속");
        }
        alert_state.body_temp_warning_active = false; // Critical overrides warning
    }
    // Check warning level
    else if (temp >= BODY_TEMP_WARNING_MIN && temp < BODY_TEMP_WARNING_MAX) {
        if (!alert_state.body_temp_warning_active && !was_critical) {
            alert_state.body_temp_warning_active = true;
            send_alert(ALERT_TYPE_BODY_TEMP, SAFETY_STATUS_WARNING, temp, "체온 주의 - 휴식 필요");
        } else if (should_send_alert(ALERT_TYPE_BODY_TEMP, SAFETY_STATUS_WARNING, timestamp)) {
            send_alert(ALERT_TYPE_BODY_TEMP, SAFETY_STATUS_WARNING, temp, "체온 주의 지속");
        }
        alert_state.body_temp_critical_active = false;
    }
    // Normal level
    else {
        alert_state.body_temp_warning_active = false;
        alert_state.body_temp_critical_active = false;
        
        if (was_warning || was_critical) {
            ESP_LOGI(TAG, "Body temperature returned to normal: %.1f°C", temp);
        }
    }
}

static void check_heart_rate(float hr, uint64_t timestamp)
{
    bool was_warning = alert_state.heart_rate_warning_active;
    bool was_critical = alert_state.heart_rate_critical_active;
    
    // Check critical level
    if (hr >= HEART_RATE_CRITICAL_MIN && hr <= HEART_RATE_CRITICAL_MAX) {
        if (!alert_state.heart_rate_critical_active) {
            alert_state.heart_rate_critical_active = true;
            send_alert(ALERT_TYPE_HEART_RATE, SAFETY_STATUS_CRITICAL, hr, "심박수 위험 - 즉각 휴식 필요");
        } else if (should_send_alert(ALERT_TYPE_HEART_RATE, SAFETY_STATUS_CRITICAL, timestamp)) {
            send_alert(ALERT_TYPE_HEART_RATE, SAFETY_STATUS_CRITICAL, hr, "심박수 위험 지속");
        }
        alert_state.heart_rate_warning_active = false;
    }
    // Check warning level
    else if (hr >= HEART_RATE_WARNING_MIN && hr < HEART_RATE_WARNING_MAX) {
        if (!alert_state.heart_rate_warning_active && !was_critical) {
            alert_state.heart_rate_warning_active = true;
            send_alert(ALERT_TYPE_HEART_RATE, SAFETY_STATUS_WARNING, hr, "심박수 주의 - 휴식 권장");
        } else if (should_send_alert(ALERT_TYPE_HEART_RATE, SAFETY_STATUS_WARNING, timestamp)) {
            send_alert(ALERT_TYPE_HEART_RATE, SAFETY_STATUS_WARNING, hr, "심박수 주의 지속");
        }
        alert_state.heart_rate_critical_active = false;
    }
    // Normal level
    else {
        alert_state.heart_rate_warning_active = false;
        alert_state.heart_rate_critical_active = false;
        
        if (was_warning || was_critical) {
            ESP_LOGI(TAG, "Heart rate returned to normal: %.1f BPM", hr);
        }
    }
}

static void check_wbgt(float wbgt, uint64_t timestamp)
{
    bool was_warning = alert_state.wbgt_warning_active;
    bool was_critical = alert_state.wbgt_critical_active;
    
    // Check critical level
    if (wbgt >= WBGT_CRITICAL) {
        if (!alert_state.wbgt_critical_active) {
            alert_state.wbgt_critical_active = true;
            send_alert(ALERT_TYPE_WBGT, SAFETY_STATUS_CRITICAL, wbgt, "WBGT 위험 - 30분 이상 휴식 필요");
        } else if (should_send_alert(ALERT_TYPE_WBGT, SAFETY_STATUS_CRITICAL, timestamp)) {
            send_alert(ALERT_TYPE_WBGT, SAFETY_STATUS_CRITICAL, wbgt, "WBGT 위험 지속");
        }
        alert_state.wbgt_warning_active = false;
    }
    // Check warning level
    else if (wbgt >= WBGT_WARNING_MIN && wbgt < WBGT_WARNING_MAX) {
        if (!alert_state.wbgt_warning_active && !was_critical) {
            alert_state.wbgt_warning_active = true;
            send_alert(ALERT_TYPE_WBGT, SAFETY_STATUS_WARNING, wbgt, "WBGT 주의 - 15분 휴식 권장");
        } else if (should_send_alert(ALERT_TYPE_WBGT, SAFETY_STATUS_WARNING, timestamp)) {
            send_alert(ALERT_TYPE_WBGT, SAFETY_STATUS_WARNING, wbgt, "WBGT 주의 지속");
        }
        alert_state.wbgt_critical_active = false;
    }
    // Normal level
    else {
        alert_state.wbgt_warning_active = false;
        alert_state.wbgt_critical_active = false;
        
        if (was_warning || was_critical) {
            ESP_LOGI(TAG, "WBGT returned to normal: %.1f°C", wbgt);
        }
    }
}

static void send_alert(alert_type_t type, safety_status_t status, float value, const char *message)
{
    safety_event_t event = {
        .type = type,
        .status = status,
        .value = value,
        .timestamp = esp_timer_get_time() / 1000
    };
    
    strncpy(event.message, message, sizeof(event.message) - 1);
    event.message[sizeof(event.message) - 1] = '\0';
    
    // Update last alert timestamp
    switch (type) {
        case ALERT_TYPE_BODY_TEMP:
            alert_state.last_body_temp_alert = event.timestamp;
            break;
        case ALERT_TYPE_HEART_RATE:
            alert_state.last_heart_rate_alert = event.timestamp;
            break;
        case ALERT_TYPE_WBGT:
            alert_state.last_wbgt_alert = event.timestamp;
            break;
        default:
            break;
    }
    
    // Send to alert queue
    if (xQueueSend(alert_queue, &event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Alert queue full, dropping alert");
    }
}

static bool should_send_alert(alert_type_t type, safety_status_t status, uint64_t timestamp)
{
    uint64_t last_alert = 0;
    uint64_t interval = (status == SAFETY_STATUS_CRITICAL) ? CRITICAL_ALERT_INTERVAL_MS : WARNING_ALERT_INTERVAL_MS;
    
    switch (type) {
        case ALERT_TYPE_BODY_TEMP:
            last_alert = alert_state.last_body_temp_alert;
            break;
        case ALERT_TYPE_HEART_RATE:
            last_alert = alert_state.last_heart_rate_alert;
            break;
        case ALERT_TYPE_WBGT:
            last_alert = alert_state.last_wbgt_alert;
            break;
        default:
            return false;
    }
    
    return (timestamp - last_alert) >= interval;
} 
/*
 * Alert Manager Header
 */

#pragma once

#include "esp_err.h"
#include "audio_manager.h"
#include "safety_analyzer.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Alert levels */
typedef enum {
    ALERT_LEVEL_NORMAL = 0,
    ALERT_LEVEL_WARNING = 1,
    ALERT_LEVEL_CRITICAL = 2
} alert_level_t;

/* LED colors */
typedef enum {
    LED_COLOR_OFF = 0,
    LED_COLOR_GREEN = 1,
    LED_COLOR_YELLOW = 2,
    LED_COLOR_RED = 3,
    LED_COLOR_BLUE = 4
} led_color_t;

/* Function declarations */
esp_err_t alert_manager_init(void);
esp_err_t alert_manager_start(void);
esp_err_t alert_manager_stop(void);
esp_err_t alert_manager_trigger_alert(alert_level_t level, risk_type_t risk, const char *message);
esp_err_t alert_manager_trigger_noise_alert(noise_level_t level, float db_value);
esp_err_t alert_manager_set_led(led_color_t color, bool blink);
esp_err_t alert_manager_set_vibration(uint8_t intensity, uint16_t duration_ms);
esp_err_t alert_manager_clear_all_alerts(void);

#ifdef __cplusplus
}
#endif 
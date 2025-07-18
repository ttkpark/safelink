/*
 * Audio Manager Header
 * 
 * 소음 측정 및 음성 경고 출력 관리
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Noise level classifications */
typedef enum {
    NOISE_LEVEL_NORMAL = 0,     // < 85 dB
    NOISE_LEVEL_ELEVATED = 1,   // 85-90 dB
    NOISE_LEVEL_WARNING = 2,    // 90-105 dB  
    NOISE_LEVEL_CRITICAL = 3,   // 105-120 dB
    NOISE_LEVEL_DANGEROUS = 4   // > 120 dB
} noise_level_t;

/* Noise measurement data */
typedef struct {
    float current_db;           // Current noise level in dB
    float average_db;           // Average over last minute
    float peak_db;              // Peak in last minute
    noise_level_t level;        // Current noise level classification
    uint32_t exposure_time_95db;  // Total exposure time above 95dB (seconds)
    uint32_t exposure_time_110db; // Total exposure time above 110dB (seconds)
    uint32_t impact_count_130db;  // Count of impacts above 130dB
    uint64_t timestamp;         // Measurement timestamp
} noise_data_t;

/* Audio alert types */
typedef enum {
    AUDIO_ALERT_BEEP_SHORT = 0,
    AUDIO_ALERT_BEEP_LONG = 1,
    AUDIO_ALERT_VOICE_WARNING = 2,
    AUDIO_ALERT_VOICE_CRITICAL = 3,
    AUDIO_ALERT_SIREN = 4
} audio_alert_type_t;

/* Voice message types */
typedef enum {
    VOICE_MSG_NOISE_WARNING = 0,
    VOICE_MSG_NOISE_CRITICAL = 1,
    VOICE_MSG_TEMP_WARNING = 2,
    VOICE_MSG_TEMP_CRITICAL = 3,
    VOICE_MSG_HEART_WARNING = 4,
    VOICE_MSG_HEART_CRITICAL = 5,
    VOICE_MSG_GENERAL_WARNING = 6,
    VOICE_MSG_SAFETY_CHECK = 7
} voice_message_t;

/* Noise alert callback function type */
typedef void (*noise_alert_callback_t)(noise_level_t level, float db_value);

/* Function declarations */
esp_err_t audio_manager_init(void);
esp_err_t audio_manager_start(void);
esp_err_t audio_manager_stop(void);
esp_err_t audio_manager_get_noise_data(noise_data_t *data);
noise_level_t audio_manager_get_current_noise_level(void);
esp_err_t audio_manager_set_noise_callback(noise_alert_callback_t callback);
esp_err_t audio_manager_play_alert(audio_alert_type_t type);
esp_err_t audio_manager_play_voice_message(voice_message_t message);
esp_err_t audio_manager_set_volume(uint8_t volume); // 0-100
esp_err_t audio_manager_enable_recording(bool enable);
esp_err_t audio_manager_calibrate_noise_sensor(void);

#ifdef __cplusplus
}
#endif 
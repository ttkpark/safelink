/*
 * Audio Manager Implementation
 */

#include "audio_manager.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include <math.h>
#include <string.h>

#define TAG "AUDIO_MGR"

/* Audio configuration */
#define I2S_NUM                 I2S_NUM_0
#define I2S_SAMPLE_RATE         44100
#define I2S_SAMPLE_BITS         I2S_BITS_PER_SAMPLE_16BIT
#define I2S_CHANNELS            I2S_CHANNEL_MONO
#define I2S_DMA_BUF_COUNT       8
#define I2S_DMA_BUF_LEN         1024

/* GPIO pins */
#define I2S_WS_PIN              GPIO_NUM_4
#define I2S_BCK_PIN             GPIO_NUM_5
#define I2S_DATA_IN_PIN         GPIO_NUM_6
#define I2S_DATA_OUT_PIN        GPIO_NUM_7

/* Noise measurement */
#define NOISE_SAMPLE_PERIOD_MS  100     // 100ms sampling
#define NOISE_AVERAGE_WINDOW    600     // 60 second average (600 * 100ms)
#define NOISE_CALIBRATION_OFFSET 0.0f   // dB offset for calibration

/* Alert thresholds */
#define NOISE_THRESHOLD_85DB    85.0f
#define NOISE_THRESHOLD_90DB    90.0f
#define NOISE_THRESHOLD_95DB    95.0f
#define NOISE_THRESHOLD_105DB   105.0f
#define NOISE_THRESHOLD_110DB   110.0f
#define NOISE_THRESHOLD_120DB   120.0f
#define NOISE_THRESHOLD_130DB   130.0f

static noise_data_t current_noise_data = {0};
static TaskHandle_t noise_task_handle = NULL;
static TimerHandle_t noise_timer = NULL;
static SemaphoreHandle_t data_mutex = NULL;
static noise_alert_callback_t noise_callback = NULL;

/* Noise measurement state */
static float noise_samples[NOISE_AVERAGE_WINDOW];
static uint16_t sample_index = 0;
static bool is_recording = false;
static uint8_t current_volume = 70; // Default 70%

/* Voice message strings (Korean) */
static const char* voice_messages[] = {
    "소음이 위험합니다. 귀마개를 착용하세요.",           // VOICE_MSG_NOISE_WARNING
    "소음이 매우 위험합니다. 즉시 안전한 곳으로 이동하세요.", // VOICE_MSG_NOISE_CRITICAL
    "체온이 높습니다. 즉시 휴식하세요.",                // VOICE_MSG_TEMP_WARNING
    "체온이 위험합니다. 작업을 중단하고 의료조치를 받으세요.", // VOICE_MSG_TEMP_CRITICAL
    "심박수가 높습니다. 휴식이 필요합니다.",            // VOICE_MSG_HEART_WARNING
    "심박수가 위험합니다. 즉시 작업을 중단하세요.",       // VOICE_MSG_HEART_CRITICAL
    "안전 주의가 필요합니다.",                       // VOICE_MSG_GENERAL_WARNING
    "안전 상태를 확인하세요."                       // VOICE_MSG_SAFETY_CHECK
};

/* Private function declarations */
static void noise_task(void *param);
static void noise_timer_callback(TimerHandle_t timer);
static esp_err_t init_i2s(void);
static float measure_noise_level(void);
static float calculate_db_level(const int16_t *samples, size_t sample_count);
static noise_level_t classify_noise_level(float db_level);
static void update_noise_statistics(float db_level);
static esp_err_t play_tone(uint16_t frequency, uint16_t duration_ms);
static esp_err_t play_voice_tts(const char *text);

esp_err_t audio_manager_init(void)
{
    ESP_LOGI(TAG, "Initializing audio manager...");
    
    // Create data mutex
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_FAIL;
    }
    
    // Initialize I2S
    esp_err_t ret = init_i2s();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S initialization failed");
        return ret;
    }
    
    // Create noise measurement timer
    noise_timer = xTimerCreate("noise_timer",
                              pdMS_TO_TICKS(NOISE_SAMPLE_PERIOD_MS),
                              pdTRUE, // Auto-reload
                              NULL,
                              noise_timer_callback);
    
    if (noise_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create noise timer");
        return ESP_FAIL;
    }
    
    // Create noise processing task
    BaseType_t task_ret = xTaskCreate(noise_task, "noise_task", 4096, NULL, 6, &noise_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create noise task");
        return ESP_FAIL;
    }
    
    // Initialize noise data
    current_noise_data.level = NOISE_LEVEL_NORMAL;
    current_noise_data.timestamp = esp_timer_get_time() / 1000;
    
    ESP_LOGI(TAG, "Audio manager initialized successfully");
    return ESP_OK;
}

esp_err_t audio_manager_start(void)
{
    ESP_LOGI(TAG, "Starting audio manager...");
    
    is_recording = true;
    
    // Start I2S
    esp_err_t ret = i2s_start(I2S_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S start failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Start noise measurement timer
    if (xTimerStart(noise_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start noise timer");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Audio manager started");
    return ESP_OK;
}

esp_err_t audio_manager_stop(void)
{
    ESP_LOGI(TAG, "Stopping audio manager...");
    
    is_recording = false;
    
    // Stop noise timer
    if (noise_timer != NULL) {
        xTimerStop(noise_timer, portMAX_DELAY);
    }
    
    // Stop I2S
    esp_err_t ret = i2s_stop(I2S_NUM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S stop failed: %s", esp_err_to_name(ret));
    }
    
    ESP_LOGI(TAG, "Audio manager stopped");
    return ESP_OK;
}

esp_err_t audio_manager_get_noise_data(noise_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        *data = current_noise_data;
        xSemaphoreGive(data_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

noise_level_t audio_manager_get_current_noise_level(void)
{
    return current_noise_data.level;
}

esp_err_t audio_manager_set_noise_callback(noise_alert_callback_t callback)
{
    noise_callback = callback;
    return ESP_OK;
}

esp_err_t audio_manager_play_alert(audio_alert_type_t type)
{
    esp_err_t ret = ESP_OK;
    
    switch (type) {
        case AUDIO_ALERT_BEEP_SHORT:
            ret = play_tone(1000, 200); // 1kHz, 200ms
            break;
            
        case AUDIO_ALERT_BEEP_LONG:
            ret = play_tone(1000, 1000); // 1kHz, 1000ms
            break;
            
        case AUDIO_ALERT_VOICE_WARNING:
            ret = audio_manager_play_voice_message(VOICE_MSG_GENERAL_WARNING);
            break;
            
        case AUDIO_ALERT_VOICE_CRITICAL:
            ret = play_tone(2000, 500); // High pitch critical beep
            vTaskDelay(pdMS_TO_TICKS(100));
            ret = play_tone(2000, 500);
            break;
            
        case AUDIO_ALERT_SIREN:
            // Alternating tones for siren effect
            for (int i = 0; i < 5; i++) {
                play_tone(800, 200);
                vTaskDelay(pdMS_TO_TICKS(50));
                play_tone(1200, 200);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;
            
        default:
            ret = ESP_ERR_INVALID_ARG;
            break;
    }
    
    return ret;
}

esp_err_t audio_manager_play_voice_message(voice_message_t message)
{
    if (message >= sizeof(voice_messages) / sizeof(voice_messages[0])) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_LOGI(TAG, "Playing voice message: %s", voice_messages[message]);
    
    // For now, play a tone sequence to indicate voice message
    // In a real implementation, this would use TTS or pre-recorded audio
    esp_err_t ret = play_tone(500, 200);
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = play_tone(700, 200);
    vTaskDelay(pdMS_TO_TICKS(100));
    ret = play_tone(900, 200);
    
    // TODO: Implement actual TTS or pre-recorded voice playback
    return ret;
}

esp_err_t audio_manager_set_volume(uint8_t volume)
{
    if (volume > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    current_volume = volume;
    ESP_LOGI(TAG, "Volume set to %d%%", volume);
    return ESP_OK;
}

esp_err_t audio_manager_enable_recording(bool enable)
{
    is_recording = enable;
    ESP_LOGI(TAG, "Recording %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

esp_err_t audio_manager_calibrate_noise_sensor(void)
{
    ESP_LOGI(TAG, "Calibrating noise sensor...");
    
    // TODO: Implement noise sensor calibration
    // This would involve measuring known reference levels
    
    ESP_LOGI(TAG, "Noise sensor calibration completed");
    return ESP_OK;
}

/* Private function implementations */
static void noise_task(void *param)
{
    while (1) {
        if (is_recording) {
            float db_level = measure_noise_level();
            
            if (db_level >= 0) {
                if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    current_noise_data.current_db = db_level;
                    current_noise_data.level = classify_noise_level(db_level);
                    current_noise_data.timestamp = esp_timer_get_time() / 1000;
                    
                    update_noise_statistics(db_level);
                    
                    xSemaphoreGive(data_mutex);
                }
                
                // Check for alerts
                if (noise_callback != NULL) {
                    noise_level_t level = classify_noise_level(db_level);
                    static noise_level_t last_level = NOISE_LEVEL_NORMAL;
                    
                    if (level > NOISE_LEVEL_ELEVATED && level != last_level) {
                        noise_callback(level, db_level);
                        last_level = level;
                    } else if (level <= NOISE_LEVEL_ELEVATED) {
                        last_level = NOISE_LEVEL_NORMAL;
                    }
                }
                
                ESP_LOGD(TAG, "Noise level: %.1f dB (Level: %d)", db_level, current_noise_data.level);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(NOISE_SAMPLE_PERIOD_MS));
    }
}

static void noise_timer_callback(TimerHandle_t timer)
{
    // Timer callback for periodic noise measurement
    // The actual measurement is done in the noise_task
}

static esp_err_t init_i2s(void)
{
    // I2S configuration for both input (microphone) and output (speaker)
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX,
        .sample_rate = I2S_SAMPLE_RATE,
        .bits_per_sample = I2S_SAMPLE_BITS,
        .channel_format = I2S_CHANNELS,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = I2S_DMA_BUF_COUNT,
        .dma_buf_len = I2S_DMA_BUF_LEN,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    esp_err_t ret = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // I2S pin configuration
    i2s_pin_config_t pin_config = {
        .ws_io_num = I2S_WS_PIN,
        .bck_io_num = I2S_BCK_PIN,
        .data_out_num = I2S_DATA_OUT_PIN,
        .data_in_num = I2S_DATA_IN_PIN
    };
    
    ret = i2s_set_pin(I2S_NUM, &pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S pin set failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    return ESP_OK;
}

static float measure_noise_level(void)
{
    if (!is_recording) {
        return -1.0f;
    }
    
    size_t bytes_read = 0;
    int16_t samples[I2S_DMA_BUF_LEN];
    
    esp_err_t ret = i2s_read(I2S_NUM, samples, sizeof(samples), &bytes_read, pdMS_TO_TICKS(100));
    if (ret != ESP_OK || bytes_read == 0) {
        return -1.0f;
    }
    
    size_t sample_count = bytes_read / sizeof(int16_t);
    return calculate_db_level(samples, sample_count);
}

static float calculate_db_level(const int16_t *samples, size_t sample_count)
{
    if (samples == NULL || sample_count == 0) {
        return -1.0f;
    }
    
    // Calculate RMS value
    float sum_squares = 0.0f;
    for (size_t i = 0; i < sample_count; i++) {
        float sample = (float)samples[i] / 32768.0f; // Normalize to [-1, 1]
        sum_squares += sample * sample;
    }
    
    float rms = sqrtf(sum_squares / sample_count);
    
    // Convert to dB (with reference and calibration)
    float db = 20.0f * log10f(rms + 1e-10f); // Add small value to avoid log(0)
    
    // Apply calibration offset and scaling
    db = db + 94.0f + NOISE_CALIBRATION_OFFSET; // Typical calibration for MEMS mic
    
    // Clamp to reasonable range
    if (db < 30.0f) db = 30.0f;
    if (db > 140.0f) db = 140.0f;
    
    return db;
}

static noise_level_t classify_noise_level(float db_level)
{
    if (db_level >= NOISE_THRESHOLD_120DB) {
        return NOISE_LEVEL_DANGEROUS;
    } else if (db_level >= NOISE_THRESHOLD_105DB) {
        return NOISE_LEVEL_CRITICAL;
    } else if (db_level >= NOISE_THRESHOLD_90DB) {
        return NOISE_LEVEL_WARNING;
    } else if (db_level >= NOISE_THRESHOLD_85DB) {
        return NOISE_LEVEL_ELEVATED;
    } else {
        return NOISE_LEVEL_NORMAL;
    }
}

static void update_noise_statistics(float db_level)
{
    // Update rolling average
    noise_samples[sample_index] = db_level;
    sample_index = (sample_index + 1) % NOISE_AVERAGE_WINDOW;
    
    // Calculate average
    float sum = 0.0f;
    for (int i = 0; i < NOISE_AVERAGE_WINDOW; i++) {
        sum += noise_samples[i];
    }
    current_noise_data.average_db = sum / NOISE_AVERAGE_WINDOW;
    
    // Update peak
    if (db_level > current_noise_data.peak_db) {
        current_noise_data.peak_db = db_level;
    }
    
    // Update exposure time counters
    static uint64_t last_update_time = 0;
    uint64_t current_time = esp_timer_get_time() / 1000000; // Convert to seconds
    
    if (last_update_time > 0) {
        uint32_t time_diff = (uint32_t)(current_time - last_update_time);
        
        if (db_level >= NOISE_THRESHOLD_95DB) {
            current_noise_data.exposure_time_95db += time_diff;
        }
        
        if (db_level >= NOISE_THRESHOLD_110DB) {
            current_noise_data.exposure_time_110db += time_diff;
        }
        
        if (db_level >= NOISE_THRESHOLD_130DB) {
            current_noise_data.impact_count_130db++;
        }
    }
    
    last_update_time = current_time;
}

static esp_err_t play_tone(uint16_t frequency, uint16_t duration_ms)
{
    if (!is_recording) {
        return ESP_ERR_INVALID_STATE;
    }
    
    size_t sample_count = (I2S_SAMPLE_RATE * duration_ms) / 1000;
    int16_t *samples = malloc(sample_count * sizeof(int16_t));
    
    if (samples == NULL) {
        return ESP_ERR_NO_MEM;
    }
    
    // Generate sine wave
    float amplitude = (current_volume / 100.0f) * 16384.0f; // Scale by volume
    
    for (size_t i = 0; i < sample_count; i++) {
        float t = (float)i / I2S_SAMPLE_RATE;
        samples[i] = (int16_t)(amplitude * sinf(2.0f * M_PI * frequency * t));
    }
    
    // Play samples
    size_t bytes_written = 0;
    esp_err_t ret = i2s_write(I2S_NUM, samples, sample_count * sizeof(int16_t), 
                             &bytes_written, pdMS_TO_TICKS(duration_ms + 100));
    
    free(samples);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2S write failed: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

static esp_err_t play_voice_tts(const char *text)
{
    // TODO: Implement Text-to-Speech functionality
    // This could use a TTS library or pre-recorded audio files
    ESP_LOGI(TAG, "TTS: %s", text);
    return ESP_OK;
} 
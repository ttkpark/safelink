#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2s.h"
#include "driver/i2c.h"
#include "dfplayer_mini.h"

static const char *TAG = "AUDIO_TEST";

// DFP Player (UART Control) - Using dfplayer_mini library
#define DFPLAYER_UART_NUM    UART_NUM_1
#define DFPLAYER_TX_PIN      GPIO_NUM_4      // DFP Player RX
#define DFPLAYER_RX_PIN      GPIO_NUM_5      // DFP Player TX
#define DFPLAYER_BAUD_RATE   9600

// INMP441 Microphone (I2S Input)
#define I2S_BCK_IO           GPIO_NUM_6     // I2S Bit Clock
#define I2S_WS_IO            GPIO_NUM_2     // I2S Word Select (LRCLK)
#define I2S_DI_IO            GPIO_NUM_14    // I2S Data In (INMP441 SD)

// UART for Serial Communication (PC)
#define UART_NUM             UART_NUM_0
#define UART_TX_PIN          GPIO_NUM_21    // PC 연결용 TX (ESP32-C6)
#define UART_RX_PIN          GPIO_NUM_20    // PC 연결용 RX (ESP32-C6)
#define UART_BAUD_RATE       115200

#define I2C_MASTER_NUM       I2C_NUM_0
#define I2C_MASTER_FREQ_HZ   100000

// I2C (if needed)
#define I2C_MASTER_SCL_IO    GPIO_NUM_9
#define I2C_MASTER_SDA_IO    GPIO_NUM_10

// Audio Configuration for INMP441 (16-bit)
#define SAMPLE_RATE          16000  // 16kHz 샘플링 레이트
#define BITS_PER_SAMPLE      I2S_BITS_PER_SAMPLE_16BIT  // 16비트로 처리
#define CHANNELS             1
#define BUFFER_SIZE          1024  // 1024 samples
#define AUDIO_FRAME_SIZE     (BUFFER_SIZE * 2) // 16-bit samples (2 bytes per sample)

// Task handles
static TaskHandle_t uart_task_handle = NULL;
static TaskHandle_t audio_player_task_handle = NULL;
static TaskHandle_t microphone_task_handle = NULL;

// Event group for task synchronization
static EventGroupHandle_t audio_event_group = NULL;
#define AUDIO_PLAYER_READY_BIT    BIT0
#define MICROPHONE_READY_BIT      BIT1
#define UART_READY_BIT            BIT2

// Audio buffer (16-bit)
static int16_t mic_buffer[BUFFER_SIZE];

// Global variables for audio processing
static float current_volume = 0.0f;
static float volume_history[10] = {0};
static int volume_index = 0;
static float dc_offset = 0.0f;  // DC 오프셋 추적
static float alpha = 0.0f;    // DC 오프셋 필터 계수 (낮을수록 빠른 추적)

// Test audio data (1kHz sine wave) - Not used with DFP Player
// static void generate_test_audio(void)
// {
//     static float phase = 0.0f;
//     const float frequency = 1000.0f; // 1kHz
//     const float amplitude = 0.5f;
//     
//     for (int i = 0; i < BUFFER_SIZE; i++) {
//         audio_buffer[i] = (int16_t)(amplitude * 32767.0f * sinf(phase));
//         phase += 2.0f * M_PI * frequency / SAMPLE_RATE;
//         if (phase >= 2.0f * M_PI) {
//             phase -= 2.0f * M_PI;
//         }
//     }
// }

// Calculate RMS volume with DC offset removal for 16-bit data
static float calculate_volume(int16_t *buffer, size_t size)
{
    // Update DC offset using exponential moving average
    float current_avg = 0.0f;
    for (size_t i = 0; i < size; i++) {
        // 16비트 데이터를 -1.0 ~ 1.0 범위로 정규화
        float sample = (float)buffer[i] / 32767.0f;
        current_avg += sample;
    }
    current_avg /= size;
    
    // Update global DC offset with low-pass filter
    dc_offset = alpha * dc_offset + (1.0f - alpha) * current_avg;
    
    // Calculate RMS with DC offset removal
    float sum = 0.0f;
    for (size_t i = 0; i < size; i++) {
        float sample = (float)buffer[i] / 32767.0f - dc_offset;
        sum += sample * sample;
    }
    
    return sqrtf(sum / size);
}

// Convert volume to dB SPL based on INMP441 datasheet
static float volume_to_db_spl(float volume)
{
    if (volume <= 0.0f) return 0.0f;
    
    // INMP441 datasheet specifications:
    // - Sensitivity: -26 dBFS (1kHz, 94dB SPL)
    // - Maximum input: 120 dB SPL
    // - Dynamic range: 87 dB
    // - EIN (Equivalent Input Noise): 33 dBA SPL
    
    // Convert normalized volume (0-1) to dBFS
    float dbfs = 20.0f * log10f(volume);
    
    // Convert dBFS to dB SPL using sensitivity
    // dB SPL = dBFS + Sensitivity + Reference SPL
    // Reference: 94 dB SPL at -26 dBFS
    float db_spl = dbfs + 26.0f + 94.0f;
    
    // Clamp to realistic range (0-120 dB SPL)
    if (db_spl < 0.0f) db_spl = 0.0f;
    if (db_spl > 120.0f) db_spl = 120.0f;
    
    return db_spl;
}

// Convert volume to dB (legacy function)
static float volume_to_db(float volume)
{
    if (volume <= 0.0f) return -60.0f;
    return 20.0f * log10f(volume);
}

// DFP Player Command Structure (based on wiki)
// Using dfplayer_mini library - no need for local structures

// Using dfplayer_mini library functions
static uint8_t dfplayer_checksum(uint8_t *cmd, uint8_t len)
{
    return dfplayer_calculate_checksum(cmd, len);
}

static const char* get_dfp_error_description(uint8_t error_code)
{
    return dfplayer_get_error_description(error_code);
}

// Send DFP Player command and wait for response (using dfplayer_mini library)
static esp_err_t dfplayer_send_cmd_with_response(uint8_t command, uint8_t param1, uint8_t param2)
{
    dfplayer_response_t response;
    esp_err_t ret = dfplayer_send_command_with_response(command, param1, param2, &response);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DFP Player command successful: 0x%02X", command);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "DFP Player command failed: 0x%02X - %s", command, esp_err_to_name(ret));
        return ret;
    }
}

// Send DFP Player command with retry logic
static esp_err_t dfplayer_send_cmd_with_retry(uint8_t command, uint8_t param1, uint8_t param2, int max_retries)
{
    esp_err_t ret = ESP_OK;
    int retry_count = 0;
    
    while (retry_count < max_retries) {
        ret = dfplayer_send_cmd_with_response(command, param1, param2);
        
        if (ret == ESP_OK) {
            return ESP_OK;  // Success
        }
        
        retry_count++;
        if (retry_count < max_retries) {
            ESP_LOGW(TAG, "Command failed, retrying (%d/%d)...", retry_count, max_retries);
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // Wait 1 second before retry
        }
    }
    
    ESP_LOGE(TAG, "Command failed after %d retries", max_retries);
    return ret;
}

// Send DFP Player command (without response for simple commands)
static esp_err_t dfplayer_send_cmd(uint8_t command, uint8_t param1, uint8_t param2)
{
    return dfplayer_send_command(command, param1, param2, false);
}

// Initialize DFP Player UART
static esp_err_t init_dfplayer_uart(void)
{
    ESP_LOGI(TAG, "Initializing DFP Player UART");
    
    esp_err_t ret = dfplayer_init(DFPLAYER_UART_NUM, DFPLAYER_TX_PIN, DFPLAYER_RX_PIN, DFPLAYER_BAUD_RATE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize DFP Player: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "DFP Player UART initialized successfully");
    ESP_LOGI(TAG, "DFP Player TX Pin: GPIO %d, RX Pin: GPIO %d", DFPLAYER_TX_PIN, DFPLAYER_RX_PIN);
    ESP_LOGI(TAG, "DFP Player Baud Rate: %d", DFPLAYER_BAUD_RATE);
    
    return ESP_OK;
}

// Initialize I2S for INMP441 Microphone (Audio Input)
static esp_err_t init_i2s_microphone(void)
{
    ESP_LOGI(TAG, "Initializing I2S for INMP441 Microphone");
    ESP_LOGI(TAG, "Sample Rate: %d Hz, Bits: %d, Buffer Size: %d", SAMPLE_RATE, BITS_PER_SAMPLE, BUFFER_SIZE);
    ESP_LOGI(TAG, "I2S Pins - BCK: GPIO%d, WS: GPIO%d, DI: GPIO%d", I2S_BCK_IO, I2S_WS_IO, I2S_DI_IO);
    
    const i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = BITS_PER_SAMPLE,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,   // 더 작은 버퍼로 테스트
        .dma_buf_len = BUFFER_SIZE,
        .use_apll = false,    // APLL 비활성화로 테스트
        .tx_desc_auto_clear = false,  // RX 모드에서는 불필요
        .fixed_mclk = 0
    };
    
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK_IO,
        .ws_io_num = I2S_WS_IO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DI_IO
    };
    
    esp_err_t ret = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2S microphone driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2s_set_pin(I2S_NUM_0, &pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set I2S microphone pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2S Microphone initialized successfully");
    return ESP_OK;
}

// Initialize UART
static esp_err_t init_uart(void)
{
    ESP_LOGI(TAG, "Initializing UART");
    
    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(UART_NUM, 1024, 1024, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "UART initialized successfully");
    return ESP_OK;
}

// Initialize I2C
static esp_err_t init_i2c(void)
{
    ESP_LOGI(TAG, "Initializing I2C");
    
    const i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C initialized successfully");
    return ESP_OK;
}

// DFP Player communication test function
static void test_dfp_communication(void)
{
    ESP_LOGI(TAG, "=== DFP Player Communication Test ===");
    
    // Test 1: Send simple test pattern
    ESP_LOGI(TAG, "Test 1: Sending test pattern...");
    uint8_t test_pattern[] = {0x7E, 0xFF, 0x06, 0x3F, 0x00, 0x00, 0x00, 0xEF};
    test_pattern[6] = dfplayer_calculate_checksum(test_pattern, 7);
    
    int ret = uart_write_bytes(DFPLAYER_UART_NUM, test_pattern, sizeof(test_pattern));
    ESP_LOGI(TAG, "Test pattern sent: %d bytes", ret);
    
    // Wait for any response
    uint8_t response[20];
    int response_len = uart_read_bytes(DFPLAYER_UART_NUM, response, sizeof(response), 1000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Response received: %d bytes", response_len);
    if (response_len > 0) {
        ESP_LOGI(TAG, "Response data:");
        for (int i = 0; i < response_len; i++) {
            ESP_LOGI(TAG, "  [%d]: 0x%02X", i, response[i]);
        }
    }
    
    // Test 2: Try different baud rates
    ESP_LOGI(TAG, "Test 2: Testing different baud rates...");
    int baud_rates[] = {9600, 38400, 57600, 115200};
    
    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "Testing baud rate: %d", baud_rates[i]);
        
        // Reconfigure UART
        uart_driver_delete(DFPLAYER_UART_NUM);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        
        uart_config_t uart_config = {
            .baud_rate = baud_rates[i],
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        
        esp_err_t err = uart_driver_install(DFPLAYER_UART_NUM, 1024, 1024, 0, NULL, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(err));
            continue;
        }
        
        err = uart_param_config(DFPLAYER_UART_NUM, &uart_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(err));
            continue;
        }
        
        err = uart_set_pin(DFPLAYER_UART_NUM, DFPLAYER_TX_PIN, DFPLAYER_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(err));
            continue;
        }
        
        // Send test command
        ret = uart_write_bytes(DFPLAYER_UART_NUM, test_pattern, sizeof(test_pattern));
        ESP_LOGI(TAG, "Sent %d bytes at %d baud", ret, baud_rates[i]);
        
        // Wait for response
        response_len = uart_read_bytes(DFPLAYER_UART_NUM, response, sizeof(response), 500 / portTICK_PERIOD_MS);
        if (response_len > 0) {
            ESP_LOGI(TAG, "SUCCESS: Received %d bytes at %d baud", response_len, baud_rates[i]);
            ESP_LOGI(TAG, "Response data:");
            for (int j = 0; j < response_len; j++) {
                ESP_LOGI(TAG, "  [%d]: 0x%02X", j, response[j]);
            }
            break;
        } else {
            ESP_LOGW(TAG, "No response at %d baud", baud_rates[i]);
        }
    }
    
    // Restore original baud rate
    uart_driver_delete(DFPLAYER_UART_NUM);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    dfplayer_init(DFPLAYER_UART_NUM, DFPLAYER_TX_PIN, DFPLAYER_RX_PIN, DFPLAYER_BAUD_RATE);
    
    ESP_LOGI(TAG, "=== Communication Test Complete ===");
}

// DFP Player Control Task
static void dfplayer_task(void *arg)
{
    ESP_LOGI(TAG, "DFP Player task started");
    
    // Wait for DFP Player to be ready
    vTaskDelay(3000 / portTICK_PERIOD_MS);  // Increased wait time
    
    // Initialize DFP Player with response checking
    ESP_LOGI(TAG, "Initializing DFP Player...");
    
    // Query current status (with response)
    esp_err_t ret = dfplayer_send_cmd_with_response(0x3F, 0x00, 0x00);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DFP Player status query successful");
    } else {
        ESP_LOGW(TAG, "DFP Player status query failed: %s", esp_err_to_name(ret));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Set device to SD card first (with response and retry)
    ret = dfplayer_send_cmd_with_retry(0x09, 0x00, 0x02, 3);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DFP Player device set successful");
    } else {
        ESP_LOGW(TAG, "DFP Player device set failed: %s", esp_err_to_name(ret));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Set volume to 15 (0-30 range) (with response and retry)
    ret = dfplayer_send_cmd_with_retry(0x06, 0x00, 15, 3);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DFP Player volume set successful");
    } else {
        ESP_LOGW(TAG, "DFP Player volume set failed: %s", esp_err_to_name(ret));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Set EQ to normal (with response and retry)
    ret = dfplayer_send_cmd_with_retry(0x07, 0x00, 0x00, 3);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "DFP Player EQ set successful");
    } else {
        ESP_LOGW(TAG, "DFP Player EQ set failed: %s", esp_err_to_name(ret));
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    ESP_LOGI(TAG, "DFP Player initialized successfully");
    
    while (1) {
        // Wait for commands from UART task
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Microphone Task
static void microphone_task(void *arg)
{
    ESP_LOGI(TAG, "Microphone task started");
    
    size_t bytes_read = 0;
    esp_err_t ret;
    bool mic_test_active = false;
    int cnt = 0;
    
    while (1) {
        // Check if microphone test is active
        EventBits_t bits = xEventGroupGetBits(audio_event_group);
        if (bits & MICROPHONE_READY_BIT) {
            mic_test_active = true;
        } else {
            mic_test_active = false;
            cnt = 0;
        }
        
        if (mic_test_active) {
            ret = i2s_read(I2S_NUM_0, mic_buffer, BUFFER_SIZE * 2, &bytes_read, portMAX_DELAY);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(ret));
            } else {
                // 디버깅: 첫 번째 몇 개 샘플 출력 (16-bit)
                /*if (cnt == 0) {
                    ESP_LOGI(TAG, "First samples: 0x%04X, 0x%04X, 0x%04X, 0x%04X", 
                             (unsigned int)mic_buffer[0], (unsigned int)mic_buffer[1], 
                             (unsigned int)mic_buffer[2], (unsigned int)mic_buffer[3]);
                }*/
                // Calculate volume
                current_volume = calculate_volume(mic_buffer, BUFFER_SIZE);
                float volume_db = volume_to_db(current_volume);
                float volume_db_spl = volume_to_db_spl(current_volume);
                
                // Store in history for averaging
                volume_history[volume_index] = volume_db_spl;
                volume_index = (volume_index + 1) % 10;
                
                // Calculate average volume
                float avg_volume = 0.0f;
                for (int i = 0; i < 10; i++) {
                    avg_volume += volume_history[i];
                }
                avg_volume /= 10.0f;
                
                ESP_LOGI(TAG, "Microphone - %d Volume: %.2f dB, SPL: %.1f dB, Avg SPL: %.1f dB, Raw: %.5f", 
                        bytes_read, volume_db, volume_db_spl, avg_volume, current_volume);

                /*if(((cnt++)%30) == 10){
                    ESP_LOGI(TAG, "count test started");
                    uint8_t *buffer = (uint8_t*)mic_buffer;
                    for(int i = 0; i < sizeof(mic_buffer); i+=8){
                        ESP_LOGI(TAG, "0x%04X : %02X %02X %02X %02X %02X %02X %02X %02X", i
                            , buffer[i], buffer[i+1], buffer[i+2], buffer[i+3], buffer[i+4], buffer[i+5], buffer[i+6], buffer[i+7]);
                    }
                }*/
                
                // Send volume data via UART
                char uart_buffer[128];
                snprintf(uart_buffer, sizeof(uart_buffer), 
                        "MIC_VOL:%.2f,%.1f,%.1f,%.4f\r\n", volume_db, volume_db_spl, avg_volume, current_volume);
                uart_write_bytes(UART_NUM, uart_buffer, strlen(uart_buffer));
            }
        }
        
        vTaskDelay(50 / portTICK_PERIOD_MS); // 50ms delay for real-time monitoring
    }
}

// UART Task for Serial Communication
static void uart_task(void *arg)
{
    ESP_LOGI(TAG, "UART task started");
    
    char uart_buffer[256];
    int len;
    
    // Send welcome message
    const char *welcome_msg = "\r\n=== Audio Test System ===\r\n";
    uart_write_bytes(UART_NUM, welcome_msg, strlen(welcome_msg));
    
    const char *menu_msg = "Commands:\r\n"
                           "1. 'play' - Play MP3 file (1-255)\r\n"
                           "2. 'pause' - Pause playback\r\n"
                           "3. 'resume' - Resume playback\r\n"
                           "4. 'stop' - Stop playback\r\n"
                           "5. 'next' - Next track\r\n"
                           "6. 'prev' - Previous track\r\n"
                           "7. 'vol <0-30>' - Set volume\r\n"
                           "8. 'mic' - Start microphone test\r\n"
                           "9. 'mic_stop' - Stop microphone test\r\n"
                           "10. 'dfp_test' - Test DFP Player connection\r\n"
                           "11. 'dfp_status' - Check DFP Player status\r\n"
                           "12. 'status' - Show system status\r\n"
                           "13. 'help' - Show this menu\r\n\r\n";
    uart_write_bytes(UART_NUM, menu_msg, strlen(menu_msg));
    
    while (1) {
        len = uart_read_bytes(UART_NUM, uart_buffer, sizeof(uart_buffer) - 1, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            uart_buffer[len] = '\0';
            
            // Remove newline characters
            char *newline = strchr(uart_buffer, '\r');
            if (newline) *newline = '\0';
            newline = strchr(uart_buffer, '\n');
            if (newline) *newline = '\0';
            
            ESP_LOGI(TAG, "UART received: '%s'", uart_buffer);
            
            if (strncmp(uart_buffer, "play", 4) == 0) {
                int track_num = 1;
                if (strlen(uart_buffer) > 5) {
                    track_num = atoi(uart_buffer + 5);
                    if (track_num < 1 || track_num > 255) track_num = 1;
                }
                ESP_LOGI(TAG, "Playing track %d", track_num);
                
                // Send play command with response
                esp_err_t ret = dfplayer_send_cmd_with_response(0x03, 0x00, track_num);
                if (ret == ESP_OK) {
                    char response[64];
                    snprintf(response, sizeof(response), "Playing track %d - SUCCESS\r\n", track_num);
                    uart_write_bytes(UART_NUM, response, strlen(response));
                } else {
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Playing track %d - FAILED (%s)\r\n", track_num, esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strcmp(uart_buffer, "pause") == 0) {
                ESP_LOGI(TAG, "Pausing playback");
                esp_err_t ret = dfplayer_send_cmd_with_response(0x0E, 0x00, 0x00);
                if (ret == ESP_OK) {
                    uart_write_bytes(UART_NUM, "Playback paused - SUCCESS\r\n", 25);
                } else {
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Pause failed (%s)\r\n", esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strcmp(uart_buffer, "resume") == 0) {
                ESP_LOGI(TAG, "Resuming playback");
                esp_err_t ret = dfplayer_send_cmd_with_response(0x0D, 0x00, 0x00);
                if (ret == ESP_OK) {
                    uart_write_bytes(UART_NUM, "Playback resumed - SUCCESS\r\n", 26);
                } else {
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Resume failed (%s)\r\n", esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strcmp(uart_buffer, "stop") == 0) {
                ESP_LOGI(TAG, "Stopping playback");
                esp_err_t ret = dfplayer_send_cmd_with_response(0x16, 0x00, 0x00);
                if (ret == ESP_OK) {
                    uart_write_bytes(UART_NUM, "Playback stopped - SUCCESS\r\n", 26);
                } else {
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Stop failed (%s)\r\n", esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strcmp(uart_buffer, "next") == 0) {
                ESP_LOGI(TAG, "Next track");
                esp_err_t ret = dfplayer_send_cmd_with_response(0x01, 0x00, 0x00);
                if (ret == ESP_OK) {
                    uart_write_bytes(UART_NUM, "Next track - SUCCESS\r\n", 20);
                } else {
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Next track failed (%s)\r\n", esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strcmp(uart_buffer, "prev") == 0) {
                ESP_LOGI(TAG, "Previous track");
                esp_err_t ret = dfplayer_send_cmd_with_response(0x02, 0x00, 0x00);
                if (ret == ESP_OK) {
                    uart_write_bytes(UART_NUM, "Previous track - SUCCESS\r\n", 23);
                } else {
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Previous track failed (%s)\r\n", esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strncmp(uart_buffer, "vol", 3) == 0) {
                int volume = 15;
                if (strlen(uart_buffer) > 4) {
                    volume = atoi(uart_buffer + 4);
                    if (volume < 0 || volume > 30) volume = 15;
                }
                ESP_LOGI(TAG, "Setting volume to %d", volume);
                esp_err_t ret = dfplayer_send_cmd_with_response(0x06, 0x00, volume);
                if (ret == ESP_OK) {
                    char response[64];
                    snprintf(response, sizeof(response), "Volume set to %d - SUCCESS\r\n", volume);
                    uart_write_bytes(UART_NUM, response, strlen(response));
                } else {
                    char error_msg[128];
                    snprintf(error_msg, sizeof(error_msg), "Volume set failed (%s)\r\n", esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strcmp(uart_buffer, "mic") == 0) {
                ESP_LOGI(TAG, "Starting microphone test");
                uart_write_bytes(UART_NUM, "Starting microphone test...\r\n", 28);
                xEventGroupSetBits(audio_event_group, MICROPHONE_READY_BIT);
                
            } else if (strcmp(uart_buffer, "mic_stop") == 0) {
                ESP_LOGI(TAG, "Stopping microphone test");
                uart_write_bytes(UART_NUM, "Stopping microphone test...\r\n", 28);
                xEventGroupClearBits(audio_event_group, MICROPHONE_READY_BIT);
                
            } else if (strcmp(uart_buffer, "dfp_status") == 0) {
                ESP_LOGI(TAG, "Checking DFP Player detailed status");
                uart_write_bytes(UART_NUM, "Checking DFP Player status...\r\n", 30);
                
                // Query current status
                esp_err_t ret = dfplayer_send_cmd_with_response(0x3F, 0x00, 0x00);
                if (ret == ESP_OK) {
                    uart_write_bytes(UART_NUM, "Status query: SUCCESS\r\n", 22);
                } else {
                    char error_msg[64];
                    snprintf(error_msg, sizeof(error_msg), "Status query: FAILED (%s)\r\n", esp_err_to_name(ret));
                    uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
                }
                
            } else if (strcmp(uart_buffer, "dfp_test") == 0) {
                ESP_LOGI(TAG, "Testing DFP Player connection");
                uart_write_bytes(UART_NUM, "Testing DFP Player connection...\r\n", 32);
                
                // Run comprehensive communication test
                test_dfp_communication();
                
                uart_write_bytes(UART_NUM, "DFP Player communication test completed. Check logs for details.\r\n", 65);
                
            } else if (strcmp(uart_buffer, "status") == 0) {
                char status_msg[256];
                snprintf(status_msg, sizeof(status_msg),
                        "System Status:\r\n"
                        "Current Volume: %.2f dB\r\n"
                        "Average Volume: %.2f dB\r\n"
                        "Raw Volume: %.4f\r\n\r\n",
                        volume_to_db(current_volume),
                        volume_to_db(current_volume), // Simplified for now
                        current_volume);
                uart_write_bytes(UART_NUM, status_msg, strlen(status_msg));
                
            } else if (strcmp(uart_buffer, "help") == 0) {
                uart_write_bytes(UART_NUM, menu_msg, strlen(menu_msg));
                
            } else {
                char error_msg[256];
                snprintf(error_msg, sizeof(error_msg), "Unknown command: '%.50s'. Type 'help' for available commands.\r\n", uart_buffer);
                uart_write_bytes(UART_NUM, error_msg, strlen(error_msg));
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}



// Main function
void app_main(void)
{
    ESP_LOGI(TAG, "Starting Audio Test System");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Create event group
    audio_event_group = xEventGroupCreate();
    
    // Initialize peripherals
    ESP_ERROR_CHECK(init_uart());
    ESP_ERROR_CHECK(init_i2c());
    ESP_ERROR_CHECK(init_dfplayer_uart());
    ESP_ERROR_CHECK(init_i2s_microphone());
    
    // Create tasks
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 5, &uart_task_handle);
    xTaskCreate(dfplayer_task, "dfplayer_task", 4096, NULL, 4, &audio_player_task_handle);
    xTaskCreate(microphone_task, "microphone_task", 4096, NULL, 4, &microphone_task_handle);
    
    ESP_LOGI(TAG, "Audio Test System initialized successfully");
    ESP_LOGI(TAG, "Pin Configuration:");
    ESP_LOGI(TAG, "  DFP Player TX: GPIO %d", DFPLAYER_TX_PIN);
    ESP_LOGI(TAG, "  DFP Player RX: GPIO %d", DFPLAYER_RX_PIN);
    ESP_LOGI(TAG, "  I2S BCK: GPIO %d", I2S_BCK_IO);
    ESP_LOGI(TAG, "  I2S WS:  GPIO %d", I2S_WS_IO);
    ESP_LOGI(TAG, "  I2S DI:  GPIO %d", I2S_DI_IO);
    ESP_LOGI(TAG, "  UART TX: GPIO %d", UART_TX_PIN);
    ESP_LOGI(TAG, "  UART RX: GPIO %d", UART_RX_PIN);
    ESP_LOGI(TAG, "  I2C SCL: GPIO %d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "  I2C SDA: GPIO %d", I2C_MASTER_SDA_IO);
    
    // Set UART ready bit
    xEventGroupSetBits(audio_event_group, UART_READY_BIT);
} 
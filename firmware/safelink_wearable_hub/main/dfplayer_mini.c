#include "dfplayer_mini.h"

static const char *TAG = "DFPLAYER";
static uart_port_t dfplayer_uart_num = UART_NUM_1;

// Calculate checksum for DFP Player command
uint16_t dfplayer_calculate_checksum(uint8_t *data, size_t length)
{
    uint16_t sum = 0;
    for (size_t i = 1; i < length-3; i++) {
        sum -= data[i];
    }
    return sum;
}

// Get error description
const char* dfplayer_get_error_description(uint8_t error_code)
{
    switch (error_code) {
        case DFPLAYER_ERROR_FILE_NOT_FOUND:
            return "File not found";
        case DFPLAYER_ERROR_ADVERTISE:
            return "Advertise";
        case DFPLAYER_ERROR_SERIAL_RECEIVING:
            return "Serial receiving error (frame not received completely)";
        case DFPLAYER_ERROR_CHECKSUM:
            return "Check sum not match";
        case DFPLAYER_ERROR_FILE_INDEX:
            return "File index out of bounds";
        case DFPLAYER_ERROR_FILE_NAME:
            return "File name mismatch";
        case DFPLAYER_ERROR_ADVERTISE_2:
        case DFPLAYER_ERROR_ADVERTISE_3:
        case DFPLAYER_ERROR_ADVERTISE_4:
        case DFPLAYER_ERROR_ADVERTISE_5:
            return "Advertise";
        default:
            return "Unknown error";
    }
}

// Initialize DFP Player
esp_err_t dfplayer_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate)
{
    dfplayer_uart_num = uart_num;
    
    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(uart_num, 1024, 1024, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_param_config(uart_num, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure UART: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = uart_set_pin(uart_num, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "DFP Player initialized successfully");
    return ESP_OK;
}

// Send command to DFP Player
esp_err_t dfplayer_send_command(uint8_t command, uint8_t param1, uint8_t param2, bool feedback)
{
    dfplayer_cmd_t cmd = {
        .start_byte = 0x7E,
        .version = 0xFF,
        .length = 0x06,
        .command = command,
        .feedback = feedback ? 0x01 : 0x00,
        .param1 = param1,
        .param2 = param2,
        .checksum1 = 0,
        .checksum2 = 0,
        .end_byte = 0xEF
    };
    
    uint16_t checksum = dfplayer_calculate_checksum((uint8_t*)&cmd, sizeof(cmd));
    cmd.checksum1 = (uint8_t)(checksum >> 8);
    cmd.checksum2 = (uint8_t)(checksum & 0xFF);
    
    int ret = uart_write_bytes(dfplayer_uart_num, (uint8_t*)&cmd, sizeof(cmd));
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGD(TAG, "Command sent: 0x%02X, Param1: 0x%02X, Param2: 0x%02X", command, param1, param2);
    return ESP_OK;
}

// Send command and wait for response
esp_err_t dfplayer_send_command_with_response(uint8_t command, uint8_t param1, uint8_t param2, dfplayer_response_t *response)
{
    esp_err_t ret = dfplayer_send_command(command, param1, param2, true);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Wait for response
    uint8_t response_data[10];
    int response_len = uart_read_bytes(dfplayer_uart_num, response_data, sizeof(response_data), 2000 / portTICK_PERIOD_MS);
    
    if (response_len <= 0) {
        ESP_LOGW(TAG, "No response received");
        return ESP_ERR_TIMEOUT;
    }
    
    if (response_len < 10) {
        ESP_LOGW(TAG, "Incomplete response: %d bytes", response_len);
        return ESP_ERR_INVALID_SIZE;
    }
    
    // Parse response
    if (response_data[0] != 0x7E || response_data[9] != 0xEF) {
        ESP_LOGW(TAG, "Invalid response format");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    if (response) {
        memcpy(response, response_data, sizeof(dfplayer_response_t));
    }
    
    // Check for error
    if (response_data[3] == DFPLAYER_RESPONSE_ERROR) {
        uint8_t error_code = response_data[6];
        const char* error_desc = dfplayer_get_error_description(error_code);
        ESP_LOGE(TAG, "DFP Player error: 0x%02X - %s", error_code, error_desc);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    ESP_LOGD(TAG, "Response received: 0x%02X", response_data[3]);
    return ESP_OK;
}

// Play specific track
esp_err_t dfplayer_play(uint8_t track)
{
    return dfplayer_send_command(DFPLAYER_CMD_PLAY, 0x00, track, false);
}

// Play track from specific folder
esp_err_t dfplayer_play_folder(uint8_t folder, uint8_t track)
{
    return dfplayer_send_command(DFPLAYER_CMD_FOLDER, folder, track, false);
}

// Pause playback
esp_err_t dfplayer_pause(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_PAUSE, 0x00, 0x00, false);
}

// Resume playback
esp_err_t dfplayer_resume(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_PLAYBACK, 0x00, 0x00, false);
}

// Stop playback
esp_err_t dfplayer_stop(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_STOP, 0x00, 0x00, false);
}

// Next track
esp_err_t dfplayer_next(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_NEXT, 0x00, 0x00, false);
}

// Previous track
esp_err_t dfplayer_prev(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_PREV, 0x00, 0x00, false);
}

// Set volume (0-30)
esp_err_t dfplayer_set_volume(uint8_t volume)
{
    if (volume > 30) volume = 30;
    return dfplayer_send_command(DFPLAYER_CMD_VOLUME, 0x00, volume, false);
}

// Volume up
esp_err_t dfplayer_volume_up(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_VOLUME_UP, 0x00, 0x00, false);
}

// Volume down
esp_err_t dfplayer_volume_down(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_VOLUME_DOWN, 0x00, 0x00, false);
}

// Set EQ
esp_err_t dfplayer_set_eq(uint8_t eq)
{
    if (eq > DFPLAYER_EQ_BASS) eq = DFPLAYER_EQ_NORMAL;
    return dfplayer_send_command(DFPLAYER_CMD_EQ, 0x00, eq, false);
}

// Set source
esp_err_t dfplayer_set_source(uint8_t source)
{
    if (source > DFPLAYER_SOURCE_FLASH) source = DFPLAYER_SOURCE_SD;
    return dfplayer_send_command(DFPLAYER_CMD_SOURCE, 0x00, source, false);
}

// Set playback mode
esp_err_t dfplayer_set_playback_mode(uint8_t mode)
{
    if (mode > DFPLAYER_MODE_RANDOM) mode = DFPLAYER_MODE_REPEAT;
    return dfplayer_send_command(DFPLAYER_CMD_PLAYBACK_MODE, 0x00, mode, false);
}

// Reset DFP Player
esp_err_t dfplayer_reset(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_RESET, 0x00, 0x00, false);
}

// Enter standby mode
esp_err_t dfplayer_standby(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_STANDBY, 0x00, 0x00, false);
}

// Exit standby mode
esp_err_t dfplayer_normal(void)
{
    return dfplayer_send_command(DFPLAYER_CMD_NORMAL, 0x00, 0x00, false);
}

// Get DFP Player status
esp_err_t dfplayer_get_status(dfplayer_status_t *status)
{
    if (!status) {
        return ESP_ERR_INVALID_ARG;
    }
    
    dfplayer_response_t response;
    esp_err_t ret = dfplayer_send_command_with_response(DFPLAYER_RESPONSE_STATUS, 0x00, 0x00, &response);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Parse status from response
    status->current_source = response.param2;
    
    // You can add more status parsing here based on your needs
    // This is a basic implementation
    
    return ESP_OK;
} 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "i2c.h"
#include "bluetooth.h"
#include "sensor.h"
#include "data_manager.h"
#include "mic_i2s.h"

static const char *TAG = "MAIN";

// VSLEEP deep sleep configuration
#define VSLEEP_PIN 3
#define VSLEEP_LOW_TIMEOUT_MS 1000  // 2초 동안 LOW 상태 유지되면 deep sleep 진입

// Task handles
static TaskHandle_t gpio_task_handle = NULL;
static TaskHandle_t bluetooth_task_handle = NULL;
static TaskHandle_t terminal_task_handle = NULL;

// Event group for task synchronization
static EventGroupHandle_t sensor_event_group = NULL;
#define BLUETOOTH_READY_BIT      BIT1

// VSLEEP 상태 모니터링 변수
static int64_t vsleep_low_start_time = 0;
static bool vsleep_low_detected = false;

// VSLEEP 초기화 함수
static esp_err_t vsleep_init(void)
{
    gpio_config_t io_conf = {};
    
    // VSLEEP을 입력으로 설정
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << VSLEEP_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;  // 내부 풀업 저항 활성화
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure VSLEEP: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "VSLEEP configured as input with pull-up");
    return ESP_OK;
}

// 모든 GPIO를 open state(고임피던스)로 설정하는 함수
static void set_all_gpio_open_state(void)
{
    ESP_LOGI(TAG, "Setting all GPIO pins to open state (high impedance) for sleep mode");
    
    // DFPlayer 전원 끄기 (Hi-Z 상태로 설정)
    dfplayer_power_off();
    
    // GPIO 설정 구조체 - 입력 모드로 설정하여 고임피던스 상태로 만듦
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 0,
        .pull_up_en = 0  // 풀업/풀다운 저항도 비활성화하여 완전한 고임피던스 상태
    };
    
    // I2C 핀들 (SDA, SCL) - 입력 모드로 설정하여 고임피던스 상태
    io_conf.pin_bit_mask = (1ULL << 8) | (1ULL << 9);
    gpio_config(&io_conf);
    
    // I2S 마이크 핀들 - 입력 모드로 설정하여 고임피던스 상태
    io_conf.pin_bit_mask = (1ULL << 4) | (1ULL << 5) | (1ULL << 6);
    gpio_config(&io_conf);
    
    // DFPlayer UART TX 핀 - 입력 모드로 설정하여 고임피던스 상태
    io_conf.pin_bit_mask = (1ULL << 2);
    gpio_config(&io_conf);
    
    // 기타 사용 가능한 GPIO 핀들도 입력 모드로 설정하여 고임피던스 상태 (안전을 위해)
    for (int i = 0; i < 22; i++) {
        if (i != VSLEEP_PIN) {  // VSLEEP 핀과 DFPlayer 전원 핀은 제외
            io_conf.pin_bit_mask = (1ULL << i);
            gpio_config(&io_conf);
        }
    }
    
    ESP_LOGI(TAG, "All GPIO pins set to open state (high impedance)");
}

// Deep sleep 진입 함수
static void enter_deep_sleep(void)
{
    ESP_LOGI(TAG, "Entering deep sleep mode - VSLEEP wake up enabled");
    
    // 모든 GPIO를 open state(고임피던스)로 설정
    set_all_gpio_open_state();
    
    // VSLEEP을 wake up source로 설정 (HIGH에서 wake up)
    // ESP32-C3에서는 ext0 wakeup 사용
    //gpio_wakeup_enable(VSLEEP_PIN, GPIO_INTR_HIGH_LEVEL ); // 1 = HIGH level
    esp_deep_sleep_enable_gpio_wakeup(1<<VSLEEP_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
    
    // Deep sleep 진입
    esp_deep_sleep_start();
}

// GPIO task
static void gpio_task(void *arg)
{
    ESP_LOGI(TAG, "GPIO task started");
    
    // VSLEEP 초기화
    esp_err_t ret = vsleep_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize VSLEEP");
        vTaskDelete(NULL);
        return;
    }
    
    while(1) {
        // VSLEEP 상태 읽기
        int vsleep_level = gpio_get_level(VSLEEP_PIN);
        int64_t current_time = esp_timer_get_time() / 1000; // ms 단위로 변환
        
        if (vsleep_level == 0) { // LOW 상태
            if (!vsleep_low_detected) {
                // LOW 상태 감지 시작
                vsleep_low_detected = true;
                vsleep_low_start_time = current_time;
                ESP_LOGI(TAG, "VSLEEP LOW detected - starting timer");
            } else {
                // LOW 상태 지속 시간 확인
                int64_t low_duration = current_time - vsleep_low_start_time;
                if (low_duration >= VSLEEP_LOW_TIMEOUT_MS) {
                    ESP_LOGI(TAG, "VSLEEP LOW for %lld ms - entering deep sleep", low_duration);
                    enter_deep_sleep();
                }
            }
        } else { // HIGH 상태
            if (vsleep_low_detected) {
                // HIGH로 복귀 - 타이머 리셋
                vsleep_low_detected = false;
                ESP_LOGI(TAG, "VSLEEP HIGH - timer reset");
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms마다 체크
    }
}



// Bluetooth task
static void bluetooth_task(void *arg)
{
    ESP_LOGI(TAG, "Bluetooth task started");
    
    // Bluetooth 초기화
    esp_err_t ret = bluetooth_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluetooth");
        vTaskDelete(NULL);
        return;
    }
    
    // Bluetooth 준비 완료 이벤트 설정
    xEventGroupSetBits(sensor_event_group, BLUETOOTH_READY_BIT);
    
    ESP_LOGI(TAG, "Bluetooth initialized successfully - Device: %s", BLE_DEVICE_NAME);
    
    while(1) {
        bluetooth_state_t state = bluetooth_get_state();
        
        switch(state) {
            case BLUETOOTH_STATE_ADVERTISING:
                // 광고 상태에서는 로그를 줄임 (10초마다)
                static uint32_t adv_log_counter = 0;
                if (++adv_log_counter >= 10) {
                    ESP_LOGI(TAG, "Bluetooth advertising - waiting for connection");
                    adv_log_counter = 0;
                }
                break;
                
            case BLUETOOTH_STATE_CONNECTED:
                // 연결된 상태에서 허브 데이터 특성 업데이트 (1초마다)
                esp_err_t update_ret = bluetooth_update_hub_data_characteristics();
                if (update_ret != ESP_OK) {
                    ESP_LOGW(TAG, "Failed to update hub data characteristics");
                }
                break;
                
            case BLUETOOTH_STATE_DISCONNECTED:
                ESP_LOGI(TAG, "Bluetooth disconnected - restarting advertising");
                break;
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

// Terminal output task
static void terminal_task(void *arg)
{
    ESP_LOGI(TAG, "Terminal task started");
    
    while(1) {
        // 데이터 매니저에서 모든 데이터를 터미널에 출력
        data_manager_print_all_data();
        
        vTaskDelay(5000 / portTICK_PERIOD_MS); // 5초마다 출력
    }
}



void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32C3 SafeLink Wearable Hub Application");
    ESP_LOGI(TAG, "BLE Device Name: %s", BLE_DEVICE_NAME);
    
    // Wake up 원인 확인
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    switch(wakeup_reason) {
        case ESP_SLEEP_WAKEUP_EXT0:
            ESP_LOGI(TAG, "Woke up from VSLEEP HIGH signal (ext0)");
            // Deep sleep에서 깨어났으므로 GPIO 복원이 필요하지 않음 (시스템 리셋됨)
            break;
        case ESP_SLEEP_WAKEUP_UNDEFINED:
            ESP_LOGI(TAG, "Normal boot (not from deep sleep)");
            break;
        default:
            ESP_LOGI(TAG, "Woke up from other source: %d", wakeup_reason);
            break;
    }
    
    esp_err_t ret;

    // NVS 초기화
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // I2C 초기화
    ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C: %s", esp_err_to_name(ret));
        return;
    }
    
    // 이벤트 그룹 생성
    sensor_event_group = xEventGroupCreate();
    if (sensor_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }
    
    // 센서 모듈 초기화
    ret = sensor_init();
    // I2S 마이크 초기화
    esp_err_t mic_ret = mic_i2s_init();
    if (mic_ret == ESP_OK) {
        mic_i2s_start();
        ESP_LOGI(TAG, "I2S microphone initialized");
    } else {
        ESP_LOGW(TAG, "I2S microphone init failed: %s", esp_err_to_name(mic_ret));
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor module");
        return;
    }
    
    // 데이터 매니저 초기화
    ret = data_manager_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize data manager");
        return;
    }
    
    // 센서 태스크들 생성
    ret = sensor_create_tasks(sensor_event_group);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create sensor tasks");
        return;
    }
    
    // GPIO 태스크 생성
    xTaskCreate(gpio_task, "gpio_task", 4096, NULL, 5, &gpio_task_handle);
    
    // Bluetooth 태스크 생성
    xTaskCreate(bluetooth_task, "bluetooth_task", 8192, NULL, 5, &bluetooth_task_handle);
    
    // Terminal 태스크 생성
    xTaskCreate(terminal_task, "terminal_task", 4096, NULL, 3, &terminal_task_handle);
    
    ESP_LOGI(TAG, "All tasks created successfully");
    ESP_LOGI(TAG, "Use nRF Connect app to scan and connect to '%s'", BLE_DEVICE_NAME);
    
    // 메인 태스크는 여기서 종료
    vTaskDelete(NULL);
}

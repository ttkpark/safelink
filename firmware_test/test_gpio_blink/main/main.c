/*
 * ESP32-C6 Safe GPIO Blink Test
 * 
 * 안전한 GPIO 핀만 사용하여 blink하는 테스트 프로그램
 * 부팅 시 사용되는 핀들과 특수 기능 핀들은 제외
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"

static const char *TAG = "SAFE_GPIO_BLINK_TEST";

/* ESP32-C6 안전한 GPIO 핀 정의 */
/* 
 * 제외된 핀들:
 * - GPIO 0: 부팅 모드 선택 (부팅 시 LOW=다운로드 모드)
 * - GPIO 1-3: UART0 (시리얼 통신)
 * - GPIO 6-11: SPI0 (플래시 메모리)
 * - GPIO 12: 부팅 시 사용 (부팅 시 HIGH=안전 모드)
 * - GPIO 16-17: UART1
 * - GPIO 18-21: SPI1
 * - GPIO 22-23: I2C0
 * - GPIO 24-25: I2C1
 * - GPIO 26-27: I2S0
 * - GPIO 28-29: I2S1
 * - GPIO 30: GPIO30 (일부 보드에서 제한적)
 */

/* 안전한 GPIO 핀들만 선별 */
#define GPIO_OUTPUT_IO_4    4
#define GPIO_OUTPUT_IO_5    5
#define GPIO_OUTPUT_IO_13   13
#define GPIO_OUTPUT_IO_14   14

/* 안전한 GPIO 핀 배열 */
static const int safe_gpio_pins[] = {
    0,1,2,3,4,5,6,7,8,9,/*10,11,*/12,13,14,15/*,16,17*/,18,19,20,21,22,23/*,24,25,26,27,28,29,30*/
};

#define SAFE_GPIO_PIN_COUNT (sizeof(safe_gpio_pins) / sizeof(safe_gpio_pins[0]))

/* Blink 모드 */
typedef enum {
    BLINK_MODE_ALL_TOGETHER = 0,    // 모든 GPIO 동시에 blink
    BLINK_MODE_SEQUENTIAL = 1,      // 순차적으로 blink
    BLINK_MODE_WAVE = 2,            // 파도처럼 blink
    BLINK_MODE_RANDOM = 3           // 랜덤 blink
} blink_mode_t;

/* 설정 가능한 매개변수 */
static blink_mode_t current_mode = BLINK_MODE_ALL_TOGETHER;
static uint32_t blink_delay_ms = 500;  // 기본 500ms
static bool test_running = true;

/* 함수 선언 */
static void gpio_init_all(void);
static void blink_all_together(void);
static void blink_sequential(void);
static void blink_wave(void);
static void blink_random(void);
static void gpio_test_task(void *arg);
static void print_usage(void);
static void handle_user_input(void);

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C6 Safe GPIO Blink Test Starting...");
    ESP_LOGI(TAG, "ESP-IDF Version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "Safe GPIO pins to test: %d", SAFE_GPIO_PIN_COUNT);
    
    /* 사용되는 안전한 GPIO 핀들 출력 */
    ESP_LOGI(TAG, "Safe GPIO pins: GPIO_%d, GPIO_%d, GPIO_%d, GPIO_%d", 
             safe_gpio_pins[0], safe_gpio_pins[1], safe_gpio_pins[2], 
             safe_gpio_pins[3]);
    
    /* GPIO 초기화 */
    gpio_init_all();
    
    /* 사용법 출력 */
    print_usage();
    
    /* GPIO 테스트 태스크 생성 */
    xTaskCreate(gpio_test_task, "gpio_test_task", 4096, NULL, 5, NULL);
    
    /* 사용자 입력 처리 */
    handle_user_input();
}

static void gpio_init_all(void)
{
    ESP_LOGI(TAG, "Initializing safe GPIO pins as outputs...");
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 0,
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    
    /* 안전한 GPIO 핀들의 비트마스크 설정 */
    for (int i = 0; i < SAFE_GPIO_PIN_COUNT; i++) {
        io_conf.pin_bit_mask |= (1ULL << safe_gpio_pins[i]);
    }
    
    esp_err_t ret = gpio_config(&io_conf);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Safe GPIO initialization completed successfully");
        vTaskDelay(1000);

        /* 모든 안전한 GPIO를 LOW로 초기화 */
        for (int i = 0; i < SAFE_GPIO_PIN_COUNT; i++) {
            gpio_set_level(safe_gpio_pins[i], 0);
        }
        
        ESP_LOGI(TAG, "All safe GPIO pins set to LOW (initial state)");
    } else {
        ESP_LOGE(TAG, "Safe GPIO initialization failed: %s", esp_err_to_name(ret));
    }
}

static void blink_all_together(void)
{
    static bool state = false;
    state = !state;
    
    ESP_LOGI(TAG, "All Safe GPIO: %s", state ? "HIGH" : "LOW");
    
    for (int i = 0; i < SAFE_GPIO_PIN_COUNT; i++) {
        gpio_set_level(safe_gpio_pins[i], state);
    }
}

static void blink_sequential(void)
{
    static int current_pin = 0;
    
    /* 이전 핀 끄기 */
    if (current_pin > 0) {
        gpio_set_level(safe_gpio_pins[current_pin - 1], 0);
    } else {
        gpio_set_level(safe_gpio_pins[SAFE_GPIO_PIN_COUNT - 1], 0);
    }
    
    /* 현재 핀 켜기 */
    gpio_set_level(safe_gpio_pins[current_pin], 1);
    
    ESP_LOGI(TAG, "Sequential: GPIO_%d HIGH", safe_gpio_pins[current_pin]);
    
    current_pin = (current_pin + 1) % SAFE_GPIO_PIN_COUNT;
}

static void blink_wave(void)
{
    static int wave_position = 0;
    static bool wave_direction = true;  // true: 증가, false: 감소
    
    /* 모든 안전한 GPIO 끄기 */
    for (int i = 0; i < SAFE_GPIO_PIN_COUNT; i++) {
        gpio_set_level(safe_gpio_pins[i], 0);
    }
    
    /* 파도 패턴 생성 (2개씩 켜기) */
    for (int i = 0; i < 2; i++) {
        int pin_index = wave_position + i;
        if (pin_index < SAFE_GPIO_PIN_COUNT) {
            gpio_set_level(safe_gpio_pins[pin_index], 1);
        }
    }
    
    ESP_LOGI(TAG, "Wave: Position %d, Direction %s", 
             wave_position, wave_direction ? "forward" : "backward");
    
    /* 파도 이동 */
    if (wave_direction) {
        wave_position++;
        if (wave_position >= SAFE_GPIO_PIN_COUNT - 1) {
            wave_direction = false;
        }
    } else {
        wave_position--;
        if (wave_position <= 0) {
            wave_direction = true;
        }
    }
}

static void blink_random(void)
{
    /* 모든 안전한 GPIO 끄기 */
    for (int i = 0; i < SAFE_GPIO_PIN_COUNT; i++) {
        gpio_set_level(safe_gpio_pins[i], 0);
    }
    
    /* 랜덤하게 1-3개 핀 켜기 */
    int num_pins_to_light = (esp_random() % 3) + 1;
    
    for (int i = 0; i < num_pins_to_light; i++) {
        int random_pin = esp_random() % SAFE_GPIO_PIN_COUNT;
        gpio_set_level(safe_gpio_pins[random_pin], 1);
        ESP_LOGI(TAG, "Random: GPIO_%d HIGH", safe_gpio_pins[random_pin]);
    }
}

static void gpio_test_task(void *arg)
{
    ESP_LOGI(TAG, "GPIO test task started");
    
    while (test_running) {
        switch (current_mode) {
            case BLINK_MODE_ALL_TOGETHER:
                blink_all_together();
                break;
            case BLINK_MODE_SEQUENTIAL:
                blink_sequential();
                break;
            case BLINK_MODE_WAVE:
                blink_wave();
                break;
            case BLINK_MODE_RANDOM:
                blink_random();
                break;
            default:
                blink_all_together();
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(blink_delay_ms));
    }
    
    /* 테스트 종료 시 모든 안전한 GPIO 끄기 */
    for (int i = 0; i < SAFE_GPIO_PIN_COUNT; i++) {
        gpio_set_level(safe_gpio_pins[i], 0);
    }
    
    ESP_LOGI(TAG, "GPIO test task ended");
    vTaskDelete(NULL);
}

static void print_usage(void)
{
    printf("\n=== ESP32-C6 Safe GPIO Blink Test ===\n");
    printf("사용 가능한 명령어:\n");
    printf("1 - 모든 안전한 GPIO 동시에 blink\n");
    printf("2 - 순차적으로 blink\n");
    printf("3 - 파도 패턴 blink\n");
    printf("4 - 랜덤 blink\n");
    printf("+ - 속도 증가 (100ms씩)\n");
    printf("- - 속도 감소 (100ms씩)\n");
    printf("q - 종료\n");
    printf("현재 모드: %d, 지연시간: %"PRIu32"ms\n", current_mode, blink_delay_ms);
    printf("안전한 GPIO 핀: GPIO_%d, GPIO_%d, GPIO_%d, GPIO_%d\n",
           safe_gpio_pins[0], safe_gpio_pins[1], safe_gpio_pins[2], 
           safe_gpio_pins[3]);
    printf("=====================================\n\n");
}

static void handle_user_input(void)
{
    char input;
    
    while (test_running) {
        input = getchar();
        
        switch (input) {
            case '1':
                current_mode = BLINK_MODE_ALL_TOGETHER;
                ESP_LOGI(TAG, "Mode changed to: All Together");
                break;
            case '2':
                current_mode = BLINK_MODE_SEQUENTIAL;
                ESP_LOGI(TAG, "Mode changed to: Sequential");
                break;
            case '3':
                current_mode = BLINK_MODE_WAVE;
                ESP_LOGI(TAG, "Mode changed to: Wave");
                break;
            case '4':
                current_mode = BLINK_MODE_RANDOM;
                ESP_LOGI(TAG, "Mode changed to: Random");
                break;
            case '+':
                if (blink_delay_ms > 100) {
                    blink_delay_ms -= 100;
                    ESP_LOGI(TAG, "Speed increased, delay: %"PRIu32"ms", blink_delay_ms);
                }
                break;
            case '-':
                blink_delay_ms += 100;
                ESP_LOGI(TAG, "Speed decreased, delay: %"PRIu32"ms", blink_delay_ms);
                break;
            case 'q':
            case 'Q':
                test_running = false;
                ESP_LOGI(TAG, "Test stopping...");
                break;
            default:
                break;
        }
    }
} 
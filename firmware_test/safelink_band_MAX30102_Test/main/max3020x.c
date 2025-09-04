#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"

// Custom includes
#include "i2c_driver.h"
#include "max3020x.h"


static const char *TAG = "MAX3020X_DRIVER";


extern i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

void sensor_task(void *arg);

fifo_sample_2slot_t data[DATA_SIZE];
int data_focus = 0;
int data_prev_focus = 0;
fifo_sample_2slot_t printData[DATA_SIZE/4];
fifo_sample_2slot_t analyze_data[DATA_SIZE];
int analyze_data_focus = 0;
SemaphoreHandle_t data_mutex = NULL;

esp_err_t i2c_max3020x_init(void){
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MAX30102_I2C_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    // 세마포어 생성
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_FAIL;
    }

    esp_err_t ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ret;
    }

    ESP_LOGI(TAG, "I2C master initialized successfully");
    return ESP_OK;
}

esp_err_t i2c_max3020x_deinit(void)
{
    return i2c_master_bus_rm_device(dev_handle);
}

uint8_t i2c_buffer[128];
void writen(uint8_t addr, uint8_t *data, int n)
{
    i2c_buffer[0] = addr;
    memcpy(i2c_buffer+1, data, n);
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, i2c_buffer, n+1, -1));
}

esp_err_t readn(uint8_t addr, uint8_t *data, int n)
{
    return i2c_master_transmit_receive(dev_handle, &addr, 1, data, n, -1);
}

uint8_t read8(uint8_t addr)
{
    uint8_t data;
    readn(addr, &data, 1);
    return data;
}

void write8(uint8_t addr, uint8_t data)
{
    writen(addr, &data, 1);
}

bool is_Part_ID_valid(void)
{
    return (read_Part_ID() == 0x15);
}
uint8_t read_Part_ID(void)
{
    return read8(0xFF);
}



uint8_t int_status[2];
bool get_INT_STATUS_1(int bit){
    uint8_t mask = 1<<bit;
    int_status[0] |= read8(INT_STATUS_1);
    if((int_status[0]&mask) != 0x00){
        int_status[0] &= ~mask;
        return true;
    }
    return false;
}
uint8_t get_INT_STATUS_2(void){
    int_status[1] = read8(INT_STATUS_2);
    return int_status[1];
}
size_t MAX30102_SAMPLE_NUMBER = 1;
float data_rate = 0;
bool is_A_FULL(void) {return get_INT_STATUS_1(7);}
bool is_PPG_RDY(void){return get_INT_STATUS_1(6);}
bool is_ALC_OVF(void){return get_INT_STATUS_1(5);}
bool is_TEMP_RDY(void){return get_INT_STATUS_2() != 0x00;}
int get_FIFO_A_FULL(void){ return read8(FIFO_CONFIG)&0xF;}
int read_FIFO(fifo_sample_t *data)
{
    int fifo_wr_ptr = read8(FIFO_WR_PTR);
    int fifo_rd_ptr = read8(FIFO_RD_PTR);
    int fifo_ovf_cnt = read8(FIFO_OVF_CNT);
    int NUM_AVAILABLE_SAMPLES = fifo_wr_ptr - fifo_rd_ptr;
    if(NUM_AVAILABLE_SAMPLES < 0 || (NUM_AVAILABLE_SAMPLES == 0 && fifo_ovf_cnt > 0)) 
        NUM_AVAILABLE_SAMPLES += 32;
    if(fifo_ovf_cnt > 0){
        ESP_LOGE(TAG, "FIFO OVF");
    }
    //ESP_LOGI(TAG, "FIFO_WR_PTR: %d, FIFO_RD_PTR: %d, NUM_AVAILABLE_SAMPLES: %d, OVF_COUNTER: %d", fifo_wr_ptr, fifo_rd_ptr, NUM_AVAILABLE_SAMPLES, fifo_ovf_cnt);

    if(NUM_AVAILABLE_SAMPLES > 0){
        if(readn(FIFO_DATA, (uint8_t*)data, NUM_AVAILABLE_SAMPLES*sizeof(fifo_sample_t)*MAX30102_SAMPLE_NUMBER) != ESP_OK){
            ESP_LOGE(TAG, "I2C read failed");
            return 0;
        }
    }

    return NUM_AVAILABLE_SAMPLES;
}
int get_FIFO_DATA_SIZE(void){ return MAX30102_SAMPLE_NUMBER;}
float get_DIE_TEMP(){
    int8_t integer = read8(DIE_TEMP_INTEGER);
    int8_t frac = read8(DIE_TEMP_FRAC);
    return (float)integer + (float)frac*0.0625f;
}

// average_mode: SMP_AVE_MODE_1
// almost_full_threshold: 2 (30 bytes should be read per one sample read)
// mode : 2(Heart Rate Only), 3(SpO2 Only), 7(Multi-LED Mode)
// led_pw_res : 15b, 16b, 17b, 18b
// sp_sample_rate : 50, 100, 200, 400, 800, 1000, 1600, 3200
// sp_adc_range : 2048, 4096, 8192, 16384
// led_amp : 1=0.2mA, 255=51.0mA (I_led = 0.2mA*led_amp)
// die_temp_en : true=enable, false=disable
void init_MAX30102(
    SMP_AVE_MODE average_mode, uint8_t almost_full_threshold, int mode,
    SPO2_LED_PW_RES led_pw_res, SPO2_SAMPLERATE sp_sample_rate, SPO2_ADC_RANGE sp_adc_range,
    uint8_t led1_amp, uint8_t led2_amp,
    bool die_temp_en
)    {
    MAX30102_SAMPLE_NUMBER = 2;
    write8(INT_ENABLE_1, 0xE0);
    write8(INT_ENABLE_2, 0x02);
    write8(FIFO_CONFIG, average_mode<<5 | (0<<4) | (almost_full_threshold&0xF));
    write8(MODE_CONFIG, (mode&0x7));
    write8(SPO2_CONFIG, (sp_adc_range<<5) | (sp_sample_rate<<2) | (led_pw_res));
    write8(LEDPLUSE_AMP_LED1, (led1_amp&0xFF));
    write8(LEDPLUSE_AMP_LED2, (led2_amp&0xFF));
    write8(MULTI_LED_MODE_REG1, (1<<4) | (2<<0));
    write8(MULTI_LED_MODE_REG2, 0x00);
    // if multi mode, slot1 = Red, slot2 = IR
    write8(DIE_TEMP_CONFIG, die_temp_en);


    data_rate = 1.f; // 1samples/s
    data_rate /= powf(2.f,(float)average_mode);

    float sample_rate = 0.f;
    switch(sp_sample_rate){
        case SPO2_SAMPLERATE_50:
            sample_rate = 50.f;
            break;
        case SPO2_SAMPLERATE_100:
            sample_rate = 100.f;
            break;
        case SPO2_SAMPLERATE_200:
            sample_rate = 200.f;
            break;
        case SPO2_SAMPLERATE_400:
            sample_rate = 400.f;
            break;
        case SPO2_SAMPLERATE_800:
            sample_rate = 800.f;
            break;
        case SPO2_SAMPLERATE_1000:
            sample_rate = 1000.f;
            break;
        case SPO2_SAMPLERATE_1600:
            sample_rate = 1600.f;
            break;
        case SPO2_SAMPLERATE_3200:
            sample_rate = 3200.f;
            break;
        default:
            sample_rate = 0.f;
            break;
    }

    data_rate *= sample_rate;
}

float get_data_rate(void){
    return data_rate;
}

BaseType_t start_sensor_task(){
    TaskHandle_t sensor_task_handle = NULL;
    BaseType_t task_created = xTaskCreate(sensor_task, "MAX_30102_TASK", 8192, NULL, 3, &sensor_task_handle);
    return task_created;
}

void read_printData(void(*print_buffer_ready)(fifo_sample_2slot_t* printbuffer, int nread), void(*analyze_buffer_ready)(fifo_sample_2slot_t* analyzebuffer, int focus, int nread)){
    // 세마포어 획득 시도
    int n_data = data_focus - data_prev_focus;
    if(n_data < 0) n_data += DATA_SIZE;
        
    if(xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE){
        // 데이터가 충분히 쌓였을 때만 처리
        if(n_data > DATA_SIZE/8){
            int copy_1 = n_data, copy_2 = 0;
            if((data_prev_focus + n_data) > DATA_SIZE) {
                copy_2 = data_focus;
                copy_1 = DATA_SIZE - data_prev_focus;
            }
            
            memcpy(printData, data + data_prev_focus, copy_1 * sizeof(fifo_sample_2slot_t));
            if (copy_2 > 0) {
                memcpy(printData + copy_1, data, copy_2 * sizeof(fifo_sample_2slot_t));
            }
            int prev1 = data_prev_focus, now1 = data_focus;
            data_prev_focus = data_focus;

            xSemaphoreGive(data_mutex);
            vTaskDelay(pdMS_TO_TICKS(1)); // 50ms로 증가

            // store PPG Data in another long buffer for heart rate analysis
            copy_1 = n_data, copy_2 = 0;
            if((analyze_data_focus + n_data) > DATA_SIZE) {
                copy_2 = n_data - (DATA_SIZE - analyze_data_focus);
                copy_1 = DATA_SIZE - analyze_data_focus;
            }

            memcpy(analyze_data + analyze_data_focus, printData, copy_1 * sizeof(fifo_sample_2slot_t));
            if (copy_2 > 0) {
                memcpy(analyze_data, printData + copy_1, copy_2 * sizeof(fifo_sample_2slot_t));
            }
            analyze_data_focus = (analyze_data_focus + n_data) % DATA_SIZE;


            //ESP_LOGI(TAG, "Processing %d samples %d -> %d", n_data, prev1, now1);
            
            if(n_data > 0 && print_buffer_ready != NULL){
                print_buffer_ready(printData, n_data);
            }
            if(n_data > 0 && analyze_buffer_ready != NULL){
                analyze_buffer_ready(analyze_data, analyze_data_focus, n_data);
            }
        } else {
            xSemaphoreGive(data_mutex);
        }
    } else {
        ESP_LOGW(TAG, "Failed to acquire data mutex in main loop");
    }

    // 메인 루프 딜레이
}

void sensor_task(void *arg){
    
    // 세마포어가 이미 생성되었는지 확인
    if (data_mutex == NULL) {
        ESP_LOGE(TAG, "Data mutex not initialized");
        vTaskDelete(NULL);
        return;
    }
    
    // MAX30102 초기화
    init_MAX30102(SMP_AVE_MODE_2,8,7,SPO2_LED_PW_RES_17b,SPO2_SAMPLERATE_100,SPO2_ADC_RANGE_8192,80,80,true);
    
    // 스택에 큰 배열 할당하지 않도록 수정
    fifo_sample_2slot_t *fifo = (fifo_sample_2slot_t*)malloc(32 * sizeof(fifo_sample_2slot_t));
    if (fifo == NULL) {
        ESP_LOGE(TAG, "Failed to allocate FIFO buffer");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Sensor task started successfully");
    
    while(1){
        // 세마포어 획득 시도
        if(is_A_FULL() || is_PPG_RDY()){
            if(xSemaphoreTake(data_mutex, pdMS_TO_TICKS(100)) == pdTRUE){
                // FIFO 데이터 읽기 (최대 10번으로 제한) 
                for(int j = 0; j < 10 && (j==0 || is_A_FULL() || is_PPG_RDY()); j++){
                    int n_read = read_FIFO((fifo_sample_t*)fifo);
                    if (n_read > 0) {
                        //ESP_LOGI(TAG, "PPG Data read %d samples", n_read);
                        for(int i = 0; i < n_read && i < 32; i++){ // 배열 범위 체크 추가
                            data[data_focus] = fifo[i];
                            if((++data_focus) >= DATA_SIZE) data_focus = 0;
                            if(data_focus == data_prev_focus){
                                ESP_LOGE(TAG, "FIFO OVF detected");
                                if((++data_prev_focus) >= DATA_SIZE) data_prev_focus = 0;
                            }
                        }
                    }
                }
                
                // ALC 오버플로우 체크
                if(is_ALC_OVF()){
                    ESP_LOGE(TAG, "ALC OVF detected");
                }
                
                // 세마포어 해제
                xSemaphoreGive(data_mutex);
            } else {
                // 세마포어 획득 실패 시 로그
                ESP_LOGW(TAG, "Failed to acquire data mutex");
            }
        }
        
        // 적절한 딜레이 (1ms는 너무 짧음)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    
    // 메모리 해제
    free(fifo);
}
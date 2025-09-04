#ifndef MAX3020X_H
#define MAX3020X_H

#include "i2c_driver.h"

// MAX30102 센서 주소
#define MAX30102_I2C_ADDR             0x57

#define DATA_SIZE 512

// 함수 선언
esp_err_t i2c_max3020x_init(void);
esp_err_t i2c_max3020x_deinit(void);

#define INT_STATUS_1 0x00
#define INT_STATUS_2 0x01
#define INT_ENABLE_1 0x02
#define INT_ENABLE_2 0x03
#define FIFO_WR_PTR 0x04
#define FIFO_OVF_CNT 0x05
#define FIFO_RD_PTR 0x06
#define FIFO_DATA 0x07

#define FIFO_CONFIG 0x08
typedef enum{
    SMP_AVE_MODE_1 = 0b00,
    SMP_AVE_MODE_2 = 0b01,
    SMP_AVE_MODE_4 = 0b10,
    SMP_AVE_MODE_8 = 0b11,
    SMP_AVE_MODE_16 = 0b100,
    SMP_AVE_MODE_32 = 0b101,
}SMP_AVE_MODE;
typedef struct{
    uint8_t data[3];
} fifo_sample_t;
typedef struct{
    uint8_t slot1[3];
    uint8_t slot2[3];
} fifo_sample_2slot_t;

#define MODE_CONFIG 0x09
#define SPO2_CONFIG 0x0A
typedef enum{
    SPO2_ADC_RANGE_2048 = 0b00,
    SPO2_ADC_RANGE_4096 = 0b01,
    SPO2_ADC_RANGE_8192 = 0b10,
    SPO2_ADC_RANGE_16384 = 0b11,
}SPO2_ADC_RANGE;//[6:5]
typedef enum{
    SPO2_SAMPLERATE_50 = 0b000,
    SPO2_SAMPLERATE_100 = 0b001,
    SPO2_SAMPLERATE_200 = 0b010,
    SPO2_SAMPLERATE_400 = 0b011,
    SPO2_SAMPLERATE_800 = 0b100,
    SPO2_SAMPLERATE_1000 = 0b101,
    SPO2_SAMPLERATE_1600 = 0b110,
    SPO2_SAMPLERATE_3200 = 0b111,
}SPO2_SAMPLERATE;//[4:2]
typedef enum{
    SPO2_LED_PW_RES_15b = 0b00,
    SPO2_LED_PW_RES_16b = 0b01,
    SPO2_LED_PW_RES_17b = 0b10,
    SPO2_LED_PW_RES_18b = 0b11,
}SPO2_LED_PW_RES;//[1:0]

#define LEDPLUSE_AMP_LED1 0x0C
#define LEDPLUSE_AMP_LED2 0x0D
#define MULTI_LED_MODE_REG1 0x11
#define MULTI_LED_MODE_REG2 0x12
#define DIE_TEMP_INTEGER 0x1F
#define DIE_TEMP_FRAC 0x20
#define DIE_TEMP_CONFIG 0x21

uint8_t read_Part_ID(void);
bool is_Part_ID_valid(void);
bool is_A_FULL(void);
bool is_PPG_RDY(void);
bool is_ALC_OVF(void);
bool is_TEMP_RDY(void);
int get_FIFO_A_FULL(void);
int read_FIFO(fifo_sample_t *data);
int get_FIFO_DATA_SIZE(void);
float get_DIE_TEMP();
float get_data_rate(void);

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
);

BaseType_t start_sensor_task();
void read_printData(void(*print_buffer_ready)(fifo_sample_2slot_t* printbuffer, int nread), void(*analyze_buffer_ready)(fifo_sample_2slot_t* analyzebuffer, int focus, int nread));

#endif // I2C_H 
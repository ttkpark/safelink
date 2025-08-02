#ifndef I2C_H
#define I2C_H

#include "esp_err.h"
#include "driver/i2c.h"

// I2C 설정
#define I2C_MASTER_SCL_IO           22      // SCL 핀
#define I2C_MASTER_SDA_IO           20      // SDA 핀
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000 // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

// AM2320 센서 주소
#define AM2320_I2C_ADDR             0x5C

// 함수 선언
esp_err_t i2c_master_init(void);
esp_err_t i2c_master_deinit(void);
esp_err_t AM2320_read(uint16_t *temperature, uint16_t *humidity);

#endif // I2C_H
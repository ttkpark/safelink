#ifndef I2C_H
#define I2C_H

#include "esp_err.h"
#include "driver/i2c.h"

// I2C 설정 (ESP32-C6용)
#define I2C_MASTER_SCL_IO           7       // SCL 핀 (ESP32-C6 GPIO7)
#define I2C_MASTER_SDA_IO           6       // SDA 핀 (ESP32-C6 GPIO6)
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000 // 100kHz
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0
#define I2C_MASTER_TIMEOUT_MS       1000

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_deinit(void);


#endif // I2C_H 
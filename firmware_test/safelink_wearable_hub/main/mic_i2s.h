#ifndef MIC_I2S_H
#define MIC_I2S_H

#include "esp_err.h"

// I2S 마이크 설정 (INMP441 등)
// 핀은 보드에 맞게 수정하세요
#ifndef I2S_MIC_BCK_IO
#define I2S_MIC_BCK_IO   6
#endif
#ifndef I2S_MIC_WS_IO
#define I2S_MIC_WS_IO    2
#endif
#ifndef I2S_MIC_DATA_IO
#define I2S_MIC_DATA_IO  14
#endif

#define I2S_MIC_SAMPLE_RATE_HZ  16000
#define I2S_MIC_BUFFER_SAMPLES  1024

// 초기화 및 태스크 시작
esp_err_t mic_i2s_init(void);
esp_err_t mic_i2s_start(void);
esp_err_t mic_i2s_stop(void);

// 최근 평균 소음 dB(SPL) 반환 (가용하지 않으면 음수)
float mic_i2s_get_average_spl_db(void);

#endif // MIC_I2S_H



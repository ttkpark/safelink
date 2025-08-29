#include "mic_i2s.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include <math.h>

static const char *TAG = "MIC_I2S";

static float s_dc_offset = 0.0f;      // DC 오프셋
static float s_dc_alpha = 0.98f;       // DC 추적 필터 계수 (0.0~1.0, 높을수록 느리게 추적)
static float s_avg_spl_db = -1.0f;     // 최근 평균 SPL

static int16_t s_buffer[I2S_MIC_BUFFER_SAMPLES];

static esp_err_t mic_i2s_configure(void)
{
    const i2s_config_t i2s_cfg = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX,
        .sample_rate = I2S_MIC_SAMPLE_RATE_HZ,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_MIC_BUFFER_SAMPLES,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0,
    };

    const i2s_pin_config_t pin_cfg = {
        .bck_io_num = I2S_MIC_BCK_IO,
        .ws_io_num = I2S_MIC_WS_IO,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_DATA_IO,
    };

    esp_err_t ret = i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2s_set_pin(I2S_NUM_0, &pin_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s set pin failed: %s", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

static float calculate_rms_without_dc(const int16_t *buf, size_t n)
{
    // 16-bit PCM을 -1.0~1.0로 정규화하고 DC 제거 후 RMS 계산
    // 1) 현재 프레임 평균으로 DC 추정
    double mean = 0.0;
    for (size_t i = 0; i < n; i++) {
        mean += (double)buf[i];
    }
    mean /= (double)n;

    // 2) 지수평활로 글로벌 DC 오프셋 업데이트
    s_dc_offset = (float)(s_dc_alpha * s_dc_offset + (1.0 - s_dc_alpha) * (mean / 32767.0));

    // 3) DC 제거 후 RMS
    double sum_sq = 0.0;
    for (size_t i = 0; i < n; i++) {
        float x = ((float)buf[i] / 32767.0f) - s_dc_offset;
        sum_sq += (double)x * (double)x;
    }
    double rms = sqrt(sum_sq / (double)n);
    return (float)rms;
}

// INMP441 기준 dB 변환: 94 dB SPL = -26 dBFS
static float rms_to_db_spl(float rms)
{
    if (rms <= 0.0f) return 0.0f;
    float dbfs = 20.0f * log10f(rms);      // -∞ ~ 0 dBFS
    float db_spl = dbfs + 26.0f + 94.0f;   // Sensitivity -26 dBFS @ 94 dB SPL
    if (db_spl < 0.0f) db_spl = 0.0f;
    if (db_spl > 130.0f) db_spl = 130.0f;  // 상한 클램프
    return db_spl;
}

esp_err_t mic_i2s_init(void)
{
    return mic_i2s_configure();
}

esp_err_t mic_i2s_start(void)
{
    // I2S는 설치 시 바로 수신 준비됨. 별도 스타트 필요 없음
    return ESP_OK;
}

esp_err_t mic_i2s_stop(void)
{
    i2s_driver_uninstall(I2S_NUM_0);
    return ESP_OK;
}

float mic_i2s_get_average_spl_db(void)
{
    // 한 프레임을 읽어 평균 SPL을 갱신 후 반환
    size_t bytes_read = 0;
    esp_err_t ret = i2s_read(I2S_NUM_0, s_buffer, sizeof(s_buffer), &bytes_read, 10 / portTICK_PERIOD_MS);
    if (ret != ESP_OK || bytes_read == 0) {
        return s_avg_spl_db;
    }

    size_t samples = bytes_read / sizeof(int16_t);
    float rms = calculate_rms_without_dc(s_buffer, samples);
    float spl = rms_to_db_spl(rms);

    // 간단한 이동 평균
    if (s_avg_spl_db < 0.0f) s_avg_spl_db = spl;
    s_avg_spl_db = 0.8f * s_avg_spl_db + 0.2f * spl;
    return s_avg_spl_db;
}



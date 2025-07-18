/*
 * Safety Analyzer Implementation
 */

#include "safety_analyzer.h"
#include "esp_log.h"
#include <string.h>

#define TAG "SAFETY_ANALYZER"

esp_err_t safety_analyzer_init(void)
{
    ESP_LOGI(TAG, "Safety analyzer initialized");
    return ESP_OK;
}

esp_err_t safety_analyzer_analyze(const wearable_device_t *devices, uint8_t device_count,
                                 const noise_data_t *noise_data, safety_result_t *result)
{
    if (result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize result
    memset(result, 0, sizeof(safety_result_t));
    result->timestamp = esp_timer_get_time() / 1000;
    result->overall_status = SAFETY_STATUS_NORMAL;
    result->primary_risk = RISK_TYPE_NONE;
    strcpy(result->message, "모든 지표가 정상입니다");
    
    float max_risk_score = 0.0f;
    uint8_t critical_count = 0;
    uint8_t warning_count = 0;
    
    // Analyze noise levels
    if (noise_data != NULL) {
        float noise_risk = 0.0f;
        
        if (noise_data->current_db >= 120.0f) {
            noise_risk = 100.0f;
            critical_count++;
            result->primary_risk = RISK_TYPE_NOISE;
            strcpy(result->message, "소음이 매우 위험합니다. 즉시 안전한 곳으로 이동하세요.");
        } else if (noise_data->current_db >= 105.0f) {
            noise_risk = 80.0f;
            warning_count++;
            if (result->primary_risk == RISK_TYPE_NONE) {
                result->primary_risk = RISK_TYPE_NOISE;
                strcpy(result->message, "소음 수준이 높습니다. 귀마개를 착용하세요.");
            }
        } else if (noise_data->current_db >= 90.0f) {
            noise_risk = 50.0f;
            warning_count++;
        }
        
        if (noise_risk > max_risk_score) {
            max_risk_score = noise_risk;
        }
    }
    
    // Analyze wearable device data
    if (devices != NULL && device_count > 0) {
        for (uint8_t i = 0; i < device_count; i++) {
            const wearable_device_t *device = &devices[i];
            
            if (!device->is_connected) {
                continue;
            }
            
            float device_risk = 0.0f;
            
            // Check heart rate
            if (device->sensor_data.heart_rate >= 130.0f) {
                device_risk = 90.0f;
                critical_count++;
                if (result->primary_risk == RISK_TYPE_NONE || max_risk_score < device_risk) {
                    result->primary_risk = RISK_TYPE_HEART_RATE;
                    strcpy(result->message, "심박수가 위험합니다. 즉시 작업을 중단하세요.");
                }
            } else if (device->sensor_data.heart_rate >= 120.0f) {
                device_risk = 60.0f;
                warning_count++;
                if (result->primary_risk == RISK_TYPE_NONE) {
                    result->primary_risk = RISK_TYPE_HEART_RATE;
                    strcpy(result->message, "심박수가 높습니다. 휴식이 필요합니다.");
                }
            }
            
            // Check temperature
            if (device->sensor_data.skin_temperature >= 38.0f) {
                float temp_risk = 95.0f;
                if (temp_risk > device_risk) {
                    device_risk = temp_risk;
                    critical_count++;
                    if (result->primary_risk == RISK_TYPE_NONE || max_risk_score < device_risk) {
                        result->primary_risk = RISK_TYPE_TEMPERATURE;
                        strcpy(result->message, "체온이 위험합니다. 즉시 휴식하고 의료조치를 받으세요.");
                    }
                }
            } else if (device->sensor_data.skin_temperature >= 37.5f) {
                float temp_risk = 70.0f;
                if (temp_risk > device_risk) {
                    device_risk = temp_risk;
                    warning_count++;
                    if (result->primary_risk == RISK_TYPE_NONE) {
                        result->primary_risk = RISK_TYPE_TEMPERATURE;
                        strcpy(result->message, "체온이 높습니다. 휴식이 필요합니다.");
                    }
                }
            }
            
            // Check WBGT
            if (device->sensor_data.wbgt >= 30.0f) {
                float wbgt_risk = 85.0f;
                if (wbgt_risk > device_risk) {
                    device_risk = wbgt_risk;
                    critical_count++;
                    if (result->primary_risk == RISK_TYPE_NONE || max_risk_score < device_risk) {
                        result->primary_risk = RISK_TYPE_HEAT_STRESS;
                        strcpy(result->message, "열스트레스 위험. 즉시 시원한 곳으로 이동하세요.");
                    }
                }
            } else if (device->sensor_data.wbgt >= 28.0f) {
                float wbgt_risk = 60.0f;
                if (wbgt_risk > device_risk) {
                    device_risk = wbgt_risk;
                    warning_count++;
                    if (result->primary_risk == RISK_TYPE_NONE) {
                        result->primary_risk = RISK_TYPE_HEAT_STRESS;
                        strcpy(result->message, "열스트레스 주의. 충분한 수분 섭취하세요.");
                    }
                }
            }
            
            if (device_risk > max_risk_score) {
                max_risk_score = device_risk;
            }
        }
        
        result->affected_devices = critical_count + warning_count;
    }
    
    // Determine overall status
    if (critical_count > 0) {
        result->overall_status = SAFETY_STATUS_CRITICAL;
    } else if (warning_count > 0) {
        result->overall_status = SAFETY_STATUS_WARNING;
    } else {
        result->overall_status = SAFETY_STATUS_NORMAL;
    }
    
    // Handle multiple risks
    if ((critical_count + warning_count) > 1) {
        result->primary_risk = RISK_TYPE_MULTIPLE;
        strcpy(result->message, "여러 위험 요소가 감지되었습니다. 즉시 안전 조치를 취하세요.");
    }
    
    result->risk_score = max_risk_score;
    
    ESP_LOGD(TAG, "Safety analysis: Status=%d, Risk=%d, Score=%.1f", 
             result->overall_status, result->primary_risk, result->risk_score);
    
    return ESP_OK;
} 
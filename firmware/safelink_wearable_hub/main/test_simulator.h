#ifndef TEST_SIMULATOR_H
#define TEST_SIMULATOR_H

#include "esp_err.h"
#include "data_manager.h"

// 시뮬레이션 데이터 생성 함수들
esp_err_t test_simulator_generate_band_data(void);
esp_err_t test_simulator_generate_hub_data(void);
esp_err_t test_simulator_start_periodic_data(void);
esp_err_t test_simulator_stop_periodic_data(void);

// 시뮬레이션 상태
void test_simulator_set_enabled(bool enabled);
bool test_simulator_is_enabled(void);

// 클라이언트 데이터 관리
void test_simulator_reset_client_data(void);

#endif // TEST_SIMULATOR_H

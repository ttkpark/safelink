# 제4회 창의혁신 공모전 코드 설명서
# SafeLink System
## 작업자와 관리자, 사람과 환경을 '안전하게 연결'하는 시스템

**Ohmazing Y**  
옴메이징 어드미턴스

## 3. 하드웨어 구성별 기능 및 코드 매핑

### 3.1 Safelink_Hub (웨어러블 허브)

#### 3.1.1 허브의 주요 기능

| 기능 | 설명 | 구현 함수 | 파일:라인 | 상세 기능 |
|------|------|-----------|-----------|-----------|
| **밴드 데이터 수신** | BLE Advertising Packet 수신 | `gap_event_cb()` | bluetooth.c:439 | BLE 스캔 이벤트 처리 |
| **밴드 데이터 파싱** | 광고 데이터에서 센서값 추출 | `parse_mfg_and_update_band()` | bluetooth.c:979 | Manufacturer Data 파싱 |
| **허브 센서 수집** | 온습도 및 소음 센서 데이터 수집 | `temp_humidity_task()` | sensor.c:57 | AM2320 온습도 센서 |
| **허브 센서 수집** | I2S 마이크 소음 측정 | `mic_i2s_get_average_spl_db()` | mic_i2s.c | I2S 마이크 소음 측정 |
| **위험 판단** | 센서값 기준 위험도 판단 | `sensor_monitor_task()` | sensor.c:130 | 경고 조건 체크 |
| **경고 알림** | 소리 및 진동 알림 | `warning_system_play_voice()` | sensor.c:495 | DFPlayer 음성 재생 |
| **경고 알림** | 진동 모터 제어 | `warning_system_vibrate()` | sensor.c:522 | GPIO 진동 제어 |
| **앱 데이터 전송** | GATT Server 운영 | `gatt_svr_access_cb()` | bluetooth.c:77 | GATT 특성 읽기/쓰기 |
| **앱 데이터 전송** | 통합 데이터 전송 | `bluetooth_update_hub_data_characteristics()` | bluetooth.c:1257 | 허브 데이터 특성 업데이트 |

#### 3.1.2 허브 센서 데이터 수신 로직

| 센서 | 통신 방식 | 구현 함수 | 파일:라인 | 측정 범위 |
|------|-----------|-----------|-----------|-----------|
| **AM2320 온습도** | I2C | `AM2320_read()` | i2c.c | 온도: -40~80°C, 습도: 0~100% |
| **I2S 마이크 소음** | I2S | `mic_i2s_get_average_spl_db()` | mic_i2s.c | 0~120dB |

#### 3.1.3 위험 판단 기준 및 구현

| 위험 항목 | 판단 기준 | 경고 레벨 | 구현 함수 | 파일:라인 |
|-----------|-----------|-----------|-----------|-----------|
| **소음** | ≥90dB 15분 지속 | 1(주의) | `warning_system_check_and_trigger()` | sensor.c:541 |
| **소음** | ≥100dB 15분 지속 | 2(경고) | `warning_system_check_and_trigger()` | sensor.c:541 |
| **소음** | ≥110dB 15분 지속 | 3(위험) | `warning_system_check_and_trigger()` | sensor.c:541 |
| **WBGT** | ≥28.0°C 15분 지속 | 1(주의) | `calc_wbgt()` | sensor.c:112 |
| **WBGT** | ≥30.0°C 15분 지속 | 3(위험) | `calc_wbgt()` | sensor.c:112 |
| **체온** | 37.5≤체온<38.0°C | 1(주의) | `sensor_monitor_task()` | sensor.c:171 |
| **체온** | 체온≥38°C, <27°C | 3(위험) | `sensor_monitor_task()` | sensor.c:171 |
| **심박수** | HR>120bpm 15분 지속 | 1(주의) | `sensor_monitor_task()` | sensor.c:175 |
| **심박수** | HR≥140bpm 15분 지속 | 3(위험) | `sensor_monitor_task()` | sensor.c:186 |

#### 3.1.4 Keep-Alive 메커니즘

| 기능 | 설명 | 구현 함수 | 파일:라인 | 동작 방식 |
|------|------|-----------|-----------|-----------|
| **연결 체크 시작** | 연결 시 5초 타이머 시작 | `gap_event_cb()` | bluetooth.c:630 | 연결 시 체크 시스템 활성화 |
| **체크 메시지 수신** | "check" 메시지 수신 시 타이머 리셋 | `gatt_svr_access_cb()` | bluetooth.c:315 | 명령어 처리 시 타이머 리셋 |
| **연결 강제 해제** | 5초 타임아웃 시 연결 해제 | `check_timeout_cb()` | bluetooth.c:1307 | 타이머 콜백으로 연결 해제 |

### 3.2 Safelink_Band (웨어러블 밴드)

#### 3.2.1 밴드의 주요 기능

| 기능 | 설명 | 구현 함수 | 파일:라인 | 상세 기능 |
|------|------|-----------|-----------|-----------|
| **센서 데이터 수집** | I2C 센서 데이터 읽기 | `sensor_beacon_task()` | main.c:334 | 주기적 센서 데이터 수집 |
| **심박수 측정** | ADC 기반 심박수 측정 | `heart_rate_sampling_task()` | main.c:581 | 10ms 간격 ADC 샘플링 |
| **심박수 계산** | DSP 알고리즘으로 BPM 계산 | `calculate_heart_rate_from_buffer()` | main.c:675 | 피크 검출 및 통계 분석 |
| **BLE 광고** | 센서 데이터 BLE 전송 | `sensor_beacon_task()` | main.c:334 | 20ms 간격 광고 전송 |
| **WiFi UDP** | 서버로 데이터 전송 | `wifi_udp_task()` | main.c:1169 | 5초 간격 UDP 전송 |

#### 3.2.2 센서 데이터 추출 과정

| 단계 | 설명 | 구현 함수 | 파일:라인 | 처리 방식 |
|------|------|-----------|-----------|-----------|
| **1단계: 원시 데이터 수집** | 10ms 간격 ADC 샘플링 | `heart_rate_sampling_task()` | main.c:581 | 순환 버퍼에 저장 |
| **2단계: 1차 필터링** | 15샘플 이동평균 | `calculate_heart_rate_from_buffer()` | main.c:689 | 고주파 노이즈 제거 |
| **3단계: 2차 필터링** | 150샘플 이동평균 차이 | `calculate_heart_rate_from_buffer()` | main.c:709 | 밴드패스 필터 효과 |
| **4단계: 피크 검출** | 피크 위치 및 간격 계산 | `calculate_heart_rate_from_buffer()` | main.c:732 | 통계적 피크 검출 |
| **5단계: 이상치 제거** | 표준편차 기반 필터링 | `calculate_heart_rate_from_buffer()` | main.c:790 | 1.5σ 이상치 제거 |
| **6단계: BPM 계산** | 최종 심박수 계산 | `calculate_heart_rate_from_buffer()` | main.c:850 | 60*100/평균간격 |

#### 3.2.3 센서별 데이터 수집

| 센서 | 통신 방식 | 구현 함수 | 파일:라인 | 측정 범위 |
|------|-----------|-----------|-----------|-----------|
| **AHT20 온습도** | I2C (0x38) | `aht20_read_data()` | main.c:478 | 온도: -40~85°C, 습도: 0~100% |
| **MCP9808 체온** | I2C (0x18) | `mcp9808_read_temperature()` | main.c:510 | -40~125°C |
| **SEN0203 심박수** | ADC (GPIO4) | `heart_rate_sampling_task()` | main.c:581 | 0-4095 (12-bit) |

#### 3.2.4 BLE Advertisement 데이터 형식

| 바이트 위치 | 설명 | 데이터 타입 | 구현 함수 | 파일:라인 |
|-------------|------|-------------|-----------|-----------|
| [0-1] | BLE Company ID | uint16_t | `build_mfg_payload()` | main.c:306 |
| [2] | BLE Version | uint8_t | `build_mfg_payload()` | main.c:313 |
| [3] | 데이터 존재 플래그 | uint8_t | `build_mfg_payload()` | main.c:314 |
| [4-5] | 외부 온도 | int16_t | `build_mfg_payload()` | main.c:315 |
| [6-7] | 외부 습도 | uint16_t | `build_mfg_payload()` | main.c:317 |
| [8-9] | 체온 | int16_t | `build_mfg_payload()` | main.c:319 |
| [10-11] | 심박수 | uint16_t | `build_mfg_payload()` | main.c:321 |
| [12] | SpO2 | uint8_t | `build_mfg_payload()` | main.c:323 |

---

## 4. 주요 기능별 분석

### 3.1 센서 시스템

#### safelink_band 센서
| 센서 | 타입 | I2C 주소 | 측정 범위 | 샘플링 주기 |
|------|------|----------|-----------|-------------|
| AHT20 | 온습도 | 0x38 | 온도: -40~85°C, 습도: 0~100% | 2초 |
| MCP9808 | 온도 | 0x18 | -40~125°C | 2초 |
| ADC4 | 심박수 | GPIO4 | 0-4095 (12-bit) | 10ms |

#### safelink_hub_c3 센서
| 센서 | 타입 | 핀 | 측정 범위 | 샘플링 주기 |
|------|------|-----|-----------|-------------|
| AM2320 | 온습도 | I2C | 온도: -40~80°C, 습도: 0~100% | 4초 |
| I2S 마이크 | 소음 | GPIO4,5,6 | 0~120dB | 연속 |

### 3.2 BLE 통신

#### safelink_band (BLE 광고)
- **디바이스명**: "link_band"
- **광고 간격**: 20ms
- **페이로드**: 13바이트 (온도, 습도, 체온, 심박수, SpO2)

#### safelink_hub_c3 (BLE 서버/클라이언트)
- **디바이스명**: "SafeLink_Test"
- **스캔 간격**: 100ms
- **연결 간격**: 7.5-15ms
- **최대 연결**: 4개

### 3.3 경고 시스템

#### 경고 레벨
| 레벨 | 값 | 설명 |
|------|-----|------|
| 주의 | 1 | 경미한 위험, 주의 필요 |
| 위험 | 2 | 중간 위험 |
| 심각 | 3 | 심각한 위험, 즉시 조치 필요 |

#### 경고 조건
| 종류 | 주의 | 위험 | 심각 |
|------|------|------|------|
| WBGT | 28°C 15분 지속 | - | 30°C 15분 지속 |
| 체온 | 37.5°C | - | 38°C 이상 또는 27°C 이하 |
| 심박수 | 120 BPM 15분 지속 | - | 140 BPM 15분 지속 |
| 소음 | 90dB 15분 지속 | 100dB 15분 지속 | 110dB 15분 지속 |

### 3.4 데이터 흐름

```
safelink_band → BLE 광고 → safelink_hub_c3 → BLE 스캔 → 데이터 파싱 → 경고 시스템 → 음성/진동 알림
```

---

## 4. 기술 스택

### 하드웨어
- **MCU**: ESP32-C3 (RISC-V, 160MHz)
- **메모리**: 400KB SRAM, 4MB Flash
- **통신**: WiFi, Bluetooth LE, I2C, UART, I2S

### 소프트웨어
- **OS**: FreeRTOS
- **프레임워크**: ESP-IDF 5.4
- **BLE 스택**: NimBLE
- **개발 언어**: C

### 외부 라이브러리
- **DFPlayer Mini**: 음성 재생
- **I2S**: 마이크 입력
- **ADC**: 심박수 측정
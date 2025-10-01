# 제4회 창의혁신 공모전 최종보고서
# SafeLink System
## 작업자와 관리자, 사람과 환경을 '안전하게 연결'하는 시스템

**Ohmazing Y**  
옴메이징 어드미턴스

**팀원**
- 2022104240 박기호
- 2022104219 김예림  
- 2021104225 김재준
- 2021104323 장호영

---

## 목차

Ⅰ. 작품 요약  
Ⅱ. 문제 인식 및 개발 목적  
Ⅲ. 시스템 구성(사용부품)  
Ⅳ. 시스템 원리(SW/HW)  
Ⅴ. 기대 효과 및 발전 가능성  

---

## Ⅰ. 작품 요약

SafeLink System은 위험한 산업 현장과 극한의 날씨에서 일하는 작업자의 안전을 혁신적으로 강화하기 위한 통합형 웨어러블 기반 안전 관리 시스템입니다. 

SafeLink System은 작업 환경을 실시간으로 모니터링하고 위험 상황을 즉각적으로 대응하기 위하여 **①밴드(SafeLink Band)**, **②웨어러블 허브(SafeLink Hub)**, **③모바일 앱(SafeLink App)**으로 크게 3가지로 구성됩니다.

### 시스템 구성 요소

**웨어러블 기기(Hub, Band)**
- 웨어러블 밴드: 작업자의 심박수, 체온, 주변 온도/습도 측정
- 웨어러블 허브: 주변 소음 측정, 데이터 종합 분석, 위험 판단 및 알림

**모바일 앱**
- 작업자: 개인 건강 상태 및 작업 환경 정보 확인
- 관리자: 전체 작업자들의 생체 및 환경 정보 모니터링
- Firebase 연동: 데이터 저장 및 실시간 동기화

### 핵심 기능
- **실시간 모니터링**: 생체 데이터와 환경 데이터 동시 측정
- **위험 감지**: 고용노동부 기준에 따른 3단계 경고 시스템
- **즉시 알림**: 음성, 진동, 앱 푸시를 통한 다중 알림
- **데이터 분석**: 누적 데이터를 통한 건강 상태 추이 분석
- **관리자 대시보드**: 다수 작업자 상태 실시간 모니터링

---

## Ⅱ. 문제 인식 및 개발 목적

### 현황 분석

오늘날 수많은 작업자들이 건설, 제조업, 조선업과 같은 위험한 산업 현장에서 근무하고 있습니다. 이들은 추락, 낙상, 장비 사고와 같은 직접적인 위험뿐만 아니라, 폭염, 한파, 유해가스, 소음 등 눈에 보이지 않는 극한의 작업 환경으로 인해 건강을 위협받고 있습니다.

**주요 문제점:**
- **2024년 뉴스**: 최근 6년간 열사병, 탈진 등의 온열질환으로 인한 산업재해 급증
- **기존 안전 관리 시스템의 한계**: 사고 발생 후 사후 대처 중심
- **개인 판단 의존**: 작업자 개인의 주관적 판단에 의존
- **실시간 모니터링 부재**: 작업자 상태를 실시간으로 파악할 수 있는 시스템 부재

### 개발 목적

SafeLink 시스템의 개발은 이러한 문제 인식에서 출발했습니다:

1. **작업자 개개인의 건강 데이터와 작업 환경 데이터를 통합적으로 분석**
2. **위험을 사전에 예측하고 대응**
3. **작업자 자신도 위험에 대비하고 관리자도 위험 상황에 즉각적인 대응 가능**
4. **산업 현장의 안전을 근본적으로 개선**

---

## Ⅲ. 시스템 구성(사용 부품)

### 하드웨어 구성

#### SafeLink Band (웨어러블 밴드)
| 부품명 | 용도 | 통신 방식 |
|--------|------|-----------|
| ESP32-C3 SuperMini | 메인 MCU | BLE, WiFi |
| AHT20 온습도센서 | 외부 온습도 측정 | I2C (0x38) |
| MCP9808 체온센서 | 피부 온도 측정 | I2C (0x18) |
| SEN0203 심박센서 | 심박수 측정 | ADC (GPIO4) |
| 400mAh Li-Po 배터리 | 전원 공급 | - |
| TP4056 충전모듈 | 배터리 충전 | - |

#### SafeLink Hub (웨어러블 허브)
| 부품명 | 용도 | 통신 방식 |
|--------|------|-----------|
| XIAO-ESP32-C3 | 메인 MCU | BLE, WiFi |
| AM2320 온습도센서 | 외부 온습도 측정 | I2C |
| THC-AS01 마이크 | 소음 측정 | I2S |
| DRF0299 음성모듈 | 경고 음성 출력 | UART |
| FIT0502 스피커 | 음성 출력 | - |
| 진동 모터 | 진동 알림 | GPIO |
| SPDT 슬라이드 스위치 | 전원 제어 | - |

### 소프트웨어 구성

#### 프로젝트 구조
```
firmware/
├── safelink_band/          # 웨어러블 밴드 펌웨어
│   └── main/main.c         # 메인 애플리케이션 (1,218 lines)
└── safelink_hub_c3/        # 허브 펌웨어
    └── main/
        ├── safelink_wearable_hub.c  # 메인 애플리케이션 (278 lines)
        ├── bluetooth.c              # BLE 통신 관리 (1,462 lines)
        ├── sensor.c                 # 센서 관리 (640 lines)
        ├── data_manager.c           # 데이터 관리 (237 lines)
        ├── i2c.c                    # I2C 통신 (115 lines)
        ├── dfplayer_mini.c          # 음성 재생 (272 lines)
        └── mic_i2s.c               # I2S 마이크 (121 lines)
```

---

## Ⅳ. 시스템 원리(SW/HW)

---

## 1. safelink_band 프로젝트

### 프로젝트 구조
```
firmware/safelink_band/
├── main/
│   └── main.c (1,208 lines) - 메인 애플리케이션
├── CMakeLists.txt
├── sdkconfig
└── README.md
```

### 주요 파일 분석

#### main.c
**파일 크기**: 1,208 lines  
**주요 기능**: 센서 데이터 수집, BLE 광고, WiFi UDP 통신, 심박수 측정

| 함수명 | 라인 | 기능 설명 |
|--------|------|-----------|
| `app_main()` | 250-272 | 메인 애플리케이션 진입점, NimBLE 및 WiFi 초기화 |
| `nimble_init()` | 152-185 | NimBLE 스택 초기화 및 NVS 설정 |
| `ble_app_on_sync()` | 188-232 | BLE 동기화 완료 후 센서 초기화 |
| `sensor_beacon_task()` | 334-425 | 센서 데이터 수집 및 BLE 광고 업데이트 |
| `heart_rate_sampling_task()` | 581-672 | ADC를 이용한 심박수 측정 |
| `calculate_heart_rate_from_buffer()` | 675-871 | 심박수 계산 알고리즘 (피크 검출, 필터링) |
| `wifi_udp_task()` | 1169-1208 | WiFi 연결 및 UDP 데이터 전송 |
| `i2c_master_init()` | 428-440 | I2C 통신 초기화 |
| `aht20_read_data()` | 478-489 | AHT20 온습도 센서 데이터 읽기 |
| `mcp9808_read_temperature()` | 510-522 | MCP9808 온도 센서 데이터 읽기 |

**주요 센서**:
- AHT20: 온습도 센서 (I2C 주소: 0x38)
- MCP9808: 온도 센서 (I2C 주소: 0x18)  
- ADC4: 심박수 측정 (GPIO4)

**BLE 광고 데이터 구조**:
- Company ID: 0x02E5 (Espressif)
- 13바이트 페이로드: 온도, 습도, 체온, 심박수, SpO2

---

## 2. safelink_hub_c3 프로젝트

### 프로젝트 구조
```
firmware/safelink_hub_c3/main/
├── safelink_wearable_hub.c (278 lines) - 메인 애플리케이션
├── bluetooth.c (1,462 lines) - BLE 통신 관리
├── bluetooth.h (102 lines) - BLE 헤더
├── sensor.c (640 lines) - 센서 관리
├── sensor.h (149 lines) - 센서 헤더
├── data_manager.c (237 lines) - 데이터 관리
├── data_manager.h (89 lines) - 데이터 관리 헤더
├── i2c.h (24 lines) - I2C 헤더
├── dfplayer_mini.h (393 lines) - DFPlayer 헤더
└── mic_i2s.h (32 lines) - I2S 마이크 헤더
```

### 주요 파일 분석

#### safelink_wearable_hub.c
**파일 크기**: 278 lines  
**주요 기능**: 메인 애플리케이션, 태스크 관리, GPIO 제어

| 함수명 | 라인 | 기능 설명 |
|--------|------|-----------|
| `app_main()` | 190-277 | 메인 애플리케이션, 모든 모듈 초기화 |
| `gpio_task()` | 80-123 | GPIO7 모니터링 및 딥슬립 제어 |
| `bluetooth_task()` | 128-173 | BLE 통신 태스크 |
| `terminal_task()` | 176-186 | 데이터 출력 태스크 |
| `enter_deep_sleep()` | 64-77 | 딥슬립 진입 |

#### bluetooth.c
**파일 크기**: 1,462 lines  
**주요 기능**: BLE 서버/클라이언트, GATT 서비스, 스캔 관리

| 함수명 | 라인 | 기능 설명 |
|--------|------|-----------|
| `bluetooth_init()` | 769-846 | BLE 스택 초기화 및 GATT 서비스 등록 |
| `gap_event_cb()` | 439-699 | BLE GAP 이벤트 처리 (스캔, 연결, 해제) |
| `gatt_svr_access_cb()` | 77-351 | GATT 서버 접근 콜백 (읽기/쓰기) |
| `start_advertising()` | 703-758 | BLE 광고 시작 |
| `start_passive_scan()` | 849-869 | BLE 스캔 시작 |
| `parse_mfg_and_update_band()` | 979-1010 | 밴드 광고 데이터 파싱 |
| `bluetooth_update_hub_data_characteristics()` | 1257-1304 | 허브 데이터 특성 업데이트 |

**GATT 서비스**:
- Hub Aggregation Service (0xFFF0): 통합 데이터
- Health Sensor Service (0x1810): 건강 센서 데이터
- Noise Sensor Service (0x1811): 소음 센서 데이터
- Control Service (0x1812): 제어 명령

#### sensor.c
**파일 크기**: 640 lines  
**주요 기능**: 센서 데이터 수집, 경고 시스템, WBGT 계산

| 함수명 | 라인 | 기능 설명 |
|--------|------|-----------|
| `sensor_init()` | 274-305 | 센서 시스템 초기화 |
| `temp_humidity_task()` | 57-109 | 온습도 센서 태스크 |
| `sensor_monitor_task()` | 130-271 | 센서 모니터링 및 경고 체크 |
| `calc_wbgt()` | 112-127 | WBGT(건구습구온도) 계산 |
| `warning_system_check_and_trigger()` | 541-629 | 경고 시스템 체크 및 트리거 |
| `warning_system_play_voice()` | 495-519 | 음성 경고 재생 |
| `analyze_health_status()` | 423-441 | 건강 상태 분석 |

**경고 시스템**:
- WBGT 경고: 28°C 이상
- 체온 경고: 37.5°C 이상
- 심박수 경고: 120 BPM 이상
- 소음 경고: 95dB 이상

#### data_manager.c
**파일 크기**: 237 lines  
**주요 기능**: 데이터 저장, 검증, 출력

| 함수명 | 라인 | 기능 설명 |
|--------|------|-----------|
| `data_manager_init()` | 19-39 | 데이터 매니저 초기화 |
| `data_manager_update_band_data()` | 53-87 | 밴드 데이터 업데이트 |
| `data_manager_get_band_data()` | 89-102 | 밴드 데이터 조회 |
| `data_manager_update_hub_data()` | 129-146 | 허브 데이터 업데이트 |
| `data_manager_print_all_data()` | 164-220 | 모든 데이터 출력 |
| `data_manager_validate_band_data()` | 104-127 | 밴드 데이터 검증 |

**데이터 구조**:
- `band_data_t`: 밴드로부터 받는 데이터 (온도, 습도, 체온, 심박수, SpO2)
- `hub_data_t`: 허브에서 수집하는 데이터 (소음, WBGT, 경보상태)

---

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
| 주의 | 1 | 경미한 위험 |
| 위험 | 2 | 중간 위험 |
| 심각 | 3 | 심각한 위험 |

#### 경고 조건
| 종류 | 주의 | 위험 | 심각 |
|------|------|------|------|
| WBGT | 28°C | 30°C | - |
| 체온 | 37.5°C | 38°C | - |
| 심박수 | 120 BPM | 140 BPM | - |
| 소음 | 95dB | 110dB | 120dB |

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

---

## 5. 성능 최적화

### 메모리 관리
- **스택 크기**: 태스크별 4KB
- **힙 사용**: 동적 할당 최소화
- **캐시**: GATT 응답 캐싱 (500ms)

### 전력 관리
- **딥슬립**: GPIO7 트리거로 진입
- **스캔 최적화**: 3초 주기 스캔
- **연결 최적화**: 빠른 연결 파라미터

### 통신 최적화
- **BLE 스캔**: 중복 필터링 비활성화
- **UDP 전송**: 5초 주기
- **데이터 압축**: 바이너리 포맷 사용

---

## 6. 보안 고려사항

### 데이터 검증
- **범위 체크**: 센서 데이터 유효성 검증
- **타임스탬프**: 데이터 신선도 확인
- **소스 추적**: 데이터 출처 구분

### 통신 보안
- **BLE 암호화**: 연결 시 자동 암호화
- **데이터 무결성**: 체크섬 검증
- **접근 제어**: GATT 특성별 권한 관리

---

## 7. 개발 및 디버깅

### 로깅 시스템
- **로그 레벨**: ERROR, WARN, INFO, DEBUG
- **태그별 분류**: 모듈별 로그 구분
- **성능 로깅**: 응답 시간 측정

### 테스트 기능
- **시뮬레이터**: 센서 데이터 시뮬레이션
- **디버그 명령**: BLE 명령어 인터페이스
- **상태 모니터링**: 실시간 시스템 상태

---

## 8. 향후 개선 사항

### 기능 개선
- **AI 분석**: 머신러닝 기반 건강 상태 분석
- **클라우드 연동**: IoT 플랫폼 연동
- **배터리 최적화**: 전력 소모 최적화

### 성능 개선
- **멀티스레딩**: 병렬 처리 최적화
- **메모리 최적화**: 메모리 사용량 감소
- **통신 최적화**: 대역폭 효율성 향상

---

*문서 생성일: 2024년*  
*프로젝트 버전: ESP-IDF 5.4*

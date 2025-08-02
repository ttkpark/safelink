# SafeLink Band (ESP32-C3 GATT Client)

ESP32-C3 기반의 웨어러블 밴드로, 센서 데이터를 수집하여 SafeLink Hub(GATT Server)로 전송하는 GATT Client 애플리케이션입니다.

## 주요 기능

### 센서 기능
- **심박수 측정**: ADC를 통한 아날로그 신호 처리 및 심박수 계산
- **온도/습도 센서**: AM2320 I2C 센서를 통한 환경 데이터 수집
- **건강 상태 모니터링**: 실시간 건강 상태 분석 및 경고

### Bluetooth 기능
- **GATT Client**: Hub(GATT Server)에 연결하여 데이터 전송
- **자동 Hub 검색**: 가장 신호가 강한 Hub를 자동으로 선택
- **연결 상태 관리**: 연결 해제 시 자동 재연결

### 시스템 기능
- **다중 태스크**: FreeRTOS 기반 병렬 처리
- **LED 상태 표시**: Hub 연결 상태를 LED로 표시
- **이벤트 기반 처리**: 센서 데이터 준비 시 자동 전송

## 하드웨어 연결

### ESP32-C3 핀 할당
```
I2C 연결:
- SDA: GPIO4
- SCL: GPIO5

심박수 센서:
- ADC 입력: GPIO0 (ADC1_CH0)

LED 상태 표시:
- LED: GPIO2
```

### 센서 연결
```
AM2320 온습도 센서:
- VCC: 3.3V
- GND: GND
- SDA: GPIO4
- SCL: GPIO5

심박수 센서:
- VCC: 3.3V
- GND: GND
- Signal: GPIO0
```

## BLE 서비스

### Hub 검색 기준
- **디바이스 이름**: "ESP32C3_Sensor" (Hub)
- **신호 강도**: RSSI 값이 가장 높은 Hub 선택
- **자동 재연결**: 연결 해제 시 자동으로 Hub 재검색

### 데이터 전송
- **심박수**: 실시간 심박수 데이터 (BPM)
- **온도**: 섭씨 온도 (0.1°C 정밀도)
- **습도**: 상대 습도 (0.1% 정밀도)
- **건강 상태**: 분석된 건강 상태 정보

## 빌드 및 플래시

### 환경 설정
```bash
# ESP-IDF 환경 설정
C:\Espressif\frameworks\esp-idf-v5.5\export.ps1
```

### 빌드
```bash
# 프로젝트 디렉토리로 이동
cd firmware/safelink_band

# 빌드
idf.py build
```

### 플래시 및 모니터링
```bash
# 플래시 (COM 포트는 실제 사용 중인 포트로 변경)
idf.py -p COM14 flash

# 플래시 및 모니터링
idf.py -p COM14 flash monitor
```

## 설정

### sdkconfig.defaults 주요 설정
```
# NimBLE GATT Client 설정
CONFIG_BT_NIMBLE_ROLE_CENTRAL=y
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=n

# 디바이스 이름
CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME="ESP32C3_Band"

# 스캔 설정
CONFIG_BT_NIMBLE_BLE_GATT_BLOB_TRANSFER=y
```

## 동작 방식

### 1. 초기화 단계
1. NVS, I2C, ADC, Bluetooth 모듈 초기화
2. 심박수 측정 시작
3. Hub 검색 시작

### 2. 센서 데이터 수집
- **심박수**: 1초마다 측정
- **온습도**: 5초마다 측정
- **건강 상태**: 실시간 분석

### 3. Hub 연결 및 데이터 전송
- **Hub 검색**: "ESP32C3_Sensor" 이름의 디바이스 검색
- **연결**: 가장 신호가 강한 Hub에 연결
- **데이터 전송**: 2초마다 센서 데이터 전송

### 4. 연결 상태 관리
- **LED 표시**: 
  - 연결됨: 빠른 깜빡임 (200ms)
  - 연결 안됨: 느린 깜빡임 (1000ms)
- **자동 재연결**: 연결 해제 시 자동 Hub 재검색

## 로그 메시지

### 초기화 로그
```
I (xxx) SAFELINK_BAND: Starting ESP32C3 Band Application (GATT Client)
I (xxx) SAFELINK_BAND: === INITIALIZING MODULES ===
I (xxx) I2C: Initializing I2C master...
I (xxx) ANALOG: Initializing analog module
I (xxx) NIMBLE_CLIENT: Initializing NimBLE Client...
I (xxx) SAFELINK_BAND: === ALL MODULES INITIALIZED ===
```

### Hub 검색 로그
```
I (xxx) NIMBLE_CLIENT: Starting BLE scan for hubs...
I (xxx) NIMBLE_CLIENT: Found device: ESP32C3_Sensor (RSSI: -45)
I (xxx) NIMBLE_CLIENT: === TARGET HUB FOUND ===
I (xxx) NIMBLE_CLIENT: Name: ESP32C3_Sensor
I (xxx) NIMBLE_CLIENT: Address: xx:xx:xx:xx:xx:xx
I (xxx) NIMBLE_CLIENT: RSSI: -45
I (xxx) NIMBLE_CLIENT: === END HUB INFO ===
```

### 연결 로그
```
I (xxx) NIMBLE_CLIENT: === CONNECTED TO HUB ===
I (xxx) NIMBLE_CLIENT: Connection handle: 0
I (xxx) NIMBLE_CLIENT: === END CONNECTION INFO ===
```

### 센서 데이터 로그
```
I (xxx) ANALOG: Signal: min=2048, max=2156, range=108, peaks=5, quality=85%
I (xxx) SAFELINK_BAND: Heart Rate: 72 BPM (Quality: 85%)
I (xxx) SAFELINK_BAND: Temperature: 23.5°C, Humidity: 45.2%
I (xxx) SAFELINK_BAND: === SENSOR DATA SENT TO HUB ===
I (xxx) SAFELINK_BAND: Heart Rate: 72 BPM
I (xxx) SAFELINK_BAND: Temperature: 23.5°C
I (xxx) SAFELINK_BAND: Humidity: 45.2%
I (xxx) SAFELINK_BAND: Health Status: Normal
I (xxx) SAFELINK_BAND: Timestamp: 12345 ms
I (xxx) SAFELINK_BAND: === END SENSOR DATA ===
```

## 파일 구조

```
firmware/safelink_band/
├── main/
│   ├── safelink_band.c      # 메인 애플리케이션
│   ├── bluetooth.h/c        # GATT Client BLE 기능
│   ├── i2c.h/c              # I2C 통신 (AM2320)
│   ├── analog.h/c           # 아날로그 처리 (심박수)
│   └── CMakeLists.txt       # 빌드 설정
├── sdkconfig.defaults       # ESP32-C3용 NimBLE 설정
└── README.md                # 이 파일
```

## 문제 해결

### Hub 연결 안됨
1. Hub가 "ESP32C3_Sensor" 이름으로 설정되어 있는지 확인
2. Hub가 BLE 광고를 하고 있는지 확인
3. 두 디바이스 간의 거리가 너무 멀지 않은지 확인

### 센서 데이터 없음
1. I2C 연결 확인 (SDA: GPIO4, SCL: GPIO5)
2. 심박수 센서 연결 확인 (GPIO0)
3. 센서 전원 공급 확인

### 빌드 오류
1. ESP-IDF 환경이 올바르게 설정되었는지 확인
2. `idf.py set-target esp32c3` 실행
3. `idf.py clean` 후 재빌드

## 개발 정보

- **타겟**: ESP32-C3
- **프레임워크**: ESP-IDF v5.5
- **Bluetooth**: NimBLE (GATT Client)
- **RTOS**: FreeRTOS
- **센서**: AM2320 (I2C), 아날로그 심박수 센서 
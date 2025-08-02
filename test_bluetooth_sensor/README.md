# ESP32C6 BLE GATT Server

This project implements a BLE (Bluetooth Low Energy) GATT server using ESP32C6. It transmits heart rate sensor and temperature/humidity sensor data via BLE.

## Features

- **BLE GATT Server**: A Bluetooth Low Energy server that other devices can connect to
- **Heart Rate Monitoring**: Real-time heart rate measurement using analog heart rate sensor
- **Temperature/Humidity Sensing**: Temperature and humidity measurement using I2C AM2320 sensor
- **Data Transmission**: Real-time sensor data transmission to connected devices

## Hardware Requirements

- **ESP32C6 Development Board** (ESP32C6 supports BLE only)
- Heart rate sensor (analog)
- AM2320 temperature/humidity sensor (I2C)
- Connection cables

## Build and Run

### 1. ESP-IDF Environment Setup

```bash
# ESP-IDF environment setup
. $HOME/esp/esp-idf/export.sh
```

### 2. Project Build

```bash
# Navigate to project directory
cd test_bluetooth_sensor

# Build
idf.py build
```

### 3. Flash and Monitor

```bash
# Flash to ESP32C6
idf.py flash

# Serial monitoring
idf.py monitor
```

## Usage

### 1. BLE Connection

1. When ESP32C6 runs, it appears as "ESP32_BLE" BLE device.
2. Open Bluetooth settings on your phone or computer and search for "ESP32_BLE".
3. Pair and connect to the device.

### 2. Data Format

When connected, ESP32C6 transmits data in the following format:

```
HR:75,QUAL:85,TEMP:25,HUM:60
```

- `HR`: Heart Rate (BPM)
- `QUAL`: Signal Quality (%)
- `TEMP`: Temperature (°C)
- `HUM`: Humidity (%)

### 3. BLE Apps

You can use BLE apps on your phone to receive data:

- **Android**: "nRF Connect" app
- **iOS**: "LightBlue" app

## 프로젝트 구조

```
test_bluetooth_sensor/
├── main/
│   ├── test_bluetooth_sensor.c  # 메인 애플리케이션
│   ├── bluetooth.c              # 블루투스 SPP 클라이언트 구현
│   ├── bluetooth.h              # 블루투스 헤더 파일
│   ├── analog.c                 # 아날로그 센서 처리
│   ├── analog.h                 # 아날로그 센서 헤더
│   ├── i2c.c                    # I2C 통신
│   ├── i2c.h                    # I2C 헤더
│   └── CMakeLists.txt           # 빌드 설정
├── sdkconfig                    # ESP-IDF 설정
├── sdkconfig.defaults           # 기본 설정
└── README.md                    # 이 파일
```

## 블루투스 API

### 초기화
```c
esp_err_t bluetooth_init(void);
```

### 상태 확인
```c
bluetooth_state_t bluetooth_get_state(void);
```

### 데이터 전송
```c
esp_err_t bluetooth_send_data(const uint8_t *data, size_t len);
```

### 연결 해제
```c
esp_err_t bluetooth_deinit(void);
```

## 블루투스 상태

- `BLUETOOTH_STATE_DISABLED`: 비활성화됨
- `BLUETOOTH_STATE_ENABLED`: 활성화됨 (연결 대기)
- `BLUETOOTH_STATE_CONNECTING`: 연결 중
- `BLUETOOTH_STATE_CONNECTED`: 연결됨
- `BLUETOOTH_STATE_DISCONNECTED`: 연결 해제됨

## 문제 해결

### 블루투스가 보이지 않는 경우
1. ESP32가 정상적으로 부팅되었는지 확인
2. 시리얼 모니터에서 블루투스 초기화 메시지 확인
3. 블루투스 설정에서 디바이스 이름이 "ESP32_SPP_Client"로 표시되는지 확인

### 연결이 안 되는 경우
1. 블루투스 페어링 상태 확인
2. ESP32 재부팅 후 재시도
3. 다른 블루투스 디바이스와의 간섭 확인

### 데이터가 전송되지 않는 경우
1. 블루투스 상태가 `BLUETOOTH_STATE_CONNECTED`인지 확인
2. 센서 데이터가 정상적으로 읽히는지 확인
3. 시리얼 모니터에서 전송 로그 확인

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 
# ESP32-C3 센서 데이터 수집 & GATT Server 데이터 형식 시뮬레이션

ESP32-C3에서 실시간 센서 데이터를 수집하고, GATT Server에 전송할 수 있는 데이터 형식을 콘솔에 출력하는 시스템입니다.

## 🎯 주요 기능

### 실시간 센서 데이터 수집
- **AHT20**: 온도, 습도 (1초마다)
- **MCP9808**: 고정밀 온도 (1초마다)
- **MAX3010x**: 심박수, SpO2, 체온 (2-5초마다)

### GATT Server 데이터 형식 시뮬레이션
- **Health Sensor Service (0x1810)** 데이터 형식 출력
- **실시간 데이터 변환**: 센서 데이터를 GATT Server 형식으로 변환
- **콘솔 출력**: 실제 GATT Server에 전송할 데이터 형식 표시

### 전력 효율성
- 30초 데이터 수집 후 딥슬립
- 10분마다 자동 재부팅

## 📋 하드웨어 요구사항

### ESP32-C3
- ESP32-C3 개발보드
- AHT20 온습도 센서
- MCP9808 온도 센서
- MAX3010x 심박수/SpO2 센서

### 연결
```
ESP32-C3    AHT20      MCP9808    MAX3010x
   SCL   ─── SCL   ─── SCL   ─── SCL
   SDA   ─── SDA   ─── SDA   ─── SDA
   VCC   ─── VCC   ─── VCC   ─── VCC
   GND   ─── GND   ─── GND   ─── GND
```

## 🚀 설치 및 실행

### ESP32-C3 펌웨어 빌드

```bash
# ESP-IDF 환경 설정
C:\Users\GH\esp\v5.4.2\esp-idf\export.ps1

# 프로젝트 빌드
idf.py build

# 펌웨어 업로드
idf.py -p COM_PORT flash monitor
```

## 📡 GATT Server 데이터 형식

### Health Sensor Service (0x1810)

| 특성 | UUID | 설명 | 데이터 형식 |
|------|------|------|-------------|
| Temperature | 0x2A6E | 온도 | 2바이트 (0.01°C 단위) |
| Humidity | 0x2A6F | 습도 | 2바이트 (0.01% 단위) |
| Heart Rate | 0x2A37 | 심박수 | 2바이트 (BPM) |
| Body Temperature | 0x2A1C | 체온 | 2바이트 (0.01°C 단위) |

### 데이터 변환 예시

#### 온도 데이터 변환
```c
// 센서 값: 25.50°C
float temperature = 25.50;
uint16_t temp_value = (uint16_t)(temperature * 100);  // 2550
uint8_t temp_data[2] = {0x9E, 0x09};  // 2550 = 0x09E
```

#### 습도 데이터 변환
```c
// 센서 값: 45.20%
float humidity = 45.20;
uint16_t humidity_value = (uint16_t)(humidity * 100);  // 4520
uint8_t humidity_data[2] = {0xA8, 0x11};  // 4520 = 0x11A8
```

#### 통합 센서 데이터 (8바이트)
```c
uint8_t sensor_data[8] = {
    0x64, 0x00,  // 심박수: 100 BPM
    0x9E, 0x09,  // 온도: 25.50°C
    0xA8, 0x11,  // 습도: 45.20%
    0x00, 0x00   // 예약
};
```

## 📊 데이터 수집 주기

| 센서 | 측정 주기 | 측정 항목 | GATT 특성 |
|------|-----------|-----------|-----------|
| AHT20 | 1초 | 온도, 습도 | 0x2A6E, 0x2A6F |
| MCP9808 | 1초 | 온도 | 0x2A6E |
| MAX3010x | 2초 | 심박수, SpO2 | 0x2A37 |
| MAX3010x | 5초 | 체온 | 0x2A1C |

## 🎯 사용법

### 1. ESP32-C3 실행

1. ESP32-C3에 펌웨어 업로드
2. 시리얼 모니터로 로그 확인
3. GATT Server 데이터 형식 정보 확인

```
=== GATT SERVER SIMULATION READY ===
서버: SafeLink_Test
센서 데이터를 GATT Server 형식으로 콘솔에 출력합니다
=== END SUCCESS INFO ===

=== GATT Server 데이터 형식 정보 ===
서버 디바이스: SafeLink_Test
Health Sensor Service UUID: 00001810-0000-1000-8000-00805f9b34fb
온도 특성 UUID: 00002a6e-0000-1000-8000-00805f9b34fb
습도 특성 UUID: 00002a6f-0000-1000-8000-00805f9b34fb
심박수 특성 UUID: 00002a37-0000-1000-8000-00805f9b34fb
체온 특성 UUID: 00002a1c-0000-1000-8000-00805f9b34fb
=== 데이터 형식 정보 끝 ===
```

### 2. 실시간 데이터 출력 예시

```
실시간 센서 데이터 수집 시작 (30초간)
[0s] AHT20 - 온도: 25.3°C, 습도: 45.2%
[0s] MCP9808 - 온도: 25.1°C
[2s] 심박수: 72.5 BPM (신뢰도: 0.85)
[2s] SpO2: 98.2% (신뢰도: 0.92)

=== GATT Server 데이터 전송 시뮬레이션 ===
[GATT] 온도 데이터 전송: 25.30°C -> 2530 (0x09E)
[GATT] 습도 데이터 전송: 45.20% -> 4520 (0x11A8)
[GATT] 심박수 데이터 전송: 72.5 BPM -> 72 (0x0048)
[GATT] 체온 데이터 전송: 36.80°C -> 3680 (0x0E60)
[GATT] 통합 센서 데이터: 48 00 9E 09 A8 11 00 00
=== GATT Server 데이터 전송 시뮬레이션 완료 ===
```

## 🔧 설정

### ESP32-C3 설정

```c
// 딥슬립 설정
#define DATA_COLLECTION_DURATION_SEC 30   // 30초간 데이터 수집
#define DEEP_SLEEP_DURATION_SEC     600   // 10분 딥슬립

// I2C 설정
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_FREQ_HZ          100000

// GATT Server 설정
#define GATT_SERVER_DEVICE_NAME     "SafeLink_Test"
#define HEALTH_SENSOR_SERVICE_UUID  "00001810-0000-1000-8000-00805f9b34fb"
```

## 📝 로그 예시

### 시작 로그
```
I (1234) SENSOR_SYSTEM: ESP32-C3 SuperMini AHT20 + MCP9808 + MAX3010x 센서 시스템 시작
I (1235) SENSOR_SYSTEM: 실시간 데이터 수집 후 10분마다 자동 재부팅
I (1236) SENSOR_SYSTEM: GATT Server 연결 시뮬레이션 시작
I (1237) SENSOR_SYSTEM: === GATT SERVER SIMULATION READY ===
I (1238) SENSOR_SYSTEM: 서버: SafeLink_Test
I (1239) SENSOR_SYSTEM: 센서 데이터를 GATT Server 형식으로 콘솔에 출력합니다
I (1240) SENSOR_SYSTEM: === END SUCCESS INFO ===
I (1241) SENSOR_SYSTEM: === GATT Server 데이터 형식 정보 ===
I (1242) SENSOR_SYSTEM: 서버 디바이스: SafeLink_Test
I (1243) SENSOR_SYSTEM: Health Sensor Service UUID: 00001810-0000-1000-8000-00805f9b34fb
I (1244) SENSOR_SYSTEM: 온도 특성 UUID: 00002a6e-0000-1000-8000-00805f9b34fb
I (1245) SENSOR_SYSTEM: 습도 특성 UUID: 00002a6f-0000-1000-8000-00805f9b34fb
I (1246) SENSOR_SYSTEM: 심박수 특성 UUID: 00002a37-0000-1000-8000-00805f9b34fb
I (1247) SENSOR_SYSTEM: 체온 특성 UUID: 00002a1c-0000-1000-8000-00805f9b34fb
I (1248) SENSOR_SYSTEM: === 데이터 형식 정보 끝 ===
I (1249) SENSOR_SYSTEM: 실시간 센서 데이터 수집 시작 (30초간)
```

### 센서 데이터 로그
```
I (1250) SENSOR_SYSTEM: [0s] AHT20 - 온도: 25.3°C, 습도: 45.2%
I (1251) SENSOR_SYSTEM: [0s] MCP9808 - 온도: 25.1°C
I (1252) SENSOR_SYSTEM: [2s] 심박수: 72.5 BPM (신뢰도: 0.85)
I (1253) SENSOR_SYSTEM: [2s] SpO2: 98.2% (신뢰도: 0.92)
I (1254) SENSOR_SYSTEM: === GATT Server 데이터 전송 시뮬레이션 ===
I (1255) SENSOR_SYSTEM: [GATT] 온도 데이터 전송: 25.30°C -> 2530 (0x09E)
I (1256) SENSOR_SYSTEM: [GATT] 습도 데이터 전송: 45.20% -> 4520 (0x11A8)
I (1257) SENSOR_SYSTEM: [GATT] 심박수 데이터 전송: 72.5 BPM -> 72 (0x0048)
I (1258) SENSOR_SYSTEM: [GATT] 체온 데이터 전송: 36.80°C -> 3680 (0x0E60)
I (1259) SENSOR_SYSTEM: [GATT] 통합 센서 데이터: 48 00 9E 09 A8 11 00 00
I (1260) SENSOR_SYSTEM: === GATT Server 데이터 전송 시뮬레이션 완료 ===
```

### 종료 로그
```
I (1261) SENSOR_SYSTEM: 데이터 수집 완료 (30초 경과)
I (1262) SENSOR_SYSTEM: GATT Server로 센서 데이터 전송 중...
I (1263) SENSOR_SYSTEM: === GATT Server 데이터 전송 시뮬레이션 ===
I (1264) SENSOR_SYSTEM: [GATT] 온도 데이터 전송: 25.30°C -> 2530 (0x09E)
I (1265) SENSOR_SYSTEM: [GATT] 습도 데이터 전송: 45.20% -> 4520 (0x11A8)
I (1266) SENSOR_SYSTEM: [GATT] 심박수 데이터 전송: 72.5 BPM -> 72 (0x0048)
I (1267) SENSOR_SYSTEM: [GATT] 체온 데이터 전송: 36.80°C -> 3680 (0x0E60)
I (1268) SENSOR_SYSTEM: [GATT] 통합 센서 데이터: 48 00 9E 09 A8 11 00 00
I (1269) SENSOR_SYSTEM: === GATT Server 데이터 전송 시뮬레이션 완료 ===
I (1270) SENSOR_SYSTEM: 딥슬립으로 진입합니다
I (1271) SENSOR_SYSTEM: 다음 측정은 10분 후에 자동으로 시작됩니다
```

## 🔍 문제 해결

### 센서 연결 실패
- I2C 연결 확인
- 센서 주소 확인
- 전원 공급 확인

### 딥슬립 문제
- RTC 메모리 설정 확인
- 타이머 설정 확인

### 데이터 형식 오류
- 센서 데이터 범위 확인
- 변환 공식 확인

## 📈 성능 최적화

- **전력 효율성**: 딥슬립으로 배터리 수명 연장
- **데이터 정확성**: 실시간 센서 데이터 수집
- **형식 표준화**: GATT Server 호환 데이터 형식

## 🛠️ 개발 환경

- **ESP-IDF**: v5.4.2
- **하드웨어**: ESP32-C3, AHT20, MCP9808, MAX3010x

## 🔄 다음 단계

이 시스템은 GATT Server 데이터 형식을 시뮬레이션합니다. 실제 BLE GATT Client 구현을 위해서는:

1. ESP32-C3 BLE 지원 확인
2. GATT Client API 구현
3. 실제 GATT Server 연결
4. 데이터 전송 구현

현재는 콘솔에 GATT Server 형식으로 데이터를 출력하여 실제 구현 시 참고할 수 있도록 구성되어 있습니다.


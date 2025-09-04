# ESP32-C3 BLE GATT Server & Client

ESP32-C3에서 실시간 센서 데이터를 BLE GATT Server로 전송하고, Python 클라이언트로 수신하는 시스템입니다.

## 🎯 주요 기능

### ESP32-C3 (GATT Server)
- **실시간 센서 데이터 수집**
  - AHT20: 온도, 습도
  - MCP9808: 고정밀 온도
  - MAX3010x: 심박수, SpO2, 체온
- **BLE GATT Server**
  - Health Sensor Service (0x1810)
  - 실시간 데이터 전송
  - 자동 재연결 지원
- **전력 효율성**
  - 30초 데이터 수집 후 딥슬립
  - 10분마다 자동 재부팅

### Python Client (GATT Client)
- **자동 디바이스 스캔**
- **실시간 데이터 수신**
- **데이터 시각화**
- **자동 재연결**

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

### 1. ESP32-C3 펌웨어 빌드

```bash
# ESP-IDF 환경 설정
C:\Users\GH\esp\v5.4.2\esp-idf\export.ps1

# 프로젝트 빌드
idf.py build

# 펌웨어 업로드
idf.py -p COM_PORT flash monitor
```

### 2. Python 클라이언트 설치

```bash
# 필요한 패키지 설치
pip install -r requirements.txt

# 클라이언트 실행
python ble_client.py
```

## 📡 BLE 서비스 및 특성

### Health Sensor Service (0x1810)

| 특성 | UUID | 설명 | 데이터 형식 |
|------|------|------|-------------|
| Temperature | 0x2A6E | 온도 | float (4 bytes) |
| Humidity | 0x2A6F | 습도 | float (4 bytes) |
| Heart Rate | 0x2A37 | 심박수 | float (4 bytes) |
| Body Temperature | 0x2A1C | 체온 | float (4 bytes) |

## 🔧 설정

### ESP32-C3 설정

```c
// BLE 설정
#define BLE_DEVICE_NAME             "SafeLink_Test"
#define DATA_COLLECTION_DURATION_SEC 30   // 30초간 데이터 수집
#define DEEP_SLEEP_DURATION_SEC     600   // 10분 딥슬립

// I2C 설정
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           8
#define I2C_MASTER_FREQ_HZ          100000
```

### Python 클라이언트 설정

```python
# BLE 디바이스 설정
DEVICE_NAME = "SafeLink_Test"
SERVICE_UUID = "00001810-0000-1000-8000-00805f9b34fb"
```

## 📊 데이터 수집 주기

| 센서 | 측정 주기 | 측정 항목 |
|------|-----------|-----------|
| AHT20 | 1초 | 온도, 습도 |
| MCP9808 | 1초 | 온도 |
| MAX3010x | 2초 | 심박수, SpO2 |
| MAX3010x | 5초 | 체온 |

## 🎯 사용법

### 1. ESP32-C3 실행

1. ESP32-C3에 펌웨어 업로드
2. 시리얼 모니터로 로그 확인
3. BLE 광고 시작 확인

```
=== BLE ADVERTISING SUCCESS ===
Device should now be visible in nRF Connect
Look for device named: SafeLink_Test
=== END SUCCESS INFO ===
```

### 2. Python 클라이언트 실행

```bash
python ble_client.py
```

출력 예시:
```
🚀 ESP32-C3 BLE GATT Client 시작
==================================================
🔍 'SafeLink_Test' 디바이스를 스캔 중...
✅ 디바이스 발견: SafeLink_Test (XX:XX:XX:XX:XX:XX)
🔗 SafeLink_Test에 연결 중...
✅ 연결 성공!
📱 디바이스: SafeLink_Test
📍 주소: XX:XX:XX:XX:XX:XX

📡 센서 데이터 구독 시작...
✅ 온도 특성 구독 완료
✅ 습도 특성 구독 완료
✅ 심박수 특성 구독 완료
✅ 체온 특성 구독 완료

🎯 실시간 센서 데이터 수신 중... (Ctrl+C로 종료)
--------------------------------------------------
[14:30:15] 🌡️  온도: 25.3°C
[14:30:15] 💧 습도: 45.2%
[14:30:16] ❤️  심박수: 72.5 BPM
[14:30:17] 🔥 체온: 36.8°C
```

## 🔍 문제 해결

### ESP32-C3 문제

1. **BLE 초기화 실패**
   - NVS 플래시 초기화 확인
   - 블루투스 컨트롤러 메모리 해제 확인

2. **센서 연결 실패**
   - I2C 연결 확인
   - 센서 주소 확인
   - 전원 공급 확인

3. **딥슬립 문제**
   - RTC 메모리 설정 확인
   - 타이머 설정 확인

### Python 클라이언트 문제

1. **디바이스 발견 실패**
   - ESP32-C3가 BLE 광고 중인지 확인
   - 블루투스 활성화 확인
   - 디바이스 이름 확인

2. **연결 실패**
   - 디바이스가 연결 가능한 상태인지 확인
   - 블루투스 권한 확인

3. **데이터 수신 실패**
   - 특성 UUID 확인
   - 데이터 형식 확인

## 📝 로그 예시

### ESP32-C3 로그
```
I (1234) SENSOR_SYSTEM: ESP32-C3 SuperMini AHT20 + MCP9808 + MAX3010x 센서 시스템 시작
I (1235) SENSOR_SYSTEM: 실시간 데이터 수집 후 10분마다 자동 재부팅
I (1236) SENSOR_SYSTEM: BLE GATT Server 초기화 시작
I (1237) SENSOR_SYSTEM: BLE 초기화 완료
I (1238) SENSOR_SYSTEM: === BLE ADVERTISING SUCCESS ===
I (1239) SENSOR_SYSTEM: Device should now be visible in nRF Connect
I (1240) SENSOR_SYSTEM: Look for device named: SafeLink_Test
I (1241) SENSOR_SYSTEM: === END SUCCESS INFO ===
I (1242) SENSOR_SYSTEM: 실시간 센서 데이터 수집 시작 (30초간)
I (1243) SENSOR_SYSTEM: [0s] AHT20 - 온도: 25.3°C, 습도: 45.2%
I (1244) SENSOR_SYSTEM: [0s] MCP9808 - 온도: 25.1°C
I (1245) SENSOR_SYSTEM: [2s] 심박수: 72.5 BPM (신뢰도: 0.85)
I (1246) SENSOR_SYSTEM: [2s] SpO2: 98.2% (신뢰도: 0.92)
I (1247) SENSOR_SYSTEM: ESP_GATTS_CONNECT_EVT, conn_id = 0
I (1248) SENSOR_SYSTEM: BLE로 센서 데이터 전송 중...
I (1249) SENSOR_SYSTEM: 데이터 수집 완료 (30초 경과)
I (1250) SENSOR_SYSTEM: 딥슬립으로 진입합니다
I (1251) SENSOR_SYSTEM: 다음 측정은 10분 후에 자동으로 시작됩니다
```

## 🔄 자동 재연결

- ESP32-C3: 클라이언트 연결 해제 시 자동으로 광고 재시작
- Python 클라이언트: 연결 실패 시 5초마다 재시도

## 📈 성능 최적화

- **전력 효율성**: 딥슬립으로 배터리 수명 연장
- **데이터 전송**: 실시간 알림으로 즉시 데이터 전송
- **연결 안정성**: 자동 재연결 및 오류 처리

## 🛠️ 개발 환경

- **ESP-IDF**: v5.4.2
- **Python**: 3.8+
- **BLE 라이브러리**: bleak
- **하드웨어**: ESP32-C3, AHT20, MCP9808, MAX3010x


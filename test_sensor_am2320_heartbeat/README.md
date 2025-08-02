# SafeLink 센서 SPP 서버 (AM2320 + 심박 센서)

ESP32-C6 개발보드를 사용하여 AM2320 온습도 센서와 아날로그 심박 센서를 SPP(Serial Port Protocol) 서버로 동작시키는 프로젝트입니다. 마스터 디바이스(PC, 스마트폰 등)의 SPP 클라이언트가 연결 요청을 보내면 센서 데이터를 전송합니다.

## 하드웨어 구성

### ESP32-C6 핀 연결

#### AM2320 온습도 센서 (I2C)
- **SCL**: GPIO9
- **SDA**: GPIO8
- **VCC**: 3.3V
- **GND**: GND

#### 아날로그 심박 센서 (ADC)
- **Signal**: GPIO4 (ADC1_CH4)
- **VCC**: 3.3V
- **GND**: GND

## 기능

### SPP 서버
- 블루투스 Classic SPP 서버 모드
- 마스터 디바이스의 연결 요청 대기
- 연결 해제 시 자동으로 다시 대기 모드로 복귀
- 연결 상태에 따른 데이터 전송 제어

### AM2320 온습도 센서
- I2C 통신을 통한 온도 및 습도 측정
- 10초마다 데이터 읽기
- 에러 발생 시 자동 재초기화
- 온도 범위: -40°C ~ 80°C
- 습도 범위: 0% ~ 100%

### 아날로그 심박 센서
- 100Hz 샘플링으로 아날로그 신호 측정
- 임계값 기반 심박 감지
- 실시간 심박수(BPM) 계산
- 12비트 ADC 해상도 사용

### SPP 데이터 전송
- SPP 클라이언트 연결 시에만 데이터 전송
- JSON 형식으로 센서 데이터 전송
- 10초마다 자동 전송
- 데이터 형식: `SENSOR_DATA:{"temperature":25.3,"humidity":45.2,"heart_rate":72.0,"timestamp":1234567890}`

## 동작 방식

1. **초기화 단계**
   - I2C, ADC, 블루투스 Classic, SPP 초기화
   - 센서 연결 상태 확인
   - SPP 서버 모드 시작

2. **대기 단계**
   - 마스터 디바이스의 연결 요청 대기
   - 센서 데이터 수집 (연결 없이도 계속)
   - 로그로 센서 상태 출력

3. **연결 단계**
   - 마스터 디바이스 연결 감지
   - 센서 데이터 수집 및 전송 시작
   - 10초마다 자동 전송

4. **재대기 단계**
   - 연결 해제 감지
   - 서버 모드로 복귀
   - 새로운 연결 요청 대기

## 빌드 및 실행

### 1. 환경 설정
```bash
# ESP-IDF 환경 설정
. $HOME/esp/esp-idf/export.sh
```

### 2. 프로젝트 빌드
```bash
cd test_sensor_am2320_heartbeat
idf.py build
```

### 3. 펌웨어 업로드
```bash
idf.py -p [PORT] flash monitor
```

## 출력 예시

### 초기화 단계
```
I (1234) SENSOR_SPP_SERVER: SafeLink Sensor SPP Server Starting...
I (1235) SENSOR_SPP_SERVER: I2C master initialized successfully
I (1236) SENSOR_SPP_SERVER: AM2320 sensor initialized
I (1237) SENSOR_SPP_SERVER: ADC1 initialized successfully
I (1238) SENSOR_SPP_SERVER: Bluetooth initialized successfully
I (1239) SENSOR_SPP_SERVER: SPP initialized successfully
I (1240) SENSOR_SPP_SERVER: All tasks created successfully
I (1241) SENSOR_SPP_SERVER: SPP Server: Waiting for client connection...
```

### 대기 단계
```
I (1242) SENSOR_SPP_SERVER: SPP server started, waiting for client connection...
I (1243) SENSOR_SPP_SERVER: Waiting for SPP client connection... Temperature: 25.3°C, Humidity: 45.2%, Heart Rate: 0.0 BPM
I (1244) SENSOR_SPP_SERVER: Heartbeat detected! Count: 1
```

### 연결 단계
```
I (1245) SENSOR_SPP_SERVER: SPP client connected
I (1246) SENSOR_SPP_SERVER: SPP Connected - Temperature: 25.3°C, Humidity: 45.2%, Heart Rate: 72.0 BPM
I (1247) SENSOR_SPP_SERVER: Sent via SPP: SENSOR_DATA:{"temperature":25.3,"humidity":45.2,"heart_rate":72.0,"timestamp":1234567890}
```

## SPP 클라이언트 설정 (마스터 디바이스)

### PC에서 SPP 클라이언트 실행
1. **Windows**: 블루투스 설정에서 "SafeLink_Sensor_SPP" 서비스 검색 및 연결
2. **Linux**: `bluetoothctl` 명령어로 SPP 클라이언트 연결
3. **Android**: SPP 클라이언트 앱 설치 및 실행

### SPP 클라이언트 앱 예시 (Android)
```java
// Android SPP 클라이언트 예시 코드
BluetoothDevice device = bluetoothAdapter.getRemoteDevice("ESP32_MAC_ADDRESS");
BluetoothSocket socket = device.createRfcommSocketToServiceRecord(UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"));
socket.connect();
// 데이터 수신 처리
```

## 설정 조정

### SPP 설정
- `SPP_SERVER_NAME`: 서버 이름 (기본: "SafeLink_Sensor_SPP")
- 연결 해제 시 자동으로 서버 모드로 복귀
- 데이터 전송 주기: `sensor_data_task`에서 `vTaskDelay` 수정

### 센서 설정
- `AM2320_I2C_MASTER_FREQ_HZ`: I2C 통신 속도 (기본: 50kHz)
- `HEARTBEAT_THRESHOLD`: 심박 감지 임계값 (기본: 2000)
- 데이터 전송 주기: `sensor_data_task`에서 `vTaskDelay` 수정

## 문제 해결

### SPP 서버가 시작되지 않는 경우
1. 블루투스 Classic 지원 확인
2. 블루투스 초기화 로그 확인
3. SPP 서비스 이름 확인
4. ESP32-C6 블루투스 설정 확인

### 클라이언트 연결이 안 되는 경우
1. 마스터 디바이스의 블루투스 설정 확인
2. 서버 이름 "SafeLink_Sensor_SPP" 검색 확인
3. 페어링 상태 확인
4. 연결 로그 확인

### 센서 데이터가 전송되지 않는 경우
1. SPP 연결 상태 확인
2. 마스터 디바이스에서 데이터 수신 확인
3. 전송 로그 확인
4. 연결 해제 및 재연결 시도

### AM2320 센서 문제
1. I2C 연결 확인 (SCL: GPIO9, SDA: GPIO8)
2. 전원 공급 확인 (3.3V)
3. I2C 스캔 결과 확인
4. 센서 웨이크업 명령 확인

### 심박 센서 문제
1. ADC 연결 확인 (GPIO4)
2. 센서 전원 공급 확인
3. 임계값 조정 (`HEARTBEAT_THRESHOLD`)
4. 센서 위치 조정

## 주의사항

- ESP32-C6는 블루투스 Classic을 지원합니다
- SPP 클라이언트 연결 시에만 센서 데이터 전송이 활성화됩니다
- 센서 데이터는 연결 상태와 관계없이 계속 수집됩니다
- SPP 클라이언트 연결 해제 시 자동으로 서버 모드로 복귀합니다
- 마스터 디바이스에서 SPP 클라이언트로 연결 요청을 보내야 합니다

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다. 
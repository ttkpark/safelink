# ESP-PICO 센서 테스트 프로젝트 (AM2320 + 심박 센서)

ESP-PICO 개발보드를 사용하여 AM2320 온습도 센서와 아날로그 심박 센서를 테스트하는 프로젝트입니다.

## 하드웨어 구성

### ESP-PICO 핀 연결

#### AM2320 온습도 센서 (I2C)
- **SCL**: GPIO8
- **SDA**: GPIO10
- **VCC**: 3.3V
- **GND**: GND

#### 아날로그 심박 센서 (ADC)
- **Signal**: GPIO4 (ADC1_CH4)
- **VCC**: 3.3V
- **GND**: GND

## 기능

### AM2320 온습도 센서
- I2C 통신을 통한 온도 및 습도 측정
- 2초마다 데이터 읽기
- 에러 발생 시 자동 재초기화
- 온도 범위: -40°C ~ 80°C
- 습도 범위: 0% ~ 100%

### 아날로그 심박 센서
- 100Hz 샘플링으로 아날로그 신호 측정
- 임계값 기반 심박 감지
- 실시간 심박수(BPM) 계산
- 12비트 ADC 해상도 사용

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

```
I (1234) SENSOR_TEST: ESP-PICO Sensor Test (AM2320 + Heartbeat) Starting...
I (1235) SENSOR_TEST: I2C master initialized successfully
I (1236) SENSOR_TEST: AM2320 sensor detected
I (1237) SENSOR_TEST: ADC initialized successfully
I (1238) SENSOR_TEST: All tasks created successfully
I (1239) SENSOR_TEST: AM2320: SCL=GPIO8, SDA=GPIO10
I (1240) SENSOR_TEST: Heartbeat: ADC1_CH4 (GPIO4)
I (1241) SENSOR_TEST: AM2320 sensor task started
I (1242) SENSOR_TEST: Heartbeat sensor task started
I (3443) SENSOR_TEST: Temperature: 25.3°C, Humidity: 45.2%
I (4444) SENSOR_TEST: Heart Rate: 72.0 BPM, Total Count: 12
I (5445) SENSOR_TEST: Heartbeat detected! Count: 13
```

## 설정 조정

### AM2320 센서 설정
- `AM2320_I2C_MASTER_FREQ_HZ`: I2C 통신 속도 (기본: 100kHz)
- 측정 주기: `am2320_task`에서 `vTaskDelay(pdMS_TO_TICKS(2000))` 수정

### 심박 센서 설정
- `HEARTBEAT_THRESHOLD`: 심박 감지 임계값 (기본: 2000)
- `HEARTBEAT_SAMPLE_RATE`: 샘플링 주파수 (기본: 100Hz)
- 심박수 계산 주기: `heartbeat_task`에서 수정

## 문제 해결

### AM2320 센서가 감지되지 않는 경우
1. I2C 연결 확인 (SCL, SDA 핀)
2. 전원 공급 확인 (3.3V)
3. 풀업 저항 확인
4. I2C 주소 확인 (0x5C)

### 심박 센서 신호가 없는 경우
1. ADC 연결 확인 (GPIO4)
2. 센서 전원 공급 확인
3. 임계값 조정 (`HEARTBEAT_THRESHOLD`)
4. 센서 위치 조정

## 주의사항

- AM2320 센서는 읽기 전에 웨이크업 명령이 필요합니다
- 심박 센서는 정확한 위치에 부착해야 정확한 측정이 가능합니다
- ADC 값은 환경에 따라 조정이 필요할 수 있습니다

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다. 
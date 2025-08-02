# SafeLink Wearable Hub - ESP32-C3

ESP32-C3 기반 웨어러블 허브 프로젝트입니다. 심박수, 온도, 습도 센서 데이터를 수집하고 Bluetooth Low Energy를 통해 전송합니다.

## 기능

- **심박수 측정**: ADC를 통한 아날로그 신호 처리
- **온도/습도 센서**: AM2320 I2C 센서 지원
- **Bluetooth Low Energy**: NimBLE 스택을 사용한 BLE 통신
- **건강 상태 모니터링**: 실시간 건강 상태 분석 및 경고
- **다중 태스크**: FreeRTOS 기반 병렬 처리

## 하드웨어 연결

### ESP32-C3 핀 할당

| 기능 | 핀 | 설명 |
|------|----|----|
| I2C SDA | GPIO4 | AM2320 센서 데이터 라인 |
| I2C SCL | GPIO5 | AM2320 센서 클럭 라인 |
| 심박수 센서 | GPIO0 (ADC1_CH0) | 아날로그 심박수 신호 입력 |

### 센서 연결

#### AM2320 온습도 센서
- VCC → 3.3V
- GND → GND
- SDA → GPIO4
- SCL → GPIO5

#### 심박수 센서
- VCC → 3.3V
- GND → GND
- Signal → GPIO0 (ADC 입력)

## BLE 서비스

### 표준 BLE 서비스
- **Heart Rate Service** (0x180D)
  - Heart Rate Measurement (0x2A37)
- **Health Thermometer Service** (0x1809)
  - Temperature Measurement (0x2A6E)
  - Temperature Type (0x2A1D)
- **Environmental Sensing Service** (0x181A)
  - Humidity (0x2A6F)
  - Pressure (0x2A6D)

### 커스텀 서비스
- **Custom Sensor Service** (0x1810)
  - Sensor Data (0x2A6F)
  - Health Status (0x2A70)

## 빌드 및 플래시

### 환경 설정
```bash
# ESP-IDF 환경 설정
C:\Espressif\frameworks\esp-idf-v5.5\export.ps1
```

### 빌드
```bash
# 프로젝트 디렉토리로 이동
cd firmware/safelink_wearable_hub

# 빌드
idf.py build
```

### 플래시 및 모니터링
```bash
# 플래시 (COM 포트는 실제 사용하는 포트로 변경)
idf.py -p COM14 flash monitor
```

## 설정

### sdkconfig.defaults
- NimBLE 활성화
- BLE 디바이스 이름: "ESP32C3_Sensor"
- 로깅 레벨 설정
- 메모리 할당 모드 설정

### 주요 설정 항목
```
CONFIG_BT_NIMBLE_ENABLED=y
CONFIG_BT_NIMBLE_ROLE_PERIPHERAL=y
CONFIG_BT_NIMBLE_SVC_GAP_DEVICE_NAME="ESP32C3_Sensor"
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
```

## 테스트

### BLE 연결 테스트
1. nRF Connect 앱 설치
2. "ESP32C3_Sensor" 디바이스 스캔
3. 연결 후 서비스 및 특성 확인
4. 실시간 데이터 모니터링

### 센서 테스트
- 심박수: GPIO0에 아날로그 신호 연결
- 온습도: AM2320 센서 I2C 연결 확인

## 로그 메시지

### 주요 로그 태그
- `MAIN`: 메인 애플리케이션
- `NIMBLE_BLE`: Bluetooth 관련
- `I2C`: I2C 통신
- `ANALOG`: 아날로그 신호 처리

### 건강 상태 경고
- `⚠️ WARNING`: 건강 상태 이상 감지
- `🚨 CRITICAL`: 위험한 건강 상태

## 파일 구조

```
safelink_wearable_hub/
├── main/
│   ├── safelink_wearable_hub.c  # 메인 애플리케이션
│   ├── bluetooth.h/c            # BLE 관련
│   ├── i2c.h/c                  # I2C 통신
│   ├── analog.h/c               # 아날로그 처리
│   └── CMakeLists.txt           # 빌드 설정
├── sdkconfig.defaults           # 기본 설정
└── README.md                    # 이 파일
```

## 문제 해결

### 빌드 오류
- ESP-IDF 환경이 제대로 설정되었는지 확인
- 모든 의존성이 설치되었는지 확인

### BLE 연결 문제
- 디바이스가 스캔되는지 확인
- nRF Connect 앱에서 서비스 확인
- 로그에서 BLE 초기화 상태 확인

### 센서 문제
- I2C 연결 확인 (SDA, SCL)
- 전원 공급 확인 (3.3V)
- 센서 주소 확인 (AM2320: 0x5C)

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다. 
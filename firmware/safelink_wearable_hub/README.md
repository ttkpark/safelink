# SafeLink Wearable Hub - Audio Test System

ESP32-C6 기반 오디오 테스트 시스템입니다. DFP Player와 INMP441 마이크 모듈을 테스트하는 프로그램입니다.

## 기능

- **DFP Player 테스트**: I2S를 통한 오디오 재생 테스트
- **INMP441 마이크 테스트**: 실시간 오디오 입력 및 음량 분석
- **시리얼 통신**: UART를 통한 명령어 기반 제어
- **실시간 음량 계산**: RMS 기반 음량 측정 및 dB 변환
- **다중 태스크**: FreeRTOS 기반 병렬 처리

## 기능

- **심박수 측정**: ADC를 통한 아날로그 신호 처리
- **온도/습도 센서**: AM2320 I2C 센서 지원
- **Bluetooth Low Energy**: NimBLE 스택을 사용한 BLE 통신
- **건강 상태 모니터링**: 실시간 건강 상태 분석 및 경고
- **다중 태스크**: FreeRTOS 기반 병렬 처리

## 하드웨어 연결

### ESP32-C6 핀 할당

| 기능 | 핀 | 설명 |
|------|----|----|
| I2S BCK | GPIO16 | I2S 비트 클럭 (DFP Player + INMP441 공유) |
| I2S WS | GPIO17 | I2S 워드 셀렉트 (DFP Player + INMP441 공유) |
| I2S DO | GPIO21 | I2S 데이터 출력 (DFP Player) |
| I2S DI | GPIO20 | I2S 데이터 입력 (INMP441) |
| UART TX | GPIO43 | 시리얼 통신 TX |
| UART RX | GPIO44 | 시리얼 통신 RX |
| I2C SCL | GPIO8 | I2C 클럭 (DFP Player 제어용) |
| I2C SDA | GPIO10 | I2C 데이터 (DFP Player 제어용) |

### 오디오 모듈 연결

#### DFP Player (I2S Audio Output)
- VCC → 3.3V
- GND → GND
- BCK → GPIO16 (I2S Bit Clock)
- WS → GPIO17 (I2S Word Select)
- DO → GPIO21 (I2S Data Out)
- SCL → GPIO8 (I2C Clock, 제어용)
- SDA → GPIO10 (I2C Data, 제어용)

#### INMP441 마이크 모듈 (I2S Audio Input)
- VCC → 3.3V
- GND → GND
- BCK → GPIO16 (I2S Bit Clock, DFP Player와 공유)
- WS → GPIO17 (I2S Word Select, DFP Player와 공유)
- SD → GPIO20 (I2S Data In)
- SEL → GND (Left Channel 선택)

## 시리얼 명령어

### 사용 가능한 명령어
- **play**: DFP Player 오디오 재생 테스트 시작
- **mic**: INMP441 마이크 테스트 시작
- **stop**: 현재 테스트 중지
- **status**: 시스템 상태 확인
- **help**: 도움말 메뉴 표시

### 오디오 설정
- **샘플링 레이트**: 16kHz
- **비트 깊이**: 16-bit
- **채널**: 모노 (1채널)
- **버퍼 크기**: 1024 샘플

## 빌드 및 플래시

### 환경 설정
```bash
# ESP-IDF 환경 설정 (ESP-IDF v5.4.2)
C:\Users\parkg\esp\v5.4.2\esp-idf\install.ps1
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

### 오디오 재생 테스트
1. 시리얼 모니터 연결 (115200 baud)
2. 'play' 명령어 입력
3. DFP Player에서 1kHz 테스트 톤 재생 확인
4. 로그에서 오디오 데이터 전송 상태 확인

### 마이크 테스트
1. 시리얼 모니터 연결 (115200 baud)
2. 'mic' 명령어 입력
3. INMP441 마이크로 소리 입력
4. 실시간 음량(dB) 및 평균 음량 확인
5. 'status' 명령어로 현재 상태 확인
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
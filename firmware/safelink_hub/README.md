# SafeLink Vest Hub

웨어러블 밴드의 센서 데이터를 수집하고 종합적인 안전 모니터링을 제공하는 허브 디바이스입니다.

## 개요

SafeLink Vest Hub는 다음과 같은 기능을 제공합니다:

1. **BLE 스캐닝**: 웨어러블 밴드의 BLE Advertising 데이터 수신
2. **소음 모니터링**: 환경 소음 실시간 측정 및 분석  
3. **종합 안전 분석**: 생체신호와 환경 데이터를 종합한 위험도 평가
4. **즉시 경고**: LED, 진동, 음성을 통한 다단계 경고 시스템
5. **모바일 연동**: BLE GATT 서버를 통한 앱 연동

## 하드웨어 요구사항

- **MCU**: ESP32-C6
- **센서**: MEMS 마이크로폰 (소음 측정)
- **출력**: RGB LED, 진동 모터, 1W 스피커
- **배터리**: 3000mAh Li-Po 배터리
- **통신**: BLE 5.0, WiFi 6

## 소프트웨어 아키텍처

```
Application Layer
├── Main Application (통합 제어)
├── Safety Monitor (종합 안전 분석)
├── Device Manager (다중 디바이스 관리)
└── Alert Manager (경고 시스템)

Communication Layer
├── BLE Scanner (웨어러블 스캔)
├── BLE Server (앱 통신)
└── WiFi Manager (클라우드 연결)

Hardware Abstraction Layer
├── Audio Manager (소음/음성)
├── Power Manager (전원 관리)
└── Data Manager (데이터 저장)
```

## 주요 컴포넌트

### 1. BLE Scanner
- 웨어러블 밴드의 BLE Advertising 데이터 스캔
- 최대 10개 디바이스 동시 모니터링
- 화이트리스트 기반 신뢰할 수 있는 디바이스 관리

### 2. Audio Manager
- 실시간 소음 측정 (30-130 dB)
- A-weighting 필터링 적용
- 음성 경고 메시지 출력
- 산업안전 기준 준수 (OSHA, IEC)

### 3. Safety Analyzer
- 다중 센서 데이터 융합 분석
- 실시간 위험도 평가
- 한국어 경고 메시지 생성
- 임계치 기반 단계별 알림

### 4. Alert Manager
- LED 색상별 시각적 경고
- PWM 제어 진동 경고
- 다단계 음성 경고
- 위험도별 맞춤 알림

## 빌드 및 플래시

```bash
# ESP32-C6 타겟 설정
idf.py set-target esp32c6

# 빌드
idf.py build

# 플래시 및 모니터링
idf.py flash monitor
```

## 설정

주요 설정은 `sdkconfig.defaults`에서 수정할 수 있습니다:

- BLE 연결 수: `CONFIG_BT_NIMBLE_MAX_CONNECTIONS`
- 로그 레벨: `CONFIG_LOG_DEFAULT_LEVEL_INFO`
- 파티션 테이블: `CONFIG_PARTITION_TABLE_CUSTOM_FILENAME`

## 통신 프로토콜

### BLE Advertising (수신)
웨어러블 밴드에서 다음 데이터를 Advertising으로 전송:
```
Company ID: 0xFFFF (SafeLink)
Data Format: [HR][TEMP][AMB_TEMP][HUM][WBGT]
각 필드는 2바이트 리틀엔디안
```

### BLE GATT (송신)
모바일 앱과의 통신을 위한 GATT 서비스:
- Service UUID: 0x1900
- Hub Status: 0x1A01
- Device List: 0x1A02  
- Safety Alert: 0x1A03
- Configuration: 0x1A04
- Noise Data: 0x1A05

## 안전 기준

### 소음 노출 기준
- **95dB 이상 10분**: 누적 노출시간 4.2% 초과시 경고
- **110dB 이상 10분**: 누적 노출시간 33% 초과시 위험
- **130dB 충격음 100회**: 일일 노출횟수 10% 초과시 위험

### 생체신호 기준
- **심박수**: 120-130 BPM 경고, 130+ BPM 위험
- **체온**: 37.5-38°C 경고, 38°C+ 위험
- **WBGT**: 28-30°C 경고, 30°C+ 위험

## 전력 관리

- **활성 모드**: 모든 기능 동작 (~150mA)
- **절약 모드**: 스캔 주기 감소 (~100mA)  
- **대기 모드**: 최소 기능만 동작 (~50mA)
- **충전 모드**: 충전 중 최적화

목표 배터리 수명: **최소 48시간** (3000mAh 배터리)

## 개발 단계

### Phase 1 (MVP) ✅
- [x] BLE 스캐닝 및 데이터 수신
- [x] 기본 소음 측정
- [x] LED/진동 경고 시스템
- [x] 모바일 앱 BLE 통신

### Phase 2 (계획)
- [ ] 음성 경고 시스템 (TTS)
- [ ] 다중 디바이스 관리 고도화
- [ ] 종합 위험도 분석 알고리즘
- [ ] 확장 포트 지원 (I2C/UART)

### Phase 3 (계획)
- [ ] WiFi 클라우드 연동
- [ ] 고급 소음 분석 (FFT)
- [ ] 머신러닝 위험 예측
- [ ] OTA 업데이트

## 문제 해결

### 빌드 오류
```bash
# 빌드 정리
idf.py clean

# 전체 재빌드
idf.py fullclean
idf.py build
```

### BLE 연결 문제
- 디바이스 MAC 주소가 화이트리스트에 등록되어 있는지 확인
- BLE 스택 로그 활성화: `idf.py menuconfig` → Component config → Bluetooth → Log level

### 소음 측정 문제  
- 마이크 연결 상태 확인
- I2S 핀 설정 확인 (`audio_manager.c`의 GPIO 정의)
- 소음 센서 캘리브레이션 실행

## 라이선스

이 프로젝트는 SafeLink System의 일부입니다. 
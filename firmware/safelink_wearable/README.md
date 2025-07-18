# SafeLink Wearable Device Firmware

SafeLink 웨어러블 디바이스는 작업자의 안전을 실시간으로 모니터링하는 ESP32-C6 기반 IoT 디바이스입니다.

## 주요 기능

### 센서 모니터링
- **심박수 측정**: PPG 센서를 통한 실시간 심박수 모니터링 (1Hz)
- **피부 온도 측정**: 적외선 센서를 통한 체온 모니터링 (1Hz)
- **환경 측정**: 온도/습도 센서를 통한 작업 환경 모니터링 (1Hz)
- **WBGT 계산**: 실시간 열스트레스 지수 계산

### 안전 모니터링
- **체온 임계치**: 37.5°C 경고, 38°C 위험
- **심박수 임계치**: 100-120 BPM 경고, 120-130 BPM 위험
- **WBGT 임계치**: 28-30°C 경고, 30°C 이상 위험
- **실시간 알림**: BLE를 통한 즉시 위험 알림

### 전원 관리
- **배터리 모니터링**: 실시간 배터리 상태 확인
- **저전력 모드**: 딥슬립/라이트슬립 자동 전환
- **7일 배터리 수명**: 최적화된 전력 관리

### 데이터 관리
- **로컬 버퍼링**: 최대 1시간 분량 데이터 저장
- **Flash 저장**: SPIFFS를 통한 데이터 영구 저장
- **BLE 전송**: 10초 간격 데이터 전송

## 하드웨어 요구사항

- **MCU**: ESP32-C6
- **센서**: 심박센서(PPG), 피부온도센서, 온습도센서
- **통신**: BLE 5.0, WiFi 6
- **전원**: 300mAh Li-Po 배터리
- **인터페이스**: I2C, UART, ADC

## 소프트웨어 아키텍처

### 컴포넌트 구조
```
├── components/
│   ├── sensor_manager/     # 센서 데이터 수집 관리
│   ├── ble_service/        # BLE GATT 서비스
│   ├── safety_monitor/     # 안전 임계치 모니터링
│   ├── power_manager/      # 배터리 및 전원 관리
│   └── data_manager/       # 데이터 버퍼링 및 저장
└── main/                   # 메인 애플리케이션
```

### 태스크 구조
- **Main Task**: 전체 시스템 조정 및 상태 관리
- **Sensor Task**: 1Hz 센서 데이터 수집
- **Safety Task**: 안전 이벤트 처리 및 알림
- **Data Task**: 데이터 버퍼링 및 동기화
- **Power Task**: 전원 관리 및 슬립 모드 제어

## 빌드 및 플래시

### 환경 설정
```bash
# ESP-IDF 설치 (v5.0 이상)
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh

# 프로젝트 빌드
cd firmware/safelink_wearable
idf.py set-target esp32c6
idf.py build
```

### 플래시 및 모니터링
```bash
# 디바이스에 플래시
idf.py flash

# 시리얼 모니터링
idf.py monitor

# 플래시 + 모니터링
idf.py flash monitor
```

### 설정
```bash
# 프로젝트 설정
idf.py menuconfig
```

## 센서 데이터 포맷

### BLE GATT 서비스
- **Service UUID**: 0x1800
- **센서 데이터 특성**: 0x2A01
- **알림 특성**: 0x2A02
- **설정 특성**: 0x2A03

### JSON 데이터 포맷
```json
{
  "hr": 75.0,        // 심박수 (BPM)
  "st": 36.5,        // 피부온도 (°C)
  "at": 28.0,        // 환경온도 (°C)
  "hum": 65.0,       // 습도 (%)
  "wbgt": 26.5,      // WBGT 지수 (°C)
  "ts": 1234567890   // 타임스탬프 (ms)
}
```

### 알림 포맷
```json
{
  "level": 2,        // 알림 레벨 (0:없음, 1:경고, 2:위험, 3:치명적)
  "msg": "체온 위험 - 즉시 작업 중단 필요"
}
```

## 전력 소비 최적화

### 저전력 모드
- **Active Mode**: 160MHz, 모든 센서 활성
- **Light Sleep**: 10MHz, 타이머 웨이크업
- **Deep Sleep**: 최소 전력, 외부 웨이크업만

### 배터리 수명
- **정상 동작**: 7일 (1Hz 센싱, 10초 BLE 전송)
- **저전력 모드**: 14일 (센싱 주기 감소)
- **대기 모드**: 30일 (딥슬립)

## 개발 계획

### Phase 1 (MVP)
- [x] 기본 센서 데이터 수집
- [x] BLE 통신 구현
- [x] 기본 임계치 감지
- [x] 전원 관리 기본 기능

### Phase 2
- [ ] 실제 센서 드라이버 구현
- [ ] WBGT 계산 알고리즘 검증
- [ ] 저전력 모드 최적화
- [ ] OTA 업데이트 지원

### Phase 3
- [ ] WiFi 직접 연결
- [ ] 클라우드 동기화
- [ ] 확장 센서 지원
- [ ] 음성 알림 기능

## 문제 해결

### 일반적인 문제
1. **빌드 오류**: ESP-IDF 버전 확인 (v5.0 이상 필요)
2. **플래시 실패**: USB 케이블 및 드라이버 확인
3. **BLE 연결 문제**: 디바이스 이름 "SafeLink-Wearable" 확인

### 디버깅
```bash
# 자세한 로그 확인
idf.py menuconfig
# Component config > Log output > Default log verbosity > Debug

# 메모리 사용량 확인
idf.py size

# 코어 덤프 분석
idf.py coredump-info
```

## 라이센스

이 프로젝트는 [MIT 라이센스](LICENSE) 하에 배포됩니다.

## 기여

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'feat: Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 연락처

프로젝트 관련 문의: [프로젝트 이슈](https://github.com/yourorg/safelink/issues) 
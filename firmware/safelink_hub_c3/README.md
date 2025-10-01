# SafeLink Wearable Hub
## 작업복 허브 펌웨어

SafeLink Wearable Hub는 작업복 부착형 환경 센서 및 알림 장치로, 밴드로부터 생체 데이터를 수신하고 환경 센서를 통해 위험 상황을 감지하여 다중 알림을 제공하는 ESP32-C3 기반 펌웨어입니다.

## 하드웨어 구성

### 센서 시스템
| 센서 | 모델명 | 기능 | 연결 방식 |
|------|--------|------|-----------|
| **온습도센서** | AM2320 | 외부 온습도 측정 | I2C |
| **소음센서** | THC-AS01 | 소음 레벨 측정 (dB) | I2S |

### 출력 시스템
| 장치 | 모델명 | 기능 | 연결 방식 |
|------|--------|------|-----------|
| **음성모듈** | DRF0299 | 경고 음성 재생 | UART |
| **스피커** | FIT0502 | 음성 출력 | - |
| **진동모터** | 316040004 | 진동 알림 | GPIO |

### 통신 시스템
- **메인 MCU**: XIAO-ESP32-C3
- **BLE 통신**: GATT 서버, 밴드 스캔

## 프로젝트 구조

### 핵심 모듈

- **safelink_wearable_hub.c**: 메인 애플리케이션 (278 lines)
- **bluetooth.c**: BLE 통신 관리 (1,462 lines)
- **sensor.c**: 센서 관리 및 경고 시스템 (640 lines)
- **data_manager.c**: 데이터 관리 (237 lines)
- **i2c.c**: I2C 통신 (115 lines)
- **dfplayer_mini.c**: 음성 재생 (272 lines)
- **mic_i2s.c**: I2S 마이크 (121 lines)

### 센서 모듈 분리

센서 처리 함수들이 별도의 `sensor.c/h` 파일로 분리되어 있습니다:

#### 주요 함수들:
- `sensor_init()`: 센서 모듈 초기화
- `sensor_deinit()`: 센서 모듈 정리
- `sensor_create_tasks()`: 센서 관련 태스크들 생성
- `sensor_get_current_data()`: 현재 센서 데이터 가져오기
- `sensor_update_health_status()`: 건강 상태 업데이트

#### 태스크들:
- `heart_rate_task`: 심박수 측정 (2초마다)
- `temp_humidity_task`: 온습도 측정 (4초마다)
- `sensor_monitor_task`: 센서 데이터 모니터링 (500ms마다)

## 빌드 및 실행

```bash
# 프로젝트 디렉토리로 이동
cd firmware/safelink_wearable_hub

# 빌드
idf.py build

# 플래시 및 모니터
idf.py flash monitor
```

## 설정

- **Target**: ESP32C3
- **Bluetooth**: NimBLE (GATT Server)
- **센서**: AM2320 (온습도), ADC (심박수)

## 주요 기능

1. **센서 데이터 수집**: 심박수, 온도, 습도 측정
2. **건강 상태 분석**: 측정된 데이터를 바탕으로 건강 상태 판단
3. **Bluetooth 통신**: 모바일 앱과 센서 데이터 공유
4. **실시간 모니터링**: 센서 데이터 실시간 로깅 및 경고

## 센서 데이터 형식

```c
typedef struct {
    uint16_t heart_rate;      // 심박수 (BPM)
    uint16_t temperature;     // 온도 (×100, 0.01°C 단위)
    uint16_t humidity;        // 습도 (×100, 0.01% 단위)
    uint32_t timestamp;       // 타임스탬프 (ms)
} sensor_data_t;
```

## 건강 상태 분류

- `HEALTH_STATUS_NORMAL`: 정상
- `HEALTH_STATUS_ELEVATED_HR`: 심박수 상승
- `HEALTH_STATUS_HIGH_TEMP`: 온도 이상
- `HEALTH_STATUS_LOW_HUMIDITY`: 습도 이상
- `HEALTH_STATUS_WARNING`: 다중 이상
- `HEALTH_STATUS_CRITICAL`: 위험 상태 
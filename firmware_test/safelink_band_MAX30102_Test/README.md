# SafeLink Band

ESP32C3 기반 웨어러블 밴드 애플리케이션입니다.

## 프로젝트 구조

### 주요 모듈

- **sensor.c/h**: 센서 처리 모듈
  - 센서 모니터링 태스크 (`sensor_monitor_task`)
  - 데이터 전송 태스크 (`data_sender_task`)
  - 센서 데이터 관리 함수들

- **bluetooth.c/h**: Bluetooth 통신 모듈
  - BLE GATT 클라이언트 구현
  - Hub 디바이스 검색 및 연결
  - 센서 데이터 전송

- **i2c.c/h**: I2C 통신 모듈
  - AM2320 온습도 센서 통신

- **analog.c/h**: 아날로그 센서 모듈
  - ADC를 통한 심박수 센서 읽기

### 센서 모듈 분리

센서 처리 함수들이 별도의 `sensor.c/h` 파일로 분리되어 있습니다:

#### 주요 함수들:
- `sensor_init()`: 센서 모듈 초기화
- `sensor_deinit()`: 센서 모듈 정리
- `sensor_create_tasks()`: 센서 관련 태스크들 생성
- `sensor_get_current_data()`: 현재 센서 데이터 가져오기
- `sensor_get_heart_rate_data()`: 심박수 데이터 가져오기
- `sensor_update_health_status()`: 건강 상태 업데이트

#### 태스크들:
- `sensor_monitor_task`: 센서 데이터 모니터링 (100ms마다)
  - 온습도 센서 읽기 (5초마다)
  - 심박수 센서 읽기 (1초마다)
  - 데이터 전송 트리거 (2초마다)
- `data_sender_task`: Hub로 데이터 전송

## 빌드 및 실행

```bash
# 프로젝트 디렉토리로 이동
cd firmware/safelink_band

# 빌드
idf.py build

# 플래시 및 모니터
idf.py flash monitor
```

## 설정

- **Target**: ESP32C3
- **Bluetooth**: NimBLE (GATT Client)
- **센서**: AM2320 (온습도), ADC (심박수)

## 주요 기능

1. **센서 데이터 수집**: 심박수, 온도, 습도 측정
2. **Hub 연결**: SafeLink Hub 디바이스 자동 검색 및 연결
3. **데이터 전송**: 측정된 센서 데이터를 Hub로 전송
4. **LED 상태 표시**: Hub 연결 상태를 LED로 표시

## 센서 데이터 형식

```c
typedef struct {
    uint16_t heart_rate;      // 심박수 (BPM)
    uint16_t temperature;     // 온도 (×100, 0.01°C 단위)
    uint16_t humidity;        // 습도 (×100, 0.01% 단위)
    uint32_t timestamp;       // 타임스탬프 (ms)
    health_status_t health_status; // 건강 상태
} sensor_data_t;
```

## Hub 연결 프로세스

1. **초기화**: Bluetooth 및 센서 모듈 초기화
2. **Hub 검색**: 주변 SafeLink Hub 디바이스 스캔
3. **연결**: Hub 발견 시 자동 연결 시도
4. **데이터 전송**: 연결 성공 시 센서 데이터 전송
5. **재연결**: 연결 끊김 시 자동 재검색 및 재연결

## LED 상태 표시

- **빠른 깜빡임 (200ms)**: Hub에 연결됨
- **느린 깜빡임 (1000ms)**: Hub 연결 안됨, 검색 중

## 센서 측정 주기

- **온습도**: 5초마다 측정
- **심박수**: 1초마다 측정
- **데이터 전송**: 2초마다 Hub로 전송
- **모니터링**: 100ms마다 상태 체크 
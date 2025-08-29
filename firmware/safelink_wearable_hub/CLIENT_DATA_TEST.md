# 클라이언트 데이터 우선순위 테스트 가이드

## 개요
이 문서는 GATT 클라이언트가 쓴 데이터가 시뮬레이터 데이터에 의해 덮어쓰여지지 않도록 하는 기능을 테스트하는 방법을 설명합니다.

## 수정된 기능

### 1. 데이터 소스 추적
- `DATA_SOURCE_SIMULATOR`: 시뮬레이터에서 생성된 데이터
- `DATA_SOURCE_CLIENT`: GATT 클라이언트가 쓴 데이터

### 2. 클라이언트 데이터 우선순위
- 클라이언트가 데이터를 쓰면 `g_has_client_data` 플래그가 설정됨
- 시뮬레이터는 이 플래그가 설정되어 있으면 데이터 생성 중단
- 클라이언트 데이터가 유지됨

### 3. 개별 특성 쓰기 지원
- 각 GATT 특성별로 개별 데이터 쓰기 지원
- 온도, 습도, 피부온도, SpO2, 심박수 각각 독립적으로 업데이트 가능

## 테스트 방법

### 1. 기본 시뮬레이션 테스트
```bash
# 빌드 및 플래시
cd firmware/safelink_wearable_hub
idf.py build
idf.py flash monitor
```

시스템 시작 후:
- 3초마다 시뮬레이터 데이터 생성
- 5초마다 터미널에 데이터 출력
- `Data Source: SIMULATOR` 표시

### 2. GATT 클라이언트 데이터 쓰기 테스트

#### nRF Connect 앱 사용:
1. "SafeLink_Test" 디바이스 연결
2. Health Sensor Service (0x1810) 선택
3. 개별 특성에 데이터 쓰기:

**온도 특성 (0x2A6E):**
- 4바이트 float 데이터
- 예: `0x5500` (85.0°C) 또는 `0x8800` (136.0°C)

**습도 특성 (0x2A6F):**
- 4바이트 float 데이터
- 예: `0x4200` (66.0%)

**피부온도 특성 (0x2A73):**
- 4바이트 float 데이터
- 예: `0x3700` (55.0°C)

**SpO2 특성 (0x2A72):**
- 4바이트 float 데이터
- 예: `0x9600` (150.0%)

**심박수 특성 (0x2A37):**
- 2바이트 uint16_t 데이터
- 예: `0x0064` (100 BPM)

### 3. 예상 결과

#### 클라이언트 데이터 쓰기 전:
```
=== SafeLink Wearable Hub Data ===
Timestamp: 12345 ms

[Band Data] VALID
  External Temp: 25.3°C
  External Humidity: 45.2%
  Skin Temperature: 36.8°C
  Heart Rate: 72 BPM
  SpO2: 97.5%
  Band Timestamp: 12340 ms
  Data Source: SIMULATOR

[System Status]
  Client Data Active: NO
================================
```

#### 클라이언트 데이터 쓰기 후:
```
=== SafeLink Wearable Hub Data ===
Timestamp: 12345 ms

[Band Data] VALID
  External Temp: 85.0°C
  External Humidity: 66.0%
  Skin Temperature: 55.0°C
  Heart Rate: 100 BPM
  SpO2: 150.0%
  Band Timestamp: 12340 ms
  Data Source: CLIENT

[System Status]
  Client Data Active: YES
================================
```

### 4. 시뮬레이터 재시작 테스트

클라이언트 데이터가 설정된 후:
- 시뮬레이터는 데이터 생성 중단
- 로그에 "Client data active - skipping simulator data generation" 메시지 표시
- 클라이언트가 쓴 데이터가 유지됨

### 5. 시뮬레이터 재시작

터미널에서 다음 명령으로 시뮬레이터 재시작:
```c
// 코드에서 호출
test_simulator_reset_client_data();
```

또는 데이터 매니저에서 직접:
```c
data_manager_clear_client_data();
```

## 문제 해결

### 1. 데이터가 덮어쓰여지는 경우
- `g_has_client_data` 플래그 확인
- 시뮬레이터 로그에서 "Client data active" 메시지 확인
- 데이터 소스가 "CLIENT"로 표시되는지 확인

### 2. 개별 특성 쓰기가 안 되는 경우
- 데이터 크기 확인 (온도/습도/피부온도/SpO2: 4바이트, 심박수: 2바이트)
- GATT 특성 UUID 확인
- 연결 상태 확인

### 3. 시뮬레이터가 계속 실행되는 경우
- `data_manager_has_client_data()` 함수 확인
- 클라이언트 데이터 플래그 설정 확인

## 로그 메시지

### 정상 동작 시:
```
I (1234) BLUETOOTH: External temperature updated by client: 85.0°C
I (1234) DATA_MANAGER: Client data received - simulator will be paused
I (1234) TEST_SIMULATOR: Client data active - skipping simulator data generation
```

### 시뮬레이터 재시작 시:
```
I (1234) DATA_MANAGER: Client data flag cleared - simulator can resume
I (1234) TEST_SIMULATOR: Client data flag reset - simulator can resume normal operation
```

## 테스트 시나리오

1. **기본 시뮬레이션**: 시스템 시작 후 시뮬레이터 데이터 확인
2. **클라이언트 데이터 쓰기**: nRF Connect로 개별 특성에 데이터 쓰기
3. **데이터 유지 확인**: 시뮬레이터가 클라이언트 데이터를 덮어쓰지 않는지 확인
4. **시뮬레이터 재시작**: 클라이언트 플래그 초기화 후 시뮬레이터 재시작
5. **복합 테스트**: 여러 특성에 순차적으로 데이터 쓰기

이 테스트를 통해 클라이언트 데이터 우선순위 기능이 정상적으로 작동하는지 확인할 수 있습니다.

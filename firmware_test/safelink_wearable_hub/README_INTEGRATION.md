# SafeLink Wearable Hub - 시스템 통합 가이드

## 개요
SafeLink Wearable Hub는 웨어러블 밴드로부터 생체 데이터를 수신하고, 허브에서 환경 데이터를 수집하여 MIT App Inventor와 GATT Server를 통해 데이터를 전송하는 시스템입니다.

## 시스템 아키텍처

### 데이터 흐름
```
밴드 데이터 → GATT Subscribe → 데이터 매니저 → 터미널 출력
허브 센서 → 데이터 매니저 → GATT Publishing → MIT App Inventor
```

### 주요 구성 요소
1. **데이터 매니저** (`data_manager.h/c`)
   - 밴드 데이터 관리 (외기 온습도, 피부온도, 심박수, 혈중산소포화농도)
   - 허브 데이터 관리 (평균소음, WBGT, 경보상태)
   - 스레드 안전한 데이터 접근

2. **블루투스 모듈** (`bluetooth.h/c`)
   - GATT Server 구현
   - MIT App Inventor 데이터 전송
   - 밴드 데이터 수신 처리
   - 디버그 커맨드 처리

3. **센서 모듈** (`sensor.h/c`)
   - 허브 센서 데이터 수집
   - 소음 레벨 측정
   - WBGT 계산
   - 경보 상태 결정

4. **테스트 시뮬레이터** (`test_simulator.h/c`)
   - 개발용 데이터 생성
   - 현실적인 범위의 시뮬레이션 데이터

## 빌드 및 실행

### 1. 환경 설정
```bash
cd firmware/safelink_wearable_hub
idf.py set-target esp32c6
```

### 2. 빌드
```bash
idf.py build
```

### 3. 플래시 및 모니터
```bash
idf.py flash monitor
```

## 시스템 테스트

### 1. 자동 시뮬레이션 테스트
시스템이 시작되면 자동으로 테스트 시뮬레이터가 실행됩니다:
- 3초마다 밴드 데이터 생성
- 3초마다 허브 데이터 생성
- 터미널에 5초마다 전체 데이터 출력

### 2. 터미널 출력 예시
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

[Hub Data] VALID
  Average Noise: 65.3 dB
  WBGT: 28.7°C
  Alarm Status: 0x00
  Hub Timestamp: 12345 ms
================================
```

### 3. 블루투스 연결 테스트
1. nRF Connect 앱으로 "SafeLink_Test" 디바이스 스캔
2. 연결 후 GATT 서비스 확인
3. 특성 읽기/쓰기 테스트

### 4. 디버그 커맨드 테스트
GATT Command 특성에 다음 명령 전송:
- `vib`: 진동 모터 1초 동작
- `play 1`: DFPlayer에서 1번 트랙 재생

## 데이터 구조

### 밴드 데이터 (GATT Subscribe)
```c
typedef struct {
    float external_temp;      // 외기 온도 (°C)
    float external_humidity;  // 외기 습도 (%)
    float skin_temp;          // 피부온도 (15.0~40.0°C)
    uint16_t heart_rate;      // 심박수 (20~160 BPM)
    float spo2;              // 혈중산소포화농도 (80.0~99.9%)
    uint32_t timestamp;      // 타임스탬프
    bool is_valid;           // 데이터 유효성
} band_data_t;
```

### 허브 데이터 (GATT Publishing)
```c
typedef struct {
    float avg_noise;         // 평균소음 (dB)
    float wbgt;              // WBGT(건구습구온도 추정치) (°C)
    uint8_t alarm_status;    // 경보상태 (비트마스크)
    uint32_t timestamp;      // 타임스탬프
    bool is_valid;           // 데이터 유효성
} hub_data_t;
```

### 경보 상태 비트
- `ALARM_WBGT_WARNING` (0x01): WBGT 경고
- `ALARM_TEMP_WARNING` (0x02): 체온 경고
- `ALARM_HR_WARNING` (0x04): 심박수 경고

## GATT 서비스 구조

### 1. Test Service (0x1800)
- Device Name Characteristic (0x2A00): 읽기/알림

### 2. Health Sensor Service (0x1810)
- Temperature Characteristic (0x2A6E): 읽기/쓰기/알림
- Humidity Characteristic (0x2A6F): 읽기/쓰기/알림
- Body Temperature Characteristic (0x2A73): 읽기/쓰기/알림
- SpO2 Characteristic (0x2A72): 읽기/쓰기/알림
- Heart Rate Characteristic (0x2A37): 읽기/쓰기/알림

### 3. Noise Sensor Service (0x1811)
- Noise Level Characteristic (0x2A71): 읽기/알림

### 4. Control Service (0x1812)
- Command Characteristic (0x2A76): 읽기/쓰기/알림

## MIT App Inventor 데이터 형식

Advertising 데이터에 포함되는 20바이트 패킷:
```
[0-3]   Skin Temperature (float)
[4-5]   Heart Rate (uint16_t)
[6-9]   SpO2 (float)
[10-13] External Temperature (float)
[14-17] External Humidity (float)
[18-19] Average Noise (uint16_t, 0.1dB 단위)
[20-23] WBGT (float)
[24]    Alarm Status (uint8_t)
```

## 문제 해결

### 1. 빌드 오류
- ESP-IDF 버전 확인 (v5.4 이상 권장)
- 타겟 설정 확인 (`esp32c6`)

### 2. 블루투스 연결 문제
- 디바이스 이름 확인: "SafeLink_Test"
- nRF Connect 앱 재시작
- ESP32 재부팅

### 3. 데이터 업데이트 문제
- 터미널 로그 확인
- 데이터 매니저 상태 확인
- 시뮬레이터 상태 확인

## 성능 최적화

### 1. 태스크 우선순위
- 블루투스 태스크: 5
- 센서 태스크: 5
- 터미널 태스크: 3
- 시뮬레이터 태스크: 3

### 2. 메모리 사용량
- 데이터 매니저: 세마포어 기반 스레드 안전
- GATT 서비스: NimBLE 최적화
- 센서 데이터: 실시간 업데이트

### 3. 전력 소모
- 블루투스 연결 시에만 데이터 전송
- 센서 읽기 주기 최적화
- 불필요한 로그 출력 제한

## 향후 개선 사항

1. **실제 하드웨어 연동**
   - AM2320 온습도 센서
   - 소음 센서
   - 진동 모터
   - DFPlayer Mini

2. **보안 강화**
   - 데이터 암호화
   - 인증 메커니즘
   - 접근 제어

3. **기능 확장**
   - 웹 인터페이스
   - 데이터베이스 저장
   - 클라우드 연동
   - 알림 시스템

## 라이선스
이 프로젝트는 MIT 라이선스 하에 배포됩니다.

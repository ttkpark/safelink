# SafeLink Band
## 웨어러블 밴드 펌웨어

SafeLink Band는 손목 착용형 생체 센서 장치로, 작업자의 심박수, 체온, 환경 데이터를 실시간으로 측정하고 BLE를 통해 허브로 전송하는 ESP32-C3 기반 펌웨어입니다.

## 하드웨어 구성

### 센서 구성
| 센서 | 모델명 | 기능 | 연결 방식 |
|------|--------|------|-----------|
| **심박센서** | SEN0203 | 심박수 측정 (BPM) | ADC (GPIO4) |
| **체온센서** | MCP9808 | 피부 온도 측정 | I2C (0x18) |
| **온습도센서** | AHT20 | 환경 온습도 측정 | I2C (0x38) |

### 전원 시스템
- **메인 MCU**: ESP32-C3 SuperMini
- **배터리**: 400mAh Li-Po
- **충전모듈**: TP4056
- **통신**: BLE 5.0

## 주요 기능

### 센서 데이터 수집
- **실시간 생체 모니터링**: 심박수, 체온, 환경 데이터 동시 측정
- **고정밀 심박수 측정**: DSP 알고리즘을 통한 정확한 BPM 계산
- **체온 측정**: 피부 온도 실시간 모니터링
- **환경 센싱**: 온습도 데이터 수집

### 통신 기능
- **BLE 광고 전송**: 20ms 간격으로 센서 데이터 브로드캐스트
- **데이터 형식**: 13바이트 페이로드 (온도, 습도, 체온, 심박수, SpO2)

### 전력 관리
- **저전력 설계**: 딥슬립 모드로 1주일 연속 사용 가능
- **배터리 관리**: 충전 상태 모니터링
- **자동 절전**: 미사용 시 자동 절전 모드

## 사용법

### 기본 사용법
```c
#include "MAX3010x_ESP32.h"
#include "MAX3010x_Algorithm.h"

// 센서 인스턴스 생성
max3010x_t sensor;
max3010x_algorithm_t algorithm;

// 초기화
max3010x_init(&sensor, I2C_NUM_0, 0x57);
max3010x_algorithm_init(&algorithm);

// 센서 설정
max3010x_setup(&sensor, 50, SAMPLE_RATE_100, MODE_SPO2, 
               SAMPLE_AVG_4, PULSE_WIDTH_411, ADC_RANGE_4096, 0);

// 데이터 읽기 및 처리
uint32_t red_buffer[32], ir_buffer[32];
uint8_t samples_read;
max3010x_read_fifo(&sensor, red_buffer, ir_buffer, NULL, &samples_read);

// 알고리즘에 데이터 추가
for (int i = 0; i < samples_read; i++) {
    max3010x_algorithm_update(&algorithm, ir_buffer[i], red_buffer[i], 0);
}

// 결과 가져오기
max3010x_heart_rate_result_t hr_result;
max3010x_algorithm_get_heart_rate(&algorithm, &hr_result);
if (hr_result.valid) {
    printf("심박수: %.1f BPM\n", hr_result.heart_rate);
}
```

## 빌드 및 실행

### 환경 설정
```bash
# ESP-IDF 환경 설정
C:\Users\yerim\esp\v5.4.2\esp-idf\export.ps1
```

### 빌드
```bash
idf.py build
```

### 플래시
```bash
idf.py flash
```

### 모니터링
```bash
idf.py monitor
```

## 예상 출력

```
I (1234) SENSOR_SYSTEM: ESP32-C3 SuperMini AHT20 + MCP9808 센서 시스템 시작
I (1235) SENSOR_SYSTEM: I2C 마스터 초기화 완료
I (1236) SENSOR_SYSTEM: I2C 스캔 시작...
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- 
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
I (1237) SENSOR_SYSTEM: I2C 스캔 완료
I (1238) SENSOR_SYSTEM: AHT20 센서 초기화 완료
I (1239) SENSOR_SYSTEM: MCP9808 센서 초기화 완료
I (1240) MAX3010x: MAX30102 센서 감지됨
I (1241) MAX3010x: MAX3010x 센서 초기화 완료
I (1242) MAX3010x_Algorithm: MAX3010x 알고리즘 초기화 완료
I (1243) SENSOR_SYSTEM: MAX3010x 센서 및 알고리즘 초기화 완료
I (1244) SENSOR_SYSTEM: 센서 초기화 완료
I (1245) SENSOR_SYSTEM: 센서 데이터를 측정합니다
I (1246) SENSOR_SYSTEM: MAX3010x - 데이터 읽기 및 심박수 계산 완료
I (1247) SENSOR_SYSTEM: 심박수: 72.5 BPM (신뢰도: 0.85)
I (1248) SENSOR_SYSTEM: SpO2: 98.2% (신뢰도: 0.92)
```

## 라이브러리 구조

```
main/
├── MAX3010x_ESP32.h          # MAX3010x 센서 드라이버 헤더 (C)
├── MAX3010x_ESP32.c          # MAX3010x 센서 드라이버 구현 (C)
├── MAX3010x_Algorithm.h      # 심박수/SpO2 알고리즘 헤더 (C)
├── MAX3010x_Algorithm.c      # 심박수/SpO2 알고리즘 구현 (C)
├── main.c                    # 메인 애플리케이션
└── CMakeLists.txt           # 빌드 설정
```

## 주요 변경사항

### C++에서 C로 포팅
- 클래스 기반 구조를 구조체 기반으로 변경
- 멤버 함수를 C 함수로 변환
- 메모리 관리 최적화
- ESP-IDF 호환성 향상

### 기능 개선
- 실시간 심박수 측정
- SpO2 계산 알고리즘
- 신호 품질 평가
- 에러 처리 강화

## 문제 해결

### 일반적인 문제들

1. **센서가 감지되지 않음**
   - I2C 연결 확인
   - 전원 공급 확인 (3.3V)
   - 센서 주소 확인 (0x57)

2. **데이터 품질이 낮음**
   - 손가락 위치 조정
   - 센서와 피부 간 접촉 확인
   - LED 전류 설정 조정

3. **심박수 측정이 불안정함**
   - 손가락을 고정하고 움직임 최소화
   - 측정 시간을 충분히 확보 (최소 10초)
   - 환경광 차단

## 라이센스

이 프로젝트는 MIT 라이센스 하에 배포됩니다.

## 기여

버그 리포트, 기능 요청, 풀 리퀘스트를 환영합니다.

## 참고 자료

- [SparkFun MAX3010x 라이브러리](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)
- [MAX30102 데이터시트](https://www.maximintegrated.com/en/products/interface/sensor-interface/MAX30102.html)
- [ESP-IDF 문서](https://docs.espressif.com/projects/esp-idf/en/latest/)

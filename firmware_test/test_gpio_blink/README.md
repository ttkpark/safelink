# ESP32-C6 Safe GPIO Blink Test

ESP32-C6에서 안전한 GPIO 핀들만 사용하여 LED blink 테스트를 수행하는 프로젝트입니다.

## 안전한 GPIO 핀 선별

ESP32-C6에서 안전하게 사용할 수 있는 GPIO 핀들만 선별했습니다:

### 사용되는 안전한 GPIO 핀들
- **GPIO 4**: 일반 GPIO (안전)
- **GPIO 5**: 일반 GPIO (안전)
- **GPIO 13**: 일반 GPIO (안전)
- **GPIO 14**: 일반 GPIO (안전)

### 제외된 GPIO 핀들 (위험/특수 기능)
- **GPIO 0**: 부팅 모드 선택 (부팅 시 LOW=다운로드 모드)
- **GPIO 1-3**: UART0 (시리얼 통신)
- **GPIO 6-11**: SPI0 (플래시 메모리)
- **GPIO 12**: 부팅 시 사용 (부팅 시 HIGH=안전 모드)
- **GPIO 15**: 문제 발생으로 제외
- **GPIO 16-17**: UART1
- **GPIO 18-21**: SPI1
- **GPIO 22-23**: I2C0
- **GPIO 24-25**: I2C1
- **GPIO 26-27**: I2S0
- **GPIO 28-29**: I2S1
- **GPIO 30**: 일부 보드에서 제한적

## 기능

### Blink 모드
1. **모든 GPIO 동시 blink**: 5개의 안전한 GPIO 핀이 동시에 켜지고 꺼짐
2. **순차적 blink**: GPIO 4 → 5 → 13 → 14 → 15 순서로 하나씩 켜짐
3. **파도 패턴 blink**: 2개씩 연속으로 켜지면서 이동
4. **랜덤 blink**: 1-3개의 핀이 랜덤하게 켜짐

### 속도 조절
- `+` 키: 속도 증가 (100ms씩 감소)
- `-` 키: 속도 감소 (100ms씩 증가)
- 기본 지연시간: 500ms

## 빌드 및 실행

### 사전 요구사항
- ESP-IDF v5.0 이상
- ESP32-C6 개발 보드
- USB 케이블

### 빌드
```bash
idf.py build
```

### 플래시 및 모니터링
```bash
idf.py flash monitor
```

### 사용법
프로그램 실행 후 시리얼 모니터에서 다음 명령어를 입력:

- `1`: 모든 안전한 GPIO 동시에 blink
- `2`: 순차적으로 blink
- `3`: 파도 패턴 blink
- `4`: 랜덤 blink
- `+`: 속도 증가
- `-`: 속도 감소
- `q`: 종료

## 하드웨어 연결

### LED 연결 (선택사항)
테스트를 위해 LED를 연결할 수 있습니다:

```
GPIO 4 ──[220Ω 저항]──[LED]── GND
GPIO 5 ──[220Ω 저항]──[LED]── GND
GPIO 13 ──[220Ω 저항]──[LED]── GND
GPIO 14 ──[220Ω 저항]──[LED]── GND
```

### 주의사항
- LED 사용 시 반드시 저항(220Ω 권장)을 직렬로 연결하세요
- LED의 애노드(+)를 GPIO 핀에, 캐소드(-)를 GND에 연결하세요
- GPIO 핀당 최대 12mA까지 안전하게 사용 가능합니다

## 로그 출력 예시

```
I (1234) SAFE_GPIO_BLINK_TEST: ESP32-C6 Safe GPIO Blink Test Starting...
I (1235) SAFE_GPIO_BLINK_TEST: ESP-IDF Version: v5.1.2
I (1236) SAFE_GPIO_BLINK_TEST: Safe GPIO pins to test: 4
I (1237) SAFE_GPIO_BLINK_TEST: Safe GPIO pins: GPIO_4, GPIO_5, GPIO_13, GPIO_14
I (1238) SAFE_GPIO_BLINK_TEST: Initializing safe GPIO pins as outputs...
I (1239) SAFE_GPIO_BLINK_TEST: Safe GPIO initialization completed successfully
I (1240) SAFE_GPIO_BLINK_TEST: All safe GPIO pins set to LOW (initial state)

=== ESP32-C6 Safe GPIO Blink Test ===
사용 가능한 명령어:
1 - 모든 안전한 GPIO 동시에 blink
2 - 순차적으로 blink
3 - 파도 패턴 blink
4 - 랜덤 blink
+ - 속도 증가 (100ms씩)
- - 속도 감소 (100ms씩)
q - 종료
현재 모드: 0, 지연시간: 500ms
안전한 GPIO 핀: GPIO_4, GPIO_5, GPIO_13, GPIO_14
=====================================
```

## 안전성 고려사항

1. **부팅 안전성**: 부팅 시 사용되는 GPIO 핀들을 제외하여 안전한 부팅 보장
2. **통신 안전성**: UART, SPI, I2C 등 통신 핀들을 제외하여 시스템 안정성 확보
3. **전류 제한**: 각 GPIO 핀의 최대 전류 제한 준수
4. **ESD 보호**: 정전기 방전 보호를 위한 적절한 저항 사용

## 문제 해결

### 빌드 오류
- ESP-IDF 환경이 올바르게 설정되었는지 확인
- `idf.py set-target esp32c6` 명령으로 타겟 설정

### 플래시 오류
- USB 케이블 연결 상태 확인
- 보드의 부팅 모드 확인 (GPIO 0을 LOW로 유지)
- 드라이버 설치 상태 확인

### GPIO 동작 안함
- LED 연결 상태 및 극성 확인
- 저항 값 확인 (220Ω 권장)
- GPIO 핀 번호 확인

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.
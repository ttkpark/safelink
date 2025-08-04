# ESP32-C3 SuperMini LED Blink Test

ESP32-C3 SuperMini의 내장 LED를 1초 주기로 깜빡이는 테스트 프로젝트입니다.

## 하드웨어 요구사항

- ESP32-C3 SuperMini 개발보드
- USB-C 케이블 (프로그래밍 및 전원 공급용)

## 프로젝트 구조

```
test_esp32c3_blink/
├── CMakeLists.txt          # 프로젝트 CMake 설정
├── main/
│   ├── CMakeLists.txt      # 메인 컴포넌트 CMake 설정
│   └── main.c              # 메인 소스 코드
└── README.md               # 이 파일
```

## 기능

- ESP32-C3 SuperMini의 내장 LED (GPIO8) 제어
- 1초 간격으로 LED ON/OFF 토글
- 시리얼 모니터를 통한 상태 로그 출력

## 빌드 및 플래시

### 1. ESP-IDF 환경 설정

```bash
# ESP-IDF 환경 변수 설정
. $HOME/esp/esp-idf/export.sh
```

### 2. 프로젝트 빌드

```bash
cd test_esp32c3_blink
idf.py build
```

### 3. ESP32-C3 SuperMini에 플래시

```bash
# ESP32-C3 SuperMini를 USB로 연결한 후
idf.py -p [PORT] flash monitor
```

예시:
```bash
idf.py -p COM3 flash monitor  # Windows
idf.py -p /dev/ttyUSB0 flash monitor  # Linux
idf.py -p /dev/tty.usbserial-0001 flash monitor  # macOS
```

## 동작 확인

프로그램이 실행되면:

1. 시리얼 모니터에 초기화 메시지가 출력됩니다
2. 내장 LED가 1초 간격으로 깜빡입니다
3. 시리얼 모니터에 LED 상태가 로그로 출력됩니다

### 예상 출력

```
I (1234) ESP32C3_LED_BLINK: ESP32-C3 SuperMini LED Blink Test Starting...
I (1234) ESP32C3_LED_BLINK: ESP-IDF Version: v5.1.2
I (1234) ESP32C3_LED_BLINK: Built LED GPIO: GPIO_8
I (1234) ESP32C3_LED_BLINK: GPIO initialization completed successfully
I (1234) ESP32C3_LED_BLINK: LED initialized to OFF state
I (1234) ESP32C3_LED_BLINK: Starting LED blink with 1000ms interval...
I (2234) ESP32C3_LED_BLINK: LED: ON
I (3234) ESP32C3_LED_BLINK: LED: OFF
I (4234) ESP32C3_LED_BLINK: LED: ON
...
```

## 하드웨어 연결

ESP32-C3 SuperMini의 내장 LED는 GPIO8에 연결되어 있습니다. 별도의 외부 연결이 필요하지 않습니다.

## 문제 해결

### LED가 깜빡이지 않는 경우

1. **GPIO 핀 확인**: 일부 ESP32-C3 보드에서는 내장 LED가 다른 GPIO에 연결되어 있을 수 있습니다.
   - GPIO2, GPIO8, GPIO10 등을 확인해보세요
   - 보드의 데이터시트를 참조하세요

2. **GPIO 핀 변경**: 다른 GPIO 핀을 사용하려면 `main.c`의 `LED_GPIO_PIN` 값을 수정하세요

```c
#define LED_GPIO_PIN    2  // GPIO2로 변경
```

### 시리얼 포트를 찾을 수 없는 경우

1. USB 드라이버가 올바르게 설치되었는지 확인
2. 다른 USB 케이블 사용
3. 다른 USB 포트 시도

## 커스터마이징

### Blink 간격 변경

`main.c`에서 `BLINK_DELAY_MS` 값을 수정하세요:

```c
#define BLINK_DELAY_MS  500  // 0.5초로 변경
```

### 다른 GPIO 핀 사용

외부 LED를 사용하려면 `LED_GPIO_PIN`을 원하는 GPIO로 변경하세요:

```c
#define LED_GPIO_PIN    4  // GPIO4 사용
```

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 
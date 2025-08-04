# DFP Player Mini Library for ESP32

[DFRobot DFP Player Mini](https://github.com/DFRobot/DFRobotDFPlayerMini) 라이브러리를 ESP32용으로 포팅한 버전입니다.

## 기능

- ✅ 모든 DFP Player 명령어 지원
- ✅ 응답 처리 및 에러 분석
- ✅ 재시도 로직
- ✅ 상세한 로깅
- ✅ ESP32 FreeRTOS 호환

## 설치

1. `dfplayer_mini.h`와 `dfplayer_mini.c` 파일을 프로젝트에 추가
2. `CMakeLists.txt`에 소스 파일 추가:
```cmake
idf_component_register(
    SRCS "main.c" "dfplayer_mini.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
```

## 사용법

### 초기화

```c
#include "dfplayer_mini.h"

// DFP Player 초기화 (UART1, TX: GPIO4, RX: GPIO5, 9600 baud)
esp_err_t ret = dfplayer_init(UART_NUM_1, GPIO_NUM_4, GPIO_NUM_5, 9600);
if (ret != ESP_OK) {
    ESP_LOGE(TAG, "DFP Player 초기화 실패");
    return;
}
```

### 기본 재생 제어

```c
// 트랙 1 재생
dfplayer_play(1);

// 폴더 00의 트랙 001 재생
dfplayer_play_folder(0, 1);

// 일시정지
dfplayer_pause();

// 재생 재개
dfplayer_resume();

// 정지
dfplayer_stop();

// 다음 트랙
dfplayer_next();

// 이전 트랙
dfplayer_prev();
```

### 볼륨 및 설정

```c
// 볼륨 설정 (0-30)
dfplayer_set_volume(20);

// 볼륨 업/다운
dfplayer_volume_up();
dfplayer_volume_down();

// EQ 설정
dfplayer_set_eq(DFPLAYER_EQ_NORMAL);  // Normal
dfplayer_set_eq(DFPLAYER_EQ_POP);     // Pop
dfplayer_set_eq(DFPLAYER_EQ_ROCK);    // Rock
dfplayer_set_eq(DFPLAYER_EQ_JAZZ);    // Jazz
dfplayer_set_eq(DFPLAYER_EQ_CLASSIC); // Classic
dfplayer_set_eq(DFPLAYER_EQ_BASS);    // Bass

// 소스 설정
dfplayer_set_source(DFPLAYER_SOURCE_SD);  // SD 카드
dfplayer_set_source(DFPLAYER_SOURCE_U_DISK); // USB
dfplayer_set_source(DFPLAYER_SOURCE_AUX); // AUX

// 재생 모드 설정
dfplayer_set_playback_mode(DFPLAYER_MODE_REPEAT);        // 반복
dfplayer_set_playback_mode(DFPLAYER_MODE_FOLDER_REPEAT); // 폴더 반복
dfplayer_set_playback_mode(DFPLAYER_MODE_SINGLE_REPEAT); // 단일 반복
dfplayer_set_playback_mode(DFPLAYER_MODE_RANDOM);        // 랜덤
```

### 응답 처리

```c
dfplayer_response_t response;
esp_err_t ret = dfplayer_send_command_with_response(
    DFPLAYER_CMD_PLAY, 0x00, 1, &response
);

if (ret == ESP_OK) {
    ESP_LOGI(TAG, "명령 성공");
} else {
    ESP_LOGE(TAG, "명령 실패: %s", esp_err_to_name(ret));
}
```

### 상태 확인

```c
dfplayer_status_t status;
if (dfplayer_get_status(&status) == ESP_OK) {
    ESP_LOGI(TAG, "현재 소스: %d", status.current_source);
}
```

### 에러 처리

```c
// 에러 코드 설명 가져오기
const char* error_desc = dfplayer_get_error_description(0x03);
ESP_LOGE(TAG, "에러: %s", error_desc);
```

## 에러 코드

| 코드 | 설명 |
|------|------|
| 0x01 | File not found |
| 0x02 | Advertise |
| 0x03 | Serial receiving error |
| 0x04 | Check sum not match |
| 0x05 | File index out of bounds |
| 0x06 | File name mismatch |
| 0x07-0x0A | Advertise |

## 파일 구조

SD 카드에 다음과 같이 파일을 배치하세요:

```
SD카드/
├── 00/
│   ├── 001.mp3
│   ├── 002.mp3
│   └── ...
├── 01/
│   ├── 001.mp3
│   └── ...
└── ...
```

## 주의사항

1. **전원 공급**: DFP Player는 안정적인 5V 전원이 필요합니다
2. **SD 카드**: FAT32 포맷된 SD 카드를 사용하세요
3. **파일명**: 8.3 형식 (예: `001.mp3`)을 권장합니다
4. **통신**: UART 통신이 불안정할 수 있으므로 재시도 로직을 사용하세요

## 예제 프로젝트

현재 프로젝트의 `main.c`에서 실제 사용 예제를 확인할 수 있습니다.

## 라이센스

원본 DFRobot 라이브러리와 동일한 라이센스를 따릅니다. 
# 심박수 데이터 UDP 서버 및 Processing 클라이언트

이 프로젝트는 UDP 서버를 통해 심박수 데이터를 실시간으로 브로드캐스트하고, Processing을 사용하여 그래프로 시각화하는 시스템입니다.

## 📁 파일 구조

```
heartrate_getting_udp_server/
├── udp_server.py          # UDP 서버 (Python)
├── HeartRateGraph.pde     # Processing 클라이언트
├── test_client.py         # 테스트용 클라이언트
└── README.md             # 이 파일
```

## 🚀 사용법

### 1. UDP 서버 실행

```bash
python udp_server.py
```

서버가 시작되면:
- 포트 8888에서 UDP 연결을 대기합니다
- 장치들이 "start" 메시지를 보내면 등록합니다
- "send\r\n"으로 시작하는 데이터를 모든 등록된 장치에 브로드캐스트합니다

### 2. Processing 클라이언트 실행

1. Processing IDE를 설치합니다
2. `HeartRateGraph.pde` 파일을 Processing에서 엽니다
3. 실행 버튼을 클릭합니다

클라이언트는:
- 서버에 자동으로 연결됩니다
- "메시지 : x축, y축1, y축2" 형식의 데이터를 수신합니다
- 실시간으로 그래프를 그립니다

### 3. 테스트 클라이언트 실행 (선택사항)

```bash
python test_client.py
```

이 클라이언트는 심박수 데이터를 시뮬레이션하여 서버에 전송합니다.

## 📊 데이터 형식

### 서버로 전송하는 메시지 형식

1. **장치 등록**: `start`
2. **데이터 전송**: `send\r\n메시지 : x값, y1값, y2값`

예시:
```
send\r\n메시지 : 1.5, 75, 52
send\r\n메시지 : 2.0, 78, 48
```

### Processing에서 표시되는 그래프

- **X축**: 시간 또는 순차적 데이터
- **Y1축 (빨간색)**: 심박수 데이터
- **Y2축 (초록색)**: 기타 센서 데이터

## ⚙️ 설정

### 서버 설정

`udp_server.py`에서 다음 값들을 수정할 수 있습니다:

```python
HOST = '0.0.0.0'  # 모든 인터페이스에서 수신
PORT = 8888       # 포트 번호
```

### Processing 클라이언트 설정

`HeartRateGraph.pde`에서 다음 값들을 수정할 수 있습니다:

```java
int serverPort = 8888;                    // 서버 포트
String serverAddress = "localhost";       // 서버 주소
int maxDataPoints = 200;                  // 화면에 표시할 최대 데이터 포인트
```

## 🎮 키보드 단축키 (Processing)

- **R**: 데이터 초기화
- **S**: 서버에 재연결

## 🔧 요구사항

### Python 서버
- Python 3.6+
- 표준 라이브러리만 사용 (추가 설치 불필요)

### Processing 클라이언트
- Processing 3.0+
- Java UDP 라이브러리 (내장)

## 📱 ESP32 연동 예시

ESP32에서 이 서버에 연결하는 예시 코드:

```cpp
#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;
const char* serverIP = "192.168.1.100";  // 서버 IP 주소
const int serverPort = 8888;

void setup() {
  // WiFi 연결 코드...
  
  // 서버에 등록
  udp.beginPacket(serverIP, serverPort);
  udp.print("start");
  udp.endPacket();
}

void loop() {
  // 센서 데이터 읽기
  float x = millis() / 1000.0;
  int heartRate = analogRead(A0);  // 심박수 센서
  int otherData = analogRead(A1);  // 기타 센서
  
  // 데이터 전송
  udp.beginPacket(serverIP, serverPort);
  udp.print("send\r\n메시지 : ");
  udp.print(x);
  udp.print(", ");
  udp.print(heartRate);
  udp.print(", ");
  udp.print(otherData);
  udp.endPacket();
  
  delay(1000);
}
```

## 🐛 문제 해결

### 서버가 시작되지 않는 경우
- 포트 8888이 이미 사용 중인지 확인
- 방화벽 설정 확인
- 관리자 권한으로 실행 시도

### Processing에서 연결되지 않는 경우
- 서버가 실행 중인지 확인
- 서버 주소가 올바른지 확인
- 네트워크 연결 상태 확인

### 데이터가 표시되지 않는 경우
- 메시지 형식이 정확한지 확인 ("메시지 : x, y1, y2")
- 서버 로그에서 데이터 수신 여부 확인

## 📈 확장 가능성

- 다중 센서 데이터 지원
- 데이터 저장 및 분석 기능
- 웹 인터페이스 추가
- 모바일 앱 연동
- 실시간 알림 시스템

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

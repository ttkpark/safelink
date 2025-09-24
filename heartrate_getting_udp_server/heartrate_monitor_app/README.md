# 심박수 모니터 앱

Processing으로 작성된 심박수 데이터 시각화 기능을 Flutter 앱으로 완전히 구현한 실시간 심박수 모니터링 애플리케이션입니다.

## 주요 기능

### 🔗 UDP 통신
- UDP 서버(211.221.184.17:8888)에 자동 연결
- 실시간 데이터 수신 및 브로드캐스트
- 연결 상태 모니터링 및 자동 재연결

### 📊 실시간 그래프
- 두 개의 Y축 데이터를 다른 색상으로 표시
  - Y축1 (심박수): 빨간색
  - Y축2 (기타 데이터): 초록색
- 실시간 데이터 업데이트
- 자동 스케일 조정 및 그리드 표시

### 📈 데이터 분석
- 실시간 통계 정보 표시
- 최솟값, 최댓값, 평균값 계산
- 데이터 포인트 수 및 최신 값 표시

### 🎛️ 사용자 인터페이스
- 다크 테마 UI
- 직관적인 버튼 컨트롤
- 키보드 단축키 지원
- 연결 상태 표시

## 키보드 단축키

- **R**: 데이터 초기화
- **S**: 서버 재연결
- **C**: 데이터 정리 (NaN 값 제거)
- **E**: 연결 종료

## 설치 및 실행

### 개발 환경에서 실행
```bash
# 의존성 설치
flutter pub get

# 앱 실행 (Windows)
flutter run -d windows

# 앱 실행 (Android)
flutter run -d android
```

### APK 설치
1. `build/app/outputs/flutter-apk/app-release.apk` 파일을 Android 기기에 복사
2. 기기에서 "알 수 없는 소스" 설치 허용
3. APK 파일을 탭하여 설치

## 데이터 형식

앱은 다음 형식의 데이터를 처리합니다:
```
x,y1,y2
```

- `x`: X축 값 (시간 또는 인덱스)
- `y1`: Y축1 값 (심박수)
- `y2`: Y축2 값 (기타 데이터)

## 서버 연결

앱은 자동으로 다음 서버에 연결을 시도합니다:
- **호스트**: 211.221.184.17
- **포트**: 8888
- **프로토콜**: UDP

## 기술 스택

- **Flutter**: 크로스 플랫폼 앱 개발
- **Provider**: 상태 관리
- **fl_chart**: 그래프 시각화
- **dart:io**: UDP 통신

## 파일 구조

```
lib/
├── main.dart                 # 앱 진입점
├── models/
│   └── heartrate_data.dart   # 데이터 모델
├── services/
│   └── udp_service.dart      # UDP 통신 서비스
├── providers/
│   └── heartrate_provider.dart # 상태 관리
├── screens/
│   └── heartrate_screen.dart # 메인 화면
└── widgets/
    ├── heartrate_chart.dart  # 그래프 위젯
    ├── status_panel.dart     # 상태 패널
    └── legend_panel.dart     # 범례 패널
```

## 테스트

```bash
# 단위 테스트 실행
flutter test

# 코드 분석
flutter analyze
```

## 빌드

### Android APK
```bash
flutter build apk --release
```

### Windows 실행 파일
```bash
flutter build windows --release
```

## 문제 해결

### 연결 문제
- 네트워크 연결 상태 확인
- 서버 주소 및 포트 확인
- 방화벽 설정 확인

### 데이터 표시 문제
- 서버에서 올바른 형식의 데이터 전송 확인
- 앱 재시작 후 재연결 시도

## 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 기여

버그 리포트나 기능 제안은 GitHub Issues를 통해 제출해 주세요.

---

**개발 완료일**: 2025년 9월 22일  
**버전**: 1.0.0  
**플랫폼**: Android, Windows
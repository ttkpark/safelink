# SafeLink System

SafeLink System은 플랫폼 기반 노동자의 안전을 실시간으로 모니터링하고 위험 상황을 즉각적으로 대응하기 위한 통합형 웨어러블 안전 관리 플랫폼입니다.

## 시스템 구성

SafeLink System은 다음과 같이 구성됩니다:
- **SafeLink Band**: 밴드 형태의 착용형 장치 (심박수, 체온 측정)
- **SafeLink Hub**: 작업복 부착용 웨어러블 허브 (환경 센싱, 알림)
- **SafeLink App**: 노동자용/관리자용 모바일 애플리케이션
- **Firebase 서버**: 실시간 데이터 저장 및 관리

## 하드웨어 구성

### SafeLink Band (밴드형 착용 장치)

#### 센서 구성
| 센서 | 모델명 | 기능 | 연결 방식 |
|------|--------|------|-----------|
| 심박센서 | MAX30102 | 심박수 측정, 산소포화도 | I²C |
| 체온센서 | 1782 | 피부 온도 측정 | I²C |
| 온습도센서 | SEN0572 | WBGT 추정용 환경 데이터 | I²C |

#### 주요 특징
- **마이크로컨트롤러**: ESP32-C6
- **통신**: BLE 5.0, Wi-Fi
- **배터리**: 약 300mAh (1주일 사용 목표)
- **전원 관리**: 딥슬립/인터럽트 방식

### SafeLink Hub (작업복 부착용 허브)

#### 센서 구성
| 센서 | 모델명 | 기능 | 연결 방식 |
|------|--------|------|-----------|
| 마이크 | HTC-AS01 | 소음 측정 (dB) | ADC |
| 온습도센서 | SEN0572 | WBGT 계산용 환경 데이터 | I²C |

#### 출력 장치
| 장치 | 모델명 | 기능 | 연결 방식 |
|------|--------|------|-----------|
| 스피커 | FIT0502 | 음성 경고 (1W) | PWM |
| 진동모터 | 3166040004 | 진동 알림 | GPIO |
| 음성출력모듈 | DFR0299 | 음성 메시지 재생 | UART |

#### 주요 특징
- **마이크로컨트롤러**: ESP32-C6
- **통신**: BLE 5.0, Wi-Fi
- **배터리**: 약 3000mAh (48시간 사용 목표)
- **전원 관리**: 스탠바이 모드

### 확장 포트
- **I²C 포트**: 산소/가스 센서, 피부전도 센서 등 외부 모듈 확장
- **UART 포트**: 추가 통신 모듈 연결

## 하드웨어 설계 요구사항

### 전원 관리
1. **저전력 설계**
   - SafeLink Band: 딥슬립 모드로 1주일 사용
   - SafeLink Hub: 스탠바이 모드로 48시간 사용
   - 배터리 잔량 모니터링 및 충전 관리

2. **전원 안정화**
   - 3.3V 레귤레이터 사용
   - 배터리 보호 회로 (과충전, 과방전 방지)
   - 전원 상태 LED 표시

### 통신 설계
1. **BLE 통신**
   - BLE 5.0 프로토콜 사용
   - 센서 데이터 실시간 전송
   - 연결 상태 모니터링

2. **Wi-Fi 통신**
   - 데이터 백업 및 동기화
   - 펌웨어 업데이트 지원

### 센서 인터페이스
1. **I²C 센서들**
   - MAX30102 (심박센서)
   - 1782 (체온센서)
   - SEN0572 (온습도센서)
   - I²C 주소 충돌 방지

2. **I2S 센서**
   - HTC-AS01 (마이크) - 소음 레벨 측정(신호처리 RMS 계산)
   - 적절한 샘플링 레이트 설정

### 출력 장치 제어
1. **음성 출력**
   - DFR0299 모듈을 통한 음성 메시지 재생
   - FIT0502 스피커로 경고음 출력
   - 볼륨 제어 기능

2. **진동 알림**
   - 3166040004 진동모터 제어
   - 패턴별 진동 신호 (경고/위험/치명적)

### PCB 설계 고려사항
1. **크기 및 형태**
   - SafeLink Band: 밴드 형태로 착용 가능한 크기
   - SafeLink Hub: 작업복에 부착 가능한 크기
   - 방수/방진 처리

2. **신호 무결성**
   - 고주파 노이즈 방지
   - 센서 신호 필터링
   - 접지 설계 최적화

3. **열 관리**
   - 전력 소모 부품의 열 방출 설계
   - 온도 센서 정확도 보장

### 안전성 및 신뢰성
1. **EMC 대응**
   - 전자파 간섭 방지
   - ESD 보호 회로

2. **환경 대응**
   - 온도 범위: -10°C ~ 50°C
   - 습도 범위: 10% ~ 90% RH
   - IP54 이상의 방수/방진 등급

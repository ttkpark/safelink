#!/usr/bin/env python3
"""
SafeLink Wearable Hub - 시스템 통합 테스트 스크립트

이 스크립트는 ESP32와 시리얼 통신을 통해 시스템 상태를 모니터링하고
기본적인 기능 테스트를 수행합니다.
"""

import serial
import time
import re
import sys
from datetime import datetime

class SafeLinkTester:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.test_results = []
        
    def connect(self):
        """시리얼 포트에 연결"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=5)
            print(f"✅ 시리얼 포트 {self.port}에 연결되었습니다.")
            return True
        except Exception as e:
            print(f"❌ 시리얼 포트 연결 실패: {e}")
            return False
    
    def disconnect(self):
        """시리얼 포트 연결 해제"""
        if self.serial:
            self.serial.close()
            print("🔌 시리얼 포트 연결이 해제되었습니다.")
    
    def send_command(self, command, timeout=5):
        """ESP32에 명령 전송"""
        if not self.serial:
            return None
        
        try:
            self.serial.write(f"{command}\n".encode())
            time.sleep(0.1)
            
            response = ""
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response += line + "\n"
                        if "I (" in line or "E (" in line or "W (" in line:
                            break
            return response
        except Exception as e:
            print(f"❌ 명령 전송 실패: {e}")
            return None
    
    def wait_for_pattern(self, pattern, timeout=30):
        """특정 패턴이 나타날 때까지 대기"""
        if not self.serial:
            return False
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if line and re.search(pattern, line):
                    print(f"✅ 패턴 발견: {line}")
                    return True
            time.sleep(0.1)
        
        print(f"❌ 패턴 '{pattern}'을 {timeout}초 내에 찾지 못했습니다.")
        return False
    
    def test_system_startup(self):
        """시스템 시작 테스트"""
        print("\n🔧 시스템 시작 테스트...")
        
        # 시스템 시작 로그 확인
        if self.wait_for_pattern("Starting ESP32C3 SafeLink Wearable Hub Application", 10):
            self.test_results.append(("시스템 시작", "PASS"))
        else:
            self.test_results.append(("시스템 시작", "FAIL"))
            return False
        
        # 데이터 매니저 초기화 확인
        if self.wait_for_pattern("Data manager initialized successfully", 5):
            self.test_results.append(("데이터 매니저 초기화", "PASS"))
        else:
            self.test_results.append(("데이터 매니저 초기화", "FAIL"))
        
        # 블루투스 초기화 확인
        if self.wait_for_pattern("Bluetooth initialized successfully", 5):
            self.test_results.append(("블루투스 초기화", "PASS"))
        else:
            self.test_results.append(("블루투스 초기화", "FAIL"))
        
        # 시뮬레이터 시작 확인
        if self.wait_for_pattern("Test simulator started successfully", 5):
            self.test_results.append(("테스트 시뮬레이터 시작", "PASS"))
        else:
            self.test_results.append(("테스트 시뮬레이터 시작", "FAIL"))
        
        return True
    
    def test_data_generation(self):
        """데이터 생성 테스트"""
        print("\n📊 데이터 생성 테스트...")
        
        # 밴드 데이터 생성 확인
        if self.wait_for_pattern("Simulated band data:", 10):
            self.test_results.append(("밴드 데이터 생성", "PASS"))
        else:
            self.test_results.append(("밴드 데이터 생성", "FAIL"))
        
        # 허브 데이터 생성 확인
        if self.wait_for_pattern("Simulated hub data:", 10):
            self.test_results.append(("허브 데이터 생성", "PASS"))
        else:
            self.test_results.append(("허브 데이터 생성", "FAIL"))
        
        # 터미널 출력 확인
        if self.wait_for_pattern("=== SafeLink Wearable Hub Data ===", 15):
            self.test_results.append(("터미널 데이터 출력", "PASS"))
        else:
            self.test_results.append(("터미널 데이터 출력", "FAIL"))
    
    def test_data_validation(self):
        """데이터 검증 테스트"""
        print("\n🔍 데이터 검증 테스트...")
        
        # 데이터 범위 검증
        start_time = time.time()
        valid_data_count = 0
        
        while time.time() - start_time < 30:  # 30초 동안 모니터링
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if "Simulated band data:" in line:
                    # 데이터 범위 검증
                    if self.validate_band_data(line):
                        valid_data_count += 1
                elif "Simulated hub data:" in line:
                    # 허브 데이터 범위 검증
                    if self.validate_hub_data(line):
                        valid_data_count += 1
        
        if valid_data_count >= 5:  # 최소 5개의 유효한 데이터
            self.test_results.append(("데이터 범위 검증", "PASS"))
        else:
            self.test_results.append(("데이터 범위 검증", "FAIL"))
    
    def validate_band_data(self, line):
        """밴드 데이터 범위 검증"""
        try:
            # 외기 온도 (15~35°C)
            temp_match = re.search(r'T=(\d+\.\d+)°C', line)
            if temp_match:
                temp = float(temp_match.group(1))
                if not (15.0 <= temp <= 35.0):
                    return False
            
            # 외기 습도 (30~80%)
            hum_match = re.search(r'H=(\d+\.\d+)%', line)
            if hum_match:
                hum = float(hum_match.group(1))
                if not (30.0 <= hum <= 80.0):
                    return False
            
            # 피부온도 (35.5~37.5°C)
            skin_match = re.search(r'Skin=(\d+\.\d+)°C', line)
            if skin_match:
                skin = float(skin_match.group(1))
                if not (35.5 <= skin <= 37.5):
                    return False
            
            # 심박수 (60~100 BPM)
            hr_match = re.search(r'HR=(\d+)', line)
            if hr_match:
                hr = int(hr_match.group(1))
                if not (60 <= hr <= 100):
                    return False
            
            # SpO2 (95~99%)
            spo2_match = re.search(r'SpO2=(\d+\.\d+)%', line)
            if spo2_match:
                spo2 = float(spo2_match.group(1))
                if not (95.0 <= spo2 <= 99.0):
                    return False
            
            return True
        except:
            return False
    
    def validate_hub_data(self, line):
        """허브 데이터 범위 검증"""
        try:
            # 평균소음 (50~80 dB)
            noise_match = re.search(r'Noise=(\d+\.\d+)dB', line)
            if noise_match:
                noise = float(noise_match.group(1))
                if not (50.0 <= noise <= 80.0):
                    return False
            
            # WBGT (20~35°C)
            wbgt_match = re.search(r'WBGT=(\d+\.\d+)°C', line)
            if wbgt_match:
                wbgt = float(wbgt_match.group(1))
                if not (20.0 <= wbgt <= 35.0):
                    return False
            
            return True
        except:
            return False
    
    def test_alarm_system(self):
        """경보 시스템 테스트"""
        print("\n🚨 경보 시스템 테스트...")
        
        # 경보 상태 로그 확인
        if self.wait_for_pattern("Alarm Status:", 10):
            self.test_results.append(("경보 상태 출력", "PASS"))
        else:
            self.test_results.append(("경보 상태 출력", "FAIL"))
    
    def print_test_results(self):
        """테스트 결과 출력"""
        print("\n" + "="*50)
        print("📋 테스트 결과 요약")
        print("="*50)
        
        pass_count = 0
        total_count = len(self.test_results)
        
        for test_name, result in self.test_results:
            status = "✅ PASS" if result == "PASS" else "❌ FAIL"
            print(f"{test_name:<25} {status}")
            if result == "PASS":
                pass_count += 1
        
        print("-"*50)
        print(f"총 테스트: {total_count}, 통과: {pass_count}, 실패: {total_count - pass_count}")
        
        if pass_count == total_count:
            print("🎉 모든 테스트가 통과했습니다!")
            return True
        else:
            print("⚠️  일부 테스트가 실패했습니다.")
            return False

def main():
    """메인 함수"""
    print("🚀 SafeLink Wearable Hub - 시스템 통합 테스트")
    print("="*50)
    
    # 포트 설정 (Windows의 경우 'COM3' 등으로 변경)
    port = '/dev/ttyUSB0'
    if len(sys.argv) > 1:
        port = sys.argv[1]
    
    tester = SafeLinkTester(port=port)
    
    if not tester.connect():
        print("❌ 연결 실패로 테스트를 중단합니다.")
        return False
    
    try:
        # 시스템 시작 테스트
        if not tester.test_system_startup():
            print("❌ 시스템 시작 테스트 실패")
            return False
        
        # 데이터 생성 테스트
        tester.test_data_generation()
        
        # 데이터 검증 테스트
        tester.test_data_validation()
        
        # 경보 시스템 테스트
        tester.test_alarm_system()
        
        # 결과 출력
        success = tester.print_test_results()
        
        return success
        
    except KeyboardInterrupt:
        print("\n⚠️  사용자에 의해 테스트가 중단되었습니다.")
        return False
    except Exception as e:
        print(f"❌ 테스트 중 오류 발생: {e}")
        return False
    finally:
        tester.disconnect()

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)

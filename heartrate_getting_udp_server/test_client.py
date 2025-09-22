#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
테스트용 UDP 클라이언트
서버에 연결하고 심박수 데이터를 시뮬레이션하여 전송
"""

import socket
import time
import random
import math
import threading
import os
import sys
import argparse
from datetime import datetime

class TestClient:
    def __init__(self, server_host='localhost', server_port=8888):
        self.server_host = server_host
        self.server_port = server_port
        self.socket = None
        self.running = False
        self.data_dir = "saved_data"  # 저장된 데이터 디렉토리
    
    def find_data_file(self, day, hour, minute, second):
        """특정 시간의 데이터 파일 찾기"""
        try:
            # 현재 년도와 월 사용 (실제로는 더 정확한 방법 필요)
            now = datetime.now()
            year = now.strftime("%Y")
            month = now.strftime("%m")
            
            # 파일 경로 구성 (새로운 파일명 형식)
            file_path = os.path.join(self.data_dir, year, month, day, hour, minute, f"{year}_{month}_{day}_{hour}_{minute}_{second}.txt")
            
            if os.path.exists(file_path):
                return file_path
            else:
                print(f"❌ 파일을 찾을 수 없습니다: {file_path}")
                return None
                
        except Exception as e:
            print(f"❌ 파일 검색 오류: {e}")
            return None
    
    def read_and_send_data(self, file_path):
        """저장된 데이터를 읽어서 재전송"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # 패킷 데이터 추출
            packets = []
            lines = content.split('\n')
            current_packet = []
            in_data_section = False
            
            for line in lines:
                if line.startswith("=== Packet at"):
                    if current_packet and in_data_section:
                        packets.append('\n'.join(current_packet))
                    current_packet = []
                    in_data_section = False
                elif line.startswith("Data:"):
                    in_data_section = True
                elif in_data_section and not line.startswith("="):
                    current_packet.append(line)
            
            # 마지막 패킷 추가
            if current_packet and in_data_section:
                packets.append('\n'.join(current_packet))
            
            print(f"📁 파일에서 {len(packets)}개의 패킷을 찾았습니다")
            
            # 각 패킷을 재전송
            for i, packet_data in enumerate(packets):
                if packet_data.strip():
                    self.send_message(packet_data.strip())
                    print(f"📤 패킷 {i+1}/{len(packets)} 재전송 완료")
                    time.sleep(0.1)  # 0.1초 간격으로 전송
            
            return True
            
        except Exception as e:
            print(f"❌ 데이터 읽기/전송 오류: {e}")
            return False
    
    def list_available_data(self):
        """사용 가능한 데이터 파일 목록 표시"""
        try:
            if not os.path.exists(self.data_dir):
                print("❌ 저장된 데이터 디렉토리가 없습니다.")
                return
            
            print("📁 사용 가능한 데이터 파일:")
            print("형식: python test_client.py --time YYYYMMDDHHMMSS")
            print("-" * 50)
            
            # 년도별로 검색
            for year in os.listdir(self.data_dir):
                year_path = os.path.join(self.data_dir, year)
                if not os.path.isdir(year_path):
                    continue
                    
                for month in os.listdir(year_path):
                    month_path = os.path.join(year_path, month)
                    if not os.path.isdir(month_path):
                        continue
                        
                    for day in os.listdir(month_path):
                        day_path = os.path.join(month_path, day)
                        if not os.path.isdir(day_path):
                            continue
                            
                        for hour in os.listdir(day_path):
                            hour_path = os.path.join(day_path, hour)
                            if not os.path.isdir(hour_path):
                                continue
                                
                            for minute in os.listdir(hour_path):
                                minute_path = os.path.join(hour_path, minute)
                                if not os.path.isdir(minute_path):
                                    continue
                                    
                                for filename in os.listdir(minute_path):
                                    if filename.endswith('.txt'):
                                        # 파일명에서 시간 정보 추출 (새로운 형식: YYYY_MM_DD_HH_MM_SS.txt)
                                        time_str = filename.replace('.txt', '')
                                        if '_' in time_str and len(time_str.split('_')) == 6:
                                            parts = time_str.split('_')
                                            file_year = parts[0]
                                            file_month = parts[1]
                                            file_day = parts[2]
                                            file_hour = parts[3]
                                            file_minute = parts[4]
                                            file_second = parts[5]
                                            
                                            print(f"📄 {file_year}{file_month}{file_day}{file_hour}{file_minute}{file_second} - {filename}")
            
        except Exception as e:
            print(f"❌ 데이터 목록 조회 오류: {e}")
        
    def connect(self):
        """서버에 연결"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.running = True
            
            # 서버에 등록
            self.send_message("start")
            print("✅ 서버에 등록됨")
            
            return True
        except Exception as e:
            print(f"❌ 연결 실패: {e}")
            return False
    
    def send_message(self, message):
        """메시지 전송"""
        try:
            self.socket.sendto(message.encode('utf-8'), (self.server_host, self.server_port))
            print(f"📤 전송: {message}")
        except Exception as e:
            print(f"❌ 전송 실패: {e}")
    
    def simulate_heart_rate_data(self):
        """심박수 데이터 시뮬레이션"""
        x_value = 0
        base_heart_rate = 70
        base_other = 50
        
        while self.running:
            # 심박수 시뮬레이션 (60-100 BPM)
            heart_rate = base_heart_rate + random.randint(-10, 10) + int(10 * math.sin(x_value * 0.1))
            heart_rate = max(60, min(100, heart_rate))
            
            # 기타 데이터 시뮬레이션
            other_data = base_other + random.randint(-5, 5) + int(5 * math.cos(x_value * 0.15))
            other_data = max(30, min(70, other_data))
            
            # 메시지 형식: "x,y1,y2" (새로운 형식)
            message = f"{x_value:.0f},{heart_rate},{other_data}"
            
            self.send_message(message)
            
            x_value += 0.5
            time.sleep(1)  # 1초마다 데이터 전송
    
    def start_simulation(self):
        """시뮬레이션 시작"""
        if not self.connect():
            return
        
        print("🚀 심박수 데이터 시뮬레이션 시작")
        print("⏹️  Ctrl+C로 중지")
        
        try:
            self.simulate_heart_rate_data()
        except KeyboardInterrupt:
            print("\n⏹️  시뮬레이션 중지됨")
        finally:
            self.disconnect()
    
    def disconnect(self):
        """연결 해제"""
        self.running = False
        if self.socket:
            self.socket.close()
        print("🔌 연결 해제됨")

def main():
    parser = argparse.ArgumentParser(description='UDP 테스트 클라이언트')
    parser.add_argument('--time', type=str, help='재전송할 데이터의 시간 (YYYYMMDDHHMMSS 형식)')
    parser.add_argument('--list', action='store_true', help='사용 가능한 데이터 파일 목록 표시')
    parser.add_argument('--host', type=str, default='localhost', help='서버 호스트 (기본값: localhost)')
    parser.add_argument('--port', type=int, default=8888, help='서버 포트 (기본값: 8888)')
    
    args = parser.parse_args()
    
    print("🧪 테스트 클라이언트 시작")
    print(f"📡 서버 주소: {args.host}:{args.port}")
    
    client = TestClient(args.host, args.port)
    
    if args.list:
        # 사용 가능한 데이터 파일 목록 표시
        client.list_available_data()
        return
    
    if args.time:
        # 특정 시간의 데이터 재전송
        if len(args.time) != 14:
            print("❌ 시간 형식이 올바르지 않습니다. YYYYMMDDHHMMSS 형식을 사용하세요.")
            return
        
        try:
            # 시간 문자열 파싱
            year = args.time[:4]
            month = args.time[4:6]
            day = args.time[6:8]
            hour = args.time[8:10]
            minute = args.time[10:12]
            second = args.time[12:14]
            
            print(f"🕐 재전송할 시간: {year}-{month}-{day} {hour}:{minute}:{second}")
            
            # 서버에 연결
            if not client.connect():
                return
            
            # 데이터 파일 찾기
            file_path = client.find_data_file(day, hour, minute, second)
            if file_path:
                print(f"📁 데이터 파일 찾음: {file_path}")
                client.read_and_send_data(file_path)
            else:
                print("❌ 해당 시간의 데이터 파일을 찾을 수 없습니다.")
            
            client.disconnect()
            
        except Exception as e:
            print(f"❌ 재전송 오류: {e}")
    else:
        # 일반 시뮬레이션 모드
        print("💡 이 클라이언트는 심박수 데이터를 시뮬레이션하여 전송합니다")
        print("💡 특정 시간의 데이터를 재전송하려면 --time 옵션을 사용하세요")
        print("💡 사용 가능한 데이터 목록을 보려면 --list 옵션을 사용하세요\n")
        client.start_simulation()

if __name__ == "__main__":
    main()

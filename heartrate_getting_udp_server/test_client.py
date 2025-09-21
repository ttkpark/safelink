#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
테스트용 UDP 클라이언트
서버에 연결하고 심박수 데이터를 시뮬레이션하여 전송
"""

import socket
import time
import random
import threading

class TestClient:
    def __init__(self, server_host='localhost', server_port=8888):
        self.server_host = server_host
        self.server_port = server_port
        self.socket = None
        self.running = False
        
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
            heart_rate = base_heart_rate + random.randint(-10, 10) + int(10 * random.sin(x_value * 0.1))
            heart_rate = max(60, min(100, heart_rate))
            
            # 기타 데이터 시뮬레이션
            other_data = base_other + random.randint(-5, 5) + int(5 * random.cos(x_value * 0.15))
            other_data = max(30, min(70, other_data))
            
            # 메시지 형식: "send\r\n메시지 : x축, y축1, y축2"
            message = f"send\r\n메시지 : {x_value:.1f}, {heart_rate}, {other_data}"
            
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
    print("🧪 테스트 클라이언트 시작")
    print("📡 서버 주소: localhost:8888")
    print("💡 이 클라이언트는 심박수 데이터를 시뮬레이션하여 전송합니다\n")
    
    client = TestClient()
    client.start_simulation()

if __name__ == "__main__":
    main()

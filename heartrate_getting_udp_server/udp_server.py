#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UDP 서버 - 심박수 데이터 브로드캐스트 서버
장치들이 'start' 메시지를 보내면 등록하고, 'send\r\n'으로 시작하는 데이터를 모든 등록된 장치에 브로드캐스트
"""

import socket
import threading
import time
import json
import os
from datetime import datetime

class HeartRateUDPServer:
    def __init__(self, host='0.0.0.0', port=8888):
        self.host = host
        self.port = port
        self.socket = None
        self.registered_devices = {}  # {address: {'last_seen': timestamp, 'name': str}}
        self.running = False
        self.data_dir = "saved_data"  # 데이터 저장 디렉토리
        self.lock = threading.Lock()
        
        # 데이터 저장 디렉토리 생성
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
    
    def save_packet_to_file(self, data, address):
        """패킷을 시간별 파일에 저장"""
        try:
            # 현재 시간 정보
            now = datetime.now()
            year = now.strftime("%Y")
            month = now.strftime("%m")
            day = now.strftime("%d")
            hour = now.strftime("%H")
            minute = now.strftime("%M")
            second = now.strftime("%S")
            
            # 디렉토리 구조: saved_data/년/월/일/시/분/
            dir_path = os.path.join(self.data_dir, year, month, day, hour, minute)
            os.makedirs(dir_path, exist_ok=True)
            
            # 파일명: 년_월_일_시_분_초.txt (사람이 읽기 쉬운 형식)
            filename = f"{year}_{month}_{day}_{hour}_{minute}_{second}.txt"
            filepath = os.path.join(dir_path, filename)
            
            # 패킷 정보와 함께 저장
            packet_info = {
                'timestamp': now.isoformat(),
                'address': f"{address[0]}:{address[1]}",
                'size': len(data),
                'data': data.decode('utf-8', errors='ignore')
            }
            
            with open(filepath, 'a', encoding='utf-8') as f:
                f.write(f"=== Packet at {now.isoformat()} ===\n")
                f.write(f"From: {address[0]}:{address[1]}\n")
                f.write(f"Size: {len(data)} bytes\n")
                f.write(f"Data:\n{data.decode('utf-8', errors='ignore')}\n")
                f.write("=" * 50 + "\n\n")
            
            print(f"💾 패킷 저장됨: {filepath} ({len(data)} bytes)")
            
        except Exception as e:
            print(f"❌ 파일 저장 오류: {e}")
    
    def start_server(self):
        """서버 시작"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.host, self.port))
            self.running = True
            
            print(f"🚀 UDP 서버가 시작되었습니다: {self.host}:{self.port}")
            print("📱 장치들이 'start' 메시지를 보내면 등록됩니다")
            print("📊 'send\\r\\n'으로 시작하는 데이터를 모든 장치에 브로드캐스트합니다")
            print("⏹️  Ctrl+C로 서버를 종료할 수 있습니다\n")
            
            # 클라이언트 메시지 처리 스레드 시작
            self.start_message_handler()
            
            # 장치 정리 스레드 시작 (5분 이상 비활성 장치 제거)
            self.start_cleanup_thread()
            
            # 서버 상태 모니터링
            self.start_status_monitor()
            
        except Exception as e:
            print(f"❌ 서버 시작 실패: {e}")
            return False
            
        return True
    
    def start_message_handler(self):
        """메시지 처리 스레드 시작"""
        def handle_messages():
            while self.running:
                try:
                    data, address = self.socket.recvfrom(6000)
                    message = data.decode('utf-8').strip()
                    
                    # 1000바이트 이상 패킷 저장
                    if len(data) >= 1000:
                        self.save_packet_to_file(data, address)
                    
                    # 메시지 처리
                    self.process_message(message, address)
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"⚠️  메시지 처리 오류: {e}")
        
        thread = threading.Thread(target=handle_messages, daemon=True)
        thread.start()
    
    def process_message(self, message, address):
        """메시지 처리"""
        current_time = time.time()
        
        with self.lock:
            # 장치 등록/업데이트
            self.registered_devices[address] = {
                'last_seen': current_time,
                'name': f"Device_{address[0]}:{address[1]}"
            }
        
        if message == "start":
            print(f"✅ 장치 등록됨: {address[0]}:{address[1]}")
            self.send_response(address, "registered")
            
        elif message.startswith("send"):
            # 데이터 추출 (send 이후 부분)
            if message.startswith("send\r\n"):
                data_part = message[6:]  # "send\r\n" 제거
            else:
                data_part = message[4:]  # "send" 제거
                if data_part.startswith(" "):
                    data_part = data_part[1:]  # 공백 제거
            
            print(f"📡 데이터 수신: {address[0]}:{address[1]} -> {data_part}")
            
            # 모든 등록된 장치에 브로드캐스트
            self.broadcast_data(data_part, exclude_address=address)
            
        elif message == "end":
            # 연결 종료 요청
            print(f"🔚 연결 종료 요청: {address[0]}:{address[1]}")
            with self.lock:
                if address in self.registered_devices:
                    del self.registered_devices[address]
            self.send_response(address, "disconnected")
            
        elif "," in message and not message.startswith("start"):
            # "x,y1,y2" 형식의 직접 데이터 전송
            print(f"📡 직접 데이터 수신: {address[0]}:{address[1]} -> {message}")
            
            # 모든 등록된 장치에 브로드캐스트
            self.broadcast_data(message, exclude_address=address)
            
        else:
            print(f"❓ 알 수 없는 메시지: {address[0]}:{address[1]} -> {message}")
    
    def broadcast_data(self, data, exclude_address=None):
        """모든 등록된 장치에 데이터 브로드캐스트"""
        if not self.registered_devices:
            print("⚠️  등록된 장치가 없습니다")
            return
        
        with self.lock:
            devices = list(self.registered_devices.keys())
        
        broadcast_count = 0
        for address in devices:
            if address != exclude_address:
                try:
                    self.socket.sendto(data.encode('utf-8'), address)
                    broadcast_count += 1
                except Exception as e:
                    print(f"⚠️  브로드캐스트 실패 {address}: {e}")
        
        print(f"📤 {broadcast_count}개 장치에 브로드캐스트 완료")
    
    def send_response(self, address, message):
        """특정 장치에 응답 전송"""
        try:
            self.socket.sendto(message.encode('utf-8'), address)
        except Exception as e:
            print(f"⚠️  응답 전송 실패 {address}: {e}")
    
    def start_cleanup_thread(self):
        """비활성 장치 정리 스레드"""
        def cleanup():
            while self.running:
                time.sleep(60)  # 1분마다 체크
                current_time = time.time()
                
                with self.lock:
                    inactive_devices = []
                    for address, info in self.registered_devices.items():
                        if current_time - info['last_seen'] > 300:  # 5분 이상 비활성
                            inactive_devices.append(address)
                    
                    for address in inactive_devices:
                        del self.registered_devices[address]
                        print(f"🗑️  비활성 장치 제거: {address[0]}:{address[1]}")
        
        thread = threading.Thread(target=cleanup, daemon=True)
        thread.start()
    
    def start_status_monitor(self):
        """서버 상태 모니터링"""
        def monitor():
            while self.running:
                time.sleep(30)  # 30초마다 상태 출력
                with self.lock:
                    device_count = len(self.registered_devices)
                print(f"📊 현재 등록된 장치: {device_count}개")
        
        thread = threading.Thread(target=monitor, daemon=True)
        thread.start()
    
    def stop_server(self):
        """서버 중지"""
        self.running = False
        if self.socket:
            self.socket.close()
        print("🛑 서버가 중지되었습니다")

def main():
    # 서버 설정
    HOST = '0.0.0.0'  # 모든 인터페이스에서 수신
    PORT = 8888       # 포트 번호
    
    server = HeartRateUDPServer(HOST, PORT)
    
    try:
        if server.start_server():
            # 서버 실행 중...
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\n⏹️  서버 종료 요청됨...")
    finally:
        server.stop_server()

if __name__ == "__main__":
    main()

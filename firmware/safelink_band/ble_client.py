#!/usr/bin/env python3
"""
ESP32-C3 BLE GATT Client
SafeLink_Test 디바이스에 연결하여 실시간 센서 데이터를 수신합니다.
"""

import asyncio
import struct
import time
from bleak import BleakScanner, BleakClient
from datetime import datetime

# BLE 디바이스 설정
DEVICE_NAME = "SafeLink_Test"
SERVICE_UUID = "00001810-0000-1000-8000-00805f9b34fb"  # Health Sensor Service
TEMPERATURE_UUID = "00002a6e-0000-1000-8000-00805f9b34fb"  # Temperature
HUMIDITY_UUID = "00002a6f-0000-1000-8000-00805f9b34fb"  # Humidity
HEART_RATE_UUID = "00002a37-0000-1000-8000-00805f9b34fb"  # Heart Rate
BODY_TEMP_UUID = "00002a1c-0000-1000-8000-00805f9b34fb"  # Body Temperature

class SensorData:
    def __init__(self):
        self.temperature = 0.0
        self.humidity = 0.0
        self.heart_rate = 0.0
        self.body_temperature = 0.0
        self.last_update = None

    def update_temperature(self, value):
        self.temperature = value
        self.last_update = datetime.now()
        print(f"[{self.last_update.strftime('%H:%M:%S')}] 🌡️  온도: {value:.1f}°C")

    def update_humidity(self, value):
        self.humidity = value
        self.last_update = datetime.now()
        print(f"[{self.last_update.strftime('%H:%M:%S')}] 💧 습도: {value:.1f}%")

    def update_heart_rate(self, value):
        self.heart_rate = value
        self.last_update = datetime.now()
        print(f"[{self.last_update.strftime('%H:%M:%S')}] ❤️  심박수: {value:.1f} BPM")

    def update_body_temperature(self, value):
        self.body_temperature = value
        self.last_update = datetime.now()
        print(f"[{self.last_update.strftime('%H:%M:%S')}] 🔥 체온: {value:.1f}°C")

    def print_summary(self):
        print("\n" + "="*50)
        print("📊 현재 센서 데이터 요약")
        print("="*50)
        print(f"🌡️  온도: {self.temperature:.1f}°C")
        print(f"💧 습도: {self.humidity:.1f}%")
        print(f"❤️  심박수: {self.heart_rate:.1f} BPM")
        print(f"🔥 체온: {self.body_temperature:.1f}°C")
        if self.last_update:
            print(f"⏰ 마지막 업데이트: {self.last_update.strftime('%Y-%m-%d %H:%M:%S')}")
        print("="*50)

# 센서 데이터 인스턴스
sensor_data = SensorData()

def temperature_callback(sender, data):
    """온도 데이터 콜백"""
    try:
        value = struct.unpack('<f', data)[0]  # float (4 bytes)
        sensor_data.update_temperature(value)
    except Exception as e:
        print(f"온도 데이터 파싱 오류: {e}")

def humidity_callback(sender, data):
    """습도 데이터 콜백"""
    try:
        value = struct.unpack('<f', data)[0]  # float (4 bytes)
        sensor_data.update_humidity(value)
    except Exception as e:
        print(f"습도 데이터 파싱 오류: {e}")

def heart_rate_callback(sender, data):
    """심박수 데이터 콜백"""
    try:
        value = struct.unpack('<f', data)[0]  # float (4 bytes)
        sensor_data.update_heart_rate(value)
    except Exception as e:
        print(f"심박수 데이터 파싱 오류: {e}")

def body_temp_callback(sender, data):
    """체온 데이터 콜백"""
    try:
        value = struct.unpack('<f', data)[0]  # float (4 bytes)
        sensor_data.update_body_temperature(value)
    except Exception as e:
        print(f"체온 데이터 파싱 오류: {e}")

async def scan_and_connect():
    """BLE 디바이스 스캔 및 연결"""
    print(f"🔍 '{DEVICE_NAME}' 디바이스를 스캔 중...")
    
    # 디바이스 스캔
    devices = await BleakScanner.discover(timeout=10.0)
    target_device = None
    
    for device in devices:
        if device.name == DEVICE_NAME:
            target_device = device
            print(f"✅ 디바이스 발견: {device.name} ({device.address})")
            break
    
    if not target_device:
        print(f"❌ '{DEVICE_NAME}' 디바이스를 찾을 수 없습니다.")
        print("사용 가능한 디바이스:")
        for device in devices:
            if device.name:
                print(f"  - {device.name} ({device.address})")
        return None
    
    return target_device

async def connect_and_monitor(device):
    """디바이스에 연결하고 센서 데이터 모니터링"""
    print(f"🔗 {device.name}에 연결 중...")
    
    try:
        async with BleakClient(device.address) as client:
            print(f"✅ 연결 성공!")
            print(f"📱 디바이스: {device.name}")
            print(f"📍 주소: {device.address}")
            print(f"🔋 배터리 레벨: {await client.read_gatt_char('00002a19-0000-1000-8000-00805f9b34fb') if '00002a19-0000-1000-8000-00805f9b34fb' in [char.uuid for char in client.services] else 'N/A'}")
            
            # 서비스 탐색
            services = await client.get_services()
            print(f"\n📋 사용 가능한 서비스:")
            for service in services:
                print(f"  - {service.uuid}")
                for char in service.characteristics:
                    print(f"    └─ {char.uuid} (읽기: {char.properties.read}, 쓰기: {char.properties.write}, 알림: {char.properties.notify})")
            
            # 특성 구독
            print(f"\n📡 센서 데이터 구독 시작...")
            
            # 온도 특성 구독
            try:
                await client.start_notify(TEMPERATURE_UUID, temperature_callback)
                print(f"✅ 온도 특성 구독 완료")
            except Exception as e:
                print(f"❌ 온도 특성 구독 실패: {e}")
            
            # 습도 특성 구독
            try:
                await client.start_notify(HUMIDITY_UUID, humidity_callback)
                print(f"✅ 습도 특성 구독 완료")
            except Exception as e:
                print(f"❌ 습도 특성 구독 실패: {e}")
            
            # 심박수 특성 구독
            try:
                await client.start_notify(HEART_RATE_UUID, heart_rate_callback)
                print(f"✅ 심박수 특성 구독 완료")
            except Exception as e:
                print(f"❌ 심박수 특성 구독 실패: {e}")
            
            # 체온 특성 구독
            try:
                await client.start_notify(BODY_TEMP_UUID, body_temp_callback)
                print(f"✅ 체온 특성 구독 완료")
            except Exception as e:
                print(f"❌ 체온 특성 구독 실패: {e}")
            
            print(f"\n🎯 실시간 센서 데이터 수신 중... (Ctrl+C로 종료)")
            print("-" * 50)
            
            # 실시간 데이터 수신
            start_time = time.time()
            while True:
                await asyncio.sleep(1)
                
                # 10초마다 요약 출력
                if int(time.time() - start_time) % 10 == 0:
                    sensor_data.print_summary()
                
    except Exception as e:
        print(f"❌ 연결 오류: {e}")
        return False
    
    return True

async def main():
    """메인 함수"""
    print("🚀 ESP32-C3 BLE GATT Client 시작")
    print("=" * 50)
    
    while True:
        try:
            # 디바이스 스캔 및 연결
            device = await scan_and_connect()
            if not device:
                print("🔄 5초 후 재시도...")
                await asyncio.sleep(5)
                continue
            
            # 연결 및 모니터링
            success = await connect_and_monitor(device)
            if not success:
                print("🔄 5초 후 재시도...")
                await asyncio.sleep(5)
                continue
                
        except KeyboardInterrupt:
            print("\n👋 프로그램을 종료합니다.")
            break
        except Exception as e:
            print(f"❌ 예상치 못한 오류: {e}")
            print("🔄 5초 후 재시도...")
            await asyncio.sleep(5)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 프로그램을 종료합니다.")


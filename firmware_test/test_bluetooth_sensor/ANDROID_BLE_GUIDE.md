# Android BLE 개발 가이드

## 개요
이 문서는 ESP32-C6 기반 BLE 센서 디바이스와 안드로이드 앱 간의 통신을 위한 개발 가이드입니다.

## BLE 서비스 및 특성

### 1. Heart Rate Service (0x180D)
- **서비스 UUID**: 0x180D
- **특성**: Heart Rate Measurement (0x2A37)
- **데이터 형식**: 2바이트
  - 바이트 0: 플래그 (0x00)
  - 바이트 1: 심박수 값 (BPM)

### 2. Health Thermometer Service (0x1809)
- **서비스 UUID**: 0x1809
- **특성**: Temperature Measurement (0x2A6E)
- **데이터 형식**: 13바이트 (IEEE 11073-20601)
  - 바이트 0: 플래그 (0x00)
  - 바이트 1-4: 온도 값 (0.01°C 단위)
  - 바이트 5-11: 타임스탬프
  - 바이트 12: 온도 타입 (0x02 = Body)

### 3. Environmental Sensing Service (0x181A)
- **서비스 UUID**: 0x181A
- **특성**: Humidity (0x2A6F)
- **데이터 형식**: 5바이트
  - 바이트 0: 플래그 (0x00)
  - 바이트 1-4: 습도 값 (0.01% 단위)

- **특성**: Pressure (0x2A6D)
- **데이터 형식**: 5바이트
  - 바이트 0: 플래그 (0x00)
  - 바이트 1-4: 압력 값 (0.1 Pa 단위)

### 4. Custom Sensor Service (0x1810) ⭐ **중요**
- **서비스 UUID**: 0x1810
- **특성**: Sensor Data (0x2A6F) - **통합 센서 데이터**
- **데이터 형식**: 8바이트
  ```
  바이트 0: 심박수 하위 바이트 (BPM)
  바이트 1: 심박수 상위 바이트 (BPM)
  바이트 2: 온도 하위 바이트 (0.1°C 단위)
  바이트 3: 온도 상위 바이트 (0.1°C 단위)
  바이트 4: 습도 하위 바이트 (0.1% 단위)
  바이트 5: 습도 상위 바이트 (0.1% 단위)
  바이트 6: 예약 (0x00)
  바이트 7: 예약 (0x00)
  ```

- **특성**: Health Status (0x2A70)
- **데이터 형식**: 문자열
  - "Normal", "Elevated Heart Rate", "Temperature Alert", "Humidity Alert", "Warning", "Critical"

## 안드로이드 구현 가이드

### 1. 권한 설정
```xml
<!-- AndroidManifest.xml -->
<uses-permission android:name="android.permission.BLUETOOTH" />
<uses-permission android:name="android.permission.BLUETOOTH_ADMIN" />
<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />
<uses-permission android:name="android.permission.BLUETOOTH_SCAN" />
<uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" />
<uses-permission android:name="android.permission.ACCESS_COARSE_LOCATION" />

<!-- Android 12+ -->
<uses-permission android:name="android.permission.BLUETOOTH_SCAN" />
<uses-permission android:name="android.permission.BLUETOOTH_CONNECT" />
```

### 2. BLE 스캔 및 연결
```kotlin
class BleManager(private val context: Context) {
    private val bluetoothAdapter: BluetoothAdapter? = BluetoothAdapter.getDefaultAdapter()
    private var bluetoothGatt: BluetoothGatt? = null
    
    fun startScan() {
        val scanFilter = ScanFilter.Builder()
            .setDeviceName("ESP32C6_Sensor") // 디바이스 이름
            .build()
            
        val scanSettings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()
            
        bluetoothAdapter?.bluetoothLeScanner?.startScan(
            listOf(scanFilter), scanSettings, scanCallback
        )
    }
    
    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            // 디바이스 발견 시 연결
            connectToDevice(result.device)
        }
    }
    
    private fun connectToDevice(device: BluetoothDevice) {
        bluetoothGatt = device.connectGatt(context, false, gattCallback)
    }
}
```

### 3. GATT 콜백 구현
```kotlin
private val gattCallback = object : BluetoothGattCallback() {
    override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
        when (newState) {
            BluetoothProfile.STATE_CONNECTED -> {
                Log.i("BLE", "Connected to GATT server")
                gatt.discoverServices()
            }
            BluetoothProfile.STATE_DISCONNECTED -> {
                Log.i("BLE", "Disconnected from GATT server")
            }
        }
    }
    
    override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
        if (status == BluetoothGatt.GATT_SUCCESS) {
            // 서비스 발견 후 특성 구독
            subscribeToCharacteristics(gatt)
        }
    }
    
    override fun onCharacteristicChanged(
        gatt: BluetoothGatt,
        characteristic: BluetoothGattCharacteristic
    ) {
        // 데이터 수신 처리
        when (characteristic.uuid) {
            // 표준 서비스들
            UUID.fromString("00002a37-0000-1000-8000-00805f9b34fb") -> {
                processHeartRateData(characteristic.value)
            }
            UUID.fromString("00002a6e-0000-1000-8000-00805f9b34fb") -> {
                processTemperatureData(characteristic.value)
            }
            UUID.fromString("00002a6f-0000-1000-8000-00805f9b34fb") -> {
                processHumidityData(characteristic.value)
            }
            UUID.fromString("00002a6d-0000-1000-8000-00805f9b34fb") -> {
                processPressureData(characteristic.value)
            }
            // ⭐ 통합 센서 데이터 처리
            UUID.fromString("00002a6f-0000-1000-8000-00805f9b34fb") -> {
                processIntegratedSensorData(characteristic.value)
            }
            // 건강 상태 문자열
            UUID.fromString("00002a70-0000-1000-8000-00805f9b34fb") -> {
                processHealthStatus(characteristic.value)
            }
        }
    }
}
```

### 4. 통합 센서 데이터 처리 ⭐ **핵심**
```kotlin
private fun processIntegratedSensorData(data: ByteArray) {
    if (data.size >= 6) {
        // 심박수 (2바이트)
        val heartRate = (data[1].toInt() shl 8) or (data[0].toInt() and 0xFF)
        
        // 온도 (2바이트) - 0.1°C 단위
        val tempRaw = (data[3].toInt() shl 8) or (data[2].toInt() and 0xFF)
        val temperature = tempRaw / 10.0f // 0.1°C 단위를 실제 온도로 변환
        
        // 습도 (2바이트) - 0.1% 단위
        val humidityRaw = (data[5].toInt() shl 8) or (data[4].toInt() and 0xFF)
        val humidity = humidityRaw / 10.0f // 0.1% 단위를 실제 습도로 변환
        
        Log.i("BLE", "통합 센서 데이터 - 심박수: ${heartRate} BPM, 온도: ${temperature}°C, 습도: ${humidity}%")
        
        // UI 업데이트
        updateSensorUI(heartRate, temperature, humidity)
    }
}

private fun processHealthStatus(data: ByteArray) {
    val status = String(data, Charsets.UTF_8)
    Log.i("BLE", "건강 상태: $status")
    
    when (status) {
        "Normal" -> updateHealthStatus("정상")
        "Elevated Heart Rate" -> updateHealthStatus("심박수 상승")
        "Temperature Alert" -> updateHealthStatus("온도 경고")
        "Humidity Alert" -> updateHealthStatus("습도 경고")
        "Warning" -> updateHealthStatus("주의")
        "Critical" -> updateHealthStatus("위험")
        else -> updateHealthStatus("알 수 없음")
    }
}
```

### 5. 특성 구독 설정
```kotlin
private fun subscribeToCharacteristics(gatt: BluetoothGatt) {
    // Heart Rate Service
    val heartRateService = gatt.getService(UUID.fromString("0000180d-0000-1000-8000-00805f9b34fb"))
    val heartRateChar = heartRateService?.getCharacteristic(UUID.fromString("00002a37-0000-1000-8000-00805f9b34fb"))
    
    // Temperature Service
    val tempService = gatt.getService(UUID.fromString("00001809-0000-1000-8000-00805f9b34fb"))
    val tempChar = tempService?.getCharacteristic(UUID.fromString("00002a6e-0000-1000-8000-00805f9b34fb"))
    
    // Environmental Service
    val envService = gatt.getService(UUID.fromString("0000181a-0000-1000-8000-00805f9b34fb"))
    val humidityChar = envService?.getCharacteristic(UUID.fromString("00002a6f-0000-1000-8000-00805f9b34fb"))
    val pressureChar = envService?.getCharacteristic(UUID.fromString("00002a6d-0000-1000-8000-00805f9b34fb"))
    
    // ⭐ Custom Sensor Service (통합 데이터)
    val customService = gatt.getService(UUID.fromString("00001810-0000-1000-8000-00805f9b34fb"))
    val sensorDataChar = customService?.getCharacteristic(UUID.fromString("00002a6f-0000-1000-8000-00805f9b34fb"))
    val healthStatusChar = customService?.getCharacteristic(UUID.fromString("00002a70-0000-1000-8000-00805f9b34fb"))
    
    // 구독 활성화
    val characteristics = listOfNotNull(
        heartRateChar, tempChar, humidityChar, pressureChar, 
        sensorDataChar, healthStatusChar
    )
    
    characteristics.forEach { characteristic ->
        gatt.setCharacteristicNotification(characteristic, true)
        
        // Client Characteristic Configuration Descriptor 설정
        val descriptor = characteristic.getDescriptor(
            UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        )
        descriptor?.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        gatt.writeDescriptor(descriptor)
    }
}
```

### 6. 데이터 변환 유틸리티
```kotlin
object BleDataConverter {
    // 16비트 값을 실제 값으로 변환
    fun convertTemperature(rawValue: Int): Float {
        return rawValue / 10.0f // 0.1°C 단위
    }
    
    fun convertHumidity(rawValue: Int): Float {
        return rawValue / 10.0f // 0.1% 단위
    }
    
    // 바이트 배열에서 16비트 값 추출
    fun extractUInt16(data: ByteArray, offset: Int): Int {
        return (data[offset + 1].toInt() shl 8) or (data[offset].toInt() and 0xFF)
    }
    
    // 바이트 배열에서 32비트 값 추출
    fun extractUInt32(data: ByteArray, offset: Int): Long {
        return (data[offset + 3].toLong() shl 24) or
               (data[offset + 2].toLong() shl 16) or
               (data[offset + 1].toLong() shl 8) or
               (data[offset].toLong() and 0xFF)
    }
}
```

### 7. UI 업데이트 예시
```kotlin
private fun updateSensorUI(heartRate: Int, temperature: Float, humidity: Float) {
    runOnUiThread {
        heartRateTextView.text = "${heartRate} BPM"
        temperatureTextView.text = "${String.format("%.1f", temperature)}°C"
        humidityTextView.text = "${String.format("%.1f", humidity)}%"
        
        // 건강 상태에 따른 색상 변경
        when {
            heartRate > 100 || heartRate < 60 -> heartRateTextView.setTextColor(Color.RED)
            else -> heartRateTextView.setTextColor(Color.GREEN)
        }
        
        when {
            temperature > 37.5f || temperature < 35.0f -> temperatureTextView.setTextColor(Color.RED)
            else -> temperatureTextView.setTextColor(Color.GREEN)
        }
        
        when {
            humidity < 30.0f || humidity > 70.0f -> humidityTextView.setTextColor(Color.RED)
            else -> humidityTextView.setTextColor(Color.GREEN)
        }
    }
}
```

## 주의사항

1. **UUID 충돌**: `SENSOR_DATA_CHAR_UUID` (0x2A6F)가 `HUMIDITY_CHAR_UUID`와 동일합니다. 실제 구현에서는 다른 UUID를 사용해야 합니다.

2. **데이터 단위**: 센서 데이터는 0.1°C, 0.1% 단위로 전송되므로 안드로이드에서 10으로 나누어야 합니다.

3. **구독 설정**: 모든 특성에 대해 Client Characteristic Configuration Descriptor (0x2902)를 설정해야 합니다.

4. **에러 처리**: BLE 연결 실패, 데이터 손실 등에 대한 적절한 에러 처리가 필요합니다.

## 테스트 방법

1. nRF Connect 앱으로 디바이스 연결
2. Custom Sensor Service (0x1810) 확인
3. Sensor Data 특성 (0x2A6F) 구독
4. 8바이트 데이터 수신 확인
5. 안드로이드 앱에서 동일한 데이터 처리 테스트 
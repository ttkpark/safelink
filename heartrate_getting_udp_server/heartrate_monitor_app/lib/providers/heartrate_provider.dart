import 'dart:async';
import 'package:flutter/foundation.dart';
import '../models/heartrate_data.dart';
import '../services/udp_service.dart';

class HeartRateProvider with ChangeNotifier {
  final UDPService _udpService = UDPService();
  final List<HeartRateData> _dataPoints = [];
  final int _maxDataPoints = 200;
  
  String _connectionStatus = '연결 중...';
  bool _isConnected = false;
  StreamSubscription<HeartRateData>? _dataSubscription;
  StreamSubscription<String>? _statusSubscription;
  
  // 통계 데이터
  double _minY1 = 0.0;
  double _maxY1 = 100.0;
  double _minY2 = 0.0;
  double _maxY2 = 100.0;
  double _avgY1 = 0.0;
  double _avgY2 = 0.0;
  
  // 이전 값 저장 (오류 시 대체용)
  double _lastValidX = 0.0;
  double _lastValidY1 = 0.0;
  double _lastValidY2 = 0.0;
  bool _hasValidData = false;

  List<HeartRateData> get dataPoints => List.unmodifiable(_dataPoints);
  String get connectionStatus => _connectionStatus;
  bool get isConnected => _isConnected;
  double get minY1 => _minY1;
  double get maxY1 => _maxY1;
  double get minY2 => _minY2;
  double get maxY2 => _maxY2;
  double get avgY1 => _avgY1;
  double get avgY2 => _avgY2;

  HeartRateProvider() {
    _initializeSubscriptions();
    connect();
  }

  void _initializeSubscriptions() {
    _dataSubscription = _udpService.dataStream.listen(
      (data) {
        _addDataPoint(data);
      },
      onError: (error) {
        print('데이터 스트림 오류: $error');
      },
    );

    _statusSubscription = _udpService.statusStream.listen(
      (status) {
        _connectionStatus = status;
        _isConnected = _udpService.isConnected;
        notifyListeners();
      },
      onError: (error) {
        print('상태 스트림 오류: $error');
      },
    );
  }

  void _addDataPoint(HeartRateData data) {
    // X 값이 1이면 데이터 초기화 (Processing 코드와 동일한 로직)
    if (data.x == 1.0 && _dataPoints.isNotEmpty) {
      _dataPoints.clear();
      _lastValidX = 0.0;
      _lastValidY1 = 0.0;
      _lastValidY2 = 0.0;
      _hasValidData = false;
    }

    // 데이터 포인트 추가
    _dataPoints.add(data);
    
    // 최대 데이터 포인트 수 제한
    if (_dataPoints.length > _maxDataPoints) {
      _dataPoints.removeAt(0);
    }

    // 유효한 데이터로 저장
    _lastValidX = data.x;
    _lastValidY1 = data.y1;
    _lastValidY2 = data.y2;
    _hasValidData = true;

    // 통계 업데이트
    _updateStatistics();
    
    notifyListeners();
  }

  void _updateStatistics() {
    if (_dataPoints.isEmpty) return;

    final y1Values = _dataPoints.map((d) => d.y1).toList();
    final y2Values = _dataPoints.map((d) => d.y2).toList();

    _minY1 = y1Values.reduce((a, b) => a < b ? a : b);
    _maxY1 = y1Values.reduce((a, b) => a > b ? a : b);
    _minY2 = y2Values.reduce((a, b) => a < b ? a : b);
    _maxY2 = y2Values.reduce((a, b) => a > b ? a : b);
    
    _avgY1 = y1Values.reduce((a, b) => a + b) / y1Values.length;
    _avgY2 = y2Values.reduce((a, b) => a + b) / y2Values.length;
  }

  Future<void> connect() async {
    await _udpService.connect();
  }

  Future<void> disconnect() async {
    await _udpService.disconnect();
  }

  void clearData() {
    _dataPoints.clear();
    _lastValidX = 0.0;
    _lastValidY1 = 0.0;
    _lastValidY2 = 0.0;
    _hasValidData = false;
    _updateStatistics();
    notifyListeners();
  }

  void cleanData() {
    // NaN 값이 포함된 데이터를 정리하는 함수 (Processing 코드와 동일한 로직)
    int removedCount = 0;
    
    for (int i = _dataPoints.length - 1; i >= 0; i--) {
      final data = _dataPoints[i];
      bool needsReplacement = false;
      
      double x = data.x;
      double y1 = data.y1;
      double y2 = data.y2;
      
      // NaN 값 체크 및 대체
      if (x.isNaN) {
        if (_hasValidData) {
          x = _lastValidX;
        } else {
          x = 0.0;
        }
        needsReplacement = true;
      }
      
      if (y1.isNaN) {
        if (_hasValidData) {
          y1 = _lastValidY1;
        } else {
          y1 = 0.0;
        }
        needsReplacement = true;
      }
      
      if (y2.isNaN) {
        if (_hasValidData) {
          y2 = _lastValidY2;
        } else {
          y2 = 0.0;
        }
        needsReplacement = true;
      }
      
      if (needsReplacement) {
        _dataPoints[i] = HeartRateData(
          x: x,
          y1: y1,
          y2: y2,
          timestamp: data.timestamp,
        );
        removedCount++;
        
        // 유효한 데이터로 업데이트
        _lastValidX = x;
        _lastValidY1 = y1;
        _lastValidY2 = y2;
        _hasValidData = true;
      }
    }
    
    print('데이터 정리: ${_dataPoints.length}개 중 $removedCount개 수정됨');
    _updateStatistics();
    notifyListeners();
  }

  @override
  void dispose() {
    _dataSubscription?.cancel();
    _statusSubscription?.cancel();
    _udpService.dispose();
    super.dispose();
  }
}


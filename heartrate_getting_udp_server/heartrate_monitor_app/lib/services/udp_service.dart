import 'dart:async';
import 'dart:io';
import 'dart:typed_data';
import '../models/heartrate_data.dart';

class UDPService {
  static const String serverHost = '211.221.184.17';
  static const int serverPort = 8888;
  
  RawDatagramSocket? _socket;
  StreamController<HeartRateData>? _dataController;
  StreamController<String>? _statusController;
  Timer? _reconnectTimer;
  bool _isConnected = false;
  bool _shouldReconnect = true;

  Stream<HeartRateData> get dataStream => _dataController!.stream;
  Stream<String> get statusStream => _statusController!.stream;
  bool get isConnected => _isConnected;

  UDPService() {
    _dataController = StreamController<HeartRateData>.broadcast();
    _statusController = StreamController<String>.broadcast();
  }

  Future<void> connect() async {
    try {
      _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, 0);
      _isConnected = true;
      _statusController?.add('서버에 연결됨');
      
      // 서버에 등록 요청
      await _sendMessage('start');
      
      // 데이터 수신 시작
      _startReceiving();
      
    } catch (e) {
      _isConnected = false;
      _statusController?.add('연결 실패: $e');
      _scheduleReconnect();
    }
  }

  Future<void> _sendMessage(String message) async {
    if (_socket != null && _isConnected) {
      try {
        final data = Uint8List.fromList(message.codeUnits);
        _socket!.send(data, InternetAddress(serverHost), serverPort);
        print('전송: $message');
      } catch (e) {
        print('메시지 전송 실패: $e');
      }
    }
  }

  void _startReceiving() {
    _socket?.listen(
      (RawSocketEvent event) {
        if (event == RawSocketEvent.read) {
          final datagram = _socket!.receive();
          if (datagram != null) {
            final message = String.fromCharCodes(datagram.data);
            _processMessage(message);
          }
        }
      },
      onError: (error) {
        print('데이터 수신 오류: $error');
        _statusController?.add('데이터 수신 오류: $error');
      },
    );
  }

  void _processMessage(String message) {
    print('수신: $message');
    
    // 연결 종료 메시지 처리
    if (message == 'disconnected') {
      _isConnected = false;
      _statusController?.add('연결 종료됨');
      return;
    }
    
    // 메시지를 "\n"을 기준으로 나누어서 각각 처리
    final lines = message.split('\n');
    
    for (final line in lines) {
      final trimmedLine = line.trim();
      
      // 빈 줄은 건너뜀
      if (trimmedLine.isEmpty) continue;
      
      // "x,y1,y2" 형식 파싱
      if (trimmedLine.contains(',')) {
        try {
          final data = HeartRateData.fromString(trimmedLine);
          _dataController?.add(data);
        } catch (e) {
          print('데이터 파싱 오류: $e');
        }
      }
    }
  }

  void _scheduleReconnect() {
    if (_shouldReconnect && _reconnectTimer == null) {
      _reconnectTimer = Timer(const Duration(seconds: 5), () {
        _reconnectTimer = null;
        if (_shouldReconnect) {
          connect();
        }
      });
    }
  }

  Future<void> disconnect() async {
    _shouldReconnect = false;
    _reconnectTimer?.cancel();
    _reconnectTimer = null;
    
    if (_socket != null && _isConnected) {
      await _sendMessage('end');
      _socket!.close();
    }
    
    _isConnected = false;
    _statusController?.add('연결 해제됨');
  }

  void dispose() {
    _shouldReconnect = false;
    _reconnectTimer?.cancel();
    _socket?.close();
    _dataController?.close();
    _statusController?.close();
  }
}

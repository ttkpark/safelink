import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:provider/provider.dart';
import '../providers/heartrate_provider.dart';
import '../widgets/heartrate_chart.dart';
import '../widgets/status_panel.dart';
import '../widgets/legend_panel.dart';

class HeartRateScreen extends StatefulWidget {
  const HeartRateScreen({super.key});

  @override
  State<HeartRateScreen> createState() => _HeartRateScreenState();
}

class _HeartRateScreenState extends State<HeartRateScreen> {
  late HeartRateProvider _provider;

  @override
  void initState() {
    super.initState();
    _provider = Provider.of<HeartRateProvider>(context, listen: false);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF141414),
      appBar: AppBar(
        title: const Text(
          '심박수 데이터 실시간 그래프',
          style: TextStyle(
            color: Colors.white,
            fontWeight: FontWeight.bold,
          ),
        ),
        backgroundColor: const Color(0xFF1E1E1E),
        elevation: 0,
        actions: [
          IconButton(
            icon: const Icon(Icons.refresh, color: Colors.white),
            onPressed: () {
              _provider.clearData();
            },
            tooltip: '데이터 초기화',
          ),
          IconButton(
            icon: const Icon(Icons.cleaning_services, color: Colors.white),
            onPressed: () {
              _provider.cleanData();
            },
            tooltip: '데이터 정리',
          ),
          IconButton(
            icon: const Icon(Icons.power_settings_new, color: Colors.white),
            onPressed: () {
              if (_provider.isConnected) {
                _provider.disconnect();
              } else {
                _provider.connect();
              }
            },
            tooltip: _provider.isConnected ? '연결 종료' : '재연결',
          ),
        ],
      ),
      body: KeyboardListener(
        focusNode: FocusNode(),
        onKeyEvent: _handleKeyEvent,
        child: SingleChildScrollView(
          padding: const EdgeInsets.all(16),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              // 메인 차트
              const HeartRateChart(),
              const SizedBox(height: 16),
              
              // 하단 패널들
              Row(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  // 상태 패널
                  const Expanded(
                    flex: 2,
                    child: StatusPanel(),
                  ),
                  const SizedBox(width: 16),
                  
                  // 범례 패널
                  const Expanded(
                    flex: 1,
                    child: LegendPanel(),
                  ),
                ],
              ),
              
              const SizedBox(height: 16),
              
              // 연결/해제 버튼
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceEvenly,
                children: [
                  ElevatedButton.icon(
                    onPressed: () {
                      _provider.clearData();
                    },
                    icon: const Icon(Icons.clear_all),
                    label: const Text('데이터 초기화'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.orange,
                      foregroundColor: Colors.white,
                    ),
                  ),
                  ElevatedButton.icon(
                    onPressed: () {
                      _provider.cleanData();
                    },
                    icon: const Icon(Icons.cleaning_services),
                    label: const Text('데이터 정리'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.blue,
                      foregroundColor: Colors.white,
                    ),
                  ),
                  Consumer<HeartRateProvider>(
                    builder: (context, provider, child) {
                      return ElevatedButton.icon(
                        onPressed: () {
                          if (provider.isConnected) {
                            provider.disconnect();
                          } else {
                            provider.connect();
                          }
                        },
                        icon: Icon(
                          provider.isConnected 
                            ? Icons.power_settings_new 
                            : Icons.power,
                        ),
                        label: Text(
                          provider.isConnected ? '연결 종료' : '재연결',
                        ),
                        style: ElevatedButton.styleFrom(
                          backgroundColor: provider.isConnected 
                            ? Colors.red 
                            : Colors.green,
                          foregroundColor: Colors.white,
                        ),
                      );
                    },
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }

  void _handleKeyEvent(KeyEvent event) {
    if (event is KeyDownEvent) {
      switch (event.logicalKey) {
        case LogicalKeyboardKey.keyR:
          _provider.clearData();
          _showSnackBar('데이터 초기화됨');
          break;
        case LogicalKeyboardKey.keyS:
          _provider.connect();
          _showSnackBar('재연결 시도 중...');
          break;
        case LogicalKeyboardKey.keyC:
          _provider.cleanData();
          _showSnackBar('데이터 정리 완료');
          break;
        case LogicalKeyboardKey.keyE:
          _provider.disconnect();
          _showSnackBar('연결 종료됨');
          break;
      }
    }
  }

  void _showSnackBar(String message) {
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        duration: const Duration(seconds: 2),
        backgroundColor: Colors.grey[800],
      ),
    );
  }
}

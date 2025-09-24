import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import '../providers/heartrate_provider.dart';

class StatusPanel extends StatelessWidget {
  const StatusPanel({super.key});

  @override
  Widget build(BuildContext context) {
    return Consumer<HeartRateProvider>(
      builder: (context, provider, child) {
        return Container(
          padding: const EdgeInsets.all(16),
          decoration: BoxDecoration(
            color: Colors.grey[900],
            borderRadius: BorderRadius.circular(12),
          ),
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const Text(
                '연결 상태',
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.white,
                ),
              ),
              const SizedBox(height: 8),
              Row(
                children: [
                  Container(
                    width: 12,
                    height: 12,
                    decoration: BoxDecoration(
                      color: provider.isConnected ? Colors.green : Colors.red,
                      shape: BoxShape.circle,
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: Text(
                      provider.connectionStatus,
                      style: TextStyle(
                        color: provider.isConnected ? Colors.green : Colors.red,
                        fontSize: 14,
                      ),
                    ),
                  ),
                ],
              ),
              const SizedBox(height: 16),
              const Text(
                '데이터 정보',
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.white,
                ),
              ),
              const SizedBox(height: 8),
              _buildDataInfo(provider),
              const SizedBox(height: 16),
              const Text(
                '통계',
                style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.white,
                ),
              ),
              const SizedBox(height: 8),
              _buildStatistics(provider),
            ],
          ),
        );
      },
    );
  }

  Widget _buildDataInfo(HeartRateProvider provider) {
    final dataPoints = provider.dataPoints;
    
    if (dataPoints.isEmpty) {
      return const Text(
        '데이터 포인트: 0',
        style: TextStyle(color: Colors.grey),
      );
    }

    final latestData = dataPoints.last;
    
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          '데이터 포인트: ${dataPoints.length}',
          style: const TextStyle(color: Colors.grey),
        ),
        const SizedBox(height: 4),
        Text(
          '최신 X: ${latestData.x.toStringAsFixed(2)}',
          style: const TextStyle(color: Colors.grey),
        ),
        const SizedBox(height: 4),
        Text(
          '최신 Y1: ${latestData.y1.toStringAsFixed(2)}',
          style: const TextStyle(color: Colors.grey),
        ),
        const SizedBox(height: 4),
        Text(
          '최신 Y2: ${latestData.y2.toStringAsFixed(2)}',
          style: const TextStyle(color: Colors.grey),
        ),
      ],
    );
  }

  Widget _buildStatistics(HeartRateProvider provider) {
    if (provider.dataPoints.isEmpty) {
      return const Text(
        '통계 데이터 없음',
        style: TextStyle(color: Colors.grey),
      );
    }

    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        Text(
          'Y1 범위: ${provider.minY1.toStringAsFixed(1)} ~ ${provider.maxY1.toStringAsFixed(1)}',
          style: const TextStyle(color: Colors.grey),
        ),
        const SizedBox(height: 4),
        Text(
          'Y2 범위: ${provider.minY2.toStringAsFixed(1)} ~ ${provider.maxY2.toStringAsFixed(1)}',
          style: const TextStyle(color: Colors.grey),
        ),
        const SizedBox(height: 4),
        Text(
          'Y1 평균: ${provider.avgY1.toStringAsFixed(1)}',
          style: const TextStyle(color: Colors.grey),
        ),
        const SizedBox(height: 4),
        Text(
          'Y2 평균: ${provider.avgY2.toStringAsFixed(1)}',
          style: const TextStyle(color: Colors.grey),
        ),
      ],
    );
  }
}


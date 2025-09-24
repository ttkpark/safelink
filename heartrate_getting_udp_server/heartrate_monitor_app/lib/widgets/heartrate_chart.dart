import 'package:flutter/material.dart';
import 'package:fl_chart/fl_chart.dart';
import 'package:provider/provider.dart';
import '../providers/heartrate_provider.dart';

class HeartRateChart extends StatelessWidget {
  const HeartRateChart({super.key});

  @override
  Widget build(BuildContext context) {
    return Consumer<HeartRateProvider>(
      builder: (context, provider, child) {
        final dataPoints = provider.dataPoints;
        
        if (dataPoints.isEmpty) {
          return Container(
            height: 300,
            decoration: BoxDecoration(
              color: Colors.grey[900],
              borderRadius: BorderRadius.circular(12),
            ),
            child: const Center(
              child: Text(
                '데이터를 기다리는 중...',
                style: TextStyle(
                  color: Colors.grey,
                  fontSize: 16,
                ),
              ),
            ),
          );
        }

        return Container(
          height: 300,
          padding: const EdgeInsets.all(16),
          decoration: BoxDecoration(
            color: Colors.grey[900],
            borderRadius: BorderRadius.circular(12),
          ),
          child: LineChart(
            LineChartData(
              gridData: FlGridData(
                show: true,
                drawVerticalLine: true,
                drawHorizontalLine: true,
                horizontalInterval: 1,
                verticalInterval: 1,
                getDrawingHorizontalLine: (value) {
                  return FlLine(
                    color: Colors.grey[700]!,
                    strokeWidth: 0.5,
                  );
                },
                getDrawingVerticalLine: (value) {
                  return FlLine(
                    color: Colors.grey[700]!,
                    strokeWidth: 0.5,
                  );
                },
              ),
              titlesData: FlTitlesData(
                show: true,
                rightTitles: const AxisTitles(
                  sideTitles: SideTitles(showTitles: false),
                ),
                topTitles: const AxisTitles(
                  sideTitles: SideTitles(showTitles: false),
                ),
                bottomTitles: AxisTitles(
                  sideTitles: SideTitles(
                    showTitles: true,
                    reservedSize: 30,
                    interval: _getXInterval(dataPoints),
                    getTitlesWidget: (double value, TitleMeta meta) {
                      return Text(
                        value.toStringAsFixed(1),
                        style: const TextStyle(
                          color: Colors.grey,
                          fontSize: 12,
                        ),
                      );
                    },
                  ),
                ),
                leftTitles: AxisTitles(
                  sideTitles: SideTitles(
                    showTitles: true,
                    interval: _getYInterval(provider.minY1, provider.maxY1),
                    reservedSize: 40,
                    getTitlesWidget: (double value, TitleMeta meta) {
                      return Text(
                        value.toStringAsFixed(0),
                        style: const TextStyle(
                          color: Colors.grey,
                          fontSize: 12,
                        ),
                      );
                    },
                  ),
                ),
              ),
              borderData: FlBorderData(
                show: true,
                border: Border.all(color: Colors.grey[600]!, width: 1),
              ),
              minX: dataPoints.first.x,
              maxX: dataPoints.last.x,
              minY: provider.minY1 - (provider.maxY1 - provider.minY1) * 0.1,
              maxY: provider.maxY1 + (provider.maxY1 - provider.minY1) * 0.1,
              lineBarsData: [
                // Y1 데이터 (심박수) - 빨간색
                LineChartBarData(
                  spots: dataPoints.map((data) => FlSpot(data.x, data.y1)).toList(),
                  isCurved: false,
                  color: const Color(0xFFFF6464),
                  barWidth: 2,
                  isStrokeCapRound: true,
                  dotData: const FlDotData(show: false),
                  belowBarData: BarAreaData(show: false),
                ),
                // Y2 데이터 (기타) - 초록색
                LineChartBarData(
                  spots: dataPoints.map((data) => FlSpot(data.x, data.y2)).toList(),
                  isCurved: false,
                  color: const Color(0xFF64FF64),
                  barWidth: 2,
                  isStrokeCapRound: true,
                  dotData: const FlDotData(show: false),
                  belowBarData: BarAreaData(show: false),
                ),
              ],
            ),
          ),
        );
      },
    );
  }

  double _getXInterval(List dataPoints) {
    if (dataPoints.isEmpty) return 1.0;
    final range = dataPoints.last.x - dataPoints.first.x;
    if (range <= 10) return 1.0;
    if (range <= 50) return 5.0;
    if (range <= 100) return 10.0;
    return 20.0;
  }

  double _getYInterval(double min, double max) {
    final range = max - min;
    if (range <= 10) return 2.0;
    if (range <= 50) return 5.0;
    if (range <= 100) return 10.0;
    return 20.0;
  }
}


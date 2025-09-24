class HeartRateData {
  final double x;
  final double y1; // 심박수
  final double y2; // 기타 데이터
  final DateTime timestamp;

  HeartRateData({
    required this.x,
    required this.y1,
    required this.y2,
    required this.timestamp,
  });

  factory HeartRateData.fromString(String data) {
    final parts = data.split(',');
    if (parts.length >= 3) {
      return HeartRateData(
        x: double.tryParse(parts[0].trim()) ?? 0.0,
        y1: double.tryParse(parts[1].trim()) ?? 0.0,
        y2: double.tryParse(parts[2].trim()) ?? 0.0,
        timestamp: DateTime.now(),
      );
    }
    throw Exception('Invalid data format: $data');
  }

  @override
  String toString() {
    return 'HeartRateData(x: $x, y1: $y1, y2: $y2, timestamp: $timestamp)';
  }
}


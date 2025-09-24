import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'screens/heartrate_screen.dart';
import 'providers/heartrate_provider.dart';

void main() {
  runApp(const HeartRateMonitorApp());
}

class HeartRateMonitorApp extends StatelessWidget {
  const HeartRateMonitorApp({super.key});

  @override
  Widget build(BuildContext context) {
    return ChangeNotifierProvider(
      create: (context) => HeartRateProvider(),
      child: MaterialApp(
        title: '심박수 모니터',
        theme: ThemeData(
          colorScheme: ColorScheme.fromSeed(
            seedColor: const Color(0xFF1E88E5),
            brightness: Brightness.dark,
          ),
          useMaterial3: true,
          fontFamily: 'NotoSansKR',
        ),
        home: const HeartRateScreen(),
        debugShowCheckedModeBanner: false,
      ),
    );
  }
}
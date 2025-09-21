/**
 * 심박수 데이터 그래프 시각화
 * UDP 서버에서 받은 "메시지 : x축, y축1, y축2" 데이터를 실시간으로 그래프로 표시
 */

import java.net.*;
import java.io.*;

// UDP 통신 관련 변수
DatagramSocket socket;
InetAddress serverAddress;
int serverPort = 8888;
byte[] buffer = new byte[1024];

// 그래프 관련 변수
ArrayList<Float> xData = new ArrayList<Float>();
ArrayList<Float> y1Data = new ArrayList<Float>();
ArrayList<Float> y2Data = new ArrayList<Float>();
int maxDataPoints = 200; // 화면에 표시할 최대 데이터 포인트 수

// UI 관련 변수
color backgroundColor = color(20, 20, 30);
color gridColor = color(60, 60, 80);
color xAxisColor = color(100, 200, 255);
color y1AxisColor = color(255, 100, 100);
color y2AxisColor = color(100, 255, 100);
color textColor = color(255, 255, 255);

// 그래프 영역 설정
int graphX = 50;
int graphY = 50;
int graphWidth = 800;
int graphHeight = 500;

// 연결 상태
boolean isConnected = false;
String connectionStatus = "연결 중...";
long lastDataTime = 0;

void setup() {
  size(900, 600);
  background(backgroundColor);
  
  // UDP 소켓 초기화
  try {
    socket = new DatagramSocket();
    serverAddress = InetAddress.getByName("localhost"); // 서버 주소 변경 가능
    
    // 서버에 등록 요청
    sendMessage("start");
    
    connectionStatus = "서버에 연결됨";
    isConnected = true;
    
  } catch (Exception e) {
    println("UDP 소켓 초기화 실패: " + e.getMessage());
    connectionStatus = "연결 실패: " + e.getMessage();
    isConnected = false;
  }
  
  // 초기 데이터 설정
  initializeData();
}

void draw() {
  background(backgroundColor);
  
  // UDP 메시지 수신
  receiveMessage();
  
  // UI 그리기
  drawUI();
  drawGraph();
  drawLegend();
  drawStatus();
  
  // 데이터가 너무 많으면 오래된 데이터 제거
  if (xData.size() > maxDataPoints) {
    xData.remove(0);
    y1Data.remove(0);
    y2Data.remove(0);
  }
}

void initializeData() {
  // 초기 더미 데이터 (선택사항)
  for (int i = 0; i < 50; i++) {
    xData.add((float)i);
    y1Data.add(sin(i * 0.1) * 50 + 100);
    y2Data.add(cos(i * 0.1) * 30 + 80);
  }
}

void sendMessage(String message) {
  try {
    byte[] data = message.getBytes();
    DatagramPacket packet = new DatagramPacket(data, data.length, serverAddress, serverPort);
    socket.send(packet);
    println("전송: " + message);
  } catch (Exception e) {
    println("메시지 전송 실패: " + e.getMessage());
  }
}

void receiveMessage() {
  try {
    // 논블로킹 방식으로 메시지 수신
    socket.setSoTimeout(10); // 10ms 타임아웃
    
    DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
    socket.receive(packet);
    
    String message = new String(packet.getData(), 0, packet.getLength());
    processMessage(message);
    
  } catch (SocketTimeoutException e) {
    // 타임아웃은 정상적인 상황
  } catch (Exception e) {
    println("메시지 수신 오류: " + e.getMessage());
  }
}

void processMessage(String message) {
  println("수신: " + message);
  
  // "메시지 : x축, y축1, y축2" 형식 파싱
  if (message.contains("메시지 :")) {
    try {
      String[] parts = message.split("메시지 :")[1].trim().split(",");
      
      if (parts.length >= 3) {
        float x = parseFloat(parts[0].trim());
        float y1 = parseFloat(parts[1].trim());
        float y2 = parseFloat(parts[2].trim());
        
        // 데이터 추가
        xData.add(x);
        y1Data.add(y1);
        y2Data.add(y2);
        
        lastDataTime = millis();
        
        println("데이터 추가: x=" + x + ", y1=" + y1 + ", y2=" + y2);
      }
    } catch (Exception e) {
      println("데이터 파싱 오류: " + e.getMessage());
    }
  }
}

void drawUI() {
  // 제목
  fill(textColor);
  textSize(24);
  textAlign(CENTER);
  text("심박수 데이터 실시간 그래프", width/2, 30);
  
  // 그래프 배경
  fill(color(30, 30, 40));
  stroke(gridColor);
  strokeWeight(1);
  rect(graphX, graphY, graphWidth, graphHeight);
}

void drawGraph() {
  if (xData.size() < 2) return;
  
  // 그리드 그리기
  drawGrid();
  
  // 데이터 범위 계산
  float minX = min(xData.toArray(new Float[0]));
  float maxX = max(xData.toArray(new Float[0]));
  float minY1 = min(y1Data.toArray(new Float[0]));
  float maxY1 = max(y1Data.toArray(new Float[0]));
  float minY2 = min(y2Data.toArray(new Float[0]));
  float maxY2 = max(y2Data.toArray(new Float[0]));
  
  // Y축 범위 통합 (더 나은 시각화를 위해)
  float minY = min(minY1, minY2) - 10;
  float maxY = max(maxY1, maxY2) + 10;
  
  // 데이터 포인트 그리기
  strokeWeight(2);
  
  // Y1 데이터 (빨간색)
  stroke(y1AxisColor);
  for (int i = 1; i < xData.size(); i++) {
    float x1 = map(xData.get(i-1), minX, maxX, graphX, graphX + graphWidth);
    float y1 = map(y1Data.get(i-1), minY, maxY, graphY + graphHeight, graphY);
    float x2 = map(xData.get(i), minX, maxX, graphX, graphX + graphWidth);
    float y2 = map(y1Data.get(i), minY, maxY, graphY + graphHeight, graphY);
    
    line(x1, y1, x2, y2);
  }
  
  // Y2 데이터 (초록색)
  stroke(y2AxisColor);
  for (int i = 1; i < xData.size(); i++) {
    float x1 = map(xData.get(i-1), minX, maxX, graphX, graphX + graphWidth);
    float y1 = map(y2Data.get(i-1), minY, maxY, graphY + graphHeight, graphY);
    float x2 = map(xData.get(i), minX, maxX, graphX, graphX + graphWidth);
    float y2 = map(y2Data.get(i), minY, maxY, graphY + graphHeight, graphY);
    
    line(x1, y1, x2, y2);
  }
  
  // 축 레이블 그리기
  drawAxisLabels(minX, maxX, minY, maxY);
}

void drawGrid() {
  stroke(gridColor);
  strokeWeight(0.5);
  
  // 수직 그리드
  for (int i = 0; i <= 10; i++) {
    float x = graphX + (graphWidth * i / 10);
    line(x, graphY, x, graphY + graphHeight);
  }
  
  // 수평 그리드
  for (int i = 0; i <= 10; i++) {
    float y = graphY + (graphHeight * i / 10);
    line(graphX, y, graphX + graphWidth, y);
  }
}

void drawAxisLabels(float minX, float maxX, float minY, float maxY) {
  fill(textColor);
  textSize(12);
  textAlign(CENTER);
  
  // X축 레이블
  for (int i = 0; i <= 5; i++) {
    float value = map(i, 0, 5, minX, maxX);
    float x = graphX + (graphWidth * i / 5);
    text(nf(value, 0, 1), x, graphY + graphHeight + 20);
  }
  
  // Y축 레이블
  textAlign(RIGHT);
  for (int i = 0; i <= 5; i++) {
    float value = map(i, 0, 5, minY, maxY);
    float y = graphY + graphHeight - (graphHeight * i / 5);
    text(nf(value, 0, 1), graphX - 10, y + 5);
  }
}

void drawLegend() {
  int legendX = graphX + graphWidth + 20;
  int legendY = graphY + 50;
  
  fill(textColor);
  textSize(16);
  textAlign(LEFT);
  text("범례", legendX, legendY);
  
  // Y1 범례
  stroke(y1AxisColor);
  strokeWeight(3);
  line(legendX, legendY + 30, legendX + 20, legendY + 30);
  fill(textColor);
  textSize(12);
  text("Y축1 (심박수)", legendX + 25, legendY + 35);
  
  // Y2 범례
  stroke(y2AxisColor);
  strokeWeight(3);
  line(legendX, legendY + 50, legendX + 20, legendY + 50);
  fill(textColor);
  textSize(12);
  text("Y축2 (기타)", legendX + 25, legendY + 55);
  
  // 데이터 정보
  fill(textColor);
  textSize(12);
  text("데이터 포인트: " + xData.size(), legendX, legendY + 100);
  
  if (xData.size() > 0) {
    text("최신 X: " + nf(xData.get(xData.size()-1), 0, 2), legendX, legendY + 120);
    text("최신 Y1: " + nf(y1Data.get(y1Data.size()-1), 0, 2), legendX, legendY + 140);
    text("최신 Y2: " + nf(y2Data.get(y2Data.size()-1), 0, 2), legendX, legendY + 160);
  }
}

void drawStatus() {
  fill(textColor);
  textSize(14);
  textAlign(LEFT);
  
  // 연결 상태
  if (isConnected) {
    fill(color(100, 255, 100));
  } else {
    fill(color(255, 100, 100));
  }
  text("상태: " + connectionStatus, 20, height - 40);
  
  // 마지막 데이터 수신 시간
  fill(textColor);
  if (lastDataTime > 0) {
    long timeSinceLastData = millis() - lastDataTime;
    text("마지막 데이터: " + (timeSinceLastData / 1000) + "초 전", 20, height - 20);
  } else {
    text("데이터 대기 중...", 20, height - 20);
  }
}

void keyPressed() {
  if (key == 'r' || key == 'R') {
    // 데이터 초기화
    xData.clear();
    y1Data.clear();
    y2Data.clear();
    println("데이터 초기화됨");
  } else if (key == 's' || key == 'S') {
    // 서버에 재연결
    sendMessage("start");
    println("서버에 재연결 요청");
  }
}

void exit() {
  if (socket != null) {
    socket.close();
  }
  super.exit();
}

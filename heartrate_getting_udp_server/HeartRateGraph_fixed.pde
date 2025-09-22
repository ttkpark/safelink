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
byte[] buffer = new byte[6000];

// 그래프 관련 변수
ArrayList<Float> xData = new ArrayList<Float>();
ArrayList<Float> y1Data = new ArrayList<Float>();
ArrayList<Float> y2Data = new ArrayList<Float>();
int maxDataPoints = 200; // 화면에 표시할 최대 데이터 포인트 수

// 이전 값 저장 (오류 시 대체용)
float lastValidX = 0;
float lastValidY1 = 0;
float lastValidY2 = 0;
boolean hasValidData = false;

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

// 한글 폰트
PFont koreanFont;

// 전체 화면 모드
boolean fullScreen = false;

// 그래프 범위 변수 (전역)
float minY1_adjusted, maxY1_adjusted, minY2_adjusted, maxY2_adjusted;

void setup() {
  size(1000, 600);
  background(backgroundColor);
  
  // 한글 폰트 설정
  try {
    // 시스템에 있는 한글 폰트 사용
    koreanFont = createFont("Malgun Gothic", 16);
    if (koreanFont == null) {
      koreanFont = createFont("Arial Unicode MS", 16);
    }
    if (koreanFont == null) {
      koreanFont = createFont("Dialog", 16);
    }
  } catch (Exception e) {
    println("한글 폰트 로드 실패, 기본 폰트 사용");
    koreanFont = createFont("Dialog", 16);
  }
  
  // UDP 소켓 초기화
  try {
    socket = new DatagramSocket();
    serverAddress = InetAddress.getByName("211.221.184.17"); // 서버 주소 변경 가능
    
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
  
  // 메시지를 "\n"을 기준으로 나누어서 각각 처리
  String[] lines = message.split("\n");
  
  for (int lineIndex = 0; lineIndex < lines.length; lineIndex++) {
    String line = lines[lineIndex].trim();
    
    // 빈 줄은 건너뜀
    if (line.isEmpty()) {
      continue;
    }
    
    println("라인 " + (lineIndex + 1) + " 처리: " + line);
    
    // "x,y1,y2" 형식 파싱 (새로운 형식)
    if (line.contains(",")) {
    try {
      String[] parts = line.split(",");
      
      if (parts.length >= 3) {
        // 빈 문자열 체크 및 처리
        String xStr = parts[0].trim();
        String y1Str = parts[1].trim();
        String y2Str = parts[2].trim();
        
        float x, y1, y2;
        
        
        // X 값 처리
        if (xStr.isEmpty()) {
          if (hasValidData) {
            x = lastValidX + 1; // 이전 X 값에 1 증가
            println("X 값 빈 문자열 - 이전 값 사용: " + x);
          } else {
            x = 0;
          }
        } else {
          x = parseFloat(xStr);
          if (Float.isNaN(x)) {
            if (hasValidData) {
              x = lastValidX + 1;
              println("X 값 NaN - 이전 값 사용: " + x);
            } else {
              x = 0;
            }
          }
        }
        
        // Y1 값 처리
        if (y1Str.isEmpty()) {
          if (hasValidData) {
            y1 = lastValidY1;
            println("Y1 값 빈 문자열 - 이전 값 사용: " + y1);
          } else {
            y1 = 0;
          }
        } else {
          y1 = parseFloat(y1Str);
          if (Float.isNaN(y1)) {
            if (hasValidData) {
              y1 = lastValidY1;
              println("Y1 값 NaN - 이전 값 사용: " + y1);
            } else {
              y1 = 0;
            }
          }
        }
        
        // Y2 값 처리
        if (y2Str.isEmpty()) {
          if (hasValidData) {
            y2 = lastValidY2;
            println("Y2 값 빈 문자열 - 이전 값 사용: " + y2);
          } else {
            y2 = 0;
          }
        } else {
          y2 = parseFloat(y2Str);
          if (Float.isNaN(y2)) {
            if (hasValidData) {
              y2 = lastValidY2;
              println("Y2 값 NaN - 이전 값 사용: " + y2);
            } else {
              y2 = 0;
            }
          }
        }
        
        // 새 데이터 추가 (기존 데이터 유지)
        xData.add(x);
        y1Data.add(y1);
        y2Data.add(y2);
        
        // 유효한 데이터로 저장
        lastValidX = x;
        lastValidY1 = y1;
        lastValidY2 = y2;
        hasValidData = true;
        
        lastDataTime = millis();
        
        println("데이터 추가: x=" + x + ", y1=" + y1 + ", y2=" + y2);
      } 
    } catch (Exception e) {
      println("데이터 파싱 오류: " + e.getMessage());
      // 오류 발생 시에도 이전 값으로 채움
      if (hasValidData) {
        float x = lastValidX + 1;
        float y1 = lastValidY1;
        float y2 = lastValidY2;
        
        // 새 데이터 추가 (기존 데이터 유지)
        xData.add(x);
        y1Data.add(y1);
        y2Data.add(y2);
        
        lastValidX = x;
        lastDataTime = millis();
        
        println("오류 발생 - 이전 값으로 채움: x=" + x + ", y1=" + y1 + ", y2=" + y2);
      }
    }
    } // for 루프 닫기
  }
}

void drawUI() {
  // 제목
  fill(textColor);
  textFont(koreanFont, 24);
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
  
  // 데이터 범위 계산
  float minX = getMinValue(xData);
  float maxX = getMaxValue(xData);
  float minY1 = getMinValue(y1Data);
  float maxY1 = getMaxValue(y1Data);
  float minY2 = getMinValue(y2Data);
  float maxY2 = getMaxValue(y2Data);
  
  // Y1과 Y2 각각의 범위 계산
  minY1_adjusted = minY1;
  maxY1_adjusted = maxY1;
  minY2_adjusted = minY2;
  maxY2_adjusted = maxY2;
  
  // NaN 값 체크
  if (Float.isNaN(minX) || Float.isNaN(maxX) || Float.isNaN(minY1) || Float.isNaN(maxY1) || Float.isNaN(minY2) || Float.isNaN(maxY2)) {
    println("그래프 범위에 NaN 값 발견 - 그래프 그리기 건너뜀");
    return;
  }
  
  // X축 범위 계산
  float xRange = maxX - minX;
  if (xRange == 0) xRange = 1; // 0으로 나누기 방지
  float xMargin = xRange * 0.05;
  float adjustedMinX = minX - xMargin;
  float adjustedMaxX = maxX + xMargin;
  
  // Y1축 범위 계산
  float y1Range = maxY1 - minY1;
  if (y1Range == 0) y1Range = 1; // 0으로 나누기 방지
  float y1Margin = y1Range * 0.1;
  minY1_adjusted = minY1 - y1Margin;
  maxY1_adjusted = maxY1 + y1Margin;
  
  // Y2축 범위 계산
  float y2Range = maxY2 - minY2;
  if (y2Range == 0) y2Range = 1; // 0으로 나누기 방지
  float y2Margin = y2Range * 0.1;
  minY2_adjusted = minY2 - y2Margin;
  maxY2_adjusted = maxY2 + y2Margin;
  
  // 그리드 그리기 (X축 범위만 사용)
  drawGrid(adjustedMinX, adjustedMaxX, minY1_adjusted, maxY1_adjusted);
  
  // 데이터 포인트 그리기
  strokeWeight(2);
  
  // Y1 데이터 (빨간색)
  stroke(y1AxisColor);
  for (int i = 1; i < xData.size(); i++) {
    float x1Val = xData.get(i-1);
    float y1Val = y1Data.get(i-1);
    float x2Val = xData.get(i);
    float y2Val = y1Data.get(i);
    
    // NaN 값 체크
    if (Float.isNaN(x1Val) || Float.isNaN(y1Val) || Float.isNaN(x2Val) || Float.isNaN(y2Val)) {
      continue; // 이 선분은 건너뜀
    }
    
    float x1 = map(x1Val, adjustedMinX, adjustedMaxX, graphX, graphX + graphWidth);
    float y1 = map(y1Val, minY1_adjusted, maxY1_adjusted, graphY + graphHeight, graphY);
    float x2 = map(x2Val, adjustedMinX, adjustedMaxX, graphX, graphX + graphWidth);
    float y2 = map(y2Val, minY1_adjusted, maxY1_adjusted, graphY + graphHeight, graphY);
    
    line(x1, y1, x2, y2);
  }
  
  // Y2 데이터 (초록색)
  stroke(y2AxisColor);
  for (int i = 1; i < xData.size(); i++) {
    float x1Val = xData.get(i-1);
    float y2Val1 = y2Data.get(i-1);
    float x2Val = xData.get(i);
    float y2Val2 = y2Data.get(i);
    
    // NaN 값 체크
    if (Float.isNaN(x1Val) || Float.isNaN(y2Val1) || Float.isNaN(x2Val) || Float.isNaN(y2Val2)) {
      continue; // 이 선분은 건너뜀
    }
    
    float x1 = map(x1Val, adjustedMinX, adjustedMaxX, graphX, graphX + graphWidth);
    float y1 = map(y2Val1, minY2_adjusted, maxY2_adjusted, graphY + graphHeight, graphY);
    float x2 = map(x2Val, adjustedMinX, adjustedMaxX, graphX, graphX + graphWidth);
    float y2 = map(y2Val2, minY2_adjusted, maxY2_adjusted, graphY + graphHeight, graphY);
    
    line(x1, y1, x2, y2);
  }
  
  // 축 레이블 그리기 (Y1 범위 기준)
  drawAxisLabels(adjustedMinX, adjustedMaxX, minY1_adjusted, maxY1_adjusted);
}

void drawGrid(float minX, float maxX, float minY, float maxY) {
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
  textFont(koreanFont, 12);
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
  textFont(koreanFont, 16);
  textAlign(LEFT);
  text("범례", legendX, legendY);
  
  // Y1 범례
  stroke(y1AxisColor);
  strokeWeight(3);
  line(legendX, legendY + 30, legendX + 20, legendY + 30);
  fill(textColor);
  textFont(koreanFont, 12);
  text("Y축1 (심박수)", legendX + 25, legendY + 35);
  
  // Y2 범례
  stroke(y2AxisColor);
  strokeWeight(3);
  line(legendX, legendY + 50, legendX + 20, legendY + 50);
  fill(textColor);
  textFont(koreanFont, 12);
  text("Y축2 (기타)", legendX + 25, legendY + 55);
  
  // 데이터 정보
  fill(textColor);
  textFont(koreanFont, 12);
  text("데이터 포인트: " + xData.size(), legendX, legendY + 100);
  
  if (xData.size() > 0) {
    text("최신 X: " + nf(xData.get(xData.size()-1), 0, 2), legendX, legendY + 120);
    text("최신 Y1: " + nf(y1Data.get(y1Data.size()-1), 0, 2), legendX, legendY + 140);
    text("최신 Y2: " + nf(y2Data.get(y2Data.size()-1), 0, 2), legendX, legendY + 160);
    
    // 그래프 범위 정보
    if (xData.size() >= 2) {
      float minX = getMinValue(xData);
      float maxX = getMaxValue(xData);
      float minY1 = getMinValue(y1Data);
      float maxY1 = getMaxValue(y1Data);
      float minY2 = getMinValue(y2Data);
      float maxY2 = getMaxValue(y2Data);
      
      text("X 범위: " + nf(minX, 0, 1) + " ~ " + nf(maxX, 0, 1), legendX, legendY + 190);
      text("Y1 범위: " + nf(minY1, 0, 1) + " ~ " + nf(maxY1, 0, 1), legendX, legendY + 210);
      text("Y2 범위: " + nf(minY2, 0, 1) + " ~ " + nf(maxY2, 0, 1), legendX, legendY + 230);
      text("Y1 확대: " + nf(minY1_adjusted, 0, 1) + " ~ " + nf(maxY1_adjusted, 0, 1), legendX, legendY + 250);
      text("Y2 확대: " + nf(minY2_adjusted, 0, 1) + " ~ " + nf(maxY2_adjusted, 0, 1), legendX, legendY + 270);
    }
  }
}

void drawStatus() {
  fill(textColor);
  textFont(koreanFont, 14);
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
  
  // 키보드 단축키 안내
  fill(textColor);
  textFont(koreanFont, 10);
  textAlign(RIGHT);
  text("단축키: R(초기화) S(재연결) Z(자동조정) F(전체화면) C(데이터정리)", width - 20, height - 5);
}

void keyPressed() {
  if (key == 'r' || key == 'R') {
    // 데이터 초기화
    println("데이터 지우기");
    int size = xData.size();
    for(int i=0;i<size;i++){
      xData.remove(0);
      y1Data.remove(0);
      y2Data.remove(0);
    } 
    
    // 이전 값들도 리셋
    lastValidX = 0;
    lastValidY1 = 0;
    lastValidY2 = 0;
    hasValidData = false;
    
    println("데이터 초기화됨");
  } else if (key == 's' || key == 'S') {
    // 서버에 재연결
    sendMessage("start");
    println("서버에 재연결 요청");
  } else if (key == 'z' || key == 'Z') {
    // 줌 리셋 (데이터 범위에 맞춰 자동 조정)
    println("그래프 자동 조정됨");
  } else if (key == 'f' || key == 'F') {
    // 전체 화면 토글
    if (fullScreen) {
      fullScreen = false;
      size(1000, 600);
    } else {
      fullScreen = true;
      fullScreen();
    }
  } else if (key == 'c' || key == 'C') {
    // NaN 값이 포함된 데이터 정리
    cleanData();
    println("NaN 값이 포함된 데이터 정리 완료");
  }
}

// NaN 값이 포함된 데이터를 정리하는 함수
void cleanData() {
  int originalSize = xData.size();
  int removedCount = 0;
  
  // 뒤에서부터 검사하여 NaN 값이 있는 인덱스를 제거하고 이전 값으로 대체
  for (int i = xData.size() - 1; i >= 0; i--) {
    boolean needsReplacement = false;
    float x = xData.get(i);
    float y1 = y1Data.get(i);
    float y2 = y2Data.get(i);
    
    // NaN 값 체크 및 대체
    if (Float.isNaN(x)) {
      if (hasValidData) {
        x = lastValidX;
        needsReplacement = true;
      } else {
        x = 0;
        needsReplacement = true;
      }
    }
    
    if (Float.isNaN(y1)) {
      if (hasValidData) {
        y1 = lastValidY1;
        needsReplacement = true;
      } else {
        y1 = 0;
        needsReplacement = true;
      }
    }
    
    if (Float.isNaN(y2)) {
      if (hasValidData) {
        y2 = lastValidY2;
        needsReplacement = true;
      } else {
        y2 = 0;
        needsReplacement = true;
      }
    }
    
    if (needsReplacement) {
      xData.set(i, x);
      y1Data.set(i, y1);
      y2Data.set(i, y2);
      removedCount++;
      
      // 유효한 데이터로 업데이트
      lastValidX = x;
      lastValidY1 = y1;
      lastValidY2 = y2;
      hasValidData = true;
    }
  }
  
  println("데이터 정리: " + originalSize + "개 중 " + removedCount + "개 수정됨");
}

void exit() {
  if (socket != null) {
    socket.close();
  }
  super.exit();
}

// ArrayList에서 최솟값을 찾는 함수 (NaN 값 제외)
float getMinValue(ArrayList<Float> list) {
  if (list.size() == 0) return 0;
  float minVal = Float.MAX_VALUE;
  for (int i = 0; i < list.size(); i++) {
    float val = list.get(i);
    if (!Float.isNaN(val) && val < minVal) {
      minVal = val;
    }
  }
  return (minVal == Float.MAX_VALUE) ? 0 : minVal;
}

// ArrayList에서 최댓값을 찾는 함수 (NaN 값 제외)
float getMaxValue(ArrayList<Float> list) {
  if (list.size() == 0) return 0;
  float maxVal = Float.MIN_VALUE;
  for (int i = 0; i < list.size(); i++) {
    float val = list.get(i);
    if (!Float.isNaN(val) && val > maxVal) {
      maxVal = val;
    }
  }
  return (maxVal == Float.MIN_VALUE) ? 0 : maxVal;
}

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define CONNECTION_POLLING_TIME 100 // 1ms = 1000 HZ
const char *ssid = "not-a-raspberrypi";
const char *password = "1145141919810";
const char recieved_message[] = "Received your UDP message";

// 1. 加速度データ (Accelerometer)
// • readFloatAccelX(): X軸方向の加速度（例: 0.02）
// • readFloatAccelY(): Y軸方向の加速度（例: -0.01）
// • readFloatAccelZ(): Z軸方向の加速度（例: 0.98 ※静止時、重力によるもの）
//
// 2. 角速度データ (Gyroscope)
// ボードがどのくらいの勢いで回転しているかを表すデータで、単位は
// dps（度/秒：1秒間に何度回転したか）
// です。静止している時は、すべての軸が「0.0」前後になる • readFloatGyroX():
// X軸周りの回転速度（ロール） • readFloatGyroY(): Y軸周りの回転速度（ピッチ）
// • readFloatGyroZ(): Z軸周りの回転速度（ヨー）

struct UDPConnectiionResult {
  bool isConnected;
  String ipAddress;
  int port;
};

struct UDPConnection {
  WiFiUDP udp;
  String targetIP;
  int targetPort;

  UDPConnectiionResult connect(String ip, int port) {
    targetIP = ip;
    targetPort = port;
    udp.begin(targetPort);
    return {true, targetIP, targetPort};
  }

  void sendData(const uint8_t *data, size_t length) {
    udp.beginPacket(targetIP.c_str(), targetPort);
    udp.write(data, length);
    udp.endPacket();
  }
};

struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

SensorData sampleData() {
  SensorData data;
  data.accelX = 1.0;
  data.accelY = 1.0;
  data.accelZ = 1.0;
  data.gyroX = 1.0;
  data.gyroY = 1.0;
  data.gyroZ = 1.0;
  return data;
}

WiFiUDP udp;
const unsigned int localPort = 8888; // 受信ポート番号
char packetBuffer[255];              // 受信バッファ
bool asReady = false;

void setup() {
  delay(500);

  // アクセスポイントモードの開始
  printf("APモードを起動中...");
  // printf("SSID: %s, PASSWORD: %s \n", ssid, password);

  WiFi.mode(WIFI_AP);
  // 成功したらtrue
  asReady = WiFi.softAP(ssid, password);

  printf("Result: %d\n", asReady);
  if (asReady == false) {
    printf("APモードの起動に失敗しました。\n");
    return;
  } else {
    printf("APモードの起動に成功しました。\n");
  }

  // 割り当てられたIPアドレスを表示（通常は 192.168.4.1）
  IPAddress myIP = WiFi.softAPIP();
  printf("AP IPアドレス: ");
  printf("%s", myIP.toString().c_str());

  // UDPリスンの開始
  udp.begin(localPort);
  printf("UDP待機ポート: ");
  printf("%d\n", localPort);
  printf("%d", localPort);
}

void loop() {
  // パケットサイズを取得
  if (asReady == false) {
    printf("APモードが起動していません。\n");
    return;
  }

  delay(CONNECTION_POLLING_TIME);
  int packetSize = udp.parsePacket();

  if (!packetSize) {
    return;
  }

  printf("受信 %d バイト (送信元 %s:%d)\n", packetSize,
         udp.remoteIP().toString().c_str(), udp.remotePort());

  // パケットをバッファに読み込む
  int len = udp.read(packetBuffer, 255);
  if (len > 0) {
    packetBuffer[len] = 0; // 文字列の終端
  }
  printf("受信データ: %s\n", packetBuffer);

  // 応答を返す（送信元へエコーバック）
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write(reinterpret_cast<const uint8_t *>(recieved_message),
            sizeof(recieved_message) - 1);
  udp.endPacket();
}

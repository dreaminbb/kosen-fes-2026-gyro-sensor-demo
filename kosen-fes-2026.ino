#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

// Godot sends HELLO packets to localPort. The ESP32 learns the PC's
// address and return port from that packet, then streams sensor data back.
const char *ssid = "not-a-raspberrypi";
const char *password = "1145141919810";
const unsigned int localPort = 8888;
const unsigned long sensorSendIntervalMs = 100; // 10 Hz
const unsigned long pcTimeoutMs = 5000;

struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

SensorData sampleData() {
  // Replace these placeholders with readings from the actual sensor.
  return {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
}

WiFiUDP udp;
char packetBuffer[256];
bool apReady = false;
bool pcKnown = false;
IPAddress pcAddress;
unsigned int pcPort = 0;
unsigned long lastPcPacketMs = 0;
unsigned long lastSensorSendMs = 0;

void sendToPc(const char *message) {
  if (!pcKnown) {
    return;
  }
  if (!udp.beginPacket(pcAddress, pcPort)) {
    printf("UDP送信開始に失敗しました\n");
    return;
  }
  udp.write(reinterpret_cast<const uint8_t *>(message), strlen(message));
  if (udp.endPacket() == 1) {
    if (strncmp(message, "DATA,", 5) != 0) {
      printf("送信: %s\n", message);
    }
  } else {
    printf("UDP送信に失敗しました\n");
  }
}

void setup() {
  delay(500);
  printf("AP起動中: %s\n", ssid);
  WiFi.mode(WIFI_AP);
  apReady = WiFi.softAP(ssid, password);
  if (!apReady) {
    printf("ERROR: Wi-Fi APを起動できませんでした\n");
    return;
  }

  printf("AP起動成功。IP: %s\n", WiFi.softAPIP().toString().c_str());
  if (udp.begin(localPort) != 1) {
    printf("ERROR: UDPポート %u を開けませんでした\n", localPort);
    apReady = false;
    return;
  }
  printf("UDP待受開始: %u\n", localPort);
  printf("GodotからHELLOが届くのを待っています\n");
}

void loop() {
  if (!apReady) {
    delay(1000);
    return;
  }

  const int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    const int length = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
    if (length >= 0) {
      packetBuffer[length] = '\0';
      pcAddress = udp.remoteIP();
      pcPort = udp.remotePort();
      pcKnown = true;
      lastPcPacketMs = millis();

      printf("受信 %d byte from %s:%u: %s\n", length,
             pcAddress.toString().c_str(), pcPort, packetBuffer);
      if (strcmp(packetBuffer, "HELLO") == 0) {
        sendToPc("HELLO_ACK");
      } else {
        char response[280];
        snprintf(response, sizeof(response), "ACK,%s", packetBuffer);
        sendToPc(response);
      }
    }
  }

  const unsigned long now = millis();
  if (pcKnown && now - lastPcPacketMs > pcTimeoutMs) {
    printf("PCからの通信がタイムアウトしました\n");
    pcKnown = false;
  }

  if (pcKnown && now - lastSensorSendMs >= sensorSendIntervalMs) {
    lastSensorSendMs = now;
    const SensorData data = sampleData();
    char message[160];
    snprintf(message, sizeof(message), "DATA,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             now, data.accelX, data.accelY, data.accelZ,
             data.gyroX, data.gyroY, data.gyroZ);
    sendToPc(message);
  }
  delay(1);
}

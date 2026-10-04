#include "sample.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <stdio.h>
#include <string.h>

const char *ssid = "not-a-raspberrypi";
const char *password = "1145141919810";
const unsigned int localPort = 8888;
const unsigned long sensorSendIntervalMs = 100; // 10 Hz
const unsigned long pcTimeoutMs = 5000;

SensorData sampleData(int loop_index) {
  if (loop_index >= SAMPLE_DATA_LENGTH) {
    loop_index %= SAMPLE_DATA_LENGTH;
  }
  printf("サンプルデータのインデックス: %d\n", loop_index);
  return rightTurn360Samples[loop_index];
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

void connectToPc() {

  unsigned long lastWaitingLogMs = millis();
  printf("PCからのUDP通信を待機中...\n");
  while (!pcKnown) {
    const int packetSize = udp.parsePacket();
    if (packetSize >= static_cast<int>(sizeof(packetBuffer))) {
      // Reserve one byte for the terminator and discard oversized datagrams.
      printf("ERROR: UDPパケットが大きすぎます: %d byte\n", packetSize);
      udp.clear();
    } else if (packetSize > 0) {
      const int length = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
      if (length > 0) {
        packetBuffer[length] = '\0';
        printf("PCからの通信を受信: %s\n", packetBuffer);
        pcAddress = udp.remoteIP();
        pcPort = udp.remotePort();
        pcKnown = true;
      } else {
        printf("ERROR: UDP読み込みに失敗しました: %d\n", length);
      }
    }

    if (millis() - lastWaitingLogMs >= 1000) {

      lastWaitingLogMs = millis();
      printf("PC待機中: Wi-Fi接続台数=%u, heap=%u\n",
             static_cast<unsigned int>(WiFi.softAPgetStationNum()),
             static_cast<unsigned int>(ESP.getFreeHeap()));
    }
    delay(100);
  }
}

void setup() {
  // Keep diagnostics visible even when the following call blocks or crashes.
  setvbuf(stdout, nullptr, _IONBF, 0);
  delay(500);
  const esp_reset_reason_t resetReason = esp_reset_reason();
  printf("BOOT: chip=%s, resetReason=%d, heap=%u\n", ESP.getChipModel(),
         static_cast<int>(resetReason),
         static_cast<unsigned int>(ESP.getFreeHeap()));

  if (resetReason == ESP_RST_BROWNOUT) {
    printf("BOOT: 前回は電圧低下によるリセットでした\n");
  } else if (resetReason == ESP_RST_PANIC) {
    printf("BOOT: 前回はパニックによるリセットでした\n");
  } else if (resetReason == ESP_RST_TASK_WDT ||
             resetReason == ESP_RST_INT_WDT || resetReason == ESP_RST_WDT) {
    printf("BOOT: 前回はウォッチドッグによるリセットでした\n");
  }

  printf("AP起動中: %s\n", ssid);
  printf("WiFi.mode(WIFI_AP): begin\n");
  const unsigned long modeStartedMs = millis();
  const bool modeReady = WiFi.mode(WIFI_AP);
  printf("WiFi.mode(WIFI_AP): result=%d, elapsed=%lu ms, heap=%u\n", modeReady,
         millis() - modeStartedMs,
         static_cast<unsigned int>(ESP.getFreeHeap()));
  if (!modeReady) {
    printf("ERROR: Wi-Fi APモードへの切り替えに失敗しました\n");
    return;
  }

  printf("WiFi.softAP(): begin\n");
  const unsigned long apStartedMs = millis();
  apReady = WiFi.softAP(ssid, password);
  printf("WiFi.softAP(): result=%d, elapsed=%lu ms, heap=%u\n", apReady,
         millis() - apStartedMs, static_cast<unsigned int>(ESP.getFreeHeap()));
  if (!apReady) {
    printf("ERROR: Wi-Fi APを起動できませんでした\n");
    return;
  } else {
    printf("Wi-Fi APを起動しました: %s\n", ssid);
  }

  printf("AP起動成功。IP: %s\n", WiFi.softAPIP().toString().c_str());
  if (udp.begin(localPort) != 1) {
    printf("ERROR: UDPポート %u を開けませんでした\n", localPort);
    apReady = false;
    return;
  }
  printf("UDP待受開始: %u\n", localPort);

  // PCとの通信を確立
  connectToPc();
}

int monitoring_index = 0; // 監視時間
unsigned long first_time_stamp = 0;
float current_time_from_start = 0;

void loop() {

  if (!apReady) {
    delay(1000);
    return;
  }

  if (!pcKnown) {
    printf("PCとの通信を待機中...\n");
    connectToPc();
    return;
  }

  lastPcPacketMs = millis();

  const unsigned long now = millis();
  current_time_from_start = (now - first_time_stamp) / 1000.0;
  // printf("経過時間: %.3f秒\n", current_time_from_start);

  if (pcKnown && now - lastPcPacketMs > pcTimeoutMs) {
    printf("PCからの通信がタイムアウトしました\n");
    pcKnown = false;
  }

  if (pcKnown && now - lastSensorSendMs >= sensorSendIntervalMs) {

    if (monitoring_index == 0) {
      first_time_stamp = millis();
    }

    lastSensorSendMs = now;

    // MARK: センサーデータ（テスト用）
    const SensorData data = sampleData(monitoring_index);

    printf("送信データ: accel(%.3f, %.3f, %.3f), gyro(%.3f, %.3f, %.3f)\n",
           data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY,
           data.gyroZ);

    // MARK: データ送信
    char message[160];
    snprintf(message, sizeof(message), "DATA,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             now, data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY,
             data.gyroZ);

    sendToPc(message);
    // サンプルデータのテストののみ
    monitoring_index++;
  }
  delay(1);
}

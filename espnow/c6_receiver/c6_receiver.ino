// 処理の流れ: ESP-NOW受信コールバックで検証済みパケットをキューへ積む。
// loopでキューから取り出し、DATA行としてprintfでUSBシリアルへ出力する。
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "../espnow_protocol.h"

namespace {

QueueHandle_t receivedPackets;
uint32_t receivedCount = 0;
uint32_t lastStatusMs = 0;

void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data,
                    int length) {
  (void)info;
  if (length != sizeof(SensorPacket) || receivedPackets == nullptr) {
    return;
  }

  SensorPacket packet;
  memcpy(&packet, data, sizeof(packet));
  if (packet.magic != ESPNOW_PACKET_MAGIC ||
      packet.version != ESPNOW_PACKET_VERSION) {
    return;
  }

  xQueueSend(receivedPackets, &packet, 0);
}

void fail(const char *message, esp_err_t error) {
  printf("ERROR,%s,0x%X\n", message, static_cast<unsigned>(error));
  while (true) {
    delay(1000);
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  printf("BOOT,ESP-NOW,C6\n");

  receivedPackets = xQueueCreate(16, sizeof(SensorPacket));
  if (receivedPackets == nullptr) {
    printf("ERROR,queue allocation failed\n");
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(50);
  }

  const esp_err_t channelStatus =
      esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelStatus != ESP_OK) {
    fail("set channel", channelStatus);
  }

  const esp_err_t initStatus = esp_now_init();
  if (initStatus != ESP_OK) {
    fail("esp_now_init", initStatus);
  }

  const esp_err_t callbackStatus = esp_now_register_recv_cb(onDataReceived);
  if (callbackStatus != ESP_OK) {
    fail("register receive callback", callbackStatus);
  }

  printf("READY,MAC=%s,CHANNEL=%u\n", WiFi.macAddress().c_str(),
         ESPNOW_WIFI_CHANNEL);
}

void loop() {
  const uint32_t now = millis();
  SensorPacket packet;
  if (receivedPackets != nullptr &&
      xQueueReceive(receivedPackets, &packet, pdMS_TO_TICKS(100)) == pdTRUE) {
    ++receivedCount;
    printf("DATA,%lu,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%lu\n",
           static_cast<unsigned long>(packet.sequence),
           static_cast<unsigned long>(packet.elapsedMs), packet.accelX,
           packet.accelY, packet.accelZ, packet.gyroX, packet.gyroY,
           packet.gyroZ, static_cast<unsigned long>(receivedCount));
  }

  if (now - lastStatusMs >= 1000) {
    lastStatusMs = now;
    printf("STATUS,WAITING_FOR_PACKET,%lu\n",
           static_cast<unsigned long>(receivedCount));
  }
}

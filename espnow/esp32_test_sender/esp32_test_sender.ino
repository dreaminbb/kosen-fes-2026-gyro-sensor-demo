#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "../espnow_protocol.h"

namespace {

const uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF,
                                    0xFF, 0xFF, 0xFF};
uint32_t sequenceNumber = 0;
unsigned long lastSendMs = 0;

} // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  printf("BOOT,ESP-NOW,test sender\n");

  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(50);
  }

  const esp_err_t channelStatus =
      esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (channelStatus != ESP_OK) {
    printf("ERROR,set channel,0x%X\n", static_cast<unsigned>(channelStatus));
    return;
  }

  const esp_err_t initStatus = esp_now_init();
  if (initStatus != ESP_OK) {
    printf("ERROR,esp_now_init,0x%X\n", static_cast<unsigned>(initStatus));
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastAddress, sizeof(broadcastAddress));
  peer.channel = ESPNOW_WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  const esp_err_t peerStatus = esp_now_add_peer(&peer);
  if (peerStatus != ESP_OK) {
    printf("ERROR,esp_now_add_peer,0x%X\n", static_cast<unsigned>(peerStatus));
    return;
  }

  printf("READY,MAC=%s,CHANNEL=%u\n", WiFi.macAddress().c_str(),
         ESPNOW_WIFI_CHANNEL);
}

void loop() {
  const unsigned long now = millis();
  if (now - lastSendMs < 100) {
    delay(1);
    return;
  }
  lastSendMs = now;

  SensorPacket packet = {};
  packet.magic = ESPNOW_PACKET_MAGIC;
  packet.version = ESPNOW_PACKET_VERSION;
  packet.sequence = sequenceNumber++;
  packet.elapsedMs = now;
  packet.accelZ = 1.0f;
  packet.gyroZ = 120.0f;

  const esp_err_t status = esp_now_send(
      broadcastAddress, reinterpret_cast<const uint8_t *>(&packet),
      sizeof(packet));
  if (status == ESP_OK) {
    printf("TX,%lu\n", static_cast<unsigned long>(packet.sequence));
  } else {
    printf("ERROR,esp_now_send,0x%X\n", static_cast<unsigned>(status));
  }
}

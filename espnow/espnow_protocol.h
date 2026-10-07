#ifndef ESPNOW_PROTOCOL_H
#define ESPNOW_PROTOCOL_H

#include <stdint.h>

constexpr uint16_t ESPNOW_PACKET_MAGIC = 0x4B46;
constexpr uint8_t ESPNOW_PACKET_VERSION = 1;
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;

struct __attribute__((packed)) SensorPacket {
  uint16_t magic;
  uint8_t version;
  uint8_t reserved;
  uint32_t sequence;
  uint32_t elapsedMs;
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

static_assert(sizeof(SensorPacket) == 36, "Unexpected ESP-NOW packet size");

#endif

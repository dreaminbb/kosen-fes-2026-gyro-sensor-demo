// 処理の流れ: GATTサービスを作ってBLE広告し、C6との接続を待つ。
// センサーデータを100msごとに通知し、START/STOP/RESETコマンドへACKを返す。
#include "sample.h"

#include "arduino_serial_config.h"
#include <Arduino.h>
#include <sl_bluetooth.h>
#include <sl_bt_api.h>
#include <stdio.h>
#include <string.h>

namespace {

const uint8_t advertisedName[] = "KosenBall";
const uint8_t serviceAdvertisingData[] = {
    0x02, 0x01, 0x06, 0x11, 0x07, 0x07, 0xb9, 0xf9, 0xd7,
    0x50, 0xa4, 0x20, 0x89, 0x77, 0x40, 0xcb, 0xfd,
    0x2c, 0xc1, 0x80, 0x48};
const uint8_t nameScanResponse[] = {
    0x0a, 0x09, 'K', 'o', 's', 'e', 'n', 'B', 'a', 'l', 'l'};
const uint16_t gattValueMaxLength = 250;
const uint16_t defaultNotificationPayloadSize = 20;
const unsigned long sensorSendIntervalMs = RIGHT_TURN_SAMPLE_INTERVAL_MS;

const uuid_128 sppServiceUuid = {.data = {0x07, 0xb9, 0xf9, 0xd7, 0x50, 0xa4,
                                          0x20, 0x89, 0x77, 0x40, 0xcb, 0xfd,
                                          0x2c, 0xc1, 0x80, 0x48}};
const uuid_128 sppDataCharacteristicUuid = {
    .data = {0xd6, 0x58, 0xd6, 0x21, 0xbc, 0x55, 0x81, 0x9f, 0x42, 0x44, 0x71,
             0x6d, 0xc4, 0x6e, 0xc2, 0xfe}};

uint16_t gattDbSessionId = 0;
uint16_t genericAccessServiceHandle = 0;
uint16_t deviceNameCharacteristicHandle = 0;
uint16_t sensorServiceHandle = 0;
uint16_t sensorDataCharacteristicHandle = 0;
uint8_t advertisingSetHandle = 0xff;
uint8_t connectionHandle = SL_BT_INVALID_CONNECTION_HANDLE;
uint16_t notificationPayloadSize = defaultNotificationPayloadSize;

bool advertisingSetInitialized = false;
bool notificationsEnabled = false;
bool streamingEnabled = true;
bool sessionClockStarted = false;
int monitoringIndex = 0;
unsigned long sessionStartedMs = 0;
unsigned long lastSensorSendMs = 0;

char commandBuffer[64];
size_t commandLength = 0;

bool reportBleStatus(const char *operation, sl_status_t status) {
  if (status == SL_STATUS_OK) {
    return true;
  }

  printf("ERROR: %s failed with BLE status 0x%08lX\n", operation,
         static_cast<unsigned long>(status));
  return false;
}

bool initializeGattDatabase() {
  sl_status_t status = sl_bt_gattdb_new_session(&gattDbSessionId);
  if (!reportBleStatus("sl_bt_gattdb_new_session", status)) {
    return false;
  }

  const uint8_t genericAccessServiceUuid[] = {0x00, 0x18};
  status = sl_bt_gattdb_add_service(
      gattDbSessionId, sl_bt_gattdb_primary_service,
      SL_BT_GATTDB_ADVERTISED_SERVICE, sizeof(genericAccessServiceUuid),
      genericAccessServiceUuid, &genericAccessServiceHandle);
  if (!reportBleStatus("add Generic Access service", status)) {
    return false;
  }

  const sl_bt_uuid_16_t deviceNameUuid = {.data = {0x00, 0x2a}};
  status = sl_bt_gattdb_add_uuid16_characteristic(
      gattDbSessionId, genericAccessServiceHandle,
      SL_BT_GATTDB_CHARACTERISTIC_READ, 0, 0, deviceNameUuid,
      sl_bt_gattdb_fixed_length_value, sizeof(advertisedName) - 1,
      sizeof(advertisedName) - 1, advertisedName,
      &deviceNameCharacteristicHandle);
  if (!reportBleStatus("add device name characteristic", status)) {
    return false;
  }

  status =
      sl_bt_gattdb_start_service(gattDbSessionId, genericAccessServiceHandle);
  if (!reportBleStatus("start Generic Access service", status)) {
    return false;
  }

  status = sl_bt_gattdb_add_service(
      gattDbSessionId, sl_bt_gattdb_primary_service,
      SL_BT_GATTDB_ADVERTISED_SERVICE, sizeof(sppServiceUuid),
      sppServiceUuid.data, &sensorServiceHandle);
  if (!reportBleStatus("add sensor service", status)) {
    return false;
  }

  const uint8_t initialCharacteristicValue = 0;
  status = sl_bt_gattdb_add_uuid128_characteristic(
      gattDbSessionId, sensorServiceHandle,
      SL_BT_GATTDB_CHARACTERISTIC_WRITE_NO_RESPONSE |
          SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,
      0, 0, sppDataCharacteristicUuid, sl_bt_gattdb_fixed_length_value,
      gattValueMaxLength, sizeof(initialCharacteristicValue),
      &initialCharacteristicValue, &sensorDataCharacteristicHandle);
  if (!reportBleStatus("add sensor data characteristic", status)) {
    return false;
  }

  status = sl_bt_gattdb_start_service(gattDbSessionId, sensorServiceHandle);
  if (!reportBleStatus("start sensor service", status)) {
    return false;
  }

  status = sl_bt_gattdb_commit(gattDbSessionId);
  return reportBleStatus("commit GATT database", status);
}

bool startAdvertising() {
  if (!advertisingSetInitialized) {
    sl_status_t status = sl_bt_advertiser_create_set(&advertisingSetHandle);
    if (!reportBleStatus("sl_bt_advertiser_create_set", status)) {
      return false;
    }

    status = sl_bt_advertiser_set_timing(advertisingSetHandle, 160, 160, 0, 0);
    if (!reportBleStatus("sl_bt_advertiser_set_timing", status)) {
      return false;
    }
    advertisingSetInitialized = true;
  }

  sl_status_t status = sl_bt_legacy_advertiser_set_data(
      advertisingSetHandle, sl_bt_advertiser_advertising_data_packet,
      sizeof(serviceAdvertisingData), serviceAdvertisingData);
  if (!reportBleStatus("set BLE service advertisement", status)) {
    return false;
  }

  status = sl_bt_legacy_advertiser_set_data(
      advertisingSetHandle, sl_bt_advertiser_scan_response_packet,
      sizeof(nameScanResponse), nameScanResponse);
  if (!reportBleStatus("set BLE name scan response", status)) {
    return false;
  }

  status = sl_bt_legacy_advertiser_start(
      advertisingSetHandle, sl_bt_advertiser_connectable_scannable);
  if (!reportBleStatus("start BLE advertisement", status)) {
    return false;
  }

  printf("BLE advertising as %s\n",
         reinterpret_cast<const char *>(advertisedName));
  return true;
}

bool sendNotificationText(const char *message) {
  if (!notificationsEnabled ||
      connectionHandle == SL_BT_INVALID_CONNECTION_HANDLE) {
    return false;
  }

  const size_t messageLength = strlen(message);
  size_t offset = 0;
  while (offset < messageLength) {
    const size_t remaining = messageLength - offset;
    const size_t chunkLength = remaining < notificationPayloadSize
                                   ? remaining
                                   : notificationPayloadSize;
    const sl_status_t status = sl_bt_gatt_server_send_notification(
        connectionHandle, sensorDataCharacteristicHandle,
        static_cast<uint8_t>(chunkLength),
        reinterpret_cast<const uint8_t *>(message + offset));
    if (!reportBleStatus("send BLE notification", status)) {
      return false;
    }
    offset += chunkLength;
  }
  return true;
}

SensorData sampleData(int loopIndex) {
  if (loopIndex < 0 || loopIndex >= SAMPLE_DATA_LENGTH) {
    loopIndex %= SAMPLE_DATA_LENGTH;
    if (loopIndex < 0) {
      loopIndex += SAMPLE_DATA_LENGTH;
    }
  }

  printf("サンプルデータのインデックス: %d\n", loopIndex);
  return rightTurn360Samples[loopIndex];
}

void processCommand(const char *command) {
  printf("BLE受信コマンド: %s\n", command);

  const char *response = "ACK,UNKNOWN\n";
  if (strcmp(command, "CMD,START") == 0) {
    streamingEnabled = true;
    response = "ACK,START\n";
  } else if (strcmp(command, "CMD,STOP") == 0) {
    streamingEnabled = false;
    response = "ACK,STOP\n";
  } else if (strcmp(command, "CMD,RESET") == 0) {
    monitoringIndex = 0;
    sessionClockStarted = false;
    streamingEnabled = true;
    response = "ACK,RESET\n";
  }

  sendNotificationText(response);
}

void processCommandByte(uint8_t byte) {
  if (byte == '\r') {
    return;
  }

  if (byte == '\n') {
    commandBuffer[commandLength] = '\0';
    if (commandLength > 0) {
      processCommand(commandBuffer);
    }
    commandLength = 0;
    return;
  }

  if (commandLength >= sizeof(commandBuffer) - 1) {
    commandLength = 0;
    printf("ERROR: BLE command is too long; discarded\n");
    return;
  }

  commandBuffer[commandLength++] = static_cast<char>(byte);
}

} // namespace

void setup() {
  Serial.begin(115200);
  // printfの出力先をSilicon Labs標準ストリームからArduino Serial/VCOMへ切り替える。
  sl_iostream_set_system_default(sl_serial_stream_handle);
  printf("XIAO MG24 BLE sensor peripheral starting\n");
}

void loop() {
  const unsigned long now = millis();
  if (!notificationsEnabled || !streamingEnabled) {
    delay(1);
    return;
  }

  if (now - lastSensorSendMs >= sensorSendIntervalMs) {
    lastSensorSendMs = now;
    if (!sessionClockStarted) {
      // 各サンプルは直前から次の100ms区間を表す。
      sessionStartedMs = now - sensorSendIntervalMs;
      sessionClockStarted = true;
    }

    const SensorData data = sampleData(monitoringIndex);
    const unsigned long elapsedMs = now - sessionStartedMs;
    char message[128];
    const int messageLength = snprintf(
        message, sizeof(message), "DATA,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        elapsedMs, data.accelX, data.accelY, data.accelZ, data.gyroX,
        data.gyroY, data.gyroZ);

    if (messageLength > 0 &&
        static_cast<size_t>(messageLength) < sizeof(message)) {
      if (sendNotificationText(message)) {
        printf("BLE送信: %s", message);
      }
    } else {
      printf("ERROR: センサーデータの整形に失敗しました\n");
    }

    monitoringIndex = (monitoringIndex + 1) % SAMPLE_DATA_LENGTH;
  }

  delay(1);
}

void sl_bt_on_event(sl_bt_msg_t *event) {
  switch (SL_BT_MSG_ID(event->header)) {
  case sl_bt_evt_system_boot_id:
    printf("BLE stack booted\n");
    if (initializeGattDatabase()) {
      startAdvertising();
    }
    break;

  case sl_bt_evt_connection_opened_id:
    connectionHandle = event->data.evt_connection_opened.connection;
    notificationsEnabled = false;
    streamingEnabled = true;
    monitoringIndex = 0;
    sessionClockStarted = false;
    printf("BLE central connected; requesting 7.5 ms interval\n");
    reportBleStatus(
        "request 7.5 ms BLE interval",
        sl_bt_connection_set_parameters(connectionHandle, 6, 6, 0, 100, 0,
                                        0xffff));
    break;

  case sl_bt_evt_connection_parameters_id: {
    const sl_bt_evt_connection_parameters_t &parameters =
        event->data.evt_connection_parameters;
    printf("BLE interval: %.2f ms, latency: %u, timeout: %u ms\n",
           parameters.interval * 1.25f,
           static_cast<unsigned>(parameters.latency),
           static_cast<unsigned>(parameters.timeout) * 10);
    break;
  }

  case sl_bt_evt_connection_closed_id:
    printf("BLE central disconnected\n");
    connectionHandle = SL_BT_INVALID_CONNECTION_HANDLE;
    notificationsEnabled = false;
    streamingEnabled = true;
    monitoringIndex = 0;
    sessionClockStarted = false;
    commandLength = 0;
    startAdvertising();
    break;

  case sl_bt_evt_gatt_mtu_exchanged_id: {
    const uint16_t mtu = event->data.evt_gatt_mtu_exchanged.mtu;
    if (mtu > 3) {
      const uint16_t payloadSize = mtu - 3;
      notificationPayloadSize =
          payloadSize < gattValueMaxLength ? payloadSize : gattValueMaxLength;
    }
    printf("BLE MTU: %u, payload: %u\n", static_cast<unsigned>(mtu),
           static_cast<unsigned>(notificationPayloadSize));
    break;
  }

  case sl_bt_evt_gatt_server_characteristic_status_id: {
    const sl_bt_evt_gatt_server_characteristic_status_t &status =
        event->data.evt_gatt_server_characteristic_status;
    if (status.characteristic == sensorDataCharacteristicHandle &&
        status.status_flags == sl_bt_gatt_server_client_config) {
      notificationsEnabled =
          status.client_config_flags == sl_bt_gatt_server_notification;
      if (notificationsEnabled) {
        monitoringIndex = 0;
        sessionClockStarted = false;
        lastSensorSendMs = millis();
        printf("BLE notifications enabled; streaming sensor data\n");
      } else {
        printf("BLE notifications disabled\n");
      }
    }
    break;
  }

  case sl_bt_evt_gatt_server_attribute_value_id: {
    const sl_bt_evt_gatt_server_attribute_value_t &value =
        event->data.evt_gatt_server_attribute_value;
    if (value.attribute == sensorDataCharacteristicHandle) {
      for (uint8_t i = 0; i < value.value.len; ++i) {
        processCommandByte(value.value.data[i]);
      }
    }
    break;
  }

  default:
    break;
  }
}

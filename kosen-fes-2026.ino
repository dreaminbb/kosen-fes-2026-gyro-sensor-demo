// PCへの送信経路:
//   MG24がGATTサービス/特性を用意してアドバタイズ ->
//   C6が発見・接続して通知を購読
//   -> 購読開始後、MG24が実センサー値をDATA行にして10msごとに通知
//   -> C6が通知をUSBシリアルへ転送 -> Godotがシリアル行を解析して画面を更新
// 逆方向はGodotのCMD行をC6がBLE Write Without
// ResponseでMG24へ転送し、ACKを通知する。
#include "arduino_serial_config.h"
#include <Arduino.h>
#include <LSM6DS3.h>
#include <Wire.h>
#include <atomic>
#include <queue.h>
#include <sl_bluetooth.h>
#include <sl_bt_api.h>
#include <stdio.h>
#include <string.h>

#define SENSOR_POLL_INTERVAL_MS 0
#define SENSOR_THRESHOLD_ACCEL 0.1f

struct SensorData {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

namespace {
// BLEイベントタスクのスタックは1000 bytes。浮動小数点のprintfや
// コマンド処理をそこで実行するとスタックを使い切るため、Arduinoタスクへ渡す。
constexpr size_t BLE_EVENT_QUEUE_LENGTH = 16;
StaticQueue_t bleEventQueueControl;
uint8_t bleEventQueueStorage[BLE_EVENT_QUEUE_LENGTH * sizeof(sl_bt_msg_t)];
QueueHandle_t bleEventQueue =
    xQueueCreateStatic(BLE_EVENT_QUEUE_LENGTH, sizeof(sl_bt_msg_t),
                       bleEventQueueStorage, &bleEventQueueControl);
std::atomic<bool> bleEventQueueOverflow{false};

const uint8_t advertisedName[] = "KosenBall";
const uint8_t serviceAdvertisingData[] = {
    0x02, 0x01, 0x06, 0x11, 0x07, 0x07, 0xb9, 0xf9, 0xd7, 0x50, 0xa4,
    0x20, 0x89, 0x77, 0x40, 0xcb, 0xfd, 0x2c, 0xc1, 0x80, 0x48};
const uint8_t nameScanResponse[] = {0x0a, 0x09, 'K', 'o', 's', 'e',
                                    'n',  'B',  'a', 'l', 'l'};
const uint16_t gattValueMaxLength = 250;
const uint16_t defaultNotificationPayloadSize = 20;
// Arduino-ESP32 3.3.12のNimBLE通知コールバックは最初のmbufだけを渡す。
// 51 bytes以内なら1つのmbufに収まり、C6側で通知が欠けない。
const uint16_t maxNotificationPayloadSize = 51;
// IMUは833 Hzで更新し、Godotの物理更新に近い100 Hzで最新値を送る。
constexpr unsigned long SENSOR_SEND_INTERVAL_US = 10000;
constexpr unsigned long NOTIFICATION_RETRY_INTERVAL_US = 5000;

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
unsigned long sessionStartedUs = 0;
unsigned long lastSensorSendUs = 0;
char pendingNotification[128];
size_t pendingNotificationLength = 0;
size_t pendingNotificationOffset = 0;
bool pendingNotificationIsControl = false;
char queuedControlNotification[20];
size_t queuedControlNotificationLength = 0;
unsigned long nextNotificationAttemptUs = 0;

// 加速度センサーのインスタンス
LSM6DS3 myIMU(I2C_MODE, 0x6A);
bool imuReady = false;

SensorData fetchSensorData() {
  // 各軸を一度ずつ読み、I2C読み出しの重複を避ける。
  const float accelX = myIMU.readFloatAccelX();
  const float accelY = myIMU.readFloatAccelY();
  const float accelZ = myIMU.readFloatAccelZ();
  if (fabs(accelX) < SENSOR_THRESHOLD_ACCEL &&
      fabs(accelY) < SENSOR_THRESHOLD_ACCEL &&
      fabs(accelZ) < SENSOR_THRESHOLD_ACCEL) {
    Serial.print("WARNING: LSM6DS3 acceleration is too small; check sensor\n");
    return SensorData{0, 0, 0, 0, 0, 0};
  }
  return SensorData{accelX, accelY, accelZ, myIMU.readFloatGyroX(),
                    myIMU.readFloatGyroY(),  myIMU.readFloatGyroZ()};
}

char commandBuffer[64];
size_t commandLength = 0;

bool reportBleStatus(const char *operation, sl_status_t status) {
  if (status == SL_STATUS_OK) {
    return true;
  }

  Serial.printf("ERROR: %s failed with BLE status 0x%08lX\n", operation,
                static_cast<unsigned long>(status));
  return false;
}

bool initializeGattDatabase() {
  // データ特性はMG24→C6のNotificationと、C6→MG24のWrite Without
  // Responseを兼ねる。
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
  if (!reportBleStatus("commit GATT database", status)) {
    return false;
  }
  Serial.printf("BLE GATT ready: service=%u characteristic=%u\n",
                static_cast<unsigned>(sensorServiceHandle),
                static_cast<unsigned>(sensorDataCharacteristicHandle));
  return true;
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

  Serial.printf("BLE advertising as %s\n",
                reinterpret_cast<const char *>(advertisedName));
  return true;
}

bool sendNotificationText(const char *message, bool isControl = false) {
  if (!notificationsEnabled ||
      connectionHandle == SL_BT_INVALID_CONNECTION_HANDLE) {
    return false;
  }

  const size_t messageLength = strlen(message);
  if (messageLength == 0 || messageLength > sizeof(pendingNotification)) {
    return false;
  }
  if (isControl && pendingNotificationLength > 0 &&
      pendingNotificationOffset > 0) {
    if (messageLength > sizeof(queuedControlNotification)) {
      return false;
    }
    memcpy(queuedControlNotification, message, messageLength);
    queuedControlNotificationLength = messageLength;
    return true;
  }
  if (pendingNotificationLength > 0 &&
      (pendingNotificationOffset > 0 || pendingNotificationIsControl ||
       queuedControlNotificationLength > 0)) {
    return false;
  }
  if (pendingNotificationLength == 0) {
    nextNotificationAttemptUs = micros();
  }
  // 未送信のDATAだけを最新値で置き換える。分割送信済みの行は最後まで送る。
  memcpy(pendingNotification, message, messageLength);
  pendingNotificationLength = messageLength;
  pendingNotificationOffset = 0;
  pendingNotificationIsControl = isControl;
  return true;
}

void flushNotification(unsigned long nowUs) {
  if (!notificationsEnabled ||
      connectionHandle == SL_BT_INVALID_CONNECTION_HANDLE ||
      pendingNotificationLength == 0 ||
      static_cast<int32_t>(nowUs - nextNotificationAttemptUs) < 0) {
    return;
  }

  const size_t remaining = pendingNotificationLength - pendingNotificationOffset;
  const size_t chunkLength = remaining < notificationPayloadSize
                                 ? remaining
                                 : notificationPayloadSize;
  const sl_status_t status = sl_bt_gatt_server_send_notification(
      connectionHandle, sensorDataCharacteristicHandle, chunkLength,
      reinterpret_cast<const uint8_t *>(pendingNotification +
                                        pendingNotificationOffset));
  if (status == SL_STATUS_NO_MORE_RESOURCE) {
    nextNotificationAttemptUs = nowUs + NOTIFICATION_RETRY_INTERVAL_US;
    return;
  }
  if (!reportBleStatus("send BLE notification", status)) {
    pendingNotificationLength = 0;
    pendingNotificationOffset = 0;
    queuedControlNotificationLength = 0;
    return;
  }
  pendingNotificationOffset += chunkLength;
  nextNotificationAttemptUs = nowUs;
  if (pendingNotificationOffset == pendingNotificationLength) {
    pendingNotificationLength = 0;
    pendingNotificationOffset = 0;
    if (queuedControlNotificationLength > 0) {
      memcpy(pendingNotification, queuedControlNotification,
             queuedControlNotificationLength);
      pendingNotificationLength = queuedControlNotificationLength;
      queuedControlNotificationLength = 0;
      pendingNotificationIsControl = true;
    }
  }
}

void processCommand(const char *command) {
  Serial.printf("BLE受信コマンド: %s\n", command);

  const char *response = "ACK,UNKNOWN\n";
  if (strcmp(command, "CMD,START") == 0) {
    streamingEnabled = true;
    response = "ACK,START\n";
  } else if (strcmp(command, "CMD,STOP") == 0) {
    streamingEnabled = false;
    response = "ACK,STOP\n";
  } else if (strcmp(command, "CMD,RESET") == 0) {
    sessionClockStarted = false;
    streamingEnabled = true;
    response = "ACK,RESET\n";
  }

  sendNotificationText(response, true);
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
    Serial.print("ERROR: BLE command is too long; discarded\n");
    return;
  }

  commandBuffer[commandLength++] = static_cast<char>(byte);
}

} // namespace

void handleBleEvent(sl_bt_msg_t *event);

void setup() {
  Serial.begin(115200);
  pinMode(PD5, OUTPUT);
  digitalWrite(PD5, HIGH);

  // Serial.printの出力先をSilicon Labs標準ストリームからArduino
  // Serial/VCOMへ切り替える。
  sl_iostream_set_system_default(sl_serial_stream_handle);
  Serial.print("XIAO MG24 BLE sensor peripheral starting\n");

  Serial.print("LSM6DS3 initialization starting\n");
  myIMU.settings.accelSampleRate = 833;
  myIMU.settings.gyroSampleRate = 833;
  myIMU.settings.accelBandWidth = 400;
  imuReady = myIMU.begin() == 0;
  if (imuReady) {
    // LSM6DS3ライブラリのI2C_MODEはWireを使用する。
    Wire.setClock(400000);
    Serial.print("LSM6DS3 initialized\n");
  } else {
    Serial.print("ERROR: LSM6DS3 initialization failed\n");
  }
}

void loop() {
  // 起動・接続・購読・コマンドを同じArduinoタスクで順番に処理する。
  // センサー未初期化/送信停止中でもBLEイベントは処理し続ける。
  sl_bt_msg_t event;
  while (xQueueReceive(bleEventQueue, &event, 0) == pdTRUE) {
    handleBleEvent(&event);
  }
  if (bleEventQueueOverflow.exchange(false)) {
    Serial.print("ERROR: BLE event queue overflow\n");
  }

  const unsigned long nowUs = micros();
  if (!imuReady || !notificationsEnabled || !streamingEnabled) {
    flushNotification(nowUs);
    delay(1);
    return;
  }

  // 接続しただけでは送らず、C6がCCCDで通知購読を有効にした後に送信する。
  if (nowUs - lastSensorSendUs >= SENSOR_SEND_INTERVAL_US) {
    // マイクロ秒の基準時刻で周期を保ち、長い停止後に追いつき送信をしない。
    lastSensorSendUs += SENSOR_SEND_INTERVAL_US;
    if (nowUs - lastSensorSendUs >= SENSOR_SEND_INTERVAL_US) {
      lastSensorSendUs = nowUs;
    }
    // 既に行の一部を送信中、または制御応答を待機中なら読み出しを省く。
    if (pendingNotificationLength == 0 ||
        (pendingNotificationOffset == 0 && !pendingNotificationIsControl &&
         queuedControlNotificationLength == 0)) {
      const SensorData data = fetchSensorData();
      const unsigned long sampleTimeUs = micros();
      if (!sessionClockStarted) {
        // 初回サンプル時刻を1周期前とし、以降はマイクロ秒単位で送る。
        sessionStartedUs = sampleTimeUs - SENSOR_SEND_INTERVAL_US;
        sessionClockStarted = true;
      }
      const unsigned long elapsedUs = sampleTimeUs - sessionStartedUs;
      char message[128];
      const int messageLength = snprintf(
          message, sizeof(message), "DATA,%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
          elapsedUs, data.accelX, data.accelY, data.accelZ, data.gyroX,
          data.gyroY, data.gyroZ);

      if (messageLength > 0 &&
          static_cast<size_t>(messageLength) < sizeof(message)) {
        sendNotificationText(message);
      } else {
        Serial.print("ERROR: センサーデータの整形に失敗しました\n");
      }
    }
  }

  flushNotification(micros());
  delay(SENSOR_POLL_INTERVAL_MS);
}

// Bluetoothのイベントタスクから呼ばれる。イベントのポインターはコールバック
// 終了後に再利用されるため、キューへ内容をコピーしてすぐに戻る。
// ログ整形やBLEコマンドの実行はloop()側のhandleBleEvent()で行う。
void sl_bt_on_event(sl_bt_msg_t *event) {
  switch (SL_BT_MSG_ID(event->header)) {
  case sl_bt_evt_system_boot_id:
  case sl_bt_evt_connection_opened_id:
  case sl_bt_evt_connection_parameters_id:
  case sl_bt_evt_connection_closed_id:
  case sl_bt_evt_gatt_mtu_exchanged_id:
  case sl_bt_evt_gatt_server_characteristic_status_id:
  case sl_bt_evt_gatt_server_attribute_value_id:
    if (xQueueSend(bleEventQueue, event, 0) != pdTRUE) {
      bleEventQueueOverflow.store(true);
    }
    break;
  default:
    break;
  }
}

// ArduinoタスクでGATT初期化、接続設定、通知購読、受信コマンドを処理する。
void handleBleEvent(sl_bt_msg_t *event) {
  switch (SL_BT_MSG_ID(event->header)) {
  case sl_bt_evt_system_boot_id:
    // BLEスタック起動時にGATTサービスと特性を登録し、C6から見つけられるようアドバタイズする。
    Serial.print("BLE stack booted\n");
    if (initializeGattDatabase()) {
      startAdvertising();
    }
    break;

  case sl_bt_evt_connection_opened_id:
    // C6との接続ハンドルを保存し、購読が完了するまではデータ通知を待機させる。
    connectionHandle = event->data.evt_connection_opened.connection;
    // MTUは接続ごとに交渉する。前の接続の大きなペイロード上限を引き継がない。
    notificationPayloadSize = defaultNotificationPayloadSize;
    notificationsEnabled = false;
    pendingNotificationLength = 0;
    pendingNotificationOffset = 0;
    queuedControlNotificationLength = 0;
    streamingEnabled = true;
    sessionClockStarted = false;
    Serial.print("BLE central connected; requesting 7.5 ms interval\n");
    reportBleStatus("request 7.5 ms BLE interval",
                    sl_bt_connection_set_parameters(connectionHandle, 6, 6, 0,
                                                    100, 0, 0xffff));
    break;

  case sl_bt_evt_connection_parameters_id: {
    // ネゴシエーション後にBLEスタックが適用した接続間隔などをログへ出す。
    const sl_bt_evt_connection_parameters_t &parameters =
        event->data.evt_connection_parameters;
    Serial.printf("BLE interval: %.2f ms, latency: %u, timeout: %u ms\n",
                  parameters.interval * 1.25f,
                  static_cast<unsigned>(parameters.latency),
                  static_cast<unsigned>(parameters.timeout) * 10);
    break;
  }

  case sl_bt_evt_connection_closed_id:
    // 接続に紐づく状態やコマンド途中の内容を初期化し、次のC6接続に備えてアドバタイズを再開する。
    Serial.printf(
        "BLE central disconnected: reason=0x%04X\n",
        static_cast<unsigned>(event->data.evt_connection_closed.reason));
    connectionHandle = SL_BT_INVALID_CONNECTION_HANDLE;
    notificationsEnabled = false;
    pendingNotificationLength = 0;
    pendingNotificationOffset = 0;
    queuedControlNotificationLength = 0;
    streamingEnabled = true;
    sessionClockStarted = false;
    commandLength = 0;
    startAdvertising();
    break;

  case sl_bt_evt_gatt_mtu_exchanged_id: {
    // 交渉されたMTUから通知1回で送れるデータ量（MTU -
    // ATTヘッダー3バイト）を更新する。
    const uint16_t mtu = event->data.evt_gatt_mtu_exchanged.mtu;
    if (mtu > 3) {
      const uint16_t payloadSize = mtu - 3;
      notificationPayloadSize = payloadSize < maxNotificationPayloadSize
                                    ? payloadSize
                                    : maxNotificationPayloadSize;
    }
    Serial.printf("BLE MTU: %u, payload: %u\n", static_cast<unsigned>(mtu),
                  static_cast<unsigned>(notificationPayloadSize));
    break;
  }

  case sl_bt_evt_gatt_server_characteristic_status_id: {
    // C6がデータ特性の通知購読を有効/無効にした状態変化を反映する。
    const sl_bt_evt_gatt_server_characteristic_status_t &status =
        event->data.evt_gatt_server_characteristic_status;
    if (status.connection == connectionHandle &&
        status.characteristic == sensorDataCharacteristicHandle &&
        status.status_flags == sl_bt_gatt_server_client_config) {
      notificationsEnabled =
          status.client_config_flags == sl_bt_gatt_server_notification;
      if (notificationsEnabled) {
        // C6の通知購読が有効になった時点から、loop()がDATA行を送り始める。
        sessionClockStarted = false;
        lastSensorSendUs = micros();
        Serial.print("BLE notifications enabled; streaming sensor data\n");
        if (!imuReady) {
          sendNotificationText("ERROR,IMU_NOT_READY\n", true);
        }
      } else {
        pendingNotificationLength = 0;
        pendingNotificationOffset = 0;
        queuedControlNotificationLength = 0;
        Serial.print("BLE notifications disabled\n");
      }
    }
    break;
  }

  case sl_bt_evt_gatt_server_attribute_value_id: {
    // C6からデータ特性へ書き込まれたバイト列を受け取り、改行区切りコマンドとして処理する。
    const sl_bt_evt_gatt_server_attribute_value_t &value =
        event->data.evt_gatt_server_attribute_value;
    if (value.attribute == sensorDataCharacteristicHandle) {
      // C6から届くコマンドは改行区切り。分割された書き込みも1文字ずつ蓄積する。
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

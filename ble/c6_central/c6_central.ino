// PCへのデータ経路:
//   MG24のアドバタイズをサービスUUIDで検索 -> GATT接続 -> サービス/特性を探索
//   -> Notificationを購読 -> MG24のDATA/ACK通知を「RX,」付きでUSBシリアルへ出力
// Godotはこのシリアルを読み取る。逆方向のCMD行はUSBシリアルから読み、BLE書き込みでMG24へ渡す。
#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <atomic>
#include <host/ble_gap.h>
#include <host/ble_gatt.h>
#include <host/ble_hs.h>
#include <host/ble_hs_id.h>
#include <string.h>

namespace {

constexpr char kDeviceName[] = "KosenBall";
constexpr char kServiceUuid[] = "4880c12c-fdcb-4077-8920-a450d7f9b907";
constexpr char kDataCharacteristicUuid[] =
    "fec26ec4-6d71-4442-9f81-55bc21d658d6";
// MG24の送信バッファは128 bytes。RX,接頭辞を含めても160 bytesで足りる。
constexpr size_t NOTIFICATION_LINE_CAPACITY = 160;
constexpr size_t SERIAL_FRAME_CAPACITY = 160;
constexpr unsigned long DATA_TIMEOUT_MS = 2000;

BLEScan *scanner = nullptr;
BLEAddress peerAddress;
BLEUUID serviceUuid(kServiceUuid);
BLEUUID dataUuid(kDataCharacteristicUuid);
BLEUUID cccdUuid(static_cast<uint16_t>(0x2902));
uint8_t peerAddressType = 0;
bool peerFound = false;
bool connected = false;
bool initialCommandSent = false;
bool streamingExpected = true;
enum class LinkState : uint8_t {
  idle,
  connecting,
  discoveringService,
  discoveringCharacteristic,
  discoveringDescriptor,
  subscribing,
  ready,
  failed,
  disconnecting
};
std::atomic<LinkState> linkState{LinkState::idle};
std::atomic<int> linkError{0};
std::atomic<uint16_t> connectionHandle{BLE_HS_CONN_HANDLE_NONE};
uint16_t serviceStartHandle = 0;
uint16_t serviceEndHandle = 0;
uint16_t dataHandle = 0;
uint16_t cccdHandle = 0;
unsigned long linkStartedAtMs = 0;
unsigned long connectedAtMs = 0;
char serialCommand[16];
size_t serialCommandLength = 0;
char notificationLine[NOTIFICATION_LINE_CAPACITY];
size_t notificationLineLength = 0;
bool discardingNotificationLine = false;
char pendingSerialFrame[SERIAL_FRAME_CAPACITY];
size_t pendingSerialFrameLength = 0;
bool pendingSerialFramePriority = false;
char serialOutputFrame[SERIAL_FRAME_CAPACITY];
size_t serialOutputFrameLength = 0;
size_t serialOutputFrameOffset = 0;
portMUX_TYPE serialFrameMux = portMUX_INITIALIZER_UNLOCKED;
std::atomic<unsigned long> lastDataFrameMs{0};

void enqueueSerialFrame(const char *line) {
  const size_t lineLength = strlen(line);
  if (lineLength + 4 > SERIAL_FRAME_CAPACITY) {
    return;
  }

  const bool isPriority = strncmp(line, "ACK,", 4) == 0 ||
                          strncmp(line, "ERROR,", 6) == 0;
  portENTER_CRITICAL(&serialFrameMux);
  if (pendingSerialFrameLength > 0 && pendingSerialFramePriority && !isPriority) {
    portEXIT_CRITICAL(&serialFrameMux);
    return;
  }
  memcpy(pendingSerialFrame, "RX,", 3);
  memcpy(pendingSerialFrame + 3, line, lineLength);
  pendingSerialFrame[3 + lineLength] = '\n';
  pendingSerialFrameLength = 4 + lineLength;
  pendingSerialFramePriority = isPriority;
  portEXIT_CRITICAL(&serialFrameMux);
}

void flushSerialFrames() {
  for (;;) {
    if (serialOutputFrameOffset == serialOutputFrameLength) {
      portENTER_CRITICAL(&serialFrameMux);
      if (pendingSerialFrameLength == 0) {
        portEXIT_CRITICAL(&serialFrameMux);
        return;
      }
      memcpy(serialOutputFrame, pendingSerialFrame, pendingSerialFrameLength);
      serialOutputFrameLength = pendingSerialFrameLength;
      serialOutputFrameOffset = 0;
      pendingSerialFrameLength = 0;
      pendingSerialFramePriority = false;
      portEXIT_CRITICAL(&serialFrameMux);
    }
    const int available = Serial.availableForWrite();
    if (available <= 0) {
      return;
    }
    const size_t remaining = serialOutputFrameLength - serialOutputFrameOffset;
    const size_t chunkLength = remaining < static_cast<size_t>(available)
                                   ? remaining
                                   : static_cast<size_t>(available);
    const size_t written = Serial.write(
        reinterpret_cast<const uint8_t *>(serialOutputFrame +
                                          serialOutputFrameOffset),
        chunkLength);
    if (written == 0) {
      return;
    }
    serialOutputFrameOffset += written;
  }
}

void receiveNotification(const uint8_t *data, size_t length) {
  // MTU 23なら1通知は20 bytes。通知境界に改行を足さず、MG24のLFまで
  // 蓄積してからRX,を付ける。Godotは完全なDATA/ACK行だけを受け取る。
  for (size_t index = 0; index < length; ++index) {
    const char incoming = static_cast<char>(data[index]);
    if (incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      if (!discardingNotificationLine && notificationLineLength > 0) {
        notificationLine[notificationLineLength] = '\0';
        if (strncmp(notificationLine, "DATA,", 5) == 0) {
          size_t separators = 0;
          for (size_t position = 0; position < notificationLineLength;
               ++position) {
            separators += notificationLine[position] == ',';
          }
          if (separators == 7) {
            lastDataFrameMs.store(millis());
          }
        }
        enqueueSerialFrame(notificationLine);
      }
      notificationLineLength = 0;
      discardingNotificationLine = false;
    } else if (!discardingNotificationLine) {
      if (notificationLineLength < sizeof(notificationLine) - 1) {
        notificationLine[notificationLineLength++] = incoming;
      } else {
        notificationLineLength = 0;
        discardingNotificationLine = true;
      }
    }
  }
}

bool scanForPeripheral() {
  // MG24はKosenBallという名前とセンサーサービスUUIDを含むアドバタイズを送信する。
  printf("BLE scanning for %s\n", kDeviceName);
  peerFound = false;
  scanner->clearResults();
  BLEScanResults *results = scanner->start(1, false);
  if (results == nullptr) {
    printf("BLE scan returned no results\n");
    return false;
  }

  printf("BLE scan saw %d device(s)\n", results->getCount());
  for (int index = 0; index < results->getCount(); ++index) {
    BLEAdvertisedDevice device = results->getDevice(index);
    const bool matchingService =
        device.haveServiceUUID() &&
        device.isAdvertisingService(BLEUUID(kServiceUuid));
    // 同名の旧ファームウェアを選ばないよう、センサーサービスを必須とする。
    if (!matchingService) {
      continue;
    }

    peerAddress = device.getAddress();
    peerAddressType = device.getAddressType();
    peerFound = true;
    printf("BLE found %s at %s address_type=%u\n", kDeviceName,
           peerAddress.toString().c_str(),
           static_cast<unsigned>(peerAddressType));
    break;
  }

  scanner->clearResults();
  return peerFound;
}

void failLink(int error) {
  linkError.store(error);
  linkState.store(LinkState::failed);
}

int onSubscriptionWritten(uint16_t handle, const ble_gatt_error *error,
                          ble_gatt_attr *attribute, void *argument) {
  (void)attribute;
  (void)argument;
  if (handle == connectionHandle.load() &&
      linkState.load() == LinkState::subscribing) {
    if (error->status == 0) {
      linkState.store(LinkState::ready);
    } else {
      failLink(error->status);
    }
  }
  return 0;
}

int onDescriptorDiscovered(uint16_t handle, const ble_gatt_error *error,
                           uint16_t characteristicHandle,
                           const ble_gatt_dsc *descriptor, void *argument) {
  (void)argument;
  if (handle != connectionHandle.load() ||
      linkState.load() != LinkState::discoveringDescriptor) {
    return 0;
  }
  if (error->status == 0 && descriptor != nullptr) {
    if (characteristicHandle == dataHandle &&
        ble_uuid_cmp(&descriptor->uuid.u, &cccdUuid.getNative()->u) == 0) {
      cccdHandle = descriptor->handle;
    }
  } else if (error->status == BLE_HS_EDONE) {
    if (cccdHandle == 0) {
      failLink(BLE_HS_ENOENT);
      return 0;
    }
    const uint8_t enableNotifications[] = {1, 0};
    linkState.store(LinkState::subscribing);
    const int result = ble_gattc_write_flat(
        handle, cccdHandle, enableNotifications, sizeof(enableNotifications),
        onSubscriptionWritten, nullptr);
    if (result != 0) {
      failLink(result);
    }
  } else if (error->status != 0) {
    failLink(error->status);
  }
  return 0;
}

int onCharacteristicDiscovered(uint16_t handle, const ble_gatt_error *error,
                                const ble_gatt_chr *characteristic,
                                void *argument) {
  (void)argument;
  if (handle != connectionHandle.load() ||
      linkState.load() != LinkState::discoveringCharacteristic) {
    return 0;
  }
  if (error->status == 0 && characteristic != nullptr) {
    const uint8_t required = BLE_GATT_CHR_F_NOTIFY |
                             BLE_GATT_CHR_F_WRITE_NO_RSP;
    if ((characteristic->properties & required) == required) {
      dataHandle = characteristic->val_handle;
    }
  } else if (error->status == BLE_HS_EDONE) {
    if (dataHandle == 0) {
      failLink(BLE_HS_ENOENT);
      return 0;
    }
    linkState.store(LinkState::discoveringDescriptor);
    const int result = ble_gattc_disc_all_dscs(
        handle, dataHandle, serviceEndHandle, onDescriptorDiscovered, nullptr);
    if (result != 0) {
      failLink(result);
    }
  } else if (error->status != 0) {
    failLink(error->status);
  }
  return 0;
}

int onServiceDiscovered(uint16_t handle, const ble_gatt_error *error,
                        const ble_gatt_svc *service, void *argument) {
  (void)argument;
  if (handle != connectionHandle.load() ||
      linkState.load() != LinkState::discoveringService) {
    return 0;
  }
  if (error->status == 0 && service != nullptr) {
    serviceStartHandle = service->start_handle;
    serviceEndHandle = service->end_handle;
  } else if (error->status == BLE_HS_EDONE) {
    if (serviceStartHandle == 0) {
      failLink(BLE_HS_ENOENT);
      return 0;
    }
    linkState.store(LinkState::discoveringCharacteristic);
    const int result = ble_gattc_disc_chrs_by_uuid(
        handle, serviceStartHandle, serviceEndHandle, &dataUuid.getNative()->u,
        onCharacteristicDiscovered, nullptr);
    if (result != 0) {
      failLink(result);
    }
  } else if (error->status != 0) {
    failLink(error->status);
  }
  return 0;
}

int onGapEvent(ble_gap_event *event, void *argument) {
  (void)argument;
  switch (event->type) {
  case BLE_GAP_EVENT_CONNECT:
    if (event->connect.status != 0) {
      failLink(event->connect.status);
      break;
    }
    if (linkState.load() != LinkState::connecting) {
      ble_gap_terminate(event->connect.conn_handle,
                        BLE_ERR_REM_USER_CONN_TERM);
      break;
    }
    connectionHandle.store(event->connect.conn_handle);
    linkState.store(LinkState::discoveringService);
    if (const int result = ble_gattc_disc_svc_by_uuid(
            event->connect.conn_handle, &serviceUuid.getNative()->u,
            onServiceDiscovered, nullptr); result != 0) {
      failLink(result);
    }
    break;
  case BLE_GAP_EVENT_DISCONNECT:
    if (event->disconnect.conn.conn_handle == connectionHandle.load()) {
      connectionHandle.store(BLE_HS_CONN_HANDLE_NONE);
      linkState.store(LinkState::idle);
    }
    break;
  case BLE_GAP_EVENT_NOTIFY_RX: {
    if (event->notify_rx.conn_handle != connectionHandle.load() ||
        event->notify_rx.attr_handle != dataHandle) {
      break;
    }
    const uint16_t length = OS_MBUF_PKTLEN(event->notify_rx.om);
    uint8_t notification[NOTIFICATION_LINE_CAPACITY];
    if (length <= sizeof(notification) &&
        os_mbuf_copydata(event->notify_rx.om, 0, length, notification) == 0) {
      receiveNotification(notification, length);
    }
    break;
  }
  default:
    break;
  }
  return 0;
}

bool connectToPeripheral() {
  if (!peerFound) {
    return false;
  }
  ble_addr_t peer;
  memcpy(peer.val, peerAddress.getNative(), sizeof(peer.val));
  peer.type = peerAddressType;
  uint8_t ownAddressType = 0;
  const int addressResult = ble_hs_id_infer_auto(0, &ownAddressType);
  if (addressResult != 0) {
    printf("ERROR,BLE address selection failed: %d\n", addressResult);
    return false;
  }

  serviceStartHandle = 0;
  serviceEndHandle = 0;
  dataHandle = 0;
  cccdHandle = 0;
  notificationLineLength = 0;
  discardingNotificationLine = false;
  connectionHandle.store(BLE_HS_CONN_HANDLE_NONE);
  linkState.store(LinkState::connecting);
  linkStartedAtMs = millis();
  printf("GATT connect request\n");
  const int result = ble_gap_connect(ownAddressType, &peer, 10000, nullptr,
                                     onGapEvent, nullptr);
  if (result != 0) {
    failLink(result);
    return false;
  }
  return true;
}

void sendCommand(const char *command, size_t length) {
  if (!connected || connectionHandle.load() == BLE_HS_CONN_HANDLE_NONE ||
      dataHandle == 0 || length == 0) {
    printf("ERROR,BLE command requires an active connection\n");
    return;
  }

  char packet[20];
  if (length + 1 > sizeof(packet) || length + 1 > 20) {
    printf("ERROR,command too long for one BLE write\n");
    return;
  }

  memcpy(packet, command, length);
  packet[length] = '\n';
  if (ble_gattc_write_no_rsp_flat(connectionHandle.load(), dataHandle, packet,
                                 length + 1) == 0) {
    if (length == strlen("CMD,STOP") &&
        memcmp(command, "CMD,STOP", length) == 0) {
      streamingExpected = false;
    } else if ((length == strlen("CMD,START") &&
                memcmp(command, "CMD,START", length) == 0) ||
               (length == strlen("CMD,RESET") &&
                memcmp(command, "CMD,RESET", length) == 0)) {
      streamingExpected = true;
      lastDataFrameMs.store(millis());
    }
    printf("TX,%.*s\n", static_cast<int>(length), command);
  } else {
    printf("ERROR,BLE write failed\n");
  }
}

void pollSerialCommands() {
  // GodotがUSBシリアルへ書いた改行区切りCMDを1行ずつBLEへ転送する。
  while (Serial.available() > 0) {
    const int incoming = Serial.read();
    if (incoming < 0 || incoming == '\r') {
      continue;
    }
    if (incoming == '\n') {
      if (serialCommandLength > 0) {
        sendCommand(serialCommand, serialCommandLength);
      }
      serialCommandLength = 0;
      continue;
    }
    if (serialCommandLength < sizeof(serialCommand) - 1) {
      serialCommand[serialCommandLength++] = static_cast<char>(incoming);
    } else {
      serialCommandLength = 0;
      printf("ERROR,serial command too long\n");
    }
  }
}

} // namespace

void setup() {
  Serial.begin(115200);
  // 完成したRX行をstdioのバッファに留めず、すぐUSBへ渡す。
  setvbuf(stdout, nullptr, _IONBF, 0);
  delay(300);
  printf("BOOT,C6 BLE central\n");

  BLEDevice::init("C6Central");
  BLEDevice::setMTU(247);
  scanner = BLEDevice::getScan();
  scanner->setActiveScan(true);
  scanner->setInterval(30);
  scanner->setWindow(30);
}

void loop() {
  flushSerialFrames();
  const LinkState state = linkState.load();
  if (state == LinkState::failed) {
    printf("ERROR,BLE connection or discovery failed: %d\n", linkError.load());
    const uint16_t handle = connectionHandle.load();
    if (handle != BLE_HS_CONN_HANDLE_NONE) {
      linkState.store(LinkState::disconnecting);
      linkStartedAtMs = millis();
      ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
    } else {
      linkState.store(LinkState::idle);
    }
    delay(100);
    return;
  }

  if (state == LinkState::idle) {
    if (connected) {
      printf("BLE disconnected; will rescan\n");
    }
    connected = false;
    initialCommandSent = false;
    // 切断中は再スキャンし、見つけたMG24への接続と通知購読をやり直す。
    if (!scanForPeripheral()) {
      delay(250);
      return;
    }
    if (!connectToPeripheral()) {
      delay(250);
    }
    return;
  }

  if (state == LinkState::disconnecting) {
    if (millis() - linkStartedAtMs > 3000) {
      connectionHandle.store(BLE_HS_CONN_HANDLE_NONE);
      linkState.store(LinkState::idle);
    }
    delay(1);
    return;
  }

  if (state != LinkState::ready) {
    if (millis() - linkStartedAtMs > 10000) {
      printf("ERROR,BLE connection or discovery timed out\n");
      const uint16_t handle = connectionHandle.load();
      if (handle == BLE_HS_CONN_HANDLE_NONE) {
        ble_gap_conn_cancel();
        linkState.store(LinkState::disconnecting);
        linkStartedAtMs = millis();
      } else {
        ble_gap_terminate(handle, BLE_ERR_REM_USER_CONN_TERM);
        linkState.store(LinkState::disconnecting);
        linkStartedAtMs = millis();
      }
    }
    delay(1);
    return;
  }

  if (!connected) {
    connected = true;
    streamingExpected = true;
    connectedAtMs = millis();
    lastDataFrameMs.store(connectedAtMs);
    printf("BLE subscribed; bidirectional GATT ready\n");
  }

  if (streamingExpected && millis() - lastDataFrameMs.load() > DATA_TIMEOUT_MS) {
    printf("ERROR,BLE subscribed but no sensor DATA for %lu ms; reconnecting\n",
           DATA_TIMEOUT_MS);
    linkState.store(LinkState::disconnecting);
    linkStartedAtMs = millis();
    ble_gap_terminate(connectionHandle.load(), BLE_ERR_REM_USER_CONN_TERM);
    return;
  }

  if (!initialCommandSent && millis() - connectedAtMs >= 250) {
    sendCommand("CMD,RESET", strlen("CMD,RESET"));
    initialCommandSent = true;
  }
  pollSerialCommands();
  flushSerialFrames();
  delay(1);
}

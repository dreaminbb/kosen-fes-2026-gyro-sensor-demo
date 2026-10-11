// PCへのデータ経路:
//   MG24のアドバタイズをサービスUUIDで検索 -> GATT接続 -> サービス/特性を探索
//   -> Notificationを購読 -> MG24のDATA/ACK通知を「RX,」付きでUSBシリアルへ出力
// Godotはこのシリアルを読み取る。逆方向のCMD行はUSBシリアルから読み、BLE書き込みでMG24へ渡す。
#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLERemoteCharacteristic.h>
#include <BLERemoteService.h>
#include <atomic>
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
BLEClient *client = nullptr;
BLERemoteCharacteristic *dataCharacteristic = nullptr;
BLEAddress peerAddress;
uint8_t peerAddressType = 0;
bool peerFound = false;
bool connected = false;
bool initialCommandSent = false;
bool streamingExpected = true;
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

class CentralClientCallbacks final : public BLEClientCallbacks {
public:
  void onConnect(BLEClient *connectedClient) override {
    (void)connectedClient;
  }

  void onDisconnect(BLEClient *disconnectedClient) override {
    (void)disconnectedClient;
  }
};

CentralClientCallbacks clientCallbacks;

void notifyCallback(BLERemoteCharacteristic *characteristic, uint8_t *data,
                    size_t length, bool isNotify) {
  (void)characteristic;
  (void)isNotify;
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

bool connectToPeripheral() {
  if (!peerFound) {
    return false;
  }

  client = BLEDevice::createClient();
  if (client == nullptr) {
    printf("ERROR,create BLE client failed\n");
    return false;
  }
  client->setClientCallbacks(&clientCallbacks);

  printf("GATT connect request\n");
  if (!client->connect(peerAddress, peerAddressType, 10000)) {
    printf("ERROR,BLE connection failed\n");
    delete client;
    client = nullptr;
    peerFound = false;
    return false;
  }

  connectedAtMs = millis();
  // MTUはBLEDevice::setMTU()で設定済み。接続処理がMTU交換を行う。
  printf("BLE connected; negotiated MTU=%u\n",
         static_cast<unsigned>(client->getMTU()));

  // 接続後にMG24のサービスとデータ特性を見つけ、双方向通信の機能を確認する。
  printf("GATT service discovery starting\n");
  BLERemoteService *service = client->getService(BLEUUID(kServiceUuid));
  printf("GATT service discovery finished: %s\n",
         service == nullptr ? "not found" : "found");
  if (service == nullptr) {
    printf("ERROR,%s\n", client->isConnected()
                              ? "sensor service not found"
                              : "BLE disconnected during service discovery");
    client->disconnect();
    connected = false;
    return false;
  }

  dataCharacteristic =
      service->getCharacteristic(BLEUUID(kDataCharacteristicUuid));
  if (dataCharacteristic == nullptr || !dataCharacteristic->canNotify() ||
      !dataCharacteristic->canWriteNoResponse()) {
    printf("ERROR,sensor characteristic lacks notify/write-no-response\n");
    client->disconnect();
    connected = false;
    return false;
  }

  // 購読完了をMG24が受け取ると、MG24側の10ms周期DATA送信が始まる。
  notificationLineLength = 0;
  discardingNotificationLine = false;
  // registerForNotify()はvoidなので、CCCDの存在と書き込み結果を直接確認する。
  if (dataCharacteristic->getDescriptor(BLEUUID(static_cast<uint16_t>(0x2902))) ==
      nullptr) {
    printf("ERROR,BLE notification descriptor not found\n");
    client->disconnect();
    return false;
  }
#if defined(CONFIG_NIMBLE_ENABLED)
  if (!dataCharacteristic->subscribe(true, notifyCallback, true)) {
    printf("ERROR,BLE notification subscription failed\n");
    client->disconnect();
    return false;
  }
#else
  dataCharacteristic->registerForNotify(notifyCallback, true, true);
#endif
  if (!client->isConnected()) {
    printf("ERROR,BLE disconnected during notification subscription\n");
    return false;
  }
  connected = true;
  streamingExpected = true;
  connectedAtMs = millis();
  lastDataFrameMs.store(connectedAtMs);
  printf("BLE subscribed; bidirectional GATT ready\n");
  return true;
}

void sendCommand(const char *command, size_t length) {
  if (!connected || dataCharacteristic == nullptr || length == 0) {
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
  if (dataCharacteristic->writeValue(
          reinterpret_cast<uint8_t *>(packet), length + 1, false)) {
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
  if (client != nullptr && (!connected || !client->isConnected())) {
    connected = false;
    dataCharacteristic = nullptr;
    delete client;
    client = nullptr;
    peerFound = false;
    initialCommandSent = false;
    printf("BLE disconnected; will rescan\n");
  }

  if (!connected) {
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

  if (streamingExpected && millis() - lastDataFrameMs.load() > DATA_TIMEOUT_MS) {
    printf("ERROR,BLE subscribed but no sensor DATA for %lu ms; reconnecting\n",
           DATA_TIMEOUT_MS);
    client->disconnect();
    connected = false;
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

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

namespace {

constexpr char kDeviceName[] = "KosenBall";
constexpr char kServiceUuid[] = "4880c12c-fdcb-4077-8920-a450d7f9b907";
constexpr char kDataCharacteristicUuid[] =
    "fec26ec4-6d71-4442-9f81-55bc21d658d6";
// LFまで複数の通知を蓄積する。終端NULを除き1行1023 bytesまで受信可能。
constexpr size_t NOTIFICATION_LINE_CAPACITY = 1024;

BLEScan *scanner = nullptr;
BLEClient *client = nullptr;
BLERemoteCharacteristic *dataCharacteristic = nullptr;
BLEAddress peerAddress;
uint8_t peerAddressType = 0;
bool peerFound = false;
bool connected = false;
bool initialCommandSent = false;
unsigned long connectedAtMs = 0;
char serialCommand[48];
size_t serialCommandLength = 0;
char notificationLine[NOTIFICATION_LINE_CAPACITY];
size_t notificationLineLength = 0;
bool discardingNotificationLine = false;

class CentralClientCallbacks final : public BLEClientCallbacks {
public:
  void onConnect(BLEClient *connectedClient) override {
    (void)connectedClient;
    printf("GATT open event received\n");
  }

  void onDisconnect(BLEClient *disconnectedClient) override {
    (void)disconnectedClient;
    printf("GATT disconnect event received\n");
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
        printf("RX,%s\n", notificationLine);
      }
      notificationLineLength = 0;
      discardingNotificationLine = false;
    } else if (!discardingNotificationLine) {
      if (notificationLineLength < sizeof(notificationLine) - 1) {
        notificationLine[notificationLineLength++] = incoming;
      } else {
        notificationLineLength = 0;
        discardingNotificationLine = true;
        printf("ERROR,BLE notification line too long\n");
      }
    }
  }
}

bool scanForPeripheral() {
  // MG24はKosenBallという名前とセンサーサービスUUIDを含むアドバタイズを送信する。
  printf("BLE scanning for %s\n", kDeviceName);
  peerFound = false;
  scanner->clearResults();
  BLEScanResults *results = scanner->start(3, false);
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

  // 購読完了をMG24が受け取ると、MG24側の20ms周期DATA送信が始まる。
  notificationLineLength = 0;
  discardingNotificationLine = false;
  dataCharacteristic->registerForNotify(notifyCallback, true, true);
  if (!client->isConnected()) {
    printf("ERROR,BLE disconnected during notification subscription\n");
    return false;
  }
  connected = true;
  printf("BLE subscribed; bidirectional GATT ready\n");
  return true;
}

void sendCommand(const char *command, size_t length) {
  if (!connected || dataCharacteristic == nullptr || length == 0) {
    printf("ERROR,BLE command requires an active connection\n");
    return;
  }

  char packet[50];
  if (length + 1 > sizeof(packet) || length + 1 > 20) {
    printf("ERROR,command too long for one BLE write\n");
    return;
  }

  memcpy(packet, command, length);
  packet[length] = '\n';
  if (dataCharacteristic->writeValue(
          reinterpret_cast<uint8_t *>(packet), length + 1, false)) {
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

  if (!initialCommandSent && millis() - connectedAtMs >= 250) {
    sendCommand("CMD,RESET", strlen("CMD,RESET"));
    initialCommandSent = true;
  }
  pollSerialCommands();
  delay(1);
}

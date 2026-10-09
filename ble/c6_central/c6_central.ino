// 処理の流れ: BLE広告からMG24を探して接続し、GATT通知を購読する。
// DATA/ACKはprintfでPCへ流し、PCからのCMDはBLE経由でMG24へ転送する。
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
  printf("RX,%.*s", static_cast<int>(length),
         reinterpret_cast<const char *>(data));
  if (length == 0 || data[length - 1] != '\n') {
    printf("\n");
  }
}

bool scanForPeripheral() {
  printf("BLE scanning for %s\n", kDeviceName);
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
    const bool matchingName = device.haveName() &&
                              device.getName().equals(kDeviceName);
    if (!matchingService && !matchingName) {
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

  connected = true;
  connectedAtMs = millis();
  printf("BLE connected; MG24 requests 7.5 ms interval\n");
  if (!client->setMTU(247)) {
    printf("WARNING,BLE MTU 247 request failed\n");
  }

  printf("GATT service discovery starting\n");
  BLERemoteService *service = client->getService(BLEUUID(kServiceUuid));
  printf("GATT service discovery finished: %s\n",
         service == nullptr ? "not found" : "found");
  if (service == nullptr) {
    printf("ERROR,sensor service not found\n");
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

  dataCharacteristic->registerForNotify(notifyCallback, true, true);
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
  if (client != nullptr && !client->isConnected()) {
    connected = false;
    dataCharacteristic = nullptr;
    delete client;
    client = nullptr;
    peerFound = false;
    initialCommandSent = false;
    printf("BLE disconnected; will rescan\n");
  }

  if (!connected) {
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

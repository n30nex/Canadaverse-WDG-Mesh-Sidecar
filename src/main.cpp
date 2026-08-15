#include <Arduino.h>
#include <DNSServer.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <SPI.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_random.h>
#include <mbedtls/aes.h>
#include <mbedtls/base64.h>
#include <mbedtls/md.h>
#include <time.h>

#include <helpers/ArduinoHelpers.h>
#include <helpers/BaseChatMesh.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>

#include "boards/BoardSupport.h"

#if defined(WDG_BOARD_RCC6)
#include "helpers/ui/NV3001BDisplay.h"
#else
#include <helpers/ui/SSD1306Display.h>
#endif

// Set only by the explicit serial screenshot command. Display drivers clear it
// after writing one CRC-protected framebuffer to USB serial.
volatile bool wdg_screenshot_requested = false;

namespace {

constexpr char kWdgUploadUrl[] = "https://wdgwars.pl/api/upload/";
constexpr char kWdgMeUrl[] = "https://wdgwars.pl/api/me";
constexpr char kUsbProvisionPrefix[] = "CFG:WDG1:";
constexpr size_t kWdgApiKeyLength = 64;
constexpr size_t kMeshUploadQueueSize = 32;
constexpr size_t kMeshAttemptedNodeCount = 64;
constexpr uint32_t kMeshUploadMaxAgeMs = 15 * 60 * 1000UL;
constexpr uint32_t kMeshUploadQuietFlushMs = 15 * 1000UL;
constexpr uint32_t kMeshUploadMaxBatchWaitMs = 60 * 1000UL;
constexpr uint32_t kWdgWifiRetryMs = 30 * 1000UL;
constexpr uint32_t kWdgRequestRetryMs = 60 * 1000UL;
constexpr uint32_t kSetupButtonHoldMs = 3 * 1000UL;
constexpr uint32_t kMeshPollCooldownMs = 30 * 60 * 1000UL;
constexpr uint32_t kMeshPollMinTimeoutMs = 10 * 1000UL;
constexpr uint32_t kMeshPostLoginDelayMs = 4 * 1000UL;
constexpr uint8_t kMeshNeighbourRequestCount = 8;
constexpr uint8_t kMeshNeighbourPrefixSize = 8;
constexpr uint8_t kMeshPollMaxAdvertHops = 1;
constexpr size_t kMeshPollRecordCount = 12;
// A 17-second prime cycle avoids phase-locking with periodic beacons. MeshCore
// polls pin the radio until complete, while five seconds is over twice the
// maximum 255-byte Meshtastic LongFast airtime.
constexpr uint32_t kMeshCoreListenMs = 12 * 1000UL;
constexpr uint32_t kMeshtasticListenMs = 5 * 1000UL;
constexpr uint32_t kMaxRadioPacketGuardMs = 2500UL;
static_assert(kMeshPollRecordCount >= MAX_CONTACTS,
              "Every contact slot needs a poll cooldown record");

// Google Trust Services Root R4, used to validate wdgwars.pl TLS. This is a
// public trust anchor, not a credential. It expires in June 2036.
constexpr char kGtsRootR4[] = R"CERT(
-----BEGIN CERTIFICATE-----
MIICCjCCAZGgAwIBAgIQbkepyIuUtui7OyrYorLBmTAKBggqhkjOPQQDAzBHMQsw
CQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZpY2VzIExMQzEU
MBIGA1UEAxMLR1RTIFJvb3QgUjQwHhcNMTYwNjIyMDAwMDAwWhcNMzYwNjIyMDAw
MDAwWjBHMQswCQYDVQQGEwJVUzEiMCAGA1UEChMZR29vZ2xlIFRydXN0IFNlcnZp
Y2VzIExMQzEUMBIGA1UEAxMLR1RTIFJvb3QgUjQwdjAQBgcqhkjOPQIBBgUrgQQA
IgNiAATzdHOnaItgrkO4NcWBMHtLSZ37wWHO5t5GvWvVYRg1rkDdc/eJkTBa6zzu
hXyiQHY7qca4R9gq55KRanPpsXI5nymfopjTX15YhmUPoYRlBtHci8nHc8iMai/l
xKvRHYqjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNVHRMBAf8EBTADAQH/MB0GA1Ud
DgQWBBSATNbrdP9JNqPV2Py1PsVq8JQdjDAKBggqhkjOPQQDAwNnADBkAjBqUFJ0
CMRw3J5QdCHojXohw0+WbhXRIjVhLfoIN+4Zba3bssx9BzT1YBkstTTZbyACMANx
sbqjYAuG7ZoIapVon+Kz4ZNkfF6Tpt95LY2F45TPI11xzPKwTdb+mciUqXWi4w==
-----END CERTIFICATE-----
)CERT";

constexpr float kMeshCoreFrequencyMhz = 910.525f;
constexpr float kMeshCoreBandwidthKhz = 62.5f;
constexpr uint8_t kMeshCoreSpreadingFactor = 7;
constexpr uint8_t kMeshCoreCodingRate = 5;
constexpr uint16_t kMeshCorePreambleSymbols = 32;
constexpr float kTcxoVoltage = 1.8f;

// Meshtastic's public Canada/US LongFast defaults. This listener never
// transmits Meshtastic packets or joins the mesh as a router.
constexpr float kMeshtasticFrequencyMhz = 906.875f;
constexpr float kMeshtasticBandwidthKhz = 250.0f;
constexpr uint8_t kMeshtasticSpreadingFactor = 11;
constexpr uint8_t kMeshtasticCodingRate = 5;
constexpr uint8_t kMeshtasticSyncWord = 0x2B;
constexpr uint16_t kMeshtasticPreambleSymbols = 16;
constexpr uint8_t kMeshtasticPublicChannelHash = 0x08;
constexpr size_t kMeshtasticHeaderSize = 16;
constexpr uint8_t kMeshtasticPositionPort = 3;
constexpr uint8_t kMeshtasticNodeInfoPort = 4;
constexpr uint8_t kMeshtasticTelemetryPort = 67;
constexpr uint8_t kMeshtasticMapReportPort = 73;
constexpr uint8_t kMeshtasticPublicKey[16] = {
    0xD4, 0xF1, 0xBB, 0x3A, 0x20, 0x29, 0x07, 0x59,
    0xF0, 0xBC, 0xFF, 0xAB, 0xCF, 0x4E, 0x69, 0x01,
};

constexpr size_t kMeshCorePublicKeySize = 32;

enum class MeshProtocol : uint8_t { MeshCore, Meshtastic };
enum class RadioListenMode : uint8_t { MeshCore, Meshtastic };

constexpr char kBiscuitServiceUuid[] = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
constexpr char kCommandUuid[] = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
constexpr char kResponseUuid[] = "beb5483e-36e1-4688-b7f5-ea07361b26a9";
constexpr char kStatusUuid[] = "beb5483e-36e1-4688-b7f5-ea07361b26aa";
constexpr char kSettingsUuid[] = "beb5483e-36e1-4688-b7f5-ea07361b26ab";

NimBLECharacteristic *responseCharacteristic = nullptr;
NimBLECharacteristic *statusCharacteristic = nullptr;
NimBLEServer *bleServer = nullptr;

#if defined(WDG_BOARD_RCC6)
NV3001BDisplay display;
#else
SSD1306Display display;
#endif

class SidecarRadioAdapter final : public mesh::Radio {
 public:
  explicit SidecarRadioAdapter(SX1262 &radio) : radio(&radio) { active = this; }

  void begin() override {
    paused = false;
    radio->setPacketReceivedAction(onRadioEvent);
    state = State::Idle;
    eventReady = false;
    ensureReceive();
  }
  int recvRaw(uint8_t *bytes, int capacity) override {
    int length = 0;
    if (state == State::Rx && eventReady) {
      eventReady = false;
      length = min(static_cast<int>(radio->getPacketLength()), capacity);
      lastRssi = radio->getRSSI(true);
      lastSnr = radio->getSNR();
      if (length <= 0 || radio->readData(bytes, length) != RADIOLIB_ERR_NONE) {
        length = 0;
      }
      state = State::Idle;
    }
    ensureReceive();
    return length;
  }
  uint32_t getEstAirtimeFor(int length) override {
    return radio->getTimeOnAir(length) / 1000;
  }
  float packetScore(float snr, int packetLength) override {
    if (snr < -7.5f) {
      return 0.0f;
    }
    const float success = (snr + 7.5f) / 10.0f;
    const float collision = 1.0f - packetLength / 256.0f;
    return constrain(success * collision, 0.0f, 1.0f);
  }
  bool startSendRaw(const uint8_t *bytes, int length) override {
    radio->standby();
    eventReady = false;
    state = State::TxWaiting;
    wdg_board::beforeTransmit();
    if (radio->startTransmit(const_cast<uint8_t *>(bytes), length) ==
        RADIOLIB_ERR_NONE) {
      return true;
    }
    wdg_board::afterTransmit();
    state = State::Idle;
    ensureReceive();
    return false;
  }
  bool isSendComplete() override {
    if (state == State::TxWaiting && eventReady) {
      eventReady = false;
      state = State::Idle;
      return true;
    }
    return false;
  }
  void onSendFinished() override {
    radio->finishTransmit();
    wdg_board::afterTransmit();
    state = State::Idle;
  }
  bool isInRecvMode() const override { return state == State::Rx; }
  float getLastRSSI() const override { return lastRssi; }
  float getLastSNR() const override { return lastSnr; }
  int getNoiseFloor() const override { return -120; }
  bool pause() {
    if (state == State::TxWaiting) {
      return false;
    }
    paused = true;
    eventReady = false;
    state = State::Idle;
    radio->standby();
    radio->clearPacketReceivedAction();
    return true;
  }
  void resume() { begin(); }

 private:
  enum class State : uint8_t { Idle, Rx, TxWaiting };
  static SidecarRadioAdapter *active;
  static void IRAM_ATTR onRadioEvent() {
    if (active != nullptr) {
      active->eventReady = true;
    }
  }
  void ensureReceive() {
    if (paused || state != State::Idle) {
      return;
    }
    wdg_board::beforeReceive();
    if (radio->startReceive() == RADIOLIB_ERR_NONE) {
      state = State::Rx;
    }
  }

  SX1262 *radio;
  volatile bool eventReady = false;
  State state = State::Idle;
  bool paused = false;
  float lastRssi = 0.0f;
  float lastSnr = 0.0f;
};

SidecarRadioAdapter *SidecarRadioAdapter::active = nullptr;

class SidecarMeshClient final : public BaseChatMesh {
 public:
  SidecarMeshClient(mesh::Radio &radio, mesh::MillisecondClock &milliseconds,
                    mesh::RNG &rng, mesh::RTCClock &rtc,
                    mesh::PacketManager &packets, mesh::MeshTables &tables)
      : BaseChatMesh(radio, milliseconds, rng, rtc, packets, tables) {}

  bool beginClient();
  void service(bool pollingEnabled);
  uint16_t neighbourCount() const { return lastNeighbourCount; }
  bool pollInProgress() const { return pollPhase != PollPhase::Idle; }

 protected:
  void onDiscoveredContact(ContactInfo &contact, bool isNew,
                           uint8_t pathLength, const uint8_t *path) override;
  ContactInfo *processAck(const uint8_t *) override { return nullptr; }
  void onContactPathUpdated(const ContactInfo &) override {}
  void onMessageRecv(const ContactInfo &, mesh::Packet *, uint32_t,
                     const char *) override {}
  void onCommandDataRecv(const ContactInfo &, mesh::Packet *, uint32_t,
                         const char *) override {}
  void onSignedMessageRecv(const ContactInfo &, mesh::Packet *, uint32_t,
                           const uint8_t *, const char *) override {}
  uint32_t calcFloodTimeoutMillisFor(uint32_t airtime) const override {
    return 1000 + airtime * 16;
  }
  uint32_t calcDirectTimeoutMillisFor(uint32_t airtime,
                                      uint8_t pathLength) const override {
    return 1000 + airtime * 6 * ((pathLength & 0x3F) + 1);
  }
  void onSendTimeout() override {}
  void onChannelMessageRecv(const mesh::GroupChannel &, mesh::Packet *,
                            uint32_t, const char *) override {}
  bool allowPacketForward(const mesh::Packet *) override { return false; }
  uint8_t onContactRequest(const ContactInfo &, uint32_t, const uint8_t *,
                           uint8_t, uint8_t *) override {
    return 0;
  }
  void onContactResponse(const ContactInfo &contact, const uint8_t *data,
                         uint8_t length) override;
  bool shouldOverwriteWhenFull() const override { return true; }

 private:
  enum class PollPhase : uint8_t {
    Idle,
    LoginQueued,
    LoginWaiting,
    NeighboursQueued,
    NeighboursWaiting,
  };
  struct PollRecord {
    uint8_t prefix[kMeshNeighbourPrefixSize]{};
    uint32_t attemptedAt = 0;
    bool used = false;
  };

  void queueRepeaterPoll(ContactInfo &contact, uint8_t pathLength,
                         const uint8_t *path);
  void servicePoll(bool pollingEnabled);
  void finishPoll(const char *result);
  bool recentlyPolled(const uint8_t *publicKey) const;
  void rememberPoll(const uint8_t *publicKey);

  PollPhase pollPhase = PollPhase::Idle;
  uint8_t pendingRepeater[kMeshCorePublicKeySize]{};
  uint32_t pendingTag = 0;
  uint32_t nextPollActionAt = 0;
  uint32_t pollDeadline = 0;
  uint32_t nextRtcSyncAt = 0;
  PollRecord pollRecords[kMeshPollRecordCount];
  size_t nextPollRecord = 0;
  uint16_t lastNeighbourCount = 0;
  bool pollingAllowed = false;
};

SPIClass loraSpi(0);
SX1262 lora = new Module(wdg_board::kLoRaNss, wdg_board::kLoRaDio1,
                         wdg_board::kLoRaReset, wdg_board::kLoRaBusy, loraSpi);
SidecarRadioAdapter meshRadio(lora);
ArduinoMillis meshMilliseconds;
StdRNG meshRng;
VolatileRTCClock meshRtc;
SimpleMeshTables meshTables;
StaticPoolPacketManager meshPackets(12);
SidecarMeshClient meshClient(meshRadio, meshMilliseconds, meshRng, meshRtc,
                             meshPackets, meshTables);

volatile bool connected = false;
volatile bool responseSubscribed = false;
volatile bool meshCoreReportingEnabled = false;
volatile bool meshStreamRequested = false;
volatile bool advertisingRestartRequested = false;
volatile bool displayDirty = true;
volatile bool setupPortalRequested = false;
volatile bool meshtasticPacketReady = false;
volatile int16_t wdgDisconnectReason = -1;

bool loraReady = false;
bool meshtasticDecoderReady = false;
bool displayReady = false;
uint32_t nextDisplayAt = 0;
uint32_t meshCoreCount = 0;
uint32_t meshtasticCount = 0;
RadioListenMode radioListenMode = RadioListenMode::MeshCore;
uint32_t nextRadioSwitchAt = 0;
uint32_t radioPacketGuardStartedAt = 0;
uint16_t negotiatedMtu = 23;
uint16_t connectedHandle = BLE_HS_CONN_HANDLE_NONE;
float lastMeshRssi = 0.0f;
float lastMeshSnr = 0.0f;
float lastMeshtasticRssi = 0.0f;
float lastMeshtasticSnr = 0.0f;
bool setupButtonDown = false;
bool setupButtonHandled = false;
uint32_t setupButtonDownAt = 0;

WebServer configServer(80);
DNSServer configDns;
bool setupPortalActive = false;
bool setupRoutesRegistered = false;
uint32_t setupRestartAt = 0;
String setupSsid;
String setupPassword;

bool wdgConfigured = false;
bool wdgWifiStarted = false;
bool wdgClockRequested = false;
bool wdgAuthChecked = false;
bool wdgAuthValid = false;
bool wdgAuthError = false;
bool wdgLastUploadFailed = false;
bool wdgUploadRequested = false;
uint32_t nextWdgWifiAttemptAt = 0;
uint32_t nextWdgRequestAt = 0;
uint32_t wdgAcceptedMeshCoreCount = 0;
uint32_t wdgAcceptedMeshtasticCount = 0;
int lastWdgWifiStatus = -1;
String wdgWifiSsid;
String wdgWifiPassword;
String wdgApiKey;

struct MeshUploadNode {
  MeshProtocol protocol = MeshProtocol::MeshCore;
  char nodeId[10]{};
  char nodeType[12]{};
  char name[33]{};
  double latitude = 0.0;
  double longitude = 0.0;
  float rssi = 0.0f;
  uint32_t observedAt = 0;
  uint32_t queuedAt = 0;
};

struct AttemptedMeshNode {
  MeshProtocol protocol = MeshProtocol::MeshCore;
  char nodeId[10]{};
};

struct MeshtasticNodeName {
  uint32_t nodeNumber = 0;
  char name[25]{};
};

MeshUploadNode meshUploadQueue[kMeshUploadQueueSize];
size_t meshUploadCount = 0;
uint32_t lastMeshQueuedAt = 0;
uint32_t firstMeshQueuedAt = 0;
AttemptedMeshNode attemptedMeshNodes[kMeshAttemptedNodeCount];
size_t attemptedMeshNodeCount = 0;
size_t nextAttemptedMeshNode = 0;
MeshtasticNodeName meshtasticNames[16];
size_t nextMeshtasticName = 0;

void startSetupPortal();
void reportVerifiedMeshCoreAdvert(const ContactInfo &contact,
                                   uint8_t pathLength, float rssi, float snr);
void reportMeshtasticPosition(uint32_t nodeNumber, int32_t latitude,
                              int32_t longitude, float rssi, float snr);

void markDisplayDirty() { displayDirty = true; }

void renderDisplay() {
  if (!displayReady) {
    return;
  }

  displayDirty = false;
  const bool isConnected = connected;
  const bool isSubscribed = responseSubscribed;
  const bool isMeshStream = meshCoreReportingEnabled;
  const bool isScreenshot = wdg_screenshot_requested;
  const uint32_t mesh = meshCoreCount;
  const uint32_t meshtastic = meshtasticCount;
  const uint16_t mtu = negotiatedMtu;

#if defined(WDG_BOARD_RCC6)
  constexpr ColorVal kBlack = 0x0000;
  constexpr ColorVal kWhite = 0xFFFF;
  constexpr ColorVal kBlue = 0x001F;
  constexpr ColorVal kCyan = 0x07FF;
  constexpr ColorVal kGreen = 0x07E0;
  constexpr ColorVal kAmber = 0xFD20;
  constexpr ColorVal kRed = 0xF800;

  if (setupPortalActive) {
    display.startFrame(kBlack);
    display.setColor(kBlue);
    display.fillRect(0, 0, 220, 20);
    display.setTextSize(1);
    display.setColor(kWhite);
    display.drawTextCentered(110, 3, "WDGWARS MESH SETUP");
    display.setColor(kCyan);
    display.drawTextCentered(110, 28, "JOIN THIS WIFI");
    display.setColor(kWhite);
    display.drawTextCentered(110, 44, setupSsid.c_str());
    display.setColor(kAmber);
    display.drawTextCentered(110, 63, "PASSWORD");
    display.setColor(kWhite);
    display.drawTextCentered(110, 79,
                             isScreenshot ? "********" : setupPassword.c_str());
    display.setColor(kGreen);
    display.drawTextCentered(110, 101, "OPEN 192.168.4.1");
    display.setColor(kWhite);
    display.drawTextCentered(110, 116, "KEYS NEVER PRINTED OR LOGGED");
    display.endFrame();
    return;
  }

  display.startFrame(kBlack);
  display.setColor(kBlue);
  display.fillRect(0, 0, 220, 20);
  display.setTextSize(1);
  display.setColor(kWhite);
  display.setCursor(6, 3);
  display.print("WDG DUAL RCC6");
  display.setColor(isSubscribed ? kGreen : (isConnected ? kAmber : kWhite));
  display.drawTextRightAlign(214, 3,
      isSubscribed ? "BLE READY" : (isConnected ? "BLE LINK" : "BLE WAIT"));

  const char *mode = "DUAL MESH SIDECAR";
  ColorVal modeColor = kCyan;
  if (isMeshStream) {
    mode = "DUAL MESH + APP";
    modeColor = kGreen;
  } else if (wdgConfigured && loraReady) {
    mode = "DUAL MESH AUTO";
    modeColor = kGreen;
  }
  display.setTextSize(2);
  display.setColor(modeColor);
  display.drawTextCentered(110, 29, mode);

  char line[48];
  display.setTextSize(1);
  display.setColor(isConnected ? kWhite : kAmber);
  if (isConnected) {
    snprintf(line, sizeof(line), "OPTIONAL APP LINK MTU %u", mtu);
  } else {
    snprintf(line, sizeof(line), "BISCUIT PRO = MANAGER");
  }
  display.drawTextCentered(110, 58, line);

  display.setColor(kWhite);
  snprintf(line, sizeof(line), "RX MC %-6lu MT %-6lu",
           static_cast<unsigned long>(mesh),
           static_cast<unsigned long>(meshtastic));
  display.drawTextCentered(110, 77, line);

  display.setColor(loraReady ? kCyan : kRed);
  if (loraReady) {
    if (!meshtasticDecoderReady) {
      snprintf(line, sizeof(line), "MC ONLY - MT DECODER ERROR");
    } else if (radioListenMode == RadioListenMode::Meshtastic) {
      snprintf(line, sizeof(line), "LISTEN MT 906.875 LONGFAST");
    } else {
      snprintf(line, sizeof(line), "LISTEN MC 910.525  NBR %u",
               static_cast<unsigned>(meshClient.neighbourCount()));
    }
  } else {
    snprintf(line, sizeof(line), "MESHCORE RADIO ERROR");
  }
  display.drawTextCentered(110, 96, line);

  if (!wdgConfigured) {
    display.setColor(kAmber);
    snprintf(line, sizeof(line), "WDG SETUP REQUIRED");
  } else if (wdgAuthError) {
    display.setColor(kRed);
    snprintf(line, sizeof(line), "WDG API KEY REJECTED");
  } else if (WiFi.status() != WL_CONNECTED) {
    display.setColor(kAmber);
    snprintf(line, sizeof(line), "WDG OFFLINE - CHECK HOTSPOT");
  } else if (!wdgAuthValid) {
    display.setColor(kCyan);
    snprintf(line, sizeof(line), "WDG CHECKING API KEY");
  } else if (wdgLastUploadFailed) {
    display.setColor(kRed);
    snprintf(line, sizeof(line), "WDG SEND FAILED - NOT RETRIED");
  } else {
    display.setColor(kGreen);
    snprintf(line, sizeof(line), "WDG MC+%lu MT+%lu Q%u",
             static_cast<unsigned long>(wdgAcceptedMeshCoreCount),
             static_cast<unsigned long>(wdgAcceptedMeshtasticCount),
             static_cast<unsigned>(meshUploadCount));
  }
  display.drawTextCentered(110, 113, line);
  display.endFrame();
#else
  constexpr ColorVal kBlack = 0;
  constexpr ColorVal kWhite = 1;
  char line[32];

  display.startFrame(kBlack);
  display.setColor(kWhite);
  display.setTextSize(1);
  if (setupPortalActive) {
    snprintf(line, sizeof(line), "WDG DUAL %s SETUP",
#if defined(WDG_BOARD_HELTEC_V3)
             "V3"
#else
             "V4"
#endif
    );
    display.drawTextCentered(64, 0, line);
    display.drawTextCentered(64, 12, setupSsid.c_str());
    snprintf(line, sizeof(line), "PIN %s",
             isScreenshot ? "********" : setupPassword.c_str());
    display.drawTextCentered(64, 25, line);
    display.drawTextCentered(64, 38, "OPEN 192.168.4.1");
    display.drawTextCentered(64, 51, "KEYS STAY ON DEVICE");
    display.endFrame();
    return;
  }

  snprintf(line, sizeof(line), "WDG DUAL %s",
#if defined(WDG_BOARD_HELTEC_V3)
           "HELTEC V3"
#else
           "HELTEC V4"
#endif
  );
  display.drawTextCentered(64, 0, line);

  snprintf(line, sizeof(line), "RX MC %lu  MT %lu",
           static_cast<unsigned long>(mesh),
           static_cast<unsigned long>(meshtastic));
  display.drawTextCentered(64, 11, line);

  if (loraReady) {
    if (!meshtasticDecoderReady) {
      snprintf(line, sizeof(line), "MC ONLY - MT ERROR");
    } else if (radioListenMode == RadioListenMode::Meshtastic) {
      snprintf(line, sizeof(line), "MT 906.875 LONGFAST");
    } else {
      snprintf(line, sizeof(line), "MC 910.525 NBR %u",
               static_cast<unsigned>(meshClient.neighbourCount()));
    }
  } else {
    snprintf(line, sizeof(line), "MESH RADIO ERROR");
  }
  display.drawTextCentered(64, 22, line);

  if (!wdgConfigured) {
    snprintf(line, sizeof(line), "WDG SETUP REQUIRED");
  } else if (wdgAuthError) {
    snprintf(line, sizeof(line), "WDG KEY REJECTED");
  } else if (WiFi.status() != WL_CONNECTED) {
    snprintf(line, sizeof(line), "WDG OFFLINE");
  } else if (!wdgAuthValid) {
    snprintf(line, sizeof(line), "WDG CHECKING KEY");
  } else if (wdgLastUploadFailed) {
    snprintf(line, sizeof(line), "WDG SEND UNCONFIRMED");
  } else {
    snprintf(line, sizeof(line), "WDG MC+%lu MT+%lu",
             static_cast<unsigned long>(wdgAcceptedMeshCoreCount),
             static_cast<unsigned long>(wdgAcceptedMeshtasticCount));
  }
  display.drawTextCentered(64, 33, line);

  snprintf(line, sizeof(line), "BLE %s  WIFI %s",
           isSubscribed ? "READY" : (isConnected ? "LINK" : "WAIT"),
           WiFi.status() == WL_CONNECTED ? "OK" : "--");
  display.drawTextCentered(64, 44, line);

  snprintf(line, sizeof(line), "PRO MANAGER | DIRECT");
  display.drawTextCentered(64, 55, line);
  display.endFrame();
#endif
}

void serviceDisplay() {
  if (!displayReady || !displayDirty ||
      static_cast<int32_t>(millis() - nextDisplayAt) < 0) {
    return;
  }
  renderDisplay();
  nextDisplayAt = millis() + 250;
}

bool sendLine(const String &line) {
  if (!connected || !responseSubscribed || responseCharacteristic == nullptr) {
    return false;
  }

  String framed = line;
  framed += '\n';
  const size_t mtuPayload = negotiatedMtu > 3 ? negotiatedMtu - 3 : 20;
  const size_t chunkSize = min(mtuPayload, static_cast<size_t>(244));
  size_t offset = 0;
  while (offset < framed.length()) {
    const size_t length = min(chunkSize, framed.length() - offset);
    if (!responseCharacteristic->notify(
            reinterpret_cast<const uint8_t *>(framed.c_str() + offset), length,
            connectedHandle)) {
      Serial.println("WARN: BLE response notification was not accepted");
      return false;
    }
    offset += length;
    if (offset < framed.length()) {
      delay(1);
    }
  }
  Serial.printf("TX %s (%u bytes)\n",
                line.startsWith("DATA:MESHCORE:") ? "DATA:MESHCORE" :
                line.startsWith("STATUS:") ? "STATUS" : "RESPONSE",
                static_cast<unsigned>(framed.length()));
  return true;
}

void setStatus(uint8_t code, const char *text, bool notify = true) {
  if (statusCharacteristic != nullptr) {
    String json = "{\"status\":" + String(code) + ",\"battery\":-1}";
    statusCharacteristic->setValue(json.c_str());
    if (connected) {
      statusCharacteristic->notify();
    }
  }
  if (notify) {
    sendLine("STATUS:" + String(code) + ":" + text);
  }
}

String cleanField(String value) {
  value.replace(",", " ");
  value.replace("\r", " ");
  value.replace("\n", " ");
  return value;
}

String hexField(const uint8_t *bytes, size_t length) {
  static constexpr char hex[] = "0123456789abcdef";
  String value;
  value.reserve(length * 2);
  for (size_t i = 0; i < length; ++i) {
    value += hex[bytes[i] >> 4];
    value += hex[bytes[i] & 0x0F];
  }
  return value;
}

bool isWdgApiKey(const String &value) {
  if (value.length() != kWdgApiKeyLength) {
    return false;
  }
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
          (c >= 'A' && c <= 'F'))) {
      return false;
    }
  }
  return true;
}

String htmlEscape(const String &value) {
  String escaped;
  escaped.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    switch (value[i]) {
      case '&':
        escaped += F("&amp;");
        break;
      case '<':
        escaped += F("&lt;");
        break;
      case '>':
        escaped += F("&gt;");
        break;
      case '"':
        escaped += F("&quot;");
        break;
      case '\'':
        escaped += F("&#39;");
        break;
      default:
        escaped += value[i];
        break;
    }
  }
  return escaped;
}

String jsonEscape(const String &value) {
  String escaped;
  escaped.reserve(value.length() + 8);
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value[i];
    if (c == '"' || c == '\\') {
      escaped += '\\';
      escaped += c;
    } else if (c == '\n') {
      escaped += F("\\n");
    } else if (c == '\r') {
      escaped += F("\\r");
    } else if (c == '\t') {
      escaped += F("\\t");
    } else if (static_cast<uint8_t>(c) >= 0x20) {
      escaped += c;
    }
  }
  return escaped;
}

String generateSetupPassword() {
  static constexpr char digits[] = "0123456789";
  String password;
  password.reserve(9);
  for (size_t i = 0; i < 8; ++i) {
    password += digits[esp_random() % 10];
  }
  return password;
}

bool isSetupPin(const String &value) {
  if (value.length() != 8) {
    return false;
  }
  for (size_t i = 0; i < value.length(); ++i) {
    const char digit = value[i];
    if (digit < '0' || digit > '9') {
      return false;
    }
  }
  return true;
}

void loadWdgConfig() {
  Preferences preferences;
  if (!preferences.begin("wdgwars", true)) {
    wdgConfigured = false;
    return;
  }
  wdgWifiSsid = preferences.getString("wifi_ssid", "");
  wdgWifiPassword = preferences.getString("wifi_pass", "");
  wdgApiKey = preferences.getString("api_key", "");
  setupPassword = preferences.getString("setup_pass", "");
  preferences.end();
  wdgConfigured = !wdgWifiSsid.isEmpty() && isWdgApiKey(wdgApiKey);
}

bool saveWdgConfig(const String &ssid, const String &wifiPassword,
                   const String &apiKey) {
  Preferences preferences;
  if (!preferences.begin("wdgwars", false)) {
    return false;
  }
  const bool ssidSaved = preferences.putString("wifi_ssid", ssid) > 0;
  preferences.putString("wifi_pass", wifiPassword);
  const bool passwordSaved =
      preferences.getString("wifi_pass", "\x01") == wifiPassword;
  const bool keySaved = preferences.putString("api_key", apiKey) > 0;
  preferences.end();
  return ssidSaved && passwordSaved && keySaved;
}

bool decodeProvisionField(const String &encoded, size_t maxLength,
                          String &decoded) {
  decoded = "";
  if (encoded.isEmpty()) {
    return true;
  }
  if (encoded.length() > 4 * ((maxLength + 2) / 3)) {
    return false;
  }

  unsigned char bytes[64] = {};
  size_t decodedLength = 0;
  const int result = mbedtls_base64_decode(
      bytes, sizeof(bytes), &decodedLength,
      reinterpret_cast<const unsigned char *>(encoded.c_str()),
      encoded.length());
  if (result != 0 || decodedLength > maxLength) {
    memset(bytes, 0, sizeof(bytes));
    return false;
  }
  decoded.reserve(decodedLength + 1);
  for (size_t i = 0; i < decodedLength; ++i) {
    if (bytes[i] == '\0') {
      decoded = "";
      memset(bytes, 0, sizeof(bytes));
      return false;
    }
    decoded += static_cast<char>(bytes[i]);
  }
  memset(bytes, 0, sizeof(bytes));
  return true;
}

bool handleUsbProvisioning(const String &command) {
  if (!command.startsWith(kUsbProvisionPrefix)) {
    return false;
  }
  Serial.println("RX command=wdgprovision");

  const size_t prefixLength = strlen(kUsbProvisionPrefix);
  const int firstSeparator = command.indexOf(':', prefixLength);
  const int secondSeparator =
      firstSeparator >= 0 ? command.indexOf(':', firstSeparator + 1) : -1;
  String ssid;
  String wifiPassword;
  String apiKey;
  bool valid = firstSeparator >= 0 && secondSeparator > firstSeparator;
  if (valid) {
    valid = decodeProvisionField(
                command.substring(prefixLength, firstSeparator), 32, ssid) &&
            decodeProvisionField(
                command.substring(firstSeparator + 1, secondSeparator), 63,
                wifiPassword);
    apiKey = command.substring(secondSeparator + 1);
    apiKey.trim();
    apiKey.toLowerCase();
    valid = valid && !ssid.isEmpty() &&
            (wifiPassword.isEmpty() || wifiPassword.length() >= 8) &&
            isWdgApiKey(apiKey);
  }
  if (!valid || !saveWdgConfig(ssid, wifiPassword, apiKey)) {
    ssid = "";
    wifiPassword = "";
    apiKey = "";
    Serial.println("RSP:wdgprovision:ERROR");
    return true;
  }

  ssid = "";
  wifiPassword = "";
  apiKey = "";
  Serial.println("RSP:wdgprovision:OK");
  Serial.flush();
  delay(250);
  ESP.restart();
  return true;
}

String setupPage() {
  String page;
  page.reserve(2800);
  page += F("<!doctype html><html><head><meta name='viewport' "
            "content='width=device-width,initial-scale=1'><meta "
            "http-equiv='Content-Security-Policy' content=\"default-src 'none'; "
            "style-src 'unsafe-inline'; form-action 'self'\"><title>WDG Mesh "
            "Setup</title><style>body{font-family:system-ui;background:#08111f;"
            "color:#eef6ff;max-width:520px;margin:2rem auto;padding:1rem}"
            "h1{color:#42d9ff}label{display:block;margin-top:1rem}input{width:"
            "100%;box-sizing:border-box;padding:.8rem;margin-top:.35rem;"
            "border-radius:.5rem;border:1px solid #466;background:#10243b;"
            "color:white}button{width:100%;margin-top:1.5rem;padding:.9rem;"
            "border:0;border-radius:.5rem;background:#22c55e;font-weight:700}"
            ".note{color:#b8c7d9;font-size:.9rem}</style></head><body>"
            "<h1>WDG Mesh Sidecar</h1><p>Device: <strong>");
  page += htmlEscape(String(wdg_board::kBoardLabel));
  page += F("</strong></p><p>Enter the 2.4 GHz Wi-Fi hotspot the device "
            "should use and your WDGWars API key. WiGLE credentials are "
            "not used here.</p><form method='post' action='/save' "
            "autocomplete='off'><label>Wi-Fi name<input name='ssid' "
            "maxlength='32' required value='");
  page += htmlEscape(wdgWifiSsid);
  page += F("'></label><label>Wi-Fi password<input name='wifi_password' "
            "type='password' maxlength='63' autocomplete='new-password' "
            "placeholder='Leave blank to keep the saved password'></label>"
            "<label>WDGWars API key<input name='api_key' type='password' "
            "minlength='64' maxlength='64' pattern='[A-Fa-f0-9]{64}' "
            "autocomplete='new-password' placeholder='Leave blank to keep "
            "the saved key'></label><button type='submit'>Save and restart"
            "</button></form><p class='note'>The key and Wi-Fi password are "
            "stored in the ESP32's device-local Preferences/NVS area. They "
            "are "
            "never placed in firmware, Git, BLE messages, or logs. Only live "
            "signed MeshCore adverts with valid coordinates are sent; no "
            "WiGLE history is read. Hold the device button for three seconds "
            "at any time to reopen this setup page.</p></body></html>");
  return page;
}

void handleSetupSave() {
  String ssid = configServer.arg("ssid");
  ssid.trim();
  String wifiPassword = configServer.arg("wifi_password");
  String apiKey = configServer.arg("api_key");
  apiKey.trim();
  apiKey.toLowerCase();

  if (wifiPassword.isEmpty() && ssid == wdgWifiSsid && wdgConfigured) {
    wifiPassword = wdgWifiPassword;
  }
  if (apiKey.isEmpty() && wdgConfigured) {
    apiKey = wdgApiKey;
  }
  if (ssid.isEmpty() || ssid.length() > 32 || wifiPassword.length() > 63 ||
      !isWdgApiKey(apiKey)) {
    configServer.sendHeader("Cache-Control", "no-store");
    configServer.send(400, "text/plain",
                      "Invalid entry. The WDGWars API key must be 64 hex "
                      "characters.");
    return;
  }
  if (!saveWdgConfig(ssid, wifiPassword, apiKey)) {
    configServer.sendHeader("Cache-Control", "no-store");
    configServer.send(500, "text/plain", "Could not save settings.");
    return;
  }

  wdgWifiSsid = ssid;
  wdgWifiPassword = wifiPassword;
  wdgApiKey = apiKey;
  wdgConfigured = true;
  configServer.sendHeader("Cache-Control", "no-store");
  configServer.send(
      200, "text/html",
      "<!doctype html><meta name='viewport' content='width=device-width'>"
      "<body style='font-family:system-ui;background:#08111f;color:white;"
      "padding:2rem'><h1 style='color:#22c55e'>Saved</h1><p>The device is "
      "restarting. Reconnect your phone to its normal hotspot, then reopen "
      "Biscuit Manager.</p></body>");
  setupRestartAt = millis() + 1500;
}

void registerSetupRoutes() {
  if (setupRoutesRegistered) {
    return;
  }
  setupRoutesRegistered = true;
  configServer.on("/", HTTP_GET, []() {
    configServer.sendHeader("Cache-Control", "no-store");
    configServer.send(200, "text/html", setupPage());
  });
  configServer.on("/save", HTTP_POST, handleSetupSave);
  configServer.onNotFound([]() {
    configServer.sendHeader("Location", "http://192.168.4.1/", true);
    configServer.send(302, "text/plain", "Open http://192.168.4.1/");
  });
}

void startSetupPortal() {
  if (setupPortalActive) {
    return;
  }
  meshCoreReportingEnabled = false;

  if (!isSetupPin(setupPassword)) {
    setupPassword = generateSetupPassword();
    Preferences preferences;
    if (preferences.begin("wdgwars", false)) {
      preferences.putString("setup_pass", setupPassword);
      preferences.end();
    }
  }
  const uint32_t suffix = static_cast<uint32_t>(ESP.getEfuseMac());
  char suffixText[7];
  snprintf(suffixText, sizeof(suffixText), "%06lX",
           static_cast<unsigned long>(suffix & 0xFFFFFF));
  setupSsid = String(wdg_board::kSetupPrefix) + suffixText;

  WiFi.setAutoReconnect(false);
  WiFi.disconnect(false, false);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(setupSsid.c_str(), setupPassword.c_str(), 1, false, 1)) {
    Serial.println("ERROR: WDGWars setup Wi-Fi failed to start");
    return;
  }
  registerSetupRoutes();
  configDns.start(53, "*", WiFi.softAPIP());
  configServer.begin();
  setupPortalActive = true;
  setupPortalRequested = false;
  display.clear();
  renderDisplay();
#if defined(WDG_BOARD_RCC6)
  Serial.printf("Display setup frame flushed: tiles=%u\n",
                static_cast<unsigned>(display.lastTilesSent()));
#endif
  Serial.println("WDGWars setup portal active at http://192.168.4.1");
  setStatus(1, "WDGWars setup details are on the device display");
}

const char *meshCoreNodeType(uint8_t type) {
  switch (type) {
    case 1:
      return "Client";
    case 2:
      return "Repeater";
    case 3:
      return "Room";
    case 4:
      return "Sensor";
    default:
      return "Unknown";
  }
}

bool wdgClockReady() {
  return time(nullptr) >= 1700000000;
}

String wdgUserAgent() {
  return String("Canadaverse-WDG-Mesh/") + wdg_board::kFirmwareVersion +
         " (" + wdg_board::kBoardLabel + ")";
}

void onWdgWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    wdgDisconnectReason = info.wifi_sta_disconnected.reason;
  }
}

void startWdgWifi() {
  if (!wdgConfigured || setupPortalActive) {
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  const wl_status_t startStatus =
      WiFi.begin(wdgWifiSsid.c_str(), wdgWifiPassword.c_str());
  wdgWifiStarted = true;
  nextWdgWifiAttemptAt = millis() + kWdgWifiRetryMs;
  Serial.printf("WDGWars uplink connecting: status=%d\n",
                static_cast<int>(startStatus));
}

void serviceSetupPortal() {
  if (setupPortalRequested && !setupPortalActive) {
    startSetupPortal();
  }
  if (!setupPortalActive) {
    return;
  }
  configDns.processNextRequest();
  configServer.handleClient();
  if (setupRestartAt != 0 &&
      static_cast<int32_t>(millis() - setupRestartAt) >= 0) {
    ESP.restart();
  }
}

void serviceSetupButton() {
  const bool pressed = digitalRead(wdg_board::kUserButtonPin) == LOW;
  const uint32_t now = millis();
  if (!pressed) {
    setupButtonDown = false;
    setupButtonHandled = false;
    setupButtonDownAt = 0;
    return;
  }
  if (!setupButtonDown) {
    setupButtonDown = true;
    setupButtonDownAt = now;
    return;
  }
  if (!setupButtonHandled && !setupPortalActive &&
      now - setupButtonDownAt >= kSetupButtonHoldMs) {
    setupButtonHandled = true;
    setupPortalRequested = true;
    Serial.println("Physical setup request accepted");
    setStatus(1, "WDGWars setup requested from device button");
    markDisplayDirty();
  }
}

void serviceWdgWifi() {
  if (!wdgConfigured || setupPortalActive) {
    return;
  }
  if (!wdgWifiStarted) {
    startWdgWifi();
  }

  const int16_t disconnectReason = wdgDisconnectReason;
  if (disconnectReason >= 0) {
    wdgDisconnectReason = -1;
    Serial.printf("WDGWars Wi-Fi disconnected: reason=%d\n",
                  disconnectReason);
  }

  const int status = static_cast<int>(WiFi.status());
  if (status != lastWdgWifiStatus) {
    lastWdgWifiStatus = status;
    Serial.printf("WDGWars Wi-Fi status: %d\n", status);
    markDisplayDirty();
  }
  if (status == WL_CONNECTED) {
    if (!wdgClockRequested) {
      configTime(0, 0, "pool.ntp.org", "time.google.com");
      wdgClockRequested = true;
    }
    return;
  }
  if (static_cast<int32_t>(millis() - nextWdgWifiAttemptAt) >= 0) {
    WiFi.disconnect(false, false);
    delay(25);
    const wl_status_t retryStatus =
        WiFi.begin(wdgWifiSsid.c_str(), wdgWifiPassword.c_str());
    Serial.printf("WDGWars clean reconnect: status=%d\n",
                  static_cast<int>(retryStatus));
    nextWdgWifiAttemptAt = millis() + kWdgWifiRetryMs;
  }
}

void copyNodeText(char *destination, size_t capacity, const String &value) {
  if (capacity == 0) {
    return;
  }
  snprintf(destination, capacity, "%s", value.c_str());
}

const char *meshProtocolName(MeshProtocol protocol) {
  return protocol == MeshProtocol::Meshtastic ? "Meshtastic" : "MeshCore";
}

const char *meshProtocolWdgType(MeshProtocol protocol) {
  return protocol == MeshProtocol::Meshtastic ? "MESHTASTIC" : "MESHCORE";
}

bool meshNodeAlreadyAttempted(MeshProtocol protocol, const String &nodeId) {
  for (size_t i = 0; i < attemptedMeshNodeCount; ++i) {
    if (protocol == attemptedMeshNodes[i].protocol &&
        nodeId == attemptedMeshNodes[i].nodeId) {
      return true;
    }
  }
  return false;
}

void rememberAttemptedMeshNode(MeshProtocol protocol, const char *nodeId) {
  for (size_t i = 0; i < attemptedMeshNodeCount; ++i) {
    if (protocol == attemptedMeshNodes[i].protocol &&
        strcmp(nodeId, attemptedMeshNodes[i].nodeId) == 0) {
      return;
    }
  }
  size_t slot = attemptedMeshNodeCount;
  if (attemptedMeshNodeCount < kMeshAttemptedNodeCount) {
    attemptedMeshNodeCount++;
  } else {
    slot = nextAttemptedMeshNode;
    nextAttemptedMeshNode =
        (nextAttemptedMeshNode + 1) % kMeshAttemptedNodeCount;
  }
  attemptedMeshNodes[slot].protocol = protocol;
  snprintf(attemptedMeshNodes[slot].nodeId,
           sizeof(attemptedMeshNodes[slot].nodeId), "%s", nodeId);
}

void rememberAttemptedMeshBatch(MeshProtocol protocol) {
  for (size_t i = 0; i < meshUploadCount; ++i) {
    if (meshUploadQueue[i].protocol == protocol) {
      rememberAttemptedMeshNode(protocol, meshUploadQueue[i].nodeId);
    }
  }
}

void queueMeshUpload(MeshProtocol protocol, const String &nodeId,
                     const char *nodeType,
                     const String &name, double latitude, double longitude,
                     float rssi) {
  if (!wdgConfigured || meshNodeAlreadyAttempted(protocol, nodeId) ||
      latitude < -90.0 || latitude > 90.0 ||
      longitude < -180.0 || longitude > 180.0 ||
      (latitude == 0.0 && longitude == 0.0)) {
    return;
  }

  const bool queueWasEmpty = meshUploadCount == 0;
  size_t slot = meshUploadCount;
  for (size_t i = 0; i < meshUploadCount; ++i) {
    if (protocol == meshUploadQueue[i].protocol &&
        nodeId == meshUploadQueue[i].nodeId) {
      slot = i;
      break;
    }
  }
  if (slot == meshUploadCount) {
    if (meshUploadCount < kMeshUploadQueueSize) {
      meshUploadCount++;
    } else {
      memmove(&meshUploadQueue[0], &meshUploadQueue[1],
              sizeof(MeshUploadNode) * (kMeshUploadQueueSize - 1));
      slot = kMeshUploadQueueSize - 1;
    }
  }

  MeshUploadNode &node = meshUploadQueue[slot];
  node.protocol = protocol;
  copyNodeText(node.nodeId, sizeof(node.nodeId), nodeId);
  copyNodeText(node.nodeType, sizeof(node.nodeType), String(nodeType));
  copyNodeText(node.name, sizeof(node.name), name);
  node.latitude = latitude;
  node.longitude = longitude;
  node.rssi = rssi;
  node.observedAt = wdgClockReady() ? static_cast<uint32_t>(time(nullptr)) : 0;
  node.queuedAt = millis();
  lastMeshQueuedAt = node.queuedAt;
  if (queueWasEmpty) {
    firstMeshQueuedAt = node.queuedAt;
  }
  if (meshUploadCount == kMeshUploadQueueSize) {
    wdgUploadRequested = true;
  }
  markDisplayDirty();
}

void pruneMeshUploadQueue() {
  const uint32_t now = millis();
  size_t writeIndex = 0;
  for (size_t i = 0; i < meshUploadCount; ++i) {
    if (now - meshUploadQueue[i].queuedAt <= kMeshUploadMaxAgeMs) {
      if (writeIndex != i) {
        meshUploadQueue[writeIndex] = meshUploadQueue[i];
      }
      writeIndex++;
    }
  }
  if (writeIndex != meshUploadCount) {
    meshUploadCount = writeIndex;
    if (meshUploadCount == 0) {
      wdgUploadRequested = false;
      lastMeshQueuedAt = 0;
      firstMeshQueuedAt = 0;
    }
    markDisplayDirty();
    Serial.println("Expired unsent live mesh sightings; no backfill retained");
  }
}

String isoUtc(uint32_t epoch) {
  time_t value = epoch != 0 ? static_cast<time_t>(epoch) : time(nullptr);
  struct tm utc {};
  gmtime_r(&value, &utc);
  char text[24];
  strftime(text, sizeof(text), "%Y-%m-%dT%H:%M:%S", &utc);
  return String(text);
}

size_t queuedMeshCount(MeshProtocol protocol) {
  size_t count = 0;
  for (size_t i = 0; i < meshUploadCount; ++i) {
    if (meshUploadQueue[i].protocol == protocol) {
      count++;
    }
  }
  return count;
}

String buildWdgMeshPayload(MeshProtocol protocol) {
  String payload = F("{\"networks\":[],\"aircraft\":[],\"meshcore_nodes\":[");
  payload.reserve(96 + meshUploadCount * 190);
  size_t written = 0;
  for (size_t i = 0; i < meshUploadCount; ++i) {
    const MeshUploadNode &node = meshUploadQueue[i];
    if (node.protocol != protocol) {
      continue;
    }
    if (written++ > 0) {
      payload += ',';
    }
    payload += F("{\"node_id\":\"");
    payload += jsonEscape(String(node.nodeId));
    payload += F("\",\"node_type\":\"");
    payload += jsonEscape(String(node.nodeType));
    payload += F("\",\"name\":\"");
    payload += jsonEscape(String(node.name));
    payload += F("\",\"lat\":");
    payload += String(node.latitude, 6);
    payload += F(",\"lon\":");
    payload += String(node.longitude, 6);
    payload += F(",\"rssi\":");
    payload += String(node.rssi, 1);
    payload += F(",\"first_seen\":\"");
    payload += isoUtc(node.observedAt);
    payload += F("\",\"type\":\"");
    payload += meshProtocolWdgType(node.protocol);
    payload += F("\"}");
  }
  payload += F("]}");
  return payload;
}

void removeAttemptedProtocol(MeshProtocol protocol) {
  size_t writeIndex = 0;
  for (size_t i = 0; i < meshUploadCount; ++i) {
    if (meshUploadQueue[i].protocol != protocol) {
      if (writeIndex != i) {
        meshUploadQueue[writeIndex] = meshUploadQueue[i];
      }
      writeIndex++;
    }
  }
  meshUploadCount = writeIndex;
  if (meshUploadCount == 0) {
    firstMeshQueuedAt = 0;
    lastMeshQueuedAt = 0;
    wdgUploadRequested = false;
    return;
  }
  firstMeshQueuedAt = meshUploadQueue[0].queuedAt;
  lastMeshQueuedAt = meshUploadQueue[0].queuedAt;
  for (size_t i = 1; i < meshUploadCount; ++i) {
    if (meshUploadQueue[i].queuedAt < firstMeshQueuedAt) {
      firstMeshQueuedAt = meshUploadQueue[i].queuedAt;
    }
    if (meshUploadQueue[i].queuedAt > lastMeshQueuedAt) {
      lastMeshQueuedAt = meshUploadQueue[i].queuedAt;
    }
  }
  wdgUploadRequested = true;
}

bool signWdgPayload(const String &payload, String &requestBody) {
  const size_t encodedCapacity = 4 * ((payload.length() + 2) / 3);
  char *encoded = static_cast<char *>(malloc(encodedCapacity + 1));
  if (encoded == nullptr) {
    return false;
  }
  size_t encodedLength = 0;
  const int base64Result = mbedtls_base64_encode(
      reinterpret_cast<unsigned char *>(encoded), encodedCapacity + 1,
      &encodedLength,
      reinterpret_cast<const unsigned char *>(payload.c_str()),
      payload.length());
  if (base64Result != 0) {
    free(encoded);
    return false;
  }
  encoded[encodedLength] = '\0';
  const String dataBase64(encoded);
  free(encoded);

  uint8_t nonceBytes[8];
  esp_fill_random(nonceBytes, sizeof(nonceBytes));
  const String nonce = hexField(nonceBytes, sizeof(nonceBytes));
  const String signedText = nonce + dataBase64;

  const mbedtls_md_info_t *sha256 =
      mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
  uint8_t digest[32];
  if (sha256 == nullptr ||
      mbedtls_md_hmac(
          sha256,
          reinterpret_cast<const unsigned char *>(wdgApiKey.c_str()),
          wdgApiKey.length(),
          reinterpret_cast<const unsigned char *>(signedText.c_str()),
          signedText.length(), digest) != 0) {
    return false;
  }

  requestBody = F("{\"data\":\"");
  requestBody.reserve(dataBase64.length() + 130);
  requestBody += dataBase64;
  requestBody += F("\",\"nonce\":\"");
  requestBody += nonce;
  requestBody += F("\",\"sig\":\"");
  requestBody += hexField(digest, sizeof(digest));
  requestBody += F("\"}");
  return true;
}

int responseInteger(const String &response, const char *key) {
  const String marker = "\"" + String(key) + "\"";
  int index = response.indexOf(marker);
  if (index < 0) {
    return -1;
  }
  index = response.indexOf(':', index + marker.length());
  if (index < 0) {
    return -1;
  }
  index++;
  while (index < static_cast<int>(response.length()) &&
         (response[index] == ' ' || response[index] == '\t')) {
    index++;
  }
  int end = index;
  while (end < static_cast<int>(response.length()) &&
         response[end] >= '0' && response[end] <= '9') {
    end++;
  }
  if (end == index) {
    return -1;
  }
  return response.substring(index, end).toInt();
}

void serviceWdgAuthentication() {
  if (!wdgConfigured || setupPortalActive || wdgAuthChecked ||
      WiFi.status() != WL_CONNECTED || !wdgClockReady() ||
      static_cast<int32_t>(millis() - nextWdgRequestAt) < 0) {
    return;
  }

  WiFiClientSecure client;
  client.setCACert(kGtsRootR4);
  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(15000);
  if (!http.begin(client, kWdgMeUrl)) {
    nextWdgRequestAt = millis() + kWdgRequestRetryMs;
    return;
  }
  http.addHeader("Accept", "application/json");
  http.addHeader("X-API-Key", wdgApiKey);
  http.addHeader("User-Agent", wdgUserAgent());
  const int code = http.GET();
  String response;
  if (code > 0) {
    response = http.getString();
  }
  http.end();

  if (code == 200 &&
      (response.indexOf("\"ok\":true") >= 0 ||
       response.indexOf("\"ok\": true") >= 0)) {
    wdgAuthChecked = true;
    wdgAuthValid = true;
    wdgAuthError = false;
    nextWdgRequestAt = 0;
    Serial.println("WDGWars authentication verified");
  } else if (code == 401 || code == 403) {
    wdgAuthChecked = true;
    wdgAuthValid = false;
    wdgAuthError = true;
    Serial.println("ERROR: WDGWars API key was rejected");
  } else {
    nextWdgRequestAt = millis() + kWdgRequestRetryMs;
    Serial.printf("WDGWars authentication check unavailable (%d)\n", code);
  }
  markDisplayDirty();
}

void serviceWdgMeshUpload() {
  pruneMeshUploadQueue();
  if (!wdgUploadRequested && meshUploadCount > 0 &&
      (millis() - lastMeshQueuedAt >= kMeshUploadQuietFlushMs ||
       millis() - firstMeshQueuedAt >= kMeshUploadMaxBatchWaitMs)) {
    wdgUploadRequested = true;
    Serial.println("Live mesh batch window ended; WDGWars upload requested");
  }
  if (!wdgUploadRequested || meshUploadCount == 0 || !wdgAuthValid ||
      setupPortalActive || meshClient.pollInProgress() ||
      radioListenMode != RadioListenMode::MeshCore ||
      WiFi.status() != WL_CONNECTED || !wdgClockReady() ||
      static_cast<int32_t>(millis() - nextWdgRequestAt) < 0) {
    return;
  }

  const MeshProtocol protocol = meshUploadQueue[0].protocol;
  const size_t sentCount = queuedMeshCount(protocol);
  const String payload = buildWdgMeshPayload(protocol);
  String requestBody;
  if (!signWdgPayload(payload, requestBody)) {
    nextWdgRequestAt = millis() + kWdgRequestRetryMs;
    Serial.printf("ERROR: Could not sign WDGWars %s payload\n",
                  meshProtocolName(protocol));
    return;
  }

  WiFiClientSecure client;
  client.setCACert(kGtsRootR4);
  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(15000);
  if (!http.begin(client, kWdgUploadUrl)) {
    nextWdgRequestAt = millis() + kWdgRequestRetryMs;
    return;  // No request was created, so a bounded retry is safe.
  }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Accept", "application/json");
  http.addHeader("X-API-Key", wdgApiKey);
  http.addHeader("User-Agent", wdgUserAgent());
  const int code = http.POST(requestBody);
  String response;
  if (code > 0) {
    response = http.getString();
  }
  http.end();

  // Once POST was attempted, never replay the batch automatically. A timeout
  // can mean the server accepted it even though the reply was lost.
  rememberAttemptedMeshBatch(protocol);
  removeAttemptedProtocol(protocol);
  if (code >= 200 && code < 300) {
    int imported = responseInteger(
        response, protocol == MeshProtocol::Meshtastic
                      ? "meshtastic_imported"
                      : "meshcore_imported");
    if (imported < 0 && protocol == MeshProtocol::Meshtastic) {
      imported = responseInteger(response, "meshcore_imported");
    }
    if (imported > 0) {
      if (protocol == MeshProtocol::Meshtastic) {
        wdgAcceptedMeshtasticCount += static_cast<uint32_t>(imported);
      } else {
        wdgAcceptedMeshCoreCount += static_cast<uint32_t>(imported);
      }
    }
    wdgLastUploadFailed = false;
    Serial.printf("WDGWars confirmed %s batch: sent=%u imported=%d\n",
                  meshProtocolName(protocol), static_cast<unsigned>(sentCount),
                  imported < 0 ? 0 : imported);
  } else {
    wdgLastUploadFailed = true;
    if (code == 401 || code == 403) {
      wdgAuthChecked = true;
      wdgAuthValid = false;
      wdgAuthError = true;
    }
    Serial.printf(
        "WDGWars %s POST was not confirmed (%d); batch not retried\n",
        meshProtocolName(protocol), code);
  }
  nextWdgRequestAt = millis() + kWdgRequestRetryMs;
  markDisplayDirty();
}

void startMeshCoreSidecar(const char *responseName) {
  const bool beginSession = !meshStreamRequested;
  meshStreamRequested = true;
  meshCoreReportingEnabled = loraReady;
  if (beginSession) {
    meshCoreCount = 0;
    meshtasticCount = 0;
    lastMeshRssi = 0.0f;
    lastMeshSnr = 0.0f;
    lastMeshtasticRssi = 0.0f;
    lastMeshtasticSnr = 0.0f;
  }
  // App session boundaries never reset the unattended live upload queue.
  markDisplayDirty();
  sendLine("RSP:" + String(responseName) + ":OK");
  if (loraReady) {
    setStatus(2, "Dual mesh sidecar active; WiFi handled by Biscuit Pro");
  } else {
    setStatus(2, "Dual mesh sidecar radio unavailable");
  }
}

void stopWardrive() {
  meshStreamRequested = false;
  meshCoreReportingEnabled = false;
  if (meshUploadCount > 0) {
    wdgUploadRequested = true;
  }
  markDisplayDirty();
  sendLine("RSP:stopscan:OK");
  setStatus(1, "Dual mesh sidecar ready");
}

String wdgStateText() {
  if (setupPortalActive || setupPortalRequested) {
    return "SETUP";
  }
  if (!wdgConfigured) {
    return "NOT_CONFIGURED";
  }
  if (wdgAuthError) {
    return "AUTH_ERROR";
  }
  if (WiFi.status() != WL_CONNECTED) {
    return "OFFLINE";
  }
  if (!wdgAuthValid) {
    return "CHECKING";
  }
  if (wdgLastUploadFailed) {
    return "LAST_SEND_UNCONFIRMED";
  }
  return "READY";
}

const char *safeCommandName(const String &command) {
  if (command == "CMD:wardrive:") return "wardrive";
  if (command == "CMD:wardriveall:") return "wardriveall";
  if (command.startsWith("CMD:stopscan:")) return "stopscan";
  if (command == "CMD:wdgsetup:") return "wdgsetup";
  if (command == "CMD:wdgstatus:") return "wdgstatus";
  if (command == "CMD:screenshot:") return "screenshot";
  if (command == "ACK:status:") return "ack-status";
  if (command == "ACK:battery:") return "ack-battery";
  if (command == "ACK:time:" || command.startsWith("TIME:SYNC:")) {
    return "time-sync";
  }
  return "unknown";
}

void handleCommand(String command) {
  command.trim();
  Serial.printf("RX command=%s bytes=%u\n", safeCommandName(command),
                static_cast<unsigned>(command.length()));

  if (command == "CMD:wardrive:") {
    startMeshCoreSidecar("wardrive");
  } else if (command == "CMD:wardriveall:") {
    startMeshCoreSidecar("wardriveall");
  } else if (command.startsWith("CMD:stopscan:")) {
    stopWardrive();
  } else if (command == "CMD:wdgsetup:") {
    setupPortalRequested = true;
    sendLine("RSP:wdgsetup:OK");
    setStatus(1, "WDGWars setup requested");
  } else if (command == "CMD:wdgstatus:") {
    const String state = wdgStateText();
    Serial.printf("WDGWars status: %s setup-clients=%u\n", state.c_str(),
                  WiFi.softAPgetStationNum());
    sendLine("RSP:wdgstatus:" + state);
  } else if (command == "CMD:screenshot:") {
    wdg_screenshot_requested = true;
    renderDisplay();
  } else if (command == "ACK:status:" || command == "ACK:battery:" ||
             command == "ACK:time:" || command.startsWith("TIME:SYNC:")) {
    // Acknowledgements and phone time sync need no reply.
  } else {
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);
    String name = firstColon >= 0 && secondColon > firstColon
                      ? command.substring(firstColon + 1, secondColon)
                      : "unknown";
    sendLine("RSP:" + name + ":ERROR");
  }
}

class CommandCallbacks final : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic *characteristic,
               NimBLEConnInfo &) override {
    const std::string value = characteristic->getValue();
    handleCommand(String(value.c_str()));
  }
};

class ResponseCallbacks final : public NimBLECharacteristicCallbacks {
 public:
  void onSubscribe(NimBLECharacteristic *, NimBLEConnInfo &connInfo,
                   uint16_t subValue) override {
    responseSubscribed = (subValue & 0x01) != 0;
    Serial.printf("BLE response notifications %s (handle=%u, value=%u)\n",
                  responseSubscribed ? "enabled" : "disabled",
                  connInfo.getConnHandle(), subValue);
    markDisplayDirty();
  }
};

class ServerCallbacks final : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer *, NimBLEConnInfo &connInfo) override {
    connected = true;
    responseSubscribed = false;
    connectedHandle = connInfo.getConnHandle();
    negotiatedMtu = connInfo.getMTU();
    Serial.printf("BLE connected (handle=%u, mtu=%u, bonded=%s)\n",
                  connInfo.getConnHandle(), negotiatedMtu,
                  connInfo.isBonded() ? "yes" : "no");
    if (meshStreamRequested) {
      meshCoreReportingEnabled = loraReady;
      Serial.println("BLE dual-mesh status stream will resume after notifications");
    }
    markDisplayDirty();
  }

  void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
    connected = false;
    responseSubscribed = false;
    connectedHandle = BLE_HS_CONN_HANDLE_NONE;
    meshCoreReportingEnabled = false;
    if (meshUploadCount > 0) {
      wdgUploadRequested = true;
    }
    advertisingRestartRequested = true;
    negotiatedMtu = 23;
    Serial.printf("BLE disconnected (reason=%d)\n", reason);
    markDisplayDirty();
  }

  void onMTUChange(uint16_t mtu, NimBLEConnInfo &connInfo) override {
    negotiatedMtu = mtu;
    Serial.printf("BLE MTU negotiated: %u (handle=%u)\n", mtu,
                  connInfo.getConnHandle());
    markDisplayDirty();
  }
};

CommandCallbacks commandCallbacks;
ResponseCallbacks responseCallbacks;
ServerCallbacks serverCallbacks;

NimBLECharacteristic *addReadCharacteristic(NimBLEService *service,
                                             const char *uuid,
                                             const String &value,
                                             bool notify = false) {
  uint32_t properties = NIMBLE_PROPERTY::READ;
  if (notify) {
    properties |= NIMBLE_PROPERTY::NOTIFY;
  }
  NimBLECharacteristic *characteristic =
      service->createCharacteristic(uuid, properties, 512);
  characteristic->setValue(value.c_str());
  return characteristic;
}

void setupBle() {
  Serial.println("BLE: initializing NimBLE");
  NimBLEDevice::init(wdg_board::kDeviceName);
  if (!NimBLEDevice::setMTU(512)) {
    Serial.println("WARN: BLE preferred MTU 512 was rejected");
  }
  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(&serverCallbacks);

  NimBLEService *service = bleServer->createService(kBiscuitServiceUuid);
  NimBLECharacteristic *commandCharacteristic = service->createCharacteristic(
      kCommandUuid, NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR, 512);
  commandCharacteristic->setCallbacks(&commandCallbacks);

  responseCharacteristic = service->createCharacteristic(
      kResponseUuid,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY, 512);
  responseCharacteristic->setValue("STATUS:1:Ready\n");
  responseCharacteristic->setCallbacks(&responseCallbacks);

  statusCharacteristic = addReadCharacteristic(
      service, kStatusUuid, "{\"status\":1,\"battery\":-1}", true);
  const String settings =
      String("{\"status\":\"OK\",\"deviceName\":\"") +
      jsonEscape(String(wdg_board::kDeviceName)) +
      "\",\"ledFeedback\":false,\"meshSidecar\":true,"
      "\"meshtasticSidecar\":true}";
  addReadCharacteristic(service, kSettingsUuid, settings);

  NimBLEService *deviceInfo = bleServer->createService("180A");
  addReadCharacteristic(deviceInfo, "2A26", wdg_board::kFirmwareVersion);
  addReadCharacteristic(deviceInfo, "2A28", wdg_board::kFirmwareVersion, true);

  uint64_t chipId = ESP.getEfuseMac();
  char serial[17];
  snprintf(serial, sizeof(serial), "%04X%08X",
           static_cast<uint16_t>(chipId >> 32),
           static_cast<uint32_t>(chipId));
  addReadCharacteristic(deviceInfo, "2A25", serial);
  addReadCharacteristic(deviceInfo, "2A29", wdg_board::kManufacturer);
  addReadCharacteristic(deviceInfo, "2A24", wdg_board::kModelNumber);
  if (!bleServer->start()) {
    Serial.println("ERROR: BLE GATT server failed to start");
    return;
  }

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(kBiscuitServiceUuid);
  advertising->enableScanResponse(true);
  advertising->setName(wdg_board::kDeviceName);
  if (advertising->start()) {
    Serial.printf("BLE advertising as %s\n", wdg_board::kDeviceName);
  } else {
    Serial.println("ERROR: BLE advertising failed to start");
  }
}

bool SidecarMeshClient::beginClient() {
  meshRng.begin(static_cast<long>(esp_random()));
  uint8_t identity[PRV_KEY_SIZE + PUB_KEY_SIZE];
  bool loaded = false;
  Preferences preferences;
  if (preferences.begin("meshclient", false)) {
    const size_t savedLength = preferences.getBytesLength("identity");
    if ((savedLength == PRV_KEY_SIZE || savedLength == sizeof(identity)) &&
        preferences.getBytes("identity", identity, savedLength) == savedLength) {
      self_id.readFrom(identity, savedLength);
      loaded = self_id.pub_key[0] != 0x00 && self_id.pub_key[0] != 0xFF;
    }
    if (!loaded) {
      for (uint8_t attempt = 0; attempt < 10; ++attempt) {
        mesh::LocalIdentity generated(&meshRng);
        if (generated.pub_key[0] == 0x00 || generated.pub_key[0] == 0xFF) {
          continue;
        }
        const size_t length = generated.writeTo(identity, sizeof(identity));
        self_id.readFrom(identity, length);
        loaded = preferences.putBytes("identity", identity, length) == length;
        break;
      }
    }
    preferences.end();
  }
  if (!loaded) {
    mesh::LocalIdentity generated(&meshRng);
    const size_t length = generated.writeTo(identity, sizeof(identity));
    self_id.readFrom(identity, length);
    Serial.println("WARN: MeshCore client identity is volatile for this boot");
  } else {
    Serial.println("MeshCore client identity loaded from device-local NVS");
  }
  BaseChatMesh::begin();
  return true;
}

bool SidecarMeshClient::recentlyPolled(const uint8_t *publicKey) const {
  const uint32_t now = millis();
  for (const PollRecord &record : pollRecords) {
    if (record.used &&
        memcmp(record.prefix, publicKey, sizeof(record.prefix)) == 0 &&
        now - record.attemptedAt < kMeshPollCooldownMs) {
      return true;
    }
  }
  return false;
}

void SidecarMeshClient::rememberPoll(const uint8_t *publicKey) {
  PollRecord &record = pollRecords[nextPollRecord];
  memcpy(record.prefix, publicKey, sizeof(record.prefix));
  record.attemptedAt = millis();
  record.used = true;
  nextPollRecord = (nextPollRecord + 1) %
                   (sizeof(pollRecords) / sizeof(pollRecords[0]));
}

void SidecarMeshClient::queueRepeaterPoll(ContactInfo &contact,
                                       uint8_t pathLength,
                                       const uint8_t *path) {
  const uint8_t pathHops = pathLength & 0x3F;
  if (!pollingAllowed || pollPhase != PollPhase::Idle ||
      contact.type != ADV_TYPE_REPEATER ||
      pathHops > kMeshPollMaxAdvertHops ||
      !mesh::Packet::isValidPathLen(pathLength) ||
      (pathHops > 0 && path == nullptr) ||
      recentlyPolled(contact.id.pub_key)) {
    return;
  }
  const uint8_t hashSize = (pathLength >> 6) + 1;
  contact.out_path_len = pathLength;
  for (uint8_t index = 0; index < pathHops; ++index) {
    memcpy(&contact.out_path[index * hashSize],
           &path[(pathHops - index - 1) * hashSize], hashSize);
  }
  memcpy(pendingRepeater, contact.id.pub_key, sizeof(pendingRepeater));
  pollPhase = PollPhase::LoginQueued;
  nextPollActionAt = millis() + 300;
  Serial.printf(
      "MeshCore repeater %s detected at %u hop(s); direct guest poll queued\n",
      hexField(pendingRepeater, 4).c_str(), static_cast<unsigned>(pathHops));
}

void SidecarMeshClient::finishPoll(const char *result) {
  Serial.printf("MeshCore repeater %s poll %s\n",
                hexField(pendingRepeater, 4).c_str(), result);
  pollPhase = PollPhase::Idle;
  pendingTag = 0;
  nextPollActionAt = 0;
  pollDeadline = 0;
}

void SidecarMeshClient::servicePoll(bool pollingEnabled) {
  if (!pollingEnabled) {
    pollPhase = PollPhase::Idle;
    return;
  }
  const uint32_t now = millis();
  if ((pollPhase == PollPhase::LoginWaiting ||
       pollPhase == PollPhase::NeighboursWaiting) &&
      static_cast<int32_t>(now - pollDeadline) >= 0) {
    finishPoll("timed out (guest login may be disabled)");
    return;
  }
  if ((pollPhase != PollPhase::LoginQueued &&
       pollPhase != PollPhase::NeighboursQueued) ||
      static_cast<int32_t>(now - nextPollActionAt) < 0) {
    return;
  }

  ContactInfo *contact =
      lookupContactByPubKey(pendingRepeater, sizeof(pendingRepeater));
  if (contact == nullptr) {
    finishPoll("cancelled (contact expired)");
    return;
  }

  uint32_t estimatedTimeout = 0;
  if (pollPhase == PollPhase::LoginQueued) {
    if (contact->out_path_len == OUT_PATH_UNKNOWN ||
        (contact->out_path_len & 0x3F) > kMeshPollMaxAdvertHops) {
      finishPoll("cancelled (advertised direct route expired)");
      return;
    }
    rememberPoll(pendingRepeater);
    const int sendResult = sendLogin(*contact, "", estimatedTimeout);
    if (sendResult != MSG_SEND_SENT_DIRECT) {
      finishPoll("cancelled (guest login was not direct)");
      return;
    }
    pollPhase = PollPhase::LoginWaiting;
    Serial.printf("MeshCore repeater %s blank guest login sent direct\n",
                  hexField(pendingRepeater, 4).c_str());
  } else {
    uint8_t request[11] = {0x06, 0, kMeshNeighbourRequestCount, 0, 0, 0,
                           kMeshNeighbourPrefixSize, 0, 0, 0, 0};
    getRNG()->random(&request[7], 4);
    const int sendResult = sendRequest(*contact, request, sizeof(request),
                                       pendingTag, estimatedTimeout);
    if (sendResult != MSG_SEND_SENT_DIRECT) {
      finishPoll("cancelled (neighbour request was not direct)");
      return;
    }
    pollPhase = PollPhase::NeighboursWaiting;
    Serial.printf("MeshCore repeater %s neighbour request sent direct\n",
                  hexField(pendingRepeater, 4).c_str());
  }
  if (estimatedTimeout < kMeshPollMinTimeoutMs) {
    estimatedTimeout = kMeshPollMinTimeoutMs;
  }
  pollDeadline = now + estimatedTimeout;
}

void SidecarMeshClient::onContactResponse(const ContactInfo &contact,
                                       const uint8_t *data, uint8_t length) {
  if (memcmp(contact.id.pub_key, pendingRepeater,
             sizeof(pendingRepeater)) != 0) {
    return;
  }
  if (pollPhase == PollPhase::LoginWaiting) {
    const bool legacyOk = length >= 6 && memcmp(&data[4], "OK", 2) == 0;
    const bool currentOk = length >= 8 && data[4] == 0;
    if (!legacyOk && !currentOk) {
      finishPoll("guest login rejected");
      return;
    }
    Serial.printf("MeshCore repeater %s accepted blank guest login\n",
                  hexField(pendingRepeater, 4).c_str());
    pollPhase = PollPhase::NeighboursQueued;
    // A flooded login response teaches the repeater our return path after
    // three seconds; wait slightly longer so its neighbour reply can be direct.
    nextPollActionAt = millis() + kMeshPostLoginDelayMs;
    pollDeadline = 0;
    return;
  }
  if (pollPhase != PollPhase::NeighboursWaiting || length < 8) {
    return;
  }
  uint32_t responseTag = 0;
  memcpy(&responseTag, data, sizeof(responseTag));
  if (responseTag != pendingTag) {
    return;
  }

  uint16_t total = 0;
  uint16_t returned = 0;
  memcpy(&total, &data[4], sizeof(total));
  memcpy(&returned, &data[6], sizeof(returned));
  constexpr size_t entrySize = kMeshNeighbourPrefixSize + 4 + 1;
  const size_t available = (length - 8) / entrySize;
  const size_t parsed =
      min(min(static_cast<size_t>(returned), available),
          static_cast<size_t>(kMeshNeighbourRequestCount));
  size_t matchedLocations = 0;
  size_t offset = 8;
  for (size_t index = 0; index < parsed; ++index) {
    const uint8_t *prefix = &data[offset];
    offset += kMeshNeighbourPrefixSize;
    uint32_t secondsAgo = 0;
    memcpy(&secondsAgo, &data[offset], sizeof(secondsAgo));
    offset += sizeof(secondsAgo);
    const int8_t snrQuarterDb = static_cast<int8_t>(data[offset++]);
    ContactInfo *neighbour =
        lookupContactByPubKey(prefix, kMeshNeighbourPrefixSize);
    const bool hasSignedLocation =
        neighbour != nullptr &&
        neighbour->gps_lat >= -90000000 && neighbour->gps_lat <= 90000000 &&
        neighbour->gps_lon >= -180000000 && neighbour->gps_lon <= 180000000 &&
        (neighbour->gps_lat != 0 || neighbour->gps_lon != 0);
    if (hasSignedLocation) {
      matchedLocations++;
    }
    Serial.printf("MeshCore neighbour %s age=%lus snr=%.1f signed-gps=%s\n",
                  hexField(prefix, 4).c_str(),
                  static_cast<unsigned long>(secondsAgo),
                  static_cast<float>(snrQuarterDb) / 4.0f,
                  hasSignedLocation ? "yes" : "no");
  }
  lastNeighbourCount = total;
  markDisplayDirty();
  Serial.printf("MeshCore neighbours: total=%u returned=%u parsed=%u "
                "gps-from-signed-adverts=%u\n",
                static_cast<unsigned>(total), static_cast<unsigned>(returned),
                static_cast<unsigned>(parsed),
                static_cast<unsigned>(matchedLocations));
  Serial.println("Neighbour replies contain IDs/age/SNR only; GPS is never "
                 "invented or copied from the polling repeater");
  finishPoll("complete");
}

void SidecarMeshClient::onDiscoveredContact(ContactInfo &contact, bool,
                                          uint8_t pathLength,
                                          const uint8_t *path) {
  reportVerifiedMeshCoreAdvert(contact, pathLength, meshRadio.getLastRSSI(),
                               meshRadio.getLastSNR());
  queueRepeaterPoll(contact, pathLength, path);
}

void SidecarMeshClient::service(bool pollingEnabled) {
  pollingAllowed = pollingEnabled;
  const uint32_t now = millis();
  if (wdgClockReady() &&
      static_cast<int32_t>(now - nextRtcSyncAt) >= 0) {
    getRTCClock()->setCurrentTime(static_cast<uint32_t>(time(nullptr)));
    nextRtcSyncAt = now + 60 * 1000UL;
  }
  getRTCClock()->tick();
  BaseChatMesh::loop();
  servicePoll(pollingEnabled);
}

void IRAM_ATTR onMeshtasticRadioEvent() { meshtasticPacketReady = true; }

uint32_t readLittleEndian32(const uint8_t *bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8) |
         (static_cast<uint32_t>(bytes[2]) << 16) |
         (static_cast<uint32_t>(bytes[3]) << 24);
}

void writeLittleEndian32(uint8_t *bytes, uint32_t value) {
  bytes[0] = value & 0xFF;
  bytes[1] = (value >> 8) & 0xFF;
  bytes[2] = (value >> 16) & 0xFF;
  bytes[3] = (value >> 24) & 0xFF;
}

bool readProtobufVarint(const uint8_t *bytes, size_t length, size_t &offset,
                        uint64_t &value) {
  value = 0;
  for (uint8_t shift = 0; shift < 64 && offset < length; shift += 7) {
    const uint8_t byte = bytes[offset++];
    value |= static_cast<uint64_t>(byte & 0x7F) << shift;
    if ((byte & 0x80) == 0) {
      return true;
    }
  }
  return false;
}

bool skipProtobufValue(const uint8_t *bytes, size_t length, size_t &offset,
                       uint8_t wireType) {
  uint64_t value = 0;
  switch (wireType) {
    case 0:
      return readProtobufVarint(bytes, length, offset, value);
    case 1:
      if (length - offset < 8) return false;
      offset += 8;
      return true;
    case 2:
      if (!readProtobufVarint(bytes, length, offset, value) ||
          value > length - offset) {
        return false;
      }
      offset += static_cast<size_t>(value);
      return true;
    case 5:
      if (length - offset < 4) return false;
      offset += 4;
      return true;
    default:
      return false;
  }
}

bool parseMeshtasticData(const uint8_t *bytes, size_t length, uint32_t &port,
                         const uint8_t *&payload, size_t &payloadLength) {
  port = 0;
  payload = nullptr;
  payloadLength = 0;
  size_t offset = 0;
  while (offset < length) {
    uint64_t key = 0;
    if (!readProtobufVarint(bytes, length, offset, key) || key == 0) {
      return false;
    }
    const uint32_t field = key >> 3;
    const uint8_t wireType = key & 0x07;
    if (field == 1 && wireType == 0) {
      uint64_t value = 0;
      if (!readProtobufVarint(bytes, length, offset, value) ||
          value > UINT32_MAX) {
        return false;
      }
      port = static_cast<uint32_t>(value);
    } else if (field == 2 && wireType == 2) {
      uint64_t value = 0;
      if (!readProtobufVarint(bytes, length, offset, value) ||
          value > length - offset) {
        return false;
      }
      payload = bytes + offset;
      payloadLength = static_cast<size_t>(value);
      offset += payloadLength;
    } else if (!skipProtobufValue(bytes, length, offset, wireType)) {
      return false;
    }
  }
  return payload != nullptr &&
         (port == kMeshtasticPositionPort || port == kMeshtasticNodeInfoPort ||
          port == kMeshtasticTelemetryPort ||
          port == kMeshtasticMapReportPort);
}

bool parseMeshtasticPosition(const uint8_t *bytes, size_t length,
                             int32_t &latitude, int32_t &longitude) {
  bool hasLatitude = false;
  bool hasLongitude = false;
  size_t offset = 0;
  while (offset < length) {
    uint64_t key = 0;
    if (!readProtobufVarint(bytes, length, offset, key) || key == 0) {
      return false;
    }
    const uint32_t field = key >> 3;
    const uint8_t wireType = key & 0x07;
    if ((field == 1 || field == 2) && wireType == 5) {
      if (length - offset < 4) {
        return false;
      }
      const int32_t value =
          static_cast<int32_t>(readLittleEndian32(bytes + offset));
      offset += 4;
      if (field == 1) {
        latitude = value;
        hasLatitude = true;
      } else {
        longitude = value;
        hasLongitude = true;
      }
    } else if (!skipProtobufValue(bytes, length, offset, wireType)) {
      return false;
    }
  }
  return hasLatitude && hasLongitude;
}

String meshtasticNodeName(uint32_t nodeNumber) {
  for (const MeshtasticNodeName &entry : meshtasticNames) {
    if (entry.nodeNumber == nodeNumber && entry.name[0] != '\0') {
      return String(entry.name);
    }
  }
  char fallback[10];
  snprintf(fallback, sizeof(fallback), "!%08lx",
           static_cast<unsigned long>(nodeNumber));
  return String(fallback);
}

void rememberMeshtasticNodeName(uint32_t nodeNumber, const String &name) {
  if (nodeNumber == 0 || name.isEmpty()) {
    return;
  }
  size_t slot = nextMeshtasticName;
  for (size_t i = 0; i < 16; ++i) {
    if (meshtasticNames[i].nodeNumber == nodeNumber ||
        meshtasticNames[i].nodeNumber == 0) {
      slot = i;
      break;
    }
  }
  meshtasticNames[slot].nodeNumber = nodeNumber;
  copyNodeText(meshtasticNames[slot].name,
               sizeof(meshtasticNames[slot].name), name);
  if (slot == nextMeshtasticName) {
    nextMeshtasticName = (nextMeshtasticName + 1) % 16;
  }
}

bool parseMeshtasticNodeInfo(const uint8_t *bytes, size_t length,
                             String &name) {
  size_t offset = 0;
  while (offset < length) {
    uint64_t key = 0;
    if (!readProtobufVarint(bytes, length, offset, key) || key == 0) {
      return false;
    }
    const uint32_t field = key >> 3;
    const uint8_t wireType = key & 0x07;
    if (field == 2 && wireType == 2) {
      uint64_t value = 0;
      if (!readProtobufVarint(bytes, length, offset, value) ||
          value > length - offset) {
        return false;
      }
      const size_t copyLength = min(static_cast<size_t>(value), size_t{24});
      char text[25]{};
      memcpy(text, bytes + offset, copyLength);
      name = cleanField(String(text));
      offset += static_cast<size_t>(value);
    } else if (!skipProtobufValue(bytes, length, offset, wireType)) {
      return false;
    }
  }
  return !name.isEmpty();
}

bool parseMeshtasticMapReport(const uint8_t *bytes, size_t length,
                              int32_t &latitude, int32_t &longitude,
                              String &name) {
  bool hasLatitude = false;
  bool hasLongitude = false;
  bool locationOptedIn = false;
  String shortName;
  size_t offset = 0;
  while (offset < length) {
    uint64_t key = 0;
    if (!readProtobufVarint(bytes, length, offset, key) || key == 0) {
      return false;
    }
    const uint32_t field = key >> 3;
    const uint8_t wireType = key & 0x07;
    if ((field == 1 || field == 2) && wireType == 2) {
      uint64_t value = 0;
      if (!readProtobufVarint(bytes, length, offset, value) ||
          value > length - offset) {
        return false;
      }
      const size_t copyLength = min(static_cast<size_t>(value), size_t{24});
      char text[25]{};
      memcpy(text, bytes + offset, copyLength);
      if (field == 1) {
        name = cleanField(String(text));
      } else {
        shortName = cleanField(String(text));
      }
      offset += static_cast<size_t>(value);
    } else if ((field == 9 || field == 10) && wireType == 5) {
      if (length - offset < 4) {
        return false;
      }
      const int32_t value =
          static_cast<int32_t>(readLittleEndian32(bytes + offset));
      offset += 4;
      if (field == 9) {
        latitude = value;
        hasLatitude = true;
      } else {
        longitude = value;
        hasLongitude = true;
      }
    } else if (field == 14 && wireType == 0) {
      uint64_t value = 0;
      if (!readProtobufVarint(bytes, length, offset, value)) {
        return false;
      }
      locationOptedIn = value != 0;
    } else if (!skipProtobufValue(bytes, length, offset, wireType)) {
      return false;
    }
  }
  if (name.isEmpty()) {
    name = shortName;
  }
  return hasLatitude && hasLongitude && locationOptedIn;
}

bool decryptMeshtasticPublicPayload(uint8_t *bytes, size_t length,
                                    uint32_t fromNode, uint32_t packetId) {
  uint8_t nonceCounter[16]{};
  uint8_t streamBlock[16]{};
  size_t nonceOffset = 0;
  writeLittleEndian32(nonceCounter, packetId);
  writeLittleEndian32(nonceCounter + 8, fromNode);

  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  const int keyResult =
      mbedtls_aes_setkey_enc(&aes, kMeshtasticPublicKey, 128);
  const int cryptResult =
      keyResult == 0
          ? mbedtls_aes_crypt_ctr(&aes, length, &nonceOffset, nonceCounter,
                                  streamBlock, bytes, bytes)
          : keyResult;
  mbedtls_aes_free(&aes);
  return cryptResult == 0;
}

bool meshtasticDecoderSelfTest() {
  uint8_t encrypted[] = {0xD4, 0x14, 0xE5, 0x21, 0xA3, 0x4B, 0xAB,
                         0xED, 0xED, 0xAB, 0xE8, 0xDC, 0x32, 0xA8};
  if (!decryptMeshtasticPublicPayload(encrypted, sizeof(encrypted),
                                      0x12345678, 0x89ABCDEF)) {
    return false;
  }
  uint32_t port = 0;
  const uint8_t *payload = nullptr;
  size_t payloadLength = 0;
  if (!parseMeshtasticData(encrypted, sizeof(encrypted), port, payload,
                           payloadLength) ||
      port != kMeshtasticPositionPort) {
    return false;
  }
  int32_t latitude = 0;
  int32_t longitude = 0;
  if (!parseMeshtasticPosition(payload, payloadLength, latitude, longitude) ||
      latitude != 436532000 || longitude != -793832000) {
    return false;
  }

  const uint8_t mapData[] = {
      0x08, 0x49, 0x12, 0x17, 0x0A, 0x09, 0x54, 0x65, 0x73,
      0x74, 0x20, 0x4E, 0x6F, 0x64, 0x65, 0x4D, 0x20, 0xF3,
      0x04, 0x1A, 0x55, 0xC0, 0x15, 0xAF, 0xD0, 0x70, 0x01,
  };
  port = 0;
  payload = nullptr;
  payloadLength = 0;
  if (!parseMeshtasticData(mapData, sizeof(mapData), port, payload,
                           payloadLength) ||
      port != kMeshtasticMapReportPort) {
    return false;
  }
  latitude = 0;
  longitude = 0;
  String name;
  return parseMeshtasticMapReport(payload, payloadLength, latitude, longitude,
                                  name) &&
         latitude == 436532000 && longitude == -793832000 &&
         name == "Test Node";
}

void reportMeshtasticPosition(uint32_t nodeNumber, int32_t latitude,
                              int32_t longitude, float rssi, float snr) {
  const double latitudeDegrees = latitude * 1e-7;
  const double longitudeDegrees = longitude * 1e-7;
  if (latitudeDegrees < -90.0 || latitudeDegrees > 90.0 ||
      longitudeDegrees < -180.0 || longitudeDegrees > 180.0 ||
      (latitude == 0 && longitude == 0)) {
    return;
  }
  char nodeIdText[10];
  snprintf(nodeIdText, sizeof(nodeIdText), "!%08lx",
           static_cast<unsigned long>(nodeNumber));
  const String nodeId(nodeIdText);
  const String name = meshtasticNodeName(nodeNumber);

  String message = "DATA:MESHTASTIC:";
  message += nodeId;
  message += ',';
  message += name;
  message += ',';
  message += String(latitudeDegrees, 7);
  message += ',';
  message += String(longitudeDegrees, 7);
  message += ',';
  message += String(rssi, 1);
  message += ',';
  message += String(snr, 1);
  if (meshCoreReportingEnabled && connected) {
    sendLine(message);
  }
  queueMeshUpload(MeshProtocol::Meshtastic, nodeId, "Meshtastic", name,
                  latitudeDegrees, longitudeDegrees, rssi);
  Serial.printf("Meshtastic public position %s received\n", nodeId.c_str());
}

void handleMeshtasticPacket() {
  uint8_t packet[255];
  const size_t length = lora.getPacketLength();
  lastMeshtasticRssi = lora.getRSSI(true);
  lastMeshtasticSnr = lora.getSNR();
  if (length <= kMeshtasticHeaderSize || length > sizeof(packet) ||
      lora.readData(packet, length) != RADIOLIB_ERR_NONE) {
    return;
  }
  if (packet[13] != kMeshtasticPublicChannelHash) {
    return;
  }

  const uint32_t fromNode = readLittleEndian32(packet + 4);
  const uint32_t packetId = readLittleEndian32(packet + 8);
  uint8_t *encrypted = packet + kMeshtasticHeaderSize;
  const size_t encryptedLength = length - kMeshtasticHeaderSize;
  if (fromNode == 0 ||
      !decryptMeshtasticPublicPayload(encrypted, encryptedLength, fromNode,
                                     packetId)) {
    return;
  }

  uint32_t port = 0;
  const uint8_t *payload = nullptr;
  size_t payloadLength = 0;
  if (!parseMeshtasticData(encrypted, encryptedLength, port, payload,
                           payloadLength)) {
    return;
  }

  if (port == kMeshtasticNodeInfoPort) {
    String name;
    if (!parseMeshtasticNodeInfo(payload, payloadLength, name)) {
      return;
    }
    rememberMeshtasticNodeName(fromNode, name);
  } else if (port == kMeshtasticPositionPort) {
    int32_t latitude = 0;
    int32_t longitude = 0;
    if (!parseMeshtasticPosition(payload, payloadLength, latitude,
                                 longitude)) {
      return;
    }
    reportMeshtasticPosition(fromNode, latitude, longitude,
                             lastMeshtasticRssi, lastMeshtasticSnr);
  } else if (port == kMeshtasticMapReportPort) {
    int32_t latitude = 0;
    int32_t longitude = 0;
    String name;
    if (!parseMeshtasticMapReport(payload, payloadLength, latitude, longitude,
                                  name)) {
      return;
    }
    rememberMeshtasticNodeName(fromNode, name);
    reportMeshtasticPosition(fromNode, latitude, longitude,
                             lastMeshtasticRssi, lastMeshtasticSnr);
  }
  meshtasticCount++;
  markDisplayDirty();
}

bool applyLoRaListener(float frequency, float bandwidth, uint8_t spreading,
                       uint8_t codingRate, uint8_t syncWord,
                       uint16_t preamble) {
  int16_t state = lora.standby();
  if (state == RADIOLIB_ERR_NONE) state = lora.setFrequency(frequency);
  if (state == RADIOLIB_ERR_NONE) state = lora.setBandwidth(bandwidth);
  if (state == RADIOLIB_ERR_NONE) state = lora.setSpreadingFactor(spreading);
  if (state == RADIOLIB_ERR_NONE) state = lora.setCodingRate(codingRate);
  if (state == RADIOLIB_ERR_NONE) state = lora.setSyncWord(syncWord);
  if (state == RADIOLIB_ERR_NONE) state = lora.setPreambleLength(preamble);
  if (state == RADIOLIB_ERR_NONE) state = lora.explicitHeader();
  if (state == RADIOLIB_ERR_NONE) state = lora.invertIQ(false);
  if (state == RADIOLIB_ERR_NONE) state = lora.setCRC(1);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: SX1262 listener reconfiguration failed: %d\n", state);
    return false;
  }
  return true;
}

bool restoreMeshCoreListener() {
  if (!applyLoRaListener(kMeshCoreFrequencyMhz, kMeshCoreBandwidthKhz,
                         kMeshCoreSpreadingFactor, kMeshCoreCodingRate,
                         RADIOLIB_SX126X_SYNC_WORD_PRIVATE,
                         kMeshCorePreambleSymbols)) {
    return false;
  }
  meshRadio.resume();
  radioListenMode = RadioListenMode::MeshCore;
  radioPacketGuardStartedAt = 0;
  nextRadioSwitchAt = millis() + kMeshCoreListenMs;
  markDisplayDirty();
  return true;
}

bool startMeshtasticListener() {
  if (!meshtasticDecoderReady || !meshRadio.pause()) {
    return false;
  }
  if (!applyLoRaListener(kMeshtasticFrequencyMhz, kMeshtasticBandwidthKhz,
                         kMeshtasticSpreadingFactor,
                         kMeshtasticCodingRate, kMeshtasticSyncWord,
                         kMeshtasticPreambleSymbols)) {
    restoreMeshCoreListener();
    return false;
  }
  meshtasticPacketReady = false;
  lora.setPacketReceivedAction(onMeshtasticRadioEvent);
  wdg_board::beforeReceive();
  const int16_t state = lora.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    lora.clearPacketReceivedAction();
    Serial.printf("ERROR: Meshtastic listener failed to start: %d\n", state);
    restoreMeshCoreListener();
    return false;
  }
  radioListenMode = RadioListenMode::Meshtastic;
  radioPacketGuardStartedAt = 0;
  nextRadioSwitchAt = millis() + kMeshtasticListenMs;
  markDisplayDirty();
  return true;
}

bool deferRadioSwitchForPacket(uint32_t now) {
  const uint32_t flags = lora.getIrqFlags();
  const bool packetInProgress =
      (flags & (RADIOLIB_SX126X_IRQ_PREAMBLE_DETECTED |
                RADIOLIB_SX126X_IRQ_HEADER_VALID)) != 0 &&
      (flags & RADIOLIB_SX126X_IRQ_RX_DONE) == 0;
  if (!packetInProgress) {
    radioPacketGuardStartedAt = 0;
    return false;
  }
  if (radioPacketGuardStartedAt == 0) {
    radioPacketGuardStartedAt = now;
  }
  return now - radioPacketGuardStartedAt < kMaxRadioPacketGuardMs;
}

void serviceMeshRadios() {
  if (!loraReady) {
    return;
  }
  const uint32_t now = millis();
  if (radioListenMode == RadioListenMode::Meshtastic) {
    if (meshtasticPacketReady) {
      meshtasticPacketReady = false;
      handleMeshtasticPacket();
      wdg_board::beforeReceive();
      lora.startReceive();
    }
    if (static_cast<int32_t>(now - nextRadioSwitchAt) >= 0 &&
        !deferRadioSwitchForPacket(now)) {
      lora.standby();
      lora.clearPacketReceivedAction();
      meshtasticPacketReady = false;
      if (!restoreMeshCoreListener()) {
        loraReady = false;
        markDisplayDirty();
      }
    }
    return;
  }

  meshClient.service(wdgConfigured && wdgAuthValid && !setupPortalActive);
  if (static_cast<int32_t>(now - nextRadioSwitchAt) >= 0 &&
      meshtasticDecoderReady && !meshClient.pollInProgress() &&
      !deferRadioSwitchForPacket(now)) {
    startMeshtasticListener();
  }
}

void reportVerifiedMeshCoreAdvert(const ContactInfo &contact,
                                  uint8_t pathLength, float rssi, float snr) {
  const double latitude = contact.gps_lat / 1000000.0;
  const double longitude = contact.gps_lon / 1000000.0;
  const bool hasLocation =
      contact.gps_lat >= -90000000 && contact.gps_lat <= 90000000 &&
      contact.gps_lon >= -180000000 && contact.gps_lon <= 180000000 &&
      (contact.gps_lat != 0 || contact.gps_lon != 0);
  const String nodeName = cleanField(String(contact.name));
  const String nodeId = hexField(contact.id.pub_key, 4);
  const String publicKeyHex =
      hexField(contact.id.pub_key, kMeshCorePublicKeySize);
  const uint8_t pathHops = pathLength & 0x3F;

  meshCoreCount++;
  lastMeshRssi = rssi;
  lastMeshSnr = snr;
  markDisplayDirty();

  String message = "DATA:MESHCORE:";
  message += nodeId;
  message += ',';
  message += meshCoreNodeType(contact.type);
  message += ',';
  message += nodeName;
  message += ',';
  message += hasLocation ? String(latitude, 6) : String();
  message += ',';
  message += hasLocation ? String(longitude, 6) : String();
  message += ',';
  message += String(rssi, 1);
  message += ',';
  message += String(snr, 1);
  message += ',';
  message += String(contact.last_advert_timestamp);
  message += ',';
  message += publicKeyHex;
  message += ',';
  message += String(pathHops);

  Serial.print("MeshCore advert ");
  Serial.println(message.substring(String("DATA:MESHCORE:").length()));
  if (meshCoreReportingEnabled && connected) {
    sendLine(message);
  }
  if (hasLocation) {
    queueMeshUpload(MeshProtocol::MeshCore, nodeId,
                    meshCoreNodeType(contact.type), nodeName, latitude,
                    longitude, rssi);
  } else {
    Serial.printf("MeshCore node %s has no signed advertised location; "
                  "WDGWars upload skipped\n",
                  nodeId.c_str());
  }
}

bool setupLora() {
  loraSpi.begin(wdg_board::kLoRaSclk, wdg_board::kLoRaMiso,
                wdg_board::kLoRaMosi);
  int16_t state = lora.begin(
      kMeshCoreFrequencyMhz, kMeshCoreBandwidthKhz,
      kMeshCoreSpreadingFactor, kMeshCoreCodingRate,
      RADIOLIB_SX126X_SYNC_WORD_PRIVATE, wdg_board::kRadioTxPowerDbm,
      kMeshCorePreambleSymbols, kTcxoVoltage, false);
  if (state == RADIOLIB_ERR_SPI_CMD_FAILED ||
      state == RADIOLIB_ERR_SPI_CMD_INVALID) {
    Serial.printf("SX1262 init failed (%d); retrying without TCXO control\n",
                  state);
    state = lora.begin(
        kMeshCoreFrequencyMhz, kMeshCoreBandwidthKhz,
        kMeshCoreSpreadingFactor, kMeshCoreCodingRate,
        RADIOLIB_SX126X_SYNC_WORD_PRIVATE, wdg_board::kRadioTxPowerDbm,
        kMeshCorePreambleSymbols, 0.0f, false);
  }
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: SX1262 init failed: %d\n", state);
    return false;
  }
  if ((state = lora.setCRC(1)) != RADIOLIB_ERR_NONE ||
      (state = lora.setCurrentLimit(140)) != RADIOLIB_ERR_NONE ||
      (state = lora.setDio2AsRfSwitch(true)) != RADIOLIB_ERR_NONE ||
      (state = lora.setRxBoostedGainMode(true)) != RADIOLIB_ERR_NONE) {
    Serial.printf("ERROR: SX1262 configuration failed: %d\n", state);
    return false;
  }
  if (!wdg_board::applyRadioConfiguration(lora)) {
    Serial.println("ERROR: board-specific SX1262 configuration failed");
    return false;
  }

  if (!meshClient.beginClient()) {
    Serial.println("ERROR: MeshCore client failed to start");
    return false;
  }

  meshtasticDecoderReady = meshtasticDecoderSelfTest();
  Serial.println(meshtasticDecoderReady
                     ? "Meshtastic public LongFast decoder self-test passed"
                     : "ERROR: Meshtastic decoder self-test failed; MeshCore only");

  radioListenMode = RadioListenMode::MeshCore;
  nextRadioSwitchAt = millis() + kMeshCoreListenMs;

  Serial.printf("MeshCore client ready: %.3f MHz, BW %.1f kHz, SF%u, "
                "CR 4/%u, TX %ddBm\n",
                kMeshCoreFrequencyMhz, kMeshCoreBandwidthKhz,
                kMeshCoreSpreadingFactor, kMeshCoreCodingRate,
                wdg_board::kEffectiveTxPowerDbm);
  Serial.println("LoRa mode: MeshCore dominant + receive-only Meshtastic LongFast");
  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.printf("Canadaverse WDG Mesh Sidecar %s on %s\n",
                wdg_board::kFirmwareVersion, wdg_board::kBoardLabel);
  Serial.println("Mode: live MeshCore + Meshtastic sidecar; Wi-Fi wardrive disabled");

  wdg_board::begin();
  displayReady = display.begin();
#if defined(WDG_BOARD_RCC6)
  Serial.println(displayReady ? "Display ready: NV3001B 220x128"
                              : "ERROR: NV3001B display failed to start");
#else
  Serial.println(displayReady ? "Display ready: SSD1306 128x64"
                              : "ERROR: SSD1306 display failed to start");
#endif

  WiFi.onEvent(onWdgWifiEvent);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  loadWdgConfig();
  loraReady = setupLora();
  markDisplayDirty();
  setupBle();
  if (wdgConfigured) {
    startWdgWifi();
  } else {
    startSetupPortal();
  }
  renderDisplay();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith(kUsbProvisionPrefix)) {
      handleUsbProvisioning(command);
    } else if (command == "CMD:wdgsetup:" || command == "CMD:wdgstatus:" ||
        command == "CMD:screenshot:") {
      handleCommand(command);
    }
  }

  if (advertisingRestartRequested) {
    advertisingRestartRequested = false;
    delay(100);
    if (NimBLEDevice::startAdvertising()) {
      Serial.println("BLE advertising restarted");
    } else {
      Serial.println("ERROR: BLE advertising restart failed");
    }
  }

  serviceSetupButton();
  serviceSetupPortal();
  serviceWdgWifi();
  serviceMeshRadios();
  serviceWdgAuthentication();
  serviceWdgMeshUpload();
  serviceDisplay();
  delay(5);
}

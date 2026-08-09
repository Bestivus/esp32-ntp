#pragma once
// SPDX-License-Identifier: Unlicense

#include <stdint.h>
#include "esp_err.h"

extern "C" {
#include "MQTTPacket.h"
}

class GpsDiscipline;
class NtpServer;
class W5500Eth;

/*
 * Publishes GPS/NTP status to an MQTT broker (with Home Assistant discovery)
 * over the W5500's own hardware TCP socket -- there is no lwIP on this path.
 *
 * loop() is polled from ntp_task's shared housekeeping slot, the same task
 * that owns every other W5500 access, per the single-SPI-owner invariant
 * documented in app_main.cpp. Every state below does at most one
 * non-blocking socket call and returns; nothing here ever waits on the
 * network, so a slow or unreachable broker cannot delay an NTP reply. This
 * is why the higher-level, synchronous MQTTClient.c/mqtt_interface.c from
 * the vendored Paho library are NOT used -- only its wire-format codec
 * (MQTTSerialize_connect/publish, MQTTDeserialize_connack) and its
 * already-non-blocking packet reader (MQTTPacket_readnb) are.
 */
class MqttPublisher {
public:
  MqttPublisher();
  ~MqttPublisher();

  esp_err_t begin(const char* brokerIp, int brokerPort,
                   const char* username, const char* password,
                   const char* clientId, const char* nodeId,
                   int publishIntervalS,
                   GpsDiscipline* gps, NtpServer* ntp, W5500Eth* eth);

  void loop();

  // Wire this to W5500Eth::onChipReset() -- a chip register-loss event
  // closes every hardware socket, including this one.
  static void chipResetTrampoline(void* arg);

  static const uint8_t kSocket = 3;

private:
  enum State {
    DISCONNECTED,
    TCP_CONNECTING,
    SENDING_CONNECT,
    AWAITING_CONNACK,
    PUBLISHING_DISCOVERY,
    CONNECTED,
  };

  struct DiscoveryEntry {
    const char* component;    // "sensor" or "binary_sensor"
    const char* objectId;     // topic-safe id, also used in unique_id
    const char* name;         // HA display name
    const char* deviceClass;  // nullable
    const char* unit;         // nullable
    const char* stateClass;   // nullable
    const char* valueKey;     // key in the state JSON payload
    bool isBinary;
    // Decimal places HA should display, matching the snprintf precision this
    // value is actually published with -- without this, HA guesses from
    // whatever value happened to be in the payload at discovery time, which
    // for sub-microsecond fields can round a real, meaningful value down to
    // a misleading "0.00000". -1 = don't send the hint (integers, binaries).
    int displayPrecision;
  };
  static const DiscoveryEntry kEntities[];
  static const int kEntityCount;

  void resetToDisconnected();
  bool pumpSend();  // true once the staged packet is fully written

  void handleDisconnected(int64_t nowUs);
  void handleTcpConnecting(int64_t nowUs);
  void handleSendingConnect(int64_t nowUs);
  void handleAwaitingConnack(int64_t nowUs);
  void handlePublishingDiscovery(int64_t nowUs);
  void handleConnected(int64_t nowUs);

  void stageConnectPacket();
  bool stageDiscoveryEntry(int index);
  bool stageStatusPublish();
  bool stageAvailabilityPublish(bool online);
  bool stagePingReq();
  int buildDiscoveryPayload(char* buf, size_t cap, const DiscoveryEntry& e);

  GpsDiscipline* gps;
  NtpServer* ntp;
  W5500Eth* eth;

  uint8_t brokerIpBytes[4];
  int brokerPort;
  // Heap-allocated, sized exactly to what's configured (MQTT 3.1.1 allows up
  // to 65535 bytes for each of these -- a fixed-size stack/member buffer
  // would either waste RAM sized for a worst case nobody hits, or silently
  // truncate a real one, which is exactly what caused a broker-side "bad
  // username/password" that took a packet capture to actually diagnose).
  char* username;
  char* password;
  char* clientId;
  char nodeId[32];
  char stateTopic[48];
  char availabilityTopic[56];
  char deviceBlock[192];
  int publishIntervalS;

  State state;
  int64_t stateEnterUs;
  int64_t nextReconnectUs;
  int64_t lastPublishUs;
  int64_t lastPingUs;
  int discoveryIndex;

  uint8_t txBuf[768];
  int txLen;
  int txOff;

  uint8_t rxBuf[64];
  MQTTTransport transport;

  char jsonScratch[600];
};

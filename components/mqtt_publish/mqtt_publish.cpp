// SPDX-License-Identifier: Unlicense

#include "mqtt_publish.h"
#include "gps.h"
#include "ntp_server.h"
#include "w5500_eth.h"
#include "w5k_tcp_wrapper.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

// Persisted boot counter, published by app_main (read-only here). Same
// diagnostic already exported on /metrics -- see ntp_stats.cpp.
extern uint32_t g_bootCount;

static const char* TAG = "MQTT";

// Bounds for the handshake stages only -- CONNECTED never waits on anything.
static const int64_t kStageTimeoutUs   = 3000000;   // 3s
static const int64_t kReconnectBackUs  = 5000000;   // 5s
static const int64_t kKeepaliveUs      = 45000000;  // 45s (< the 60s told to the broker)

const MqttPublisher::DiscoveryEntry MqttPublisher::kEntities[] = {
  // component        objectId            name                    deviceClass     unit     stateClass          valueKey              isBinary  displayPrecision
  {"binary_sensor", "gps_lock",        "GPS Lock",             "connectivity", nullptr, nullptr,            "gps_lock",        true,  -1},
  {"binary_sensor", "eth_link",        "Ethernet Link",        "connectivity", nullptr, nullptr,            "eth_link",        true,  -1},
  {"binary_sensor", "holdover",        "Holdover",             "problem",      nullptr, nullptr,            "holdover",        true,  -1},
  {"sensor",        "stratum",         "Stratum",              nullptr,        nullptr, "measurement",      "stratum",         false, -1},
  // offset/pps_jitter/root_dispersion are published in MICROSECONDS
  // (%.3f -- 1ns resolution) rather than seconds: at this hardware's actual
  // scale (single-digit-ns to low-us), a seconds unit needs 9 decimal places
  // to show anything meaningful, which exceeds HA's own display-precision
  // ceiling of 7 (both the manual picker and this hint). In microseconds,
  // small values just read as ordinary small numbers instead.
  {"sensor",        "offset",          "Clock Offset",         nullptr,        "\xc2\xb5s", "measurement",  "offset_us",        false, 3},
  {"sensor",        "pps_jitter",      "PPS Jitter",           nullptr,        "\xc2\xb5s", "measurement",  "pps_jitter_us",    false, 3},
  {"sensor",        "root_dispersion", "Root Dispersion",      nullptr,        "\xc2\xb5s", "measurement",  "root_dispersion_us", false, 3},
  {"sensor",        "freq_ppm",        "Frequency Error",      nullptr,        "ppm",   "measurement",      "freq_ppm",        false, 6},
  {"sensor",        "sats_used",       "Satellites Used",      nullptr,        nullptr, "measurement",      "sats_used",       false, -1},
  {"sensor",        "uptime",          "Uptime",               nullptr,        "s",     "measurement",      "uptime_s",        false, 1},
  {"sensor",        "boot_count",      "Boot Count",           nullptr,        nullptr, "total_increasing", "boot_count",      false, -1},
  {"sensor",        "free_heap",       "Free Heap",            nullptr,        "B",     "measurement",      "free_heap",       false, -1},
  {"sensor",        "chip_resets",     "W5500 Chip Resets",    nullptr,        nullptr, "total_increasing", "chip_resets",     false, -1},
  {"sensor",        "ntp_requests",    "NTP Requests Served",  nullptr,        nullptr, "total_increasing", "ntp_requests",    false, -1},
};
const int MqttPublisher::kEntityCount = sizeof(kEntities) / sizeof(kEntities[0]);

static bool parseIp4(const char* str, uint8_t* out) {
  unsigned a, b, c, d;
  if (sscanf(str, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
  if (a > 255 || b > 255 || c > 255 || d > 255) return false;
  out[0] = (uint8_t)a; out[1] = (uint8_t)b; out[2] = (uint8_t)c; out[3] = (uint8_t)d;
  return true;
}

// snprintf() into a fixed-size buffer truncates silently -- for auth fields
// that's the kind of thing that turns into a confusing broker-side "bad
// username/password" instead of an obvious firmware-side error, so flag it
// loudly for fields that ARE fixed-size (nodeId: it's a topic-path segment,
// meant to be short).
static void copyChecked(char* dst, size_t dstCap, const char* src, const char* fieldName) {
  if (!src) src = "";
  size_t len = strlen(src);
  if (len >= dstCap) {
    ESP_LOGW(TAG, "%s is %u bytes but only %u fit -- TRUNCATED to '%.*s...'; "
                  "authentication will likely fail",
             fieldName, (unsigned)len, (unsigned)(dstCap - 1), (int)dstCap - 4, src);
  }
  snprintf(dst, dstCap, "%s", src);
}

// username/password/clientId have no such length limit here -- MQTT 3.1.1
// allows up to 65535 bytes for each (2-byte length prefix on the wire), and
// a broker-issued credential could plausibly be a long generated token, so
// these are heap-allocated to fit exactly rather than risking truncation
// against any fixed cap.
static char* dupString(const char* src) {
  if (!src) src = "";
  size_t len = strlen(src);
  char* copy = new char[len + 1];
  memcpy(copy, src, len + 1);
  return copy;
}

// getfn for MQTTPacket_readnb(): must return -1 on error, 0 for "call again",
// or the number of bytes actually read (may be less than requested). Backed
// by the already-non-blocking w5k_tcp_recv() -- one register/data read, never
// a wait.
static int mqttNonBlockingRead(void* /*sck*/, unsigned char* buf, int len) {
  // Try the read FIRST, regardless of connection state: a broker that
  // rejects the connection commonly sends its CONNACK and then immediately
  // closes -- by the time this runs, the socket may already show
  // CLOSE_WAIT even though the CONNACK is still sitting in the RX buffer
  // right behind it. Checking status before reading discarded that data
  // and misreported a real rejection as a bare "connection closed".
  int32_t n = w5k_tcp_recv(MqttPublisher::kSocket, buf, (uint16_t)len);
  if (n > 0) return (int)n;
  uint8_t status = w5k_tcp_status(MqttPublisher::kSocket);
  if (status == W5K_SOCK_ESTABLISHED || status == W5K_SOCK_CLOSE_WAIT) return 0;  // call again
  return -1;
}

MqttPublisher::MqttPublisher()
  : gps(nullptr), ntp(nullptr), eth(nullptr), brokerIpBytes{}, brokerPort(0),
    username(nullptr), password(nullptr), clientId(nullptr),
    nodeId{}, stateTopic{}, availabilityTopic{},
    deviceBlock{}, publishIntervalS(30), state(DISCONNECTED), stateEnterUs(0),
    nextReconnectUs(0), lastPublishUs(0), lastPingUs(0), discoveryIndex(0),
    txBuf{}, txLen(0), txOff(0), rxBuf{}, transport{},
    jsonScratch{} {
}

MqttPublisher::~MqttPublisher() {
  delete[] username;
  delete[] password;
  delete[] clientId;
}

esp_err_t MqttPublisher::begin(const char* brokerIp, int brokerPort_,
                                const char* username_, const char* password_,
                                const char* clientId_, const char* nodeId_,
                                int publishIntervalS_,
                                GpsDiscipline* gps_, NtpServer* ntp_, W5500Eth* eth_) {
  if (!brokerIp || !brokerIp[0] || !parseIp4(brokerIp, brokerIpBytes)) {
    ESP_LOGE(TAG, "Invalid/empty MQTT broker IP; MQTT publishing disabled");
    return ESP_ERR_INVALID_ARG;
  }
  brokerPort = brokerPort_;
  gps = gps_;
  ntp = ntp_;
  eth = eth_;
  publishIntervalS = publishIntervalS_ > 0 ? publishIntervalS_ : 30;

  delete[] username; delete[] password; delete[] clientId;  // in case begin() is ever re-run
  username = dupString(username_ ? username_ : "");
  password = dupString(password_ ? password_ : "");
  clientId = dupString(clientId_ && clientId_[0] ? clientId_ : "esp32-ntp");
  copyChecked(nodeId, sizeof(nodeId), nodeId_ && nodeId_[0] ? nodeId_ : "esp32ntp", "MQTT node ID");
  snprintf(stateTopic, sizeof(stateTopic), "%s/state", nodeId);
  snprintf(availabilityTopic, sizeof(availabilityTopic), "%s/availability", nodeId);
  snprintf(deviceBlock, sizeof(deviceBlock),
    "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"ESP32 NTP Clock\","
    "\"model\":\"esp32-ntp W5500/GPS\",\"manufacturer\":\"esp32-ntp\"}",
    nodeId);

  transport.getfn = mqttNonBlockingRead;
  transport.sck = nullptr;
  transport.state = 0;

  state = DISCONNECTED;
  nextReconnectUs = esp_timer_get_time();

  ESP_LOGI(TAG, "MQTT configured: broker %s:%d, node '%s', publish every %ds",
           brokerIp, brokerPort, nodeId, publishIntervalS);
  return ESP_OK;
}

void MqttPublisher::resetToDisconnected() {
  w5k_tcp_close(kSocket);
  state = DISCONNECTED;
  txLen = 0;
  txOff = 0;
  transport.state = 0;
  discoveryIndex = 0;
  nextReconnectUs = esp_timer_get_time() + kReconnectBackUs;
}

void MqttPublisher::chipResetTrampoline(void* arg) {
  static_cast<MqttPublisher*>(arg)->resetToDisconnected();
}

bool MqttPublisher::pumpSend() {
  if (txOff >= txLen) return true;
  int32_t r = w5k_tcp_send(kSocket, txBuf + txOff, (uint16_t)(txLen - txOff));
  if (r > 0) {
    txOff += r;
  } else if (r < 0) {
    resetToDisconnected();
    return false;
  }
  return txOff >= txLen;
}

void MqttPublisher::loop() {
  if (!eth) return;
  int64_t now = esp_timer_get_time();
  switch (state) {
    case DISCONNECTED:          handleDisconnected(now); break;
    case TCP_CONNECTING:        handleTcpConnecting(now); break;
    case SENDING_CONNECT:       handleSendingConnect(now); break;
    case AWAITING_CONNACK:      handleAwaitingConnack(now); break;
    case PUBLISHING_DISCOVERY:  handlePublishingDiscovery(now); break;
    case CONNECTED:             handleConnected(now); break;
  }
}

void MqttPublisher::handleDisconnected(int64_t nowUs) {
  if (nowUs < nextReconnectUs) return;
  uint32_t ip = 0;
  if (!eth->getIpAddr(ip) || ip == 0) return;

  ESP_LOGI(TAG, "Connecting to broker %u.%u.%u.%u:%d (sn=%d)",
           brokerIpBytes[0], brokerIpBytes[1], brokerIpBytes[2], brokerIpBytes[3],
           brokerPort, kSocket);
  int rc = w5k_tcp_connect(kSocket, brokerIpBytes, (uint16_t)brokerPort);
  if (rc != 0) {
    ESP_LOGW(TAG, "w5k_tcp_connect() failed (rc=%d) -- retrying in %llds",
             rc, (long long)(kReconnectBackUs / 1000000));
    nextReconnectUs = nowUs + kReconnectBackUs;
    return;
  }
  state = TCP_CONNECTING;
  stateEnterUs = nowUs;
}

void MqttPublisher::handleTcpConnecting(int64_t nowUs) {
  uint8_t status = w5k_tcp_status(kSocket);
  if (status == W5K_SOCK_ESTABLISHED) {
    ESP_LOGI(TAG, "TCP connected to broker, sending MQTT CONNECT");
    stageConnectPacket();
    pumpSend();
    state = SENDING_CONNECT;
    stateEnterUs = nowUs;
    return;
  }
  if (status == W5K_SOCK_CLOSED || (nowUs - stateEnterUs) > kStageTimeoutUs) {
    ESP_LOGW(TAG, "TCP connect to broker failed/timed out (sock status=0x%02x) -- "
                  "retrying in %llds", status, (long long)(kReconnectBackUs / 1000000));
    resetToDisconnected();
  }
}

void MqttPublisher::handleSendingConnect(int64_t nowUs) {
  if (pumpSend()) {
    transport.state = 0;
    state = AWAITING_CONNACK;
    stateEnterUs = nowUs;
    return;
  }
  if ((nowUs - stateEnterUs) > kStageTimeoutUs) {
    ESP_LOGW(TAG, "Timed out sending MQTT CONNECT packet (%d/%d bytes written)", txOff, txLen);
    resetToDisconnected();
  }
}

void MqttPublisher::handleAwaitingConnack(int64_t nowUs) {
  if ((nowUs - stateEnterUs) > kStageTimeoutUs) {
    ESP_LOGW(TAG, "Timed out waiting for CONNACK -- broker accepted the TCP connection "
                  "but never (fully) replied");
    resetToDisconnected();
    return;
  }
  int rc = MQTTPacket_readnb(rxBuf, sizeof(rxBuf), &transport);
  if (rc == 0) return;   // nothing complete yet
  if (rc < 0) {
    ESP_LOGW(TAG, "Error reading CONNACK (peer closed the connection?)");
    resetToDisconnected();
    return;
  }
  if (rc != CONNACK) {
    ESP_LOGW(TAG, "Expected CONNACK, got MQTT packet type %d instead", rc);
    resetToDisconnected();
    return;
  }

  unsigned char sessionPresent = 0, connackRc = 0;
  if (MQTTDeserialize_connack(&sessionPresent, &connackRc, rxBuf, sizeof(rxBuf)) != 1 ||
      connackRc != 0) {
    ESP_LOGW(TAG, "MQTT CONNECT rejected (rc=%u; 1=bad protocol version, 2=bad client id, "
                  "3=server unavailable, 4=bad username/password, 5=not authorized)", connackRc);
    resetToDisconnected();
    return;
  }
  ESP_LOGI(TAG, "MQTT connected");
  discoveryIndex = 0;
  state = PUBLISHING_DISCOVERY;
  stateEnterUs = nowUs;
}

void MqttPublisher::handlePublishingDiscovery(int64_t nowUs) {
  if (txOff < txLen) { pumpSend(); return; }

  if (discoveryIndex < kEntityCount) {
    if (!stageDiscoveryEntry(discoveryIndex)) {
      ESP_LOGW(TAG, "Discovery payload for '%s' too large for the buffer -- skipped",
               kEntities[discoveryIndex].objectId);
    } else {
      pumpSend();
    }
    discoveryIndex++;
    return;
  }
  if (discoveryIndex == kEntityCount) {
    if (stageAvailabilityPublish(true)) pumpSend();
    discoveryIndex++;
    return;
  }

  ESP_LOGI(TAG, "MQTT discovery published (%d entities)", kEntityCount);
  state = CONNECTED;
  lastPublishUs = nowUs;
  lastPingUs = nowUs;
}

void MqttPublisher::handleConnected(int64_t nowUs) {
  uint8_t status = w5k_tcp_status(kSocket);
  if (status == W5K_SOCK_CLOSED || status == W5K_SOCK_CLOSE_WAIT) {
    ESP_LOGW(TAG, "MQTT connection dropped, reconnecting");
    resetToDisconnected();
    return;
  }

  if (txOff < txLen) { pumpSend(); return; }

  if (nowUs - lastPublishUs >= (int64_t)publishIntervalS * 1000000) {
    bool ok = stageStatusPublish();
    if (ok) pumpSend();
    ESP_LOGD(TAG, "Published state (%d bytes)%s", txLen, ok ? "" : " -- FAILED to build payload");
    lastPublishUs = nowUs;
    return;
  }
  if (nowUs - lastPingUs >= kKeepaliveUs) {
    if (stagePingReq()) pumpSend();
    lastPingUs = nowUs;
    return;
  }

  // Drain anything pending (PINGRESP etc.) so the RX ring never backs up.
  int rc = MQTTPacket_readnb(rxBuf, sizeof(rxBuf), &transport);
  if (rc < 0) resetToDisconnected();
}

void MqttPublisher::stageConnectPacket() {
  static char kOffline[] = "offline";

  MQTTPacket_connectData opts = MQTTPacket_connectData_initializer;
  opts.MQTTVersion = 4;
  opts.cleansession = 1;
  opts.keepAliveInterval = 60;
  opts.clientID.cstring = clientId;
  if (username[0]) opts.username.cstring = username;
  if (password[0]) opts.password.cstring = password;
  opts.willFlag = 1;
  opts.will.topicName.cstring = availabilityTopic;
  opts.will.message.cstring = kOffline;
  opts.will.qos = 0;
  opts.will.retained = 1;

  txLen = MQTTSerialize_connect(txBuf, sizeof(txBuf), &opts);
  if (txLen <= 0) txLen = 0;
  txOff = 0;
}

int MqttPublisher::buildDiscoveryPayload(char* buf, size_t cap, const DiscoveryEntry& e) {
  int n = snprintf(buf, cap,
    "{\"name\":\"%s\",\"unique_id\":\"%s_%s\",\"state_topic\":\"%s\","
    "\"value_template\":\"{{ value_json.%s }}\",\"availability_topic\":\"%s\"",
    e.name, nodeId, e.objectId, stateTopic, e.valueKey, availabilityTopic);
  if (n < 0 || (size_t)n >= cap) return -1;
  if (e.isBinary) {
    n += snprintf(buf + n, cap - n, ",\"payload_on\":\"1\",\"payload_off\":\"0\"");
  }
  if (e.deviceClass) n += snprintf(buf + n, cap - n, ",\"device_class\":\"%s\"", e.deviceClass);
  if (e.unit)        n += snprintf(buf + n, cap - n, ",\"unit_of_measurement\":\"%s\"", e.unit);
  if (e.stateClass)  n += snprintf(buf + n, cap - n, ",\"state_class\":\"%s\"", e.stateClass);
  if (e.displayPrecision >= 0)
    n += snprintf(buf + n, cap - n, ",\"suggested_display_precision\":%d", e.displayPrecision);
  n += snprintf(buf + n, cap - n, ",%s}", deviceBlock);
  if (n < 0 || (size_t)n >= cap) return -1;
  return n;
}

bool MqttPublisher::stageDiscoveryEntry(int index) {
  const DiscoveryEntry& e = kEntities[index];
  char topic[96];
  snprintf(topic, sizeof(topic), "homeassistant/%s/%s/%s/config", e.component, nodeId, e.objectId);

  int plen = buildDiscoveryPayload(jsonScratch, sizeof(jsonScratch), e);
  if (plen < 0) { txLen = 0; return false; }

  MQTTString topicStr = MQTTString_initializer;
  topicStr.cstring = topic;
  txLen = MQTTSerialize_publish(txBuf, sizeof(txBuf), 0, 0, 1, 0, topicStr,
                                (unsigned char*)jsonScratch, plen);
  if (txLen <= 0) { txLen = 0; return false; }
  txOff = 0;
  return true;
}

bool MqttPublisher::stageStatusPublish() {
  GpsStats gs = {};
  if (gps) gps->getStats(gs);
  bool locked = (gps && gps->isLocked());
  int stratum = locked ? 1 : 16;
  double rootDispersion = gps ? gps->getRootDispersion() : 1.0;
  uint32_t reqCount = ntp ? ntp->getRequestCount() : 0;
  int ethLink = eth->isLinkUp() ? 1 : 0;
  uint32_t chipResets = eth->getChipResetCount();
  double uptimeSec = (double)esp_timer_get_time() / 1e6;
  uint32_t freeHeap = (uint32_t)esp_get_free_heap_size();

  // offset/jitter/dispersion in microseconds rather than seconds: at this
  // hardware's actual scale (single-digit-ns to low-us), a seconds unit
  // needs 9 decimal places to show anything meaningful, which exceeds HA's
  // own display-precision ceiling (7, both the manual picker and the
  // suggested_display_precision hint). In microseconds, 3 decimals gives 1ns
  // resolution and reads as an ordinary small number instead.
  double offsetUs = gs.lastOffsetSec * 1e6;
  double jitterUs = gs.ppsJitterSec * 1e6;
  double dispersionUs = rootDispersion * 1e6;

  int plen = snprintf(jsonScratch, sizeof(jsonScratch),
    "{\"gps_lock\":%d,\"holdover\":%d,\"stratum\":%d,\"offset_us\":%.3f,"
    "\"pps_jitter_us\":%.3f,\"root_dispersion_us\":%.3f,\"freq_ppm\":%.6f,"
    "\"sats_used\":%u,\"uptime_s\":%.1f,\"boot_count\":%" PRIu32 ","
    "\"free_heap\":%" PRIu32 ",\"chip_resets\":%" PRIu32 ",\"ntp_requests\":%" PRIu32 ","
    "\"eth_link\":%d}",
    locked ? 1 : 0, gs.holdover ? 1 : 0, stratum, offsetUs,
    jitterUs, dispersionUs, gs.frequencyPpm,
    (unsigned)gs.satsUsed, uptimeSec, g_bootCount,
    freeHeap, chipResets, reqCount, ethLink);
  if (plen <= 0 || (size_t)plen >= sizeof(jsonScratch)) return false;

  MQTTString topicStr = MQTTString_initializer;
  topicStr.cstring = stateTopic;
  txLen = MQTTSerialize_publish(txBuf, sizeof(txBuf), 0, 0, 1, 0, topicStr,
                                (unsigned char*)jsonScratch, plen);
  if (txLen <= 0) { txLen = 0; return false; }
  txOff = 0;
  return true;
}

bool MqttPublisher::stageAvailabilityPublish(bool online) {
  char payload[8];
  snprintf(payload, sizeof(payload), "%s", online ? "online" : "offline");

  MQTTString topicStr = MQTTString_initializer;
  topicStr.cstring = availabilityTopic;
  txLen = MQTTSerialize_publish(txBuf, sizeof(txBuf), 0, 0, 1, 0, topicStr,
                                (unsigned char*)payload, (int)strlen(payload));
  if (txLen <= 0) { txLen = 0; return false; }
  txOff = 0;
  return true;
}

bool MqttPublisher::stagePingReq() {
  txLen = MQTTSerialize_pingreq(txBuf, sizeof(txBuf));
  if (txLen <= 0) { txLen = 0; return false; }
  txOff = 0;
  return true;
}

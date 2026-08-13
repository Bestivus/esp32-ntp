// SPDX-License-Identifier: Unlicense

#include "ntp_stats.h"
#include "config.h"
#include "gps.h"
#include "ntp_server.h"
#include "ntp_prof.h"
#include "w5500_eth.h"
#include "wifi_sta.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <stdlib.h>
#include <fcntl.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "w5k_tcp_wrapper.h"
#include "w5500_drv.h"
#include "w5500_dhcp.h"
#include "config_store.h"

// Liveness/boot diagnostics published by app_main (read-only here).
extern volatile uint32_t g_mainLoopBeats;
extern uint32_t g_bootCount;

static const char* TAG = "NTP_STATS";

static const uint8_t STATS_SOCKET = 2;

// Request and response staging. File scope rather than stack: this all runs in
// the NTP task, whose stack has no room for kilobyte buffers.
static char g_req[3072];
static char g_resp[26624];
static const int hdrReserve = 128;

// strcasestr is a GNU extension; spell it out so the build does not depend on
// which libc variant is configured.
static const char* ci_find(const char* hay, const char* needle) {
  size_t n = strlen(needle);
  for (const char* p = hay; *p; ++p)
    if (strncasecmp(p, needle, n) == 0) return p;
  return nullptr;
}

NtpStats::NtpStats() : sock(-1), port(0), gps(nullptr), ntp(nullptr),
                       eth(nullptr), wifi(nullptr), useWifi(false),
                       listening(false), disconnecting(false), startupLogged(false),
                       listen_sock(-1), client_sock(-1),
                       reqLen(0), hdrEnd(-1), contentLen(0), reqStartUs(0) {}

esp_err_t NtpStats::begin(int port_, GpsDiscipline* gps_, NtpServer* ntp_, W5500Eth* eth_, WifiSta* wifi_) {
  port = port_;
  gps = gps_;
  ntp = ntp_;
  eth = eth_;
  wifi = wifi_;
  useWifi = (wifi_ != nullptr);
  if (useWifi) {
    sock = -1;
    ESP_LOGI(TAG, "Stats HTTP server configured for port %d (WiFi), waiting for IP", port);
  } else {
    sock = STATS_SOCKET;
    ESP_LOGI(TAG, "Stats HTTP server configured for port %d (sn=%d), waiting for IP", port, sock);
  }
  return ESP_OK;
}

bool NtpStats::tryStartListener() {
  uint32_t ipVal = 0;
  if (useWifi) {
    if (!wifi || !wifi->getIpAddr(ipVal) || ipVal == 0) return false;
    listen_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sock < 0) return false;
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(listen_sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
      close(listen_sock);
      listen_sock = -1;
      return false;
    }
    if (listen(listen_sock, 1) != 0) {
      close(listen_sock);
      listen_sock = -1;
      return false;
    }
    int flags = fcntl(listen_sock, F_GETFL, 0);
    if (flags >= 0) fcntl(listen_sock, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGI(TAG, "Stats HTTP server listening on %lu.%lu.%lu.%lu:%d (WiFi)",
             (ipVal >> 24) & 0xff, (ipVal >> 16) & 0xff, (ipVal >> 8) & 0xff, ipVal & 0xff, port);
    startupLogged = true;
    listening = true;
    return true;
  }
  if (!eth) return false;
  bool gotIp = eth->getIpAddr(ipVal);
  if (!gotIp || ipVal == 0) {
    static int suppressCount = 0;
    if (suppressCount++ % 100 == 0)
      ESP_LOGW(TAG, "tryStartListener: no IP yet (gotIp=%d ipVal=0x%08" PRIx32 ")", gotIp, ipVal);
    return false;
  }
  uint8_t ip[4] = {
    (uint8_t)(ipVal >> 24), (uint8_t)(ipVal >> 16),
    (uint8_t)(ipVal >> 8),  (uint8_t)(ipVal)
  };
  int rc = w5k_tcp_listen((uint8_t)sock, (uint16_t)port);
  if (rc != 0) {
    ESP_LOGW(TAG, "TCP listen failed on s=%d port=%d rc=%d (will retry)", sock, port, rc);
    return false;
  }
  ESP_LOGI(TAG, "Stats HTTP server listening on %d.%d.%d.%d:%d (sn=%d)",
           ip[0], ip[1], ip[2], ip[3], port, sock);
  startupLogged = true;
  listening = true;
  resetRequest();
  return true;
}

void NtpStats::loop() {
  if (useWifi) {
    if (!listening && listen_sock < 0) {
      tryStartListener();
      return;
    }
    if (listen_sock < 0) return;
    if (client_sock < 0) {
      struct sockaddr_in from = {};
      socklen_t fromlen = sizeof(from);
      client_sock = accept(listen_sock, (struct sockaddr*)&from, &fromlen);
      if (client_sock >= 0) resetRequest();
      return;
    }
    handleConnection();
    return;
  }

  if (sock < 0) return;
  if (!listening && !disconnecting) {
    tryStartListener();
    return;
  }

  uint8_t status = w5k_tcp_status((uint8_t)sock);
  static uint8_t lastStatus = 255;
  if (status != lastStatus) {
    ESP_LOGI(TAG, "Socket status changed: %d -> %d", lastStatus, status);
    lastStatus = status;
  }

  if (disconnecting) {
    if (status == W5K_SOCK_CLOSED) {
      disconnecting = false;
      listening = false;
      tryStartListener();
    } else if (status == W5K_SOCK_ESTABLISHED || status == W5K_SOCK_CLOSE_WAIT) {
      w5k_tcp_disconnect((uint8_t)sock);
    }
    return;
  }

  switch (status) {
    case W5K_SOCK_ESTABLISHED:
      handleConnection();
      break;
    case W5K_SOCK_CLOSE_WAIT:
      w5k_tcp_disconnect((uint8_t)sock);
      disconnecting = true;
      break;
    case W5K_SOCK_CLOSED:
      ESP_LOGW(TAG, "Socket %d unexpectedly closed, re-listening", sock);
      listening = false;
      disconnecting = false;
      tryStartListener();
      break;
    case W5K_SOCK_LISTEN:
      break;
    default:
      break;
  }
}


void NtpStats::resetRequest() {
  reqLen = 0;
  hdrEnd = -1;
  contentLen = 0;
  reqStartUs = 0;
}

void NtpStats::closeConn() {
  if (useWifi) {
    if (client_sock >= 0) close(client_sock);
    client_sock = -1;
  } else {
    w5k_tcp_disconnect((uint8_t)sock);
    disconnecting = true;
  }
  resetRequest();
}

void NtpStats::sendAll(const char* data, int len) {
  if (useWifi) {
    int off = 0;
    const int64_t deadline = esp_timer_get_time() + 2000000;
    while (off < len && esp_timer_get_time() < deadline) {
      int r = send(client_sock, data + off, len - off, 0);
      if (r > 0) off += r;
      else if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) vTaskDelay(1);
      else break;
    }
    return;
  }
  // The W5500 socket TX buffer is 2KB; send in <=1KB chunks so a response
  // larger than the buffer chains cleanly through the driver's
  // one-chunk-in-flight send.
  /*
   * w5k_tcp_send() returns 0 (busy) — not an error — whenever the PREVIOUS
   * chunk has not reached SENDOK yet or the TX buffer lacks room. Treating
   * that as fatal silently truncated every response past the first chunk or
   * two, which is why the body length varied with load while Content-Length
   * stayed correct. Retry on 0, bail only on a negative return, and bound
   * the whole thing in time so a dead peer cannot wedge the housekeeping
   * slot.
   */
  int off = 0;
  const int64_t deadline = esp_timer_get_time() + 2000000;   /* 2 s */
  while (off < len && esp_timer_get_time() < deadline) {
    int chunk = len - off;
    if (chunk > 1024) chunk = 1024;
    int32_t r = w5k_tcp_send((uint8_t)sock, (const uint8_t*)data + off, (uint16_t)chunk);
    if (r > 0) off += r;
    else if (r < 0) break;     // socket closed/errored
    else vTaskDelay(1);        // SOCK_BUSY (waiting for prior chunk's SENDOK) --
                                // yield instead of spinning, so a slow SENDOK
                                // can no longer starve other tasks/the idle task
                                // for the whole 2s deadline
  }
}

void NtpStats::sendStatus(const char* status, const char* ctype, const char* body) {
  char hdr[192];
  int blen = (int)strlen(body);
  int hlen = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 %s\r\n"
    "Content-Type: %s\r\n"
    "Connection: close\r\n"
    "%s"
    "Content-Length: %d\r\n"
    "\r\n",
    status, ctype,
    strncmp(status, "401", 3) == 0
      ? "WWW-Authenticate: Basic realm=\"esp32-ntp\"\r\n" : "",
    blen);
  sendAll(hdr, hlen);
  sendAll(body, blen);
  closeConn();
}

/*
 * Accumulate whatever has arrived and report whether a whole request is in
 * hand. Returning false means "not yet" — the caller returns to the NTP loop
 * and tries again next pass, so a body split across packets costs no blocking
 * time in the task that also answers time requests.
 */
bool NtpStats::pumpRequest() {
  if (reqStartUs == 0) reqStartUs = esp_timer_get_time();

  for (;;) {
    if (reqLen >= (int)sizeof(g_req) - 1) break;
    int32_t r;
    if (useWifi) {
      r = recv(client_sock, g_req + reqLen, sizeof(g_req) - 1 - reqLen, 0);
      if (r == 0) { closeConn(); return false; }            // peer closed
      if (r < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        closeConn();
        return false;
      }
    } else {
      r = w5k_tcp_recv((uint8_t)sock, (uint8_t*)(g_req + reqLen),
                       (uint16_t)(sizeof(g_req) - 1 - reqLen));
      if (r < 0) { closeConn(); return false; }
      if (r == 0) break;
    }
    reqLen += r;
    g_req[reqLen] = '\0';
  }

  if (hdrEnd < 0) {
    char* p = strstr((char*)g_req, "\r\n\r\n");
    if (p) {
      hdrEnd = (int)(p - (char*)g_req) + 4;
      for (char* q = (char*)g_req; q < p; ++q) {
        if (strncasecmp(q, "Content-Length:", 15) == 0) {
          contentLen = atoi(q + 15);
          if (contentLen < 0) contentLen = 0;
          break;
        }
      }
    }
  }

  if (hdrEnd >= 0 && reqLen >= hdrEnd + contentLen) return true;

  // A client that opens a connection and then says nothing must not hold the
  // single stats socket forever.
  if (esp_timer_get_time() - reqStartUs > 5000000) closeConn();
  return false;
}

bool NtpStats::authorized(const char* req) {
  const char* want = cfg_str(CFG_UI_PASS);
  if (want[0] == '\0') return true;    // no password set: open, and the page says so

  const char* h = ci_find(req, "Authorization: Basic ");
  if (!h) return false;
  h += 21;

  // "user:pass" base64, any username accepted.
  static const char* B64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  char dec[96];
  int dlen = 0, bits = 0, acc = 0;
  for (; *h && *h != '\r' && *h != '\n' && *h != ' '; ++h) {
    if (*h == '=') break;
    const char* p = strchr(B64, *h);
    if (!p) return false;
    acc = (acc << 6) | (int)(p - B64);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      if (dlen < (int)sizeof(dec) - 1) dec[dlen++] = (char)((acc >> bits) & 0xFF);
    }
  }
  dec[dlen] = '\0';

  const char* colon = strchr(dec, ':');
  if (!colon) return false;
  return strcmp(colon + 1, want) == 0;
}

void NtpStats::handleConnection() {
  if (!pumpRequest()) return;

  const char* req = (const char*)g_req;
  const char* body = req + hdrEnd;

  bool isGet  = strncmp(req, "GET ",  4) == 0;
  bool isPost = strncmp(req, "POST ", 5) == 0;
  const char* path = req + (isPost ? 5 : 4);

  auto pathIs = [&](const char* p) {
    size_t n = strlen(p);
    return strncmp(path, p, n) == 0 &&
           (path[n] == ' ' || path[n] == '?' || path[n] == '\0');
  };

  if (isGet && pathIs("/metrics")) {
    sendMetrics();
    return;
  }

  if (isGet && (pathIs("/") || pathIs("/config"))) {
    if (!authorized(req)) {
      sendStatus("401 Unauthorized", "text/plain", "Authentication required");
      return;
    }
    sendConfigPage(nullptr);
    return;
  }

  if (isPost && pathIs("/config")) {
    if (!authorized(req)) {
      sendStatus("401 Unauthorized", "text/plain", "Authentication required");
      return;
    }
    handleConfigPost(body);
    return;
  }

  if (isPost && pathIs("/factory-reset")) {
    if (!authorized(req)) {
      sendStatus("401 Unauthorized", "text/plain", "Authentication required");
      return;
    }
    cfg_factory_reset();
    sendStatus("200 OK", "text/plain",
               "Settings erased. Rebooting into build-time defaults.");
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_restart();
    return;
  }

  sendStatus("404 Not Found", "text/plain", "Not Found");
}

void NtpStats::sendMetrics() {
  GpsStats gs = {};
  if (gps) gps->getStats(gs);
  bool locked = (gps && gps->isLocked());
  uint32_t reqCount = ntp ? ntp->getRequestCount() : 0;
  uint32_t rxIrqCount = ntp ? ntp->getRxIrqCount() : 0;
  uint32_t primeSkips = ntp ? ntp->getPrimeSkips() : 0;
  uint32_t capRejects = ntp ? ntp->getCapRejects() : 0;
  uint32_t lateOk = ntp ? ntp->getLateStampOk() : 0;
  uint32_t lateFb = ntp ? ntp->getLateStampFallbacks() : 0;
  int lastStageRc = ntp ? ntp->getLastStageRc() : 0;
  int lastWrDelta = ntp ? ntp->getLastWrDelta() : 0;
  uint32_t turnN = ntp ? ntp->getTurnSamples() : 0;
  double turnUs = ntp ? ntp->getTurnUs() : 0.0;
  double txCorrUs = ntp ? ntp->getTxCorrectionUs() : 0.0;
  double uptimeSec = (double)esp_timer_get_time() / 1e6;
  int stratum = locked ? 1 : 16;
  double rootDispersion = gps ? gps->getRootDispersion() : 1.0;

  // Diagnostics (read-only; not on the timekeeping path).
  int resetReason     = (int)esp_reset_reason();
  uint32_t freeHeap   = (uint32_t)esp_get_free_heap_size();
  uint32_t minFreeHeap= (uint32_t)esp_get_minimum_free_heap_size();
  uint32_t loopBeats  = g_mainLoopBeats;
  uint32_t bootCount  = g_bootCount;
  int ethLink         = eth ? (eth->isLinkUp() ? 1 : 0) : -1;
  int w5500Ver        = eth ? (int)eth->readVersion() : -1;
  uint32_t chipResets = eth ? eth->getChipResetCount() : 0;

  char* resp = g_resp;
  char* body = resp + hdrReserve;
  int blen = snprintf(body, sizeof(g_resp) - hdrReserve,
    "# HELP ntp_clock_offset_seconds Last measured clock offset\n"
    "# TYPE ntp_clock_offset_seconds gauge\n"
    "ntp_clock_offset_seconds %.9f\n"
    "# HELP ntp_rms_offset_seconds EW-RMS of the capture-fit endpoint residual. NOT a clock phase error: it is how far the newest PPS deviates from the oscillator's own 240 s trend line, so it responds to frequency RAMP as a*T^2/12. See ntp_gps_freq_drift_ppm_per_hour and ntp_gps_residual_predicted_seconds.\n"
    "# TYPE ntp_rms_offset_seconds gauge\n"
    "ntp_rms_offset_seconds %.9f\n"
    "# HELP ntp_frequency_ppm Estimated frequency error\n"
    "# TYPE ntp_frequency_ppm gauge\n"
    "ntp_frequency_ppm %.6f\n"
    "# HELP ntp_pps_jitter_seconds PPS pulse jitter\n"
    "# TYPE ntp_pps_jitter_seconds gauge\n"
    "ntp_pps_jitter_seconds %.9f\n"
    "# HELP ntp_root_dispersion_seconds Estimated root dispersion\n"
    "# TYPE ntp_root_dispersion_seconds gauge\n"
    "ntp_root_dispersion_seconds %.6f\n"
    "# HELP ntp_gps_lock GPS lock status\n"
    "# TYPE ntp_gps_lock gauge\n"
    "ntp_gps_lock %d\n"
    "# HELP ntp_stratum NTP stratum\n"
    "# TYPE ntp_stratum gauge\n"
    "ntp_stratum %d\n"
    "# HELP ntp_uptime_seconds Seconds since boot\n"
    "# TYPE ntp_uptime_seconds gauge\n"
    "ntp_uptime_seconds %.1f\n"
    "# HELP ntp_requests_total Total NTP requests served\n"
    "# TYPE ntp_requests_total counter\n"
    "ntp_requests_total %" PRIu32 "\n"
    "# HELP ntp_pps_count Total PPS edges received\n"
    "# TYPE ntp_pps_count counter\n"
    "ntp_pps_count %" PRIu32 "\n"
    "# HELP ntp_pps_rejects_total PPS pulses rejected as outliers\n"
    "# TYPE ntp_pps_rejects_total counter\n"
    "ntp_pps_rejects_total %" PRIu32 "\n"
    "# HELP ntp_nmea_mispair_total PPS pulses skipped as suspected PPS/NMEA second mispairs\n"
    "# TYPE ntp_nmea_mispair_total counter\n"
    "ntp_nmea_mispair_total %" PRIu32 "\n"
    "# HELP ntp_gps_holdover 1 when coasting on the oscillator (GPS lost, sync still credible)\n"
    "# TYPE ntp_gps_holdover gauge\n"
    "ntp_gps_holdover %d\n"
    "# HELP ntp_tx_late_stamp_total Replies sent with t3 patched in place before SEND\n"
    "# TYPE ntp_tx_late_stamp_total counter\n"
    "# HELP ntp_tx_late_stamp_fallback_total Replies that fell back to the library send\n"
    "# TYPE ntp_tx_late_stamp_fallback_total counter\n"
    "# HELP ntp_rx_irq_total W5500 RX interrupts captured (hardware arrival edges)\n"
    "# TYPE ntp_rx_irq_total counter\n"
    "ntp_tx_late_stamp_total %" PRIu32 "\n"
    "ntp_tx_late_stamp_fallback_total %" PRIu32 "\n"
    "ntp_tx_stage_last_rc %d\n"
    "ntp_tx_wr_delta %d\n"
    "ntp_tx_turnaround_us %.2f\n"
    "ntp_tx_turnaround_samples %" PRIu32 "\n"
    "ntp_rx_irq_total %" PRIu32 "\n"
    "# HELP ntp_tx_correction_us Self-calibrated transmit-path correction added to t3\n"
    "# TYPE ntp_tx_correction_us gauge\n"
    "ntp_tx_correction_us %.1f\n"
    "# HELP ntp_reset_reason esp_reset_reason() of the last boot (1=POWERON,4=SW,7=TASK_WDT,8=INT_WDT,9=BROWNOUT)\n"
    "# TYPE ntp_reset_reason gauge\n"
    "ntp_reset_reason %d\n"
    "# HELP ntp_boot_count Boots since flash (persisted in NVS; a jump = auto-recovery)\n"
    "# TYPE ntp_boot_count counter\n"
    "ntp_boot_count %" PRIu32 "\n"
    "# HELP ntp_main_loop_beats Core-0 main-loop iterations (liveness heartbeat)\n"
    "# TYPE ntp_main_loop_beats counter\n"
    "ntp_main_loop_beats %" PRIu32 "\n"
    "# HELP ntp_free_heap_bytes Current free heap\n"
    "# TYPE ntp_free_heap_bytes gauge\n"
    "ntp_free_heap_bytes %" PRIu32 "\n"
    "# HELP ntp_min_free_heap_bytes Lowest free heap since boot\n"
    "# TYPE ntp_min_free_heap_bytes gauge\n"
    "ntp_min_free_heap_bytes %" PRIu32 "\n"
    "# HELP ntp_eth_link_up W5500 link health (1=up,0=down,-1=n/a)\n"
    "# TYPE ntp_eth_link_up gauge\n"
    "ntp_eth_link_up %d\n"
    "# HELP ntp_w5500_version W5500 VERSIONR (4=healthy; other=wedged SPI; -1=n/a)\n"
    "# TYPE ntp_w5500_version gauge\n"
    "ntp_w5500_version %d\n"
    "# HELP ntp_w5500_chip_resets_total W5500 register-loss events recovered in place (chip reset without a device reboot)\n"
    "# TYPE ntp_w5500_chip_resets_total counter\n"
    "ntp_w5500_chip_resets_total %" PRIu32 "\n"
    "# HELP ntp_w5500_mgmt_spin_timeouts_total Bounded management-path register waits that expired (chip stopped acknowledging commands)\n"
    "# TYPE ntp_w5500_mgmt_spin_timeouts_total counter\n"
    "ntp_w5500_mgmt_spin_timeouts_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_lease_seconds Lease duration granted by the DHCP server\n"
    "# TYPE ntp_dhcp_lease_seconds gauge\n"
    "ntp_dhcp_lease_seconds %" PRIu32 "\n"
    "# HELP ntp_dhcp_acks_total DHCP ACKs bound (acquisitions and renewals)\n"
    "# TYPE ntp_dhcp_acks_total counter\n"
    "ntp_dhcp_acks_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_naks_total DHCP NAKs received\n"
    "# TYPE ntp_dhcp_naks_total counter\n"
    "ntp_dhcp_naks_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_timeouts_total DHCP per-phase resend/expiry events\n"
    "# TYPE ntp_dhcp_timeouts_total counter\n"
    "ntp_dhcp_timeouts_total %" PRIu32 "\n"
    "# HELP ntp_dhcp_renews_total Unicast renewal REQUESTs sent at T1\n"
    "# TYPE ntp_dhcp_renews_total counter\n"
    "ntp_dhcp_renews_total %" PRIu32 "\n"
    "# HELP ntp_gps_fix_quality GGA fix quality (0 none, 1 GPS, 2 DGPS, 4 RTK fix, 5 RTK float)\n"
    "# TYPE ntp_gps_fix_quality gauge\n"
    "ntp_gps_fix_quality %d\n"
    "# HELP gps_fix_type GNSS fix type (0=none,2=2D,3=3D)\n"
    "# TYPE gps_fix_type gauge\n"
    "gps_fix_type %d\n"
    "# HELP gps_satellites_used Satellites in the position solution\n"
    "# TYPE gps_satellites_used gauge\n"
    "gps_satellites_used %d\n"
    "# HELP ntp_gps_satellites_visible Satellites in view across all constellations\n"
    "# TYPE ntp_gps_satellites_visible gauge\n"
    "ntp_gps_satellites_visible %d\n"
    "# HELP ntp_gps_satellites_in_view Satellites in view, by constellation\n"
    "# TYPE ntp_gps_satellites_in_view gauge\n"
    "ntp_gps_satellites_in_view{constellation=\"gps\"} %d\n"
    "ntp_gps_satellites_in_view{constellation=\"glonass\"} %d\n"
    "ntp_gps_satellites_in_view{constellation=\"galileo\"} %d\n"
    "ntp_gps_satellites_in_view{constellation=\"beidou\"} %d\n"
    "# HELP ntp_gps_satellites_tracked Satellites reporting a non-zero C/N0\n"
    "# TYPE ntp_gps_satellites_tracked gauge\n"
    "ntp_gps_satellites_tracked %d\n"
    "# HELP ntp_gps_cn0_max_db Strongest tracked signal, dB-Hz\n"
    "# TYPE ntp_gps_cn0_max_db gauge\n"
    "ntp_gps_cn0_max_db %d\n"
    "# HELP ntp_gps_cn0_mean_db Mean C/N0 over tracked satellites, dB-Hz\n"
    "# TYPE ntp_gps_cn0_mean_db gauge\n"
    "ntp_gps_cn0_mean_db %d\n"
    "# HELP gps_time_valid RMC reports a valid fix\n"
    "# TYPE gps_time_valid gauge\n"
    "gps_time_valid %d\n"
    "# HELP gps_latitude_degrees\n"
    "# TYPE gps_latitude_degrees gauge\n"
    "gps_latitude_degrees %.7f\n"
    "# HELP gps_longitude_degrees\n"
    "# TYPE gps_longitude_degrees gauge\n"
    "gps_longitude_degrees %.7f\n"
    "# HELP gps_pdop Position dilution of precision\n"
    "# TYPE gps_pdop gauge\n"
    "gps_pdop %.2f\n"
    "# HELP gps_hdop Horizontal dilution of precision\n"
    "# TYPE gps_hdop gauge\n"
    "gps_hdop %.2f\n"
    "# HELP gps_vdop Vertical dilution of precision\n"
    "# TYPE gps_vdop gauge\n"
    "gps_vdop %.2f\n"
    "# HELP gps_altitude_msl_meters GGA altitude above mean sea level\n"
    "# TYPE gps_altitude_msl_meters gauge\n"
    "gps_altitude_msl_meters %.1f\n"
    "# HELP ntp_gps_nmea_age_seconds Age of the newest valid RMC fix\n"
    "# TYPE ntp_gps_nmea_age_seconds gauge\n"
    "ntp_gps_nmea_age_seconds %.3f\n"
    "# HELP ntp_arp_prime_skips_total Replies that reused the socket's resolved peer\n"
    "# TYPE ntp_arp_prime_skips_total counter\n"
    "ntp_arp_prime_skips_total %" PRIu32 "\n"
    "# HELP ntp_capture_rejects_total Arrival captures rejected as mis-paired\n"
    "# TYPE ntp_capture_rejects_total counter\n"
    "ntp_capture_rejects_total %" PRIu32 "\n"
    "# HELP ts2phc_offset_ns PPS offset against the capture clock, nanoseconds\n"
    "# TYPE ts2phc_offset_ns gauge\n"
    "ts2phc_offset_ns{clock=\"mcpwm0\"} %.1f\n"
    "# HELP ts2phc_freq_ppb Capture-clock frequency error, parts per billion\n"
    "# TYPE ts2phc_freq_ppb gauge\n"
    "ts2phc_freq_ppb{clock=\"mcpwm0\"} %.1f\n"
    "# HELP ntp_gps_fit_valid Capture-to-GPS-second least-squares fit is solved\n"
    "# TYPE ntp_gps_fit_valid gauge\n"
    "ntp_gps_fit_valid %d\n"
    "# HELP ntp_gps_fit_samples Points in the current fit window\n"
    "# TYPE ntp_gps_fit_samples gauge\n"
    "ntp_gps_fit_samples %" PRIu32 "\n"
    "# HELP ntp_gps_fit_ticks_per_second Fitted capture-timer rate (nominal 80e6); 0 while unsolved\n"
    "# TYPE ntp_gps_fit_ticks_per_second gauge\n"
    "ntp_gps_fit_ticks_per_second %.3f\n"
    "# HELP ntp_pps_handle_latency_us PPS hardware capture to discipline-handler entry\n"
    "# TYPE ntp_pps_handle_latency_us gauge\n"
    "ntp_pps_handle_latency_us %.1f\n"
    "# HELP ntp_pps_handle_latency_max_us Worst PPS capture-to-handler latency seen\n"
    "# TYPE ntp_pps_handle_latency_max_us gauge\n"
    "ntp_pps_handle_latency_max_us %.1f\n"
    "# HELP ntp_gps_freq_drift_ppm_per_hour Rate of change of the fitted oscillator frequency\n"
    "# TYPE ntp_gps_freq_drift_ppm_per_hour gauge\n"
    "ntp_gps_freq_drift_ppm_per_hour %.4f\n"
    "# HELP ntp_gps_residual_predicted_seconds Fit endpoint residual predicted from that drift alone (a*T^2/12); compare against ntp_rms_offset_seconds\n"
    "# TYPE ntp_gps_residual_predicted_seconds gauge\n"
    "ntp_gps_residual_predicted_seconds %.9f\n",
    gs.lastOffsetSec,
    gs.rmsOffsetSec,
    gs.frequencyPpm,
    gs.ppsJitterSec,
    rootDispersion,
    locked ? 1 : 0,
    stratum,
    uptimeSec,
    reqCount,
    gs.ppsCount,
    gs.ppsRejectCount,
    gs.nmeaMispairCount,
    gs.holdover ? 1 : 0,
    lateOk,
    lateFb,
    lastStageRc,
    lastWrDelta,
    turnUs,
    turnN,
    rxIrqCount,
    txCorrUs,
    resetReason,
    bootCount,
    loopBeats,
    freeHeap,
    minFreeHeap,
    ethLink,
    w5500Ver,
    chipResets,
    w5500_mgmt_spin_timeouts(),
    w5500_dhcp_lease_seconds(),
    w5500_dhcp_acks_total(),
    w5500_dhcp_naks_total(),
    w5500_dhcp_timeouts_total(),
    w5500_dhcp_renews_total(),

    gs.fixQuality,
    gs.fixType,
    gs.satsUsed,
    gs.satsVisible,
    gs.satsGps,
    gs.satsGlonass,
    gs.satsGalileo,
    gs.satsBeidou,
    gs.satsTracked,
    gs.cn0Max,
    gs.cn0Mean,
    gs.timeValid ? 1 : 0,
    gs.latitudeDeg,
    gs.longitudeDeg,
    gs.pdop,
    gs.hdop,
    gs.vdop,
    gs.altitudeM,
    gs.nmeaAgeMs / 1000.0,
    primeSkips,
    capRejects,
    gs.lastOffsetSec * 1e9,
    gs.fitValid ? (gs.fitTicksPerSec / 80000000.0 - 1.0) * 1e9 : 0.0,
    gs.fitValid ? 1 : 0,
    gs.fitSamples,
    gs.fitTicksPerSec,
    gs.ppsHandleLatUs,
    gs.ppsHandleLatMaxUs,
    gs.freqDriftPpmPerHour,
    gs.residualPredictedSec);

  /*
   * Reply-path attribution. Labelled series rather than one metric per span so
   * a single query renders the whole budget, and so adding a checkpoint does
   * not change the metric namespace.
   */
  if (blen > 0) {
    int cap = (int)(sizeof(g_resp) - hdrReserve);
    blen += snprintf(body + blen, cap - blen,
      "# HELP ntp_path_span_us Honest software-timed reply-path spans (EWMA)\n"
      "# TYPE ntp_path_span_us gauge\n");
    for (int i = 0; i < NTP_PROF_COUNT && blen < cap - 128; ++i)
      blen += snprintf(body + blen, cap - blen,
        "ntp_path_span_us{span=\"%s\"} %.3f\n",
        ntp_prof_name(i), ntp_prof_ewma_us(i));
    blen += snprintf(body + blen, cap - blen,
      "# HELP ntp_path_span_max_us Worst observed value of each span\n"
      "# TYPE ntp_path_span_max_us gauge\n");
    for (int i = 0; i < NTP_PROF_COUNT && blen < cap - 128; ++i)
      blen += snprintf(body + blen, cap - blen,
        "ntp_path_span_max_us{span=\"%s\"} %.3f\n",
        ntp_prof_name(i), ntp_prof_max_us(i));
    blen += snprintf(body + blen, cap - blen,
      "# HELP ntp_path_span_min_us Best observed value of each span (the floor)\n"
      "# TYPE ntp_path_span_min_us gauge\n");
    for (int i = 0; i < NTP_PROF_COUNT && blen < cap - 128; ++i)
      blen += snprintf(body + blen, cap - blen,
        "ntp_path_span_min_us{span=\"%s\"} %.3f\n",
        ntp_prof_name(i), ntp_prof_min_us(i));
    blen += snprintf(body + blen, cap - blen,
      "# HELP ntp_int_to_send_us Arrival edge to SEND accepted (the honest turnaround)\n"
      "# TYPE ntp_int_to_send_us gauge\n"
      "ntp_int_to_send_us %.3f\n"
      "# HELP ntp_path_samples Packets folded into the span EWMAs\n"
      "# TYPE ntp_path_samples counter\n"
      "ntp_path_samples %" PRIu32 "\n"
      "# HELP ntp_spi_txns_per_reply SPI transactions per loop() body, ARP prime INCLUDED\n"
      "# TYPE ntp_spi_txns_per_reply gauge\n"
      "ntp_spi_txns_per_reply %.2f\n"
      "# HELP ntp_spi_txns_per_reply_serve SPI transactions for the reply itself, ARP prime excluded\n"
      "# TYPE ntp_spi_txns_per_reply_serve gauge\n"
      "ntp_spi_txns_per_reply_serve %.2f\n"
      "# HELP ntp_spi_txns_per_prime SPI transactions spent inside w5k_arp_prime (mostly Sn_IR polls, so this RISES as SPI gets faster)\n"
      "# TYPE ntp_spi_txns_per_prime gauge\n"
      "ntp_spi_txns_per_prime %.2f\n"
      "# HELP ntp_spi_bytes_per_reply Bytes clocked per loop() body, headers included\n"
      "# TYPE ntp_spi_bytes_per_reply gauge\n"
      "ntp_spi_bytes_per_reply %.2f\n"
      "# HELP ntp_spi_bytes_per_reply_serve Bytes clocked for the reply itself, ARP prime excluded\n"
      "# TYPE ntp_spi_bytes_per_reply_serve gauge\n"
      "ntp_spi_bytes_per_reply_serve %.2f\n"
      "# HELP ntp_spi_selects_per_reply wizchip_select() calls per served reply\n"
      "# TYPE ntp_spi_selects_per_reply gauge\n"
      "ntp_spi_selects_per_reply %.2f\n"
      "# HELP ntp_spi_reap_polls_per_reply Sn_IR reads spent spinning on SENDOK\n"
      "# TYPE ntp_spi_reap_polls_per_reply gauge\n"
      "ntp_spi_reap_polls_per_reply %.2f\n"
      "# HELP ntp_spi_prime_polls_per_reply Sn_IR reads spent in w5k_arp_prime\n"
      "# TYPE ntp_spi_prime_polls_per_reply gauge\n"
      "ntp_spi_prime_polls_per_reply %.2f\n"
      "# HELP ntp_arp_primes_total ARP primes actually issued (paid before stamping t3)\n"
      "# TYPE ntp_arp_primes_total counter\n"
      "ntp_arp_primes_total %" PRIu32 "\n",
      ntp_prof_ewma_us(NTP_PROF_INT_TO_SEND),
      ntp_prof_samples(),
      ntp_prof_txns(), ntp_prof_txns_serve(), ntp_prof_prime_txns(),
      ntp_prof_bytes(), ntp_prof_bytes_serve(), ntp_prof_sels(),
      ntp_prof_reap_polls(), ntp_prof_prime_polls(), ntp_prof_primes());

    /* Cumulative histogram of int_to_send, so the bimodality is visible and a
     * percentile can be read off directly rather than inferred from an EWMA. */
    blen += snprintf(body + blen, cap - blen,
      "# HELP ntp_int_to_send_us_bucket Cumulative histogram of arrival-edge to SEND\n"
      "# TYPE ntp_int_to_send_us_bucket histogram\n");
    uint32_t cum = 0;
    for (int i = 0; i < NTP_PROF_BUCKETS && blen < cap - 128; ++i) {
      cum += ntp_prof_bucket(i);
      uint32_t e = ntp_prof_bucket_edge_us(i);
      if (e)
        blen += snprintf(body + blen, cap - blen,
          "ntp_int_to_send_us_bucket{le=\"%" PRIu32 "\"} %" PRIu32 "\n", e, cum);
      else
        blen += snprintf(body + blen, cap - blen,
          "ntp_int_to_send_us_bucket{le=\"+Inf\"} %" PRIu32 "\n", cum);
    }
  }

  /*
   * Per-satellite series, matching ts2phc-go's label scheme so one dashboard
   * query covers every clock in the fleet. Appended rather than folded into the
   * snprintf above because the count varies with the sky.
   */
  if (blen > 0) {
    static const char* const kGnss[GNSS_COUNT] =
        { "gps", "sbas", "glonass", "galileo", "beidou", "qzss" };
    GpsSatellite sats[40];
    int nsat = gps ? gps->getSatellites(sats, 40) : 0;
    int cap = (int)(sizeof(g_resp) - hdrReserve);
    if (nsat > 0 && blen < cap - 1) {
      blen += snprintf(body + blen, cap - blen,
        "# HELP gps_satellite_cno_dbhz C/N0 per satellite\n"
        "# TYPE gps_satellite_cno_dbhz gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, cap - blen,
          "gps_satellite_cno_dbhz{gnss=\"%s\",svid=\"%u\"} %u\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, (unsigned)sats[i].cn0);

      blen += snprintf(body + blen, cap - blen,
        "# HELP gps_satellite_elevation_degrees\n"
        "# TYPE gps_satellite_elevation_degrees gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, cap - blen,
          "gps_satellite_elevation_degrees{gnss=\"%s\",svid=\"%u\"} %d\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, (int)sats[i].elev);

      blen += snprintf(body + blen, cap - blen,
        "# HELP gps_satellite_azimuth_degrees\n"
        "# TYPE gps_satellite_azimuth_degrees gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, cap - blen,
          "gps_satellite_azimuth_degrees{gnss=\"%s\",svid=\"%u\"} %u\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, (unsigned)sats[i].azim);

      blen += snprintf(body + blen, cap - blen,
        "# HELP gps_satellite_used 1 if SV used in nav solution\n"
        "# TYPE gps_satellite_used gauge\n");
      for (int i = 0; i < nsat && blen < cap - 96; ++i)
        blen += snprintf(body + blen, cap - blen,
          "gps_satellite_used{gnss=\"%s\",svid=\"%u\"} %d\n",
          kGnss[sats[i].gnss < GNSS_COUNT ? sats[i].gnss : 0],
          (unsigned)sats[i].svid, sats[i].used ? 1 : 0);
    }
  }

  if (blen < 0 || blen >= (int)(sizeof(g_resp) - hdrReserve)) {
    blen = (int)(sizeof(g_resp) - hdrReserve) - 1;
    body[blen] = '\0';
  }

  char hdr[128];
  int hlen = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/plain; version=0.0.4; charset=utf-8\r\n"
    "Connection: close\r\n"
    "Content-Length: %d\r\n"
    "\r\n",
    blen);

  memmove(resp + hlen, body, blen);
  memcpy(resp, hdr, hlen);
  sendAll(resp, hlen + blen);
  closeConn();
}

/* %XX and + decoding, in place. */
static void url_decode(char* s) {
  char* out = s;
  for (; *s; ++s) {
    if (*s == '+') {
      *out++ = ' ';
    } else if (*s == '%' && isxdigit((unsigned char)s[1]) && isxdigit((unsigned char)s[2])) {
      char hex[3] = { s[1], s[2], '\0' };
      *out++ = (char)strtol(hex, nullptr, 16);
      s += 2;
    } else {
      *out++ = *s;
    }
  }
  *out = '\0';
}

static void html_escape(const char* in, char* out, size_t cap) {
  size_t o = 0;
  for (const char* p = in; *p && o + 7 < cap; ++p) {
    switch (*p) {
      case '&':  memcpy(out + o, "&amp;", 5);  o += 5; break;
      case '<':  memcpy(out + o, "&lt;", 4);   o += 4; break;
      case '>':  memcpy(out + o, "&gt;", 4);   o += 4; break;
      case '"':  memcpy(out + o, "&quot;", 6); o += 6; break;
      default:   out[o++] = *p;
    }
  }
  out[o] = '\0';
}

static const char* const kGroupOrder[] = {
  "Network", "System", "Display", "Service",
  "Display wiring", "W5500 wiring", "GPS wiring", nullptr
};

void NtpStats::renderField(char** pp, char* end, int i) {
  char* p = *pp;
  const cfg_field_t* f = &g_cfg_fields[i];

  p += snprintf(p, end - p, "<tr><td class=l><label for=\"%s\">%s</label>%s",
                f->key, f->label, f->reboot ? "<span class=r>*</span>" : "");
  if (f->help) p += snprintf(p, end - p, "<div class=h>%s</div>", f->help);
  p += snprintf(p, end - p, "</td><td>");

  switch (f->type) {
    case CF_BOOL:
      p += snprintf(p, end - p,
        "<input type=hidden name=\"%s\" value=\"0\">"
        "<input type=checkbox id=\"%s\" name=\"%s\" value=\"1\"%s>",
        f->key, f->key, f->key, cfg_int((cfg_id_t)i) ? " checked" : "");
      break;
    case CF_ENUM:
      p += snprintf(p, end - p, "<select id=\"%s\" name=\"%s\">", f->key, f->key);
      for (int v = f->imin; v <= f->imax; ++v)
        p += snprintf(p, end - p, "<option value=\"%d\"%s>%s</option>",
                      v, cfg_int((cfg_id_t)i) == v ? " selected" : "", f->names[v]);
      p += snprintf(p, end - p, "</select>");
      break;
    case CF_PASS:
      p += snprintf(p, end - p,
        "<input type=password id=\"%s\" name=\"%s\" value=\"\" placeholder=\"%s\">",
        f->key, f->key, cfg_str((cfg_id_t)i)[0] ? "unchanged" : "not set");
      if (cfg_str((cfg_id_t)i)[0])
        p += snprintf(p, end - p,
          "<label class=h><input type=checkbox name=\"%s.clr\" value=\"1\"> remove</label>",
          f->key);
      break;
    case CF_STR: {
      const char* cur = cfg_str((cfg_id_t)i);
      if (f->opts) {
        bool matched = false;
        for (int k = 0; f->opts[k]; k += 2)
          if (strcmp(f->opts[k], cur) == 0) { matched = true; break; }
        p += snprintf(p, end - p, "<select id=\"%s\" name=\"%s\">", f->key, f->key);
        if (!matched) {
          char esc[CFG_STR_MAX * 6];
          html_escape(cur, esc, sizeof(esc));
          p += snprintf(p, end - p, "<option value=\"%s\" selected>%s (custom)</option>", esc, esc);
        }
        for (int k = 0; f->opts[k]; k += 2)
          p += snprintf(p, end - p, "<option value=\"%s\"%s>%s</option>",
                        f->opts[k], strcmp(f->opts[k], cur) == 0 ? " selected" : "",
                        f->opts[k + 1]);
        p += snprintf(p, end - p, "</select>");
      } else {
        char esc[CFG_STR_MAX * 6];
        html_escape(cur, esc, sizeof(esc));
        p += snprintf(p, end - p,
          "<input id=\"%s\" name=\"%s\" value=\"%s\" maxlength=%d>",
          f->key, f->key, esc, CFG_STR_MAX - 1);
      }
      break;
    }
    case CF_INT:
      p += snprintf(p, end - p,
        "<input type=number id=\"%s\" name=\"%s\" value=\"%d\" min=%d max=%d>",
        f->key, f->key, (int)cfg_int((cfg_id_t)i), (int)f->imin, (int)f->imax);
      break;
  }
  p += snprintf(p, end - p, "</td></tr>");
  *pp = p;
}

void NtpStats::renderSection(char** pp, char* end, bool advanced) {
  for (int g = 0; kGroupOrder[g]; ++g) {
    bool opened = false;
    for (int i = 0; i < CFG_COUNT; ++i) {
      const cfg_field_t* f = &g_cfg_fields[i];
      if (f->advanced != advanced) continue;
      if (strcmp(f->group, kGroupOrder[g]) != 0) continue;
      if (!opened) {
        *pp += snprintf(*pp, end - *pp, "<h2>%s</h2><table>", kGroupOrder[g]);
        opened = true;
      }
      renderField(pp, end, i);
    }
    if (opened) *pp += snprintf(*pp, end - *pp, "</table>");
  }
}

void NtpStats::sendConfigPage(const char* notice) {
  char* p = g_resp;
  char* end = g_resp + sizeof(g_resp);

  p += snprintf(p, end - p,
    "<!doctype html><meta charset=utf-8>"
    "<meta name=viewport content=\"width=device-width,initial-scale=1\">"
    "<title>esp32-ntp settings</title>"
    "<style>"
    ":root{color-scheme:light dark;--b:#8883;--m:#8888}"
    "*{box-sizing:border-box}"
    "body{font:15px/1.5 ui-sans-serif,system-ui,sans-serif;max-width:44rem;margin:0 auto;padding:1.5rem 1rem 4rem}"
    "h1{font-size:1.35rem;margin:0}"
    "h2{font-size:.8rem;text-transform:uppercase;letter-spacing:.06em;color:var(--m);"
    "margin:1.75rem 0 .35rem;font-weight:600}"
    "table{width:100%%;border-collapse:collapse}"
    "tr{border-top:1px solid var(--b)}"
    "tr:first-child{border-top:0}"
    "td{padding:.55rem 0;vertical-align:top}"
    "td.l{width:54%%;padding-right:1rem}"
    "label{font-weight:500}"
    ".h{color:var(--m);font-size:.82rem;line-height:1.35;margin-top:.15rem}"
    ".r{color:#d97706;margin-left:.25rem;font-weight:700}"
    "input,select{width:100%%;padding:.4rem .5rem;font:inherit;"
    "border:1px solid var(--b);border-radius:.375rem;background:transparent;color:inherit}"
    "input:focus,select:focus{outline:2px solid #3b82f6;outline-offset:1px}"
    "input[type=checkbox]{width:1.1rem;height:1.1rem;margin-top:.3rem}"
    ".bar{display:flex;align-items:baseline;justify-content:space-between;gap:1rem;flex-wrap:wrap}"
    ".st{color:var(--m);font-size:.85rem;font-variant-numeric:tabular-nums}"
    ".note{padding:.65rem .85rem;border-radius:.375rem;margin:1rem 0;font-size:.9rem}"
    ".warn{background:#fef3c7;color:#713f12;border:1px solid #fcd34d}"
    ".ok{background:#dcfce7;color:#14532d;border:1px solid #86efac}"
    "details{margin-top:2rem;border-top:1px solid var(--b);padding-top:1rem}"
    "summary{cursor:pointer;font-weight:600;font-size:.9rem}"
    "button{font:inherit;font-weight:500;padding:.5rem 1.1rem;border-radius:.375rem;"
    "border:1px solid var(--b);background:#3b82f6;color:#fff;cursor:pointer}"
    "button.sec{background:transparent;color:inherit}"
    ".acts{position:sticky;bottom:0;background:Canvas;padding:1rem 0;margin-top:1.5rem;"
    "border-top:1px solid var(--b);display:flex;gap:.6rem;align-items:center;flex-wrap:wrap}"
    "a{color:#3b82f6}"
    "</style>");

  uint32_t ipv = 0;
  if (eth) eth->getIpAddr(ipv);
  else if (wifi) wifi->getIpAddr(ipv);
  uint8_t mac[6] = {0};
  if (eth) eth->getMacAddr(mac);

  p += snprintf(p, end - p,
    "<div class=bar><h1>esp32-ntp</h1>"
    "<div class=st>%u.%u.%u.%u &middot; %02x:%02x:%02x:%02x:%02x:%02x &middot; up %us</div></div>",
    (unsigned)((ipv >> 24) & 0xff), (unsigned)((ipv >> 16) & 0xff),
    (unsigned)((ipv >> 8) & 0xff), (unsigned)(ipv & 0xff),
    mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
    (unsigned)(esp_timer_get_time() / 1000000));

  if (notice)
    p += snprintf(p, end - p, "<div class=\"note ok\">%s</div>", notice);

  if (Config::isSafeMode())
    p += snprintf(p, end - p,
      "<div class=\"note warn\"><b>Safe mode.</b> Stored settings were ignored this boot "
      "because %d starts in a row did not reach the network. These are the build-time "
      "defaults; your stored values are still in flash, and saving from here overwrites "
      "them. A restart that reaches the network clears this on its own.</div>",
      CFG_SAFE_MODE_FAILS);

  if (cfg_str(CFG_UI_PASS)[0] == '\0')
    p += snprintf(p, end - p,
      "<div class=\"note warn\">No management password is set, so anyone who can reach this "
      "address can change these settings or reboot the clock.</div>");

  p += snprintf(p, end - p, "<form method=post action=/config>");
  renderSection(&p, end, false);

  p += snprintf(p, end - p,
    "<details><summary>Advanced: wiring, pins and timing</summary>"
    "<div class=\"note warn\">These describe how the board is physically wired and how the "
    "clock is calibrated. If you built it to the published wiring, leave them alone. A wrong "
    "pin here takes the clock off the network on the next restart; interrupt startup twice, "
    "with RESET or the power lead, and the third boot comes up on the build-time defaults "
    "so you can undo it.</div>");
  renderSection(&p, end, true);
  p += snprintf(p, end - p, "</details>");

  p += snprintf(p, end - p,
    "<div class=acts><button type=submit>Save to flash</button>"
    "<span class=st><span class=r>*</span> needs a restart</span></div></form>"
    "<form method=post action=/factory-reset "
    "onsubmit=\"return confirm('Erase all stored settings and reboot?')\">"
    "<button class=sec type=submit>Erase stored settings</button></form>"
    "<p class=st><a href=/metrics>/metrics</a></p>");

  int blen = (int)(p - g_resp);
  if (blen >= (int)sizeof(g_resp) - 1) blen = (int)sizeof(g_resp) - 1;

  char hdr[160];
  int hlen = snprintf(hdr, sizeof(hdr),
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: text/html; charset=utf-8\r\n"
    "Connection: close\r\n"
    "Content-Length: %d\r\n"
    "\r\n", blen);
  sendAll(hdr, hlen);
  sendAll(g_resp, blen);
  closeConn();
}

/*
 * Stage every field the form sent, then commit once. A value that fails
 * validation is reported and nothing is written, so a typo in one pin cannot
 * half-apply a settings change.
 */
void NtpStats::handleConfigPost(const char* body) {
  static char buf[2048];
  snprintf(buf, sizeof(buf), "%s", body);

  char bad[64] = {0};
  int staged = 0;

  char* saveptr = nullptr;
  for (char* tok = strtok_r(buf, "&", &saveptr); tok; tok = strtok_r(nullptr, "&", &saveptr)) {
    char* eq = strchr(tok, '=');
    if (!eq) continue;
    *eq = '\0';
    char* key = tok;
    char* val = eq + 1;
    url_decode(key);
    url_decode(val);

    size_t klen = strlen(key);
    if (klen > 4 && strcmp(key + klen - 4, ".clr") == 0) {
      key[klen - 4] = '\0';
      cfg_id_t cid = cfg_lookup(key);
      if (cid != CFG_COUNT && strcmp(val, "1") == 0) {
        cfg_clear(cid);
        staged++;
      }
      continue;
    }

    cfg_id_t id = cfg_lookup(key);
    if (id == CFG_COUNT) continue;
    if (!cfg_stage(id, val)) {
      snprintf(bad, sizeof(bad), "%s", g_cfg_fields[id].label);
      break;
    }
    staged++;
  }

  if (bad[0]) {
    char msg[160];
    snprintf(msg, sizeof(msg),
             "Rejected: %s is out of range. Nothing was written.", bad);
    sendStatus("400 Bad Request", "text/plain", msg);
    return;
  }

  bool changed = cfg_dirty();
  esp_err_t err = cfg_commit();
  if (err != ESP_OK) {
    sendStatus("500 Internal Server Error", "text/plain", "Could not write to flash");
    return;
  }

  if (!changed) {
    sendConfigPage("No changes; flash was not written.");
    return;
  }

  sendStatus("200 OK", "text/html",
    "<!doctype html><meta charset=utf-8>"
    "<meta http-equiv=refresh content=\"8;url=/\">"
    "<p>Saved to flash. Restarting.</p>"
    "<p>If the address or interface changed, the device will come back somewhere else.</p>");
  ESP_LOGW(TAG, "settings saved over the management UI, restarting");
  vTaskDelay(pdMS_TO_TICKS(300));
  esp_restart();
}

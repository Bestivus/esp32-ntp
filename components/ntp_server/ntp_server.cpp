// SPDX-License-Identifier: Unlicense

#include "ntp_server.h"
#include "config.h"
#include "gps.h"
#include <string.h>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_log.h"
#include <fcntl.h>
#include "lwip/sockets.h"
#include "w5k_udp_wrapper.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "NTP_SRV";

// W5500 RX interrupt capture: the INTn pin (active-low) asserts on packet
// arrival. The GPIO ISR latches a monotonic timestamp at the edge — the same
// "capture in hardware, not in a poll" trick used for PPS — so t2 reflects true
// arrival instead of when the poll loop happened to read the socket.
static volatile uint64_t s_rxCaptureUs = 0;
static volatile uint32_t s_rxIrqCount = 0;
static uint32_t s_primeSkips = 0;
static uint32_t s_capRejects = 0;
static volatile uint32_t s_rxIrqSeq = 0;

// Self-calibrating transmit correction (µs). t3 must be written into the packet
// before the SPI send completes, so the wire egress is always ~one send-duration
// later than t3. We measure each send's duration and feed an EWMA back as a
// pre-correction added to the next packet's t3, so t3 tracks true departure.
// Seeded from the FIRST measurement rather than a constant: the old 700us
// figure described a whole 48-byte blocking send, but the split send measures
// only the 8-byte tail plus the SEND command, so a fixed seed took ~83 replies
// to decay and stamped t3 up to 0.7 ms into the future the whole time.
static double s_txCorrectionUs = 0.0;
/* Which transmit path ran: late in-place stamp vs library fallback. */
/* Hardware-measured request-to-egress turnaround, in capture ticks. Both ends
 * are MCPWM captures of INTn (arrival, then SENDOK), so this is measured on the
 * same counter that timestamps the GPS pulse — no software timing involved. */
static double s_turnTicks = 0.0;
static bool s_turnSeeded = false;
static uint32_t s_turnSamples = 0;
static uint32_t s_lateStampOk = 0;
static uint32_t s_lateStampFallbacks = 0;
/* Last nonzero rc from w5k_send_stage: -1 = TX free-space check, -2 = the
 * Sn_TX_WR advance did not equal the frame length. */
static int s_lastStageRc = 0;
static int s_lastWrDelta = -1;
static bool s_txCorrectionSeeded = false;

/* Task to wake when INTn asserts. Set once the NTP task exists; until then the
 * ISR only bookkeeps and the task's timeout drains the socket. */
static volatile TaskHandle_t s_ntpTask = nullptr;

void ntp_register_task(void* handle) { s_ntpTask = (TaskHandle_t)handle; }

/* Block until INTn fires or the timeout expires. Returns true if notified. */
bool ntp_wait_for_packet(uint32_t timeout_ms) {
  if (s_ntpTask == nullptr) ntp_register_task(xTaskGetCurrentTaskHandle());
  return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0;
}

static void IRAM_ATTR w5500_rx_isr(void* arg) {
  /* Seqlock: a 64-bit store is two words on Xtensa, and the task reads it as
   * two loads, so without this the value tears when the low word wraps (every
   * 71.6 min) and t2 lands thousands of seconds away. */
  s_rxIrqSeq = s_rxIrqSeq + 1;
  __sync_synchronize();
  s_rxCaptureUs = (uint64_t)esp_timer_get_time();
  __sync_synchronize();
  s_rxIrqSeq = s_rxIrqSeq + 1;
  s_rxIrqCount = s_rxIrqCount + 1;
  /* Wake the NTP task immediately: this replaces a timer-paced poll, removing
   * up to a full tick of random latency from every reply. */
  if (s_ntpTask) {
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)s_ntpTask, &hpw);
    if (hpw) portYIELD_FROM_ISR();
  }
}

/* Consistent snapshot of the ISR timestamp; false if it could not be read. */
static bool rx_isr_stamp(uint64_t* out) {
  for (int i = 0; i < 8; i++) {
    uint32_t s1 = s_rxIrqSeq;
    if (s1 & 1) continue;
    __sync_synchronize();
    uint64_t v = s_rxCaptureUs;
    __sync_synchronize();
    if (s_rxIrqSeq == s1) { *out = v; return true; }
  }
  return false;
}

static void wr32(uint8_t* p, int idx, uint32_t v) {
  p[idx+0] = (uint8_t)(v >> 24);
  p[idx+1] = (uint8_t)(v >> 16);
  p[idx+2] = (uint8_t)(v >> 8);
  p[idx+3] = (uint8_t)(v);
}

/* Subtract microseconds from an NTP sec/frac pair, borrowing correctly. */
static void ntp_ts_sub_us(uint32_t& sec, uint32_t& frac, int us) {
  if (us == 0) return;
  int64_t f = (int64_t)frac - ((int64_t)us * 4294967296LL) / 1000000LL;
  while (f < 0) { f += 4294967296LL; sec -= 1; }
  while (f >= 4294967296LL) { f -= 4294967296LL; sec += 1; }
  frac = (uint32_t)f;
}

static void wr_ntp_ts(uint8_t* p, int idx, uint32_t sec, uint32_t frac) {
  wr32(p, idx, sec);
  wr32(p, idx + 4, frac);
}

NtpServer::NtpServer() : sock(-1), port(0), gps(nullptr), requestCount(0), lastRxIrqConsumed(0), lastRxCapSeq(0), useWifi(false) {
}

esp_err_t NtpServer::begin(int port_, GpsDiscipline* gps_) {
  port = port_;
  gps = gps_;
  useWifi = Config::getNetworkWifi();

  if (useWifi) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
      ESP_LOGE(TAG, "WiFi UDP socket create failed");
      return ESP_FAIL;
    }
    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)port);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
      ESP_LOGE(TAG, "WiFi UDP bind failed on port %d", port);
      close(sock);
      sock = -1;
      return ESP_FAIL;
    }
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags >= 0) fcntl(sock, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGI(TAG, "NTP server listening on port %d (WiFi)", port);
    return ESP_OK;
  }

  sock = 1;
  if (w5k_udp_open((uint8_t)sock, (uint16_t)port) != 0) {
    ESP_LOGE(TAG, "W5500 UDP socket open failed on s=%d port=%d", sock, port);
    sock = -1;
    return ESP_FAIL;
  }
  w5k_set_nonblock((uint8_t)sock);
  setupRxInterrupt();
  ESP_LOGI(TAG, "NTP server listening on port %d (Wiznet sn=%d)", port, sock);
  return ESP_OK;
}

void NtpServer::reopenSocket() {
  if (useWifi || sock < 0) return;
  // The W5500's register file was rebuilt after a chip reset, which closed
  // every hardware socket. Reopen ours and re-arm the chip-side RECV
  // interrupt enable — the GPIO ISR itself survives.
  if (w5k_udp_open((uint8_t)sock, (uint16_t)port) != 0) {
    ESP_LOGE(TAG, "W5500 UDP socket reopen failed (s=%d port=%d)", sock, port);
    return;
  }
  w5k_set_nonblock((uint8_t)sock);
  if (Config::getW5500IntPin() >= 0) {
    w5k_enable_rx_irq((uint8_t)sock);
  }
  ESP_LOGI(TAG, "NTP W5500 socket reopened (sn=%d port=%d)", sock, port);
}

uint32_t NtpServer::getPrimeSkips() const { return s_primeSkips; }
uint32_t NtpServer::getCapRejects() const { return s_capRejects; }
uint32_t NtpServer::getRxIrqCount() const { return s_rxIrqCount; }
uint32_t NtpServer::getLateStampOk() const { return s_lateStampOk; }
uint32_t NtpServer::getLateStampFallbacks() const { return s_lateStampFallbacks; }
int NtpServer::getLastStageRc() const { return s_lastStageRc; }
int NtpServer::getLastWrDelta() const { return s_lastWrDelta; }
uint32_t NtpServer::getTurnSamples() const { return s_turnSamples; }
double NtpServer::getTurnUs() const { return s_turnTicks / 80.0; }
double NtpServer::getTxCorrectionUs() const { return s_txCorrectionUs; }

void NtpServer::setupRxInterrupt() {
  int pin = Config::getW5500IntPin();
  if (pin < 0) {
    ESP_LOGW(TAG, "No W5500 INT pin configured; RX timestamping stays poll-based");
    return;
  }
  // GPIO34-39 are input-only with no internal pulls; the W5500 INTn line needs
  // an external pull-up (present on the module). Trigger on the falling edge.
  gpio_config_t io = {};
  io.pin_bit_mask = (1ULL << pin);
  io.mode = GPIO_MODE_INPUT;
  io.intr_type = GPIO_INTR_NEGEDGE;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io);

  // gpio_install_isr_service is idempotent across the app; ignore "already installed".
  esp_err_t err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGW(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
    return;
  }
  gpio_isr_handler_add((gpio_num_t)pin, w5500_rx_isr, nullptr);

  // Enable the socket's RECV interrupt so INTn actually asserts on arrival.
  w5k_enable_sendok_irq((uint8_t)sock);
  ESP_LOGI(TAG, "W5500 RX+SENDOK capture armed on GPIO%d (sn=%d)", pin, sock);
}

void NtpServer::computeNtpTimestamp(uint64_t monoUs, bool locked, uint32_t& sec1900, uint32_t& frac) {
  if (locked && gps) {
    uint64_t lastPpsUs = gps->getLastPpsMonotonicUs();
    uint32_t ppsSec1900, ppsFrac;
    if (gps->getLastPps(ppsSec1900, ppsFrac)) {
      /* Signed: app_main runs gps->loop() before ntpServer->loop(), so a packet
       * stamped just before a pulse is regularly processed after that pulse
       * moved the anchor. The unsigned form wrapped to ~1.8e19 and produced a
       * random timestamp for roughly 1% of packets. */
      int64_t signedDelta = (int64_t)monoUs - (int64_t)lastPpsUs;
      if (signedDelta < 0) signedDelta = 0;
      uint64_t rawDelta = (uint64_t)signedDelta;
      int64_t freqCorr = (int64_t)(-gps->getFrequencyPpm() * 1e-6 * (double)rawDelta);
      uint64_t correctedDelta = rawDelta + freqCorr;
      sec1900 = ppsSec1900 + (uint32_t)(correctedDelta / 1000000ULL);
      frac = (uint32_t)(((correctedDelta % 1000000ULL) << 32) / 1000000ULL);
      return;
    }
  }
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  sec1900 = (uint32_t)((uint64_t)tv.tv_sec + 2208988800ULL);
  frac = 0;
}

/*
 * The W5500 resolves and remembers one destination per socket, so a prime is
 * only needed when the peer differs from the last one we primed. Priming is not
 * optional politeness: it forces ARP to complete *before* t3 is patched in, so
 * SEND egresses immediately after the stamp instead of stalling on resolution
 * and shipping a timestamp that is already stale. The TTL re-primes a
 * long-lived peer in case the chip has aged its entry out, which it gives us no
 * way to query.
 */
static uint8_t s_primedIp[4] = {0, 0, 0, 0};
static uint64_t s_primedUs = 0;
static const uint64_t kPrimeTtlUs = 10ULL * 1000000ULL;

static bool needsArpPrime(const uint8_t* ip) {
  uint64_t now = esp_timer_get_time();
  bool same = (s_primedIp[0] == ip[0] && s_primedIp[1] == ip[1] &&
               s_primedIp[2] == ip[2] && s_primedIp[3] == ip[3]);
  if (same && s_primedUs && (now - s_primedUs) < kPrimeTtlUs) {
    s_primeSkips++;
    return false;
  }
  s_primedIp[0] = ip[0]; s_primedIp[1] = ip[1];
  s_primedIp[2] = ip[2]; s_primedIp[3] = ip[3];
  s_primedUs = now;
  return true;
}

void NtpServer::loop() {
  if (sock < 0) return;

  static uint32_t windowSec = 0;
  static int windowCount = 0;

  /* Large enough for NTP plus a MAC (authenticated NTP is 68 bytes): reading
   * only 48 left the W5500 socket's remainder non-zero, so the next recvfrom
   * skipped the packet header and returned an uninitialised source address. */
  uint8_t req[128];
  bool haveRxCap = false;
  uint32_t rxCapTick = 0;
  uint8_t from_ip[4];
  uint16_t from_port = 0;
  struct sockaddr_in wifi_from = {};
  socklen_t wifi_fromlen = sizeof(wifi_from);
  int32_t n;
  uint64_t t2_us;
  if (useWifi) {
    n = recvfrom(sock, req, sizeof(req), 0, (struct sockaddr*)&wifi_from, &wifi_fromlen);
    // lwIP has already buffered the datagram; stamp after recvfrom returns.
    t2_us = esp_timer_get_time();
    if (n > 0) {
      uint32_t a = ntohl(wifi_from.sin_addr.s_addr);
      from_ip[0] = (a >> 24) & 0xff;
      from_ip[1] = (a >> 16) & 0xff;
      from_ip[2] = (a >> 8) & 0xff;
      from_ip[3] = a & 0xff;
      from_port = ntohs(wifi_from.sin_port);
    }
  } else {
    // W5500: a full NTP request is 48 bytes plus the 8-byte PACKET-INFO header
    // the chip prepends in the RX buffer. Detect arrival with a cheap RSR read
    // and stamp t2 *before* clocking the payload out over SPI, so the SPI read
    // duration no longer inflates the receive timestamp.
    if (w5k_rx_ready((uint8_t)sock) < 48 + 8) return;
    // Prefer the hardware-captured arrival edge (INTn ISR). If a fresh IRQ has
    // fired since the last packet we handled, use its latched timestamp — this
    // removes the poll-loop quantization from t2. Otherwise (e.g. a second
    // datagram queued behind one INTn assertion) fall back to the poll instant.
    // Best: the MCPWM-latched arrival tick, in the same 80 MHz counter as
    // PPS — no ISR latency, 12.5 ns resolution, converted straight through the
    // PPS fit. Falls back to the GPIO-ISR stamp, then to the poll instant.
    uint32_t capTick, capSeq;
    if (gps && gps->getRxCapture(capTick, capSeq) && capSeq != lastRxCapSeq) {
      lastRxCapSeq = capSeq;
      haveRxCap = true;
      rxCapTick = capTick;
    } else {
      haveRxCap = false;
    }
    /* Read the timestamp BEFORE the counter: the original order let an ISR
     * landing in between hand packet N the stamp belonging to packet N+1. */
    uint64_t isrStamp = 0;
    bool haveIsrStamp = rx_isr_stamp(&isrStamp);
    uint32_t irqc = s_rxIrqCount;
    if (haveIsrStamp && irqc != lastRxIrqConsumed) {
      t2_us = isrStamp;
      lastRxIrqConsumed = irqc;
    } else {
      t2_us = esp_timer_get_time();
    }
    n = w5k_recvfrom((uint8_t)sock, req, sizeof(req), from_ip, &from_port);
    // Ack the RECV interrupt so INTn de-asserts and the next arrival re-fires.
    w5k_clear_rx_irq((uint8_t)sock);
  }

  if (n <= 0) return;
  if (n < 48) {
    ESP_LOGW(TAG, "Short NTP request (%d bytes) from %d.%d.%d.%d", n, from_ip[0], from_ip[1], from_ip[2], from_ip[3]);
    return;
  }

  // Only answer client requests (mode 3; mode 0 for ancient v3 hosts).
  // Replying to anything else — e.g. a spoofed mode-4 server response —
  // invites reflection loops between servers.
  uint8_t reqMode = req[0] & 0x07;
  if (reqMode != 3 && reqMode != 0) {
    ESP_LOGD(TAG, "Ignoring mode-%d packet from %d.%d.%d.%d", reqMode, from_ip[0], from_ip[1], from_ip[2], from_ip[3]);
    return;
  }

  bool locked = (gps && gps->isLocked());

  uint8_t rsp[48];
  memset(rsp, 0, sizeof(rsp));

  uint8_t clientVersion = (req[0] >> 3) & 0x7;
  if (clientVersion < 3) clientVersion = 3;
  if (clientVersion > 4) clientVersion = 4;

  uint8_t li = locked ? 0 : 3;
  rsp[0] = (li << 6) | (clientVersion << 3) | 4;
  rsp[1] = locked ? 1 : 16;
  rsp[2] = 6;
  rsp[3] = 0xEC;  // precision: 2^-20 (~1µs)

  auto to_fixed_16_16 = [](double secs) -> uint32_t {
    if (secs < 0) secs = 0;
    double x = secs * 65536.0;
    if (x > 4294967295.0) x = 4294967295.0;
    return (uint32_t)(x + 0.5);
  };
  double rootDelay = (locked && gps) ? gps->getRootDelay() : 1.0;
  double dispersion = (locked && gps) ? gps->getRootDispersion() : 1.0;
  wr32(rsp, 4, to_fixed_16_16(rootDelay));
  wr32(rsp, 8, to_fixed_16_16(dispersion));

  if (locked) {
    rsp[12] = 'P'; rsp[13] = 'P'; rsp[14] = 'S'; rsp[15] = 0;
  }

  uint32_t refSec1900 = 0, refFrac = 0;
  if (locked && gps && gps->getLastPps(refSec1900, refFrac)) {
    wr_ntp_ts(rsp, 16, refSec1900, refFrac);
  }

  // Originate timestamp (client's transmit time)
  memcpy(&rsp[24], &req[40], 8);

  // Receive timestamp (t2)
  uint32_t t2_sec, t2_frac;
  bool usedCapture = false;
  if (haveRxCap && locked && gps &&
      gps->captureToNtp(rxCapTick, t2_sec, t2_frac)) {
    /*
     * Cross-check the hardware timestamp against the poll instant before
     * trusting it. Nothing structurally ties a capture to the datagram
     * w5k_recvfrom() returns: a runt packet that hit the rx_ready early-return
     * leaves its capture unconsumed, and an arrival racing the rx_ready read
     * can advance the counter so the older queued datagram gets the newer tick.
     * Both show up as a capture that disagrees with when we actually looked.
     */
    uint32_t poll_sec, poll_frac;
    computeNtpTimestamp(t2_us, locked, poll_sec, poll_frac);
    int32_t dsec = (int32_t)(t2_sec - poll_sec);
    double ddiff = (double)dsec +
                   ((double)t2_frac - (double)poll_frac) / 4294967296.0;
    if (ddiff > -0.002 && ddiff < 0.002) {
      usedCapture = true;
    } else {
      s_capRejects++;
    }
  }
  if (!usedCapture)
    computeNtpTimestamp(t2_us, locked, t2_sec, t2_frac);
  /* Apply the served-time calibration to t2 (and below to t3): compensates the
   * W5500's receive-store latency, which otherwise reads as clock error. */
  ntp_ts_sub_us(t2_sec, t2_frac, Config::getServeCalibrationUs());
  wr_ntp_ts(rsp, 32, t2_sec, t2_frac);

  // KoD rate limiting, per client. A single global counter meant one chatty
  // host RATE-limited every other client on the LAN; buckets are keyed by a
  // hash of the source address so an abusive client only silences itself.
  // 64 buckets is plenty for a home LAN and collisions merely share a budget.
  uint32_t currentSec = t2_sec - 2208988800ULL;
  /* Initialised to a second that cannot occur so the first packet in each
   * bucket resets rather than accumulating (a zero-initialised array collides
   * with a legitimate currentSec of 0). */
  static uint32_t bucketSec[64];
  static uint32_t bucketCount[64];
  /* Marks a bucket whose peer we have already ARP-resolved, so an over-budget
   * reply can be sent without paying for resolution. */
  static bool bucketWarm[64];
  static bool bucketInit = false;
  if (!bucketInit) {
    /* A zero-initialised bucketSec collides with a legitimate currentSec of 0,
     * which would leave the window never resetting. */
    for (int i = 0; i < 64; i++) bucketSec[i] = 0xFFFFFFFFu;
    bucketInit = true;
  }
  uint32_t cip = ((uint32_t)from_ip[0] << 24) | ((uint32_t)from_ip[1] << 16) |
                 ((uint32_t)from_ip[2] << 8) | (uint32_t)from_ip[3];
  uint32_t h = cip * 2654435761u;          // Knuth multiplicative
  uint32_t b = (h >> 26) & 63;
  if (bucketSec[b] != currentSec) {
    bucketSec[b] = currentSec;
    bucketCount[b] = 0;
  }
  bucketCount[b]++;
  // Also keep a global ceiling so a distributed flood still can't monopolise
  // the single core; well above any legitimate aggregate.
  if (currentSec != windowSec) {
    windowSec = currentSec;
    windowCount = 0;
  }
  windowCount++;
  bool overBudget = (bucketCount[b] > 20 || windowCount > 200);
  if (overBudget) {
    /*
     * Kiss-o'-Death. LI MUST be 3 (unsynchronised) *together with* stratum 0:
     * chrony and ntpd both require that conjunction before they even look at
     * the refid, so a KoD sent with LI=0 is discarded as unsynchronised and the
     * client never backs off. The origin timestamp must stay filled or the KoD
     * is rejected as spoofed.
     */
    rsp[0] = (uint8_t)((3 << 6) | (clientVersion << 3) | 4);
    rsp[1] = 0;
    rsp[12] = 'R'; rsp[13] = 'A'; rsp[14] = 'T'; rsp[15] = 'E';
    /* Root delay/dispersion are meaningless at stratum 0. */
    wr32(rsp, 4, 0);
    wr32(rsp, 8, 0);
  }

  if (useWifi) {
    // WiFi: stamp t3 as late as possible, then send
    uint32_t t3_sec, t3_frac;
    computeNtpTimestamp(esp_timer_get_time(), locked, t3_sec, t3_frac);
    wr_ntp_ts(rsp, 40, t3_sec, t3_frac);
    if (sendto(sock, rsp, 48, 0, (struct sockaddr*)&wifi_from, wifi_fromlen) == 48) {
      requestCount++;
      ESP_LOGD(TAG, "Replied to %d.%d.%d.%d:%u (stratum %d, LI=%d)",
               from_ip[0], from_ip[1], from_ip[2], from_ip[3], (unsigned)from_port, rsp[1], li);
    }
    return;
  }

  // W5500: prime ARP first. This is load-bearing, not just a timing nicety — the
  // chip's UDP send only completes (and advances Sn_TX_RD) once the destination
  // MAC is resolved. Skipping it leaves Sn_TX_RD stuck while Sn_TX_WR grows, so
  // every reply becomes an oversized/garbage packet. arp_prime blocks until the
  // ARP is warm, after which the real send departs promptly.
  // If ARP can't resolve (spoofed or unroutable source), drop the reply — the
  // real send would stall the same way and the answer would go nowhere anyway.
  /*
   * Over budget: answer only if ARP is already warm, never pay for resolution.
   * w5k_arp_prime() deliberately blocks up to ~80 ms on an unresolvable
   * address, so without this a mere dozen spoofed-source packets per second
   * consumed the whole main loop — the same task that runs the PPS discipline.
   * Rate limiting that still does the expensive work is not rate limiting.
   */
  if (overBudget) {
    /* The W5500 exposes no way to query its ARP cache, so rely on having
     * resolved this peer earlier: a real client that is now over budget was
     * warmed by its own first request, while a spoofed source never was. */
    if (!bucketWarm[b]) return;
  } else if (needsArpPrime(from_ip) &&
             w5k_arp_prime((uint8_t)sock, from_ip) != 0) {
    ESP_LOGW(TAG, "ARP unresolved for %d.%d.%d.%d — dropping reply",
             from_ip[0], from_ip[1], from_ip[2], from_ip[3]);
    return;
  }

  /*
   * Single-shot send. Two attempts at stamping t3 later than this both failed
   * on hardware and are recorded in REVIEW.md: splitting wiz_send_data() emits
   * 8-byte frames (the Sn_TX_WR advance does not survive between calls), and
   * staging-then-patching the timestamp in place also stopped the server
   * answering. Until the cause is instrumented properly, correctness wins.
   */
  uint32_t t3_sec, t3_frac;
  computeNtpTimestamp(esp_timer_get_time() + (uint64_t)(s_txCorrectionUs + 0.5),
                      locked, t3_sec, t3_frac);
  wr_ntp_ts(rsp, 40, t3_sec, t3_frac);

  /*
   * Late-stamped send with verification and fallback.
   *
   * Goal: get t3 as close to the wire as possible. Preferred path stages the
   * whole 48-byte datagram in ONE wiz_send_data() (so there is no Sn_TX_WR
   * advance to lose between calls), then patches the 8 t3 bytes in place at
   * their offset in the socket TX buffer and issues SEND — only those 8 bytes
   * and the command separate the timestamp from transmission.
   *
   * Every step is verified. If staging does not advance Sn_TX_WR by exactly
   * the frame length, the frame is NOT transmitted from that state; the code
   * falls back to the plain library send, which is known good. An earlier
   * unverified split emitted 8-byte frames to every peer, so correctness here
   * is enforced rather than assumed.
   */
  uint8_t tail[8];
  uint16_t stampBase = 0;
  int32_t sret = -1;
  uint64_t txStart = 0, txEnd = 0;
  bool late = false;

  int wrDelta = -1;
  int stageRc = w5k_send_stage((uint8_t)sock, rsp, sizeof(rsp), from_ip,
                               from_port, &stampBase, &wrDelta);
  s_lastWrDelta = wrDelta;
  if (stageRc != 0) s_lastStageRc = stageRc;
  if (stageRc == 0) {
    /*
     * Preferred: place t3 at the PREDICTED egress instant, expressed in the
     * same hardware capture timebase as t2 — arrival tick plus the turnaround
     * measured from previous packets' SENDOK captures. This removes software
     * timing from t3 entirely and makes (t3 - t2) a hardware interval.
     */
    /*
     * t3 stays on the esp_timer path. Deriving it from the SENDOK-capture
     * turnaround was tried and made served time WORSE (offset +8 -> +124 us,
     * stddev 32 -> 166 us) because the interval measured 3042 us against a real
     * turnaround of ~250 us: the capture being read is not reliably this
     * packet's SENDOK edge. INTn is level-asserted and shared with RECV, so
     * pairing an edge to a specific packet needs more than a counter delta.
     * The turnaround is still measured and exported as a diagnostic.
     */
    computeNtpTimestamp(esp_timer_get_time() + (uint64_t)(s_txCorrectionUs + 0.5),
                        locked, t3_sec, t3_frac);
    ntp_ts_sub_us(t3_sec, t3_frac, Config::getServeCalibrationUs());
    wr_ntp_ts(tail, 0, t3_sec, t3_frac);
    uint32_t capBefore = 0, capTmp = 0;
    if (gps) gps->getRxCapture(capTmp, capBefore);
    txStart = esp_timer_get_time();
    int rc = w5k_send_stamp_and_fire((uint8_t)sock, (uint16_t)(stampBase + 40),
                                     tail, sizeof(tail));
    /* Stop timing HERE: the frame is on the wire once SEND is accepted. The
     * SENDOK reap below happens after egress and must not inflate the t3
     * pre-correction. */
    txEnd = esp_timer_get_time();
    if (rc == 0) rc = w5k_send_reap((uint8_t)sock);
    /*
     * SENDOK has now fired, so its INTn edge should be latched. Accept the
     * sample only if exactly one new capture appeared (an interleaved arrival
     * would keep INTn low and swallow the SENDOK edge, or add a second one).
     */
    if (rc == 0 && haveRxCap && gps) {
      uint32_t egressTick = 0, capAfter = 0;
      if (gps->getRxCapture(egressTick, capAfter) &&
          (uint32_t)(capAfter - capBefore) == 2 &&
          w5k_rx_ready((uint8_t)sock) == 0) {
        double turn = (double)(int32_t)(egressTick - rxCapTick);
        /* 80 MHz: a plausible turnaround is tens to hundreds of us. */
        if (turn > 800.0 && turn < 80000.0 * 1000.0) {
          if (!s_turnSeeded) { s_turnTicks = turn; s_turnSeeded = true; }
          else s_turnTicks += 0.05 * (turn - s_turnTicks);
          s_turnSamples++;
        }
      }
    }
    if (rc == 0) { sret = (int32_t)sizeof(rsp); late = true; }
  }

  if (!late) {
    /* Fallback: stamp t3 then hand the whole frame to the library. */
    s_lateStampFallbacks++;
    computeNtpTimestamp(esp_timer_get_time() + (uint64_t)(s_txCorrectionUs + 0.5),
                        locked, t3_sec, t3_frac);
    wr_ntp_ts(rsp, 40, t3_sec, t3_frac);
    txStart = esp_timer_get_time();
    sret = w5k_sendto((uint8_t)sock, rsp, sizeof(rsp), from_ip, from_port);
    txEnd = esp_timer_get_time();
  } else {
    s_lateStampOk++;
  }
  int commitRc = (sret == (int32_t)sizeof(rsp)) ? 0 : -1;
  if (commitRc == 0 && !overBudget) bucketWarm[b] = true;
  if (commitRc != 0) {
    /* An unacknowledged send leaves its bytes in the TX buffer with Sn_TX_RD
     * not advanced, so the space is gone until the socket is reopened. */
    ESP_LOGW(TAG, "W5500 send did not complete — reopening socket");
    reopenSocket();
    return;
  }
  if (sret == (int32_t)sizeof(rsp)) {
    requestCount++;
    // EWMA of actual send duration → next packet's t3 pre-correction.
    double txDur = (double)(txEnd - txStart);
    if (!s_txCorrectionSeeded) {
      s_txCorrectionUs = txDur;
      s_txCorrectionSeeded = true;
    } else {
      s_txCorrectionUs += 0.05 * (txDur - s_txCorrectionUs);
    }
    ESP_LOGD(TAG, "Replied to %d.%d.%d.%d:%u (stratum %d, LI=%d)",
             from_ip[0], from_ip[1], from_ip[2], from_ip[3], (unsigned)from_port, rsp[1], li);
  } else {
    ESP_LOGW(TAG, "W5500 sendto failed");
  }
}

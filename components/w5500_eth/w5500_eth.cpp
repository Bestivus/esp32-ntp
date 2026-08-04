// SPDX-License-Identifier: Unlicense

#include "w5500_eth.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "hal/spi_ll.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static bool parse_ip4(const char* str, uint8_t* out) {
  unsigned a, b, c, d;
  if (sscanf(str, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
  if (a > 255 || b > 255 || c > 255 || d > 255) return false;
  out[0] = (uint8_t)a;
  out[1] = (uint8_t)b;
  out[2] = (uint8_t)c;
  out[3] = (uint8_t)d;
  return true;
}

extern "C" {
#include "wizchip_conf.h"
#include "W5500/w5500.h"
#include "dhcp.h"
}

static const char* TAG = "W5500";

static spi_device_handle_t g_spi_handle = nullptr;
static int g_cs_pin = -1;
static int g_rst_pin = -1;

static const uint8_t DHCP_SOCKET_NUM = 0;
static uint8_t s_dhcp_buf[548];

/*
 * Coalescing SPI shim.
 *
 * A W5500 SPI frame is [addr_hi][addr_lo][control][data...] with
 * auto-incrementing addresses in VDM (OM=00), so ONE transaction of 3+N bytes
 * performs an entire access. The ioLibrary, however, emits the three header
 * bytes through single-byte callbacks, so a plain register read cost four
 * separate spi_device_polling_transmit() calls — and each call's fixed
 * overhead dwarfs the ~0.4 us of actual clocking at 20 MHz. That per-call
 * tax, not SPI bandwidth, is what made a 48-byte UDP send take ~700 us.
 *
 * CS is a manual GPIO here (spics_io_num = -1) and is held low across the
 * whole frame, so bytes can be accumulated between select and deselect and
 * issued as one transaction. Within a frame the W5500 protocol is either read
 * or write (the RWB bit says which), never both, so buffering cannot reorder
 * anything. Reads are full-duplex: the accumulated header is clocked out while
 * the payload is clocked in.
 *
 * Falls back to per-call transactions if a frame ever exceeds the buffer.
 */
/* 3-byte header + 2048-byte socket buffer = 2051 max; rounded up and word
 * aligned because with DMA enabled the driver may write up to 3 bytes past an
 * unaligned RX length. */
#define W5K_FRAME_MAX 2064
static WORD_ALIGNED_ATTR uint8_t s_frame_tx[W5K_FRAME_MAX + 4];
static WORD_ALIGNED_ATTR uint8_t s_frame_rx[W5K_FRAME_MAX + 4];
static uint16_t s_frame_len = 0;    /* bytes accumulated, not yet on the wire */

/*
 * Attribution counters. Every microsecond in the reply path is either SPI
 * clocking (bytes / clock rate) or per-call fixed overhead (transactions), so
 * counting both lets `bytes*t_byte + txns*t_txn` be checked against the
 * measured span instead of guessed at. Plain increments on a single-owner path.
 */
extern "C" {
volatile uint32_t g_w5k_txns  = 0;   /* spi_device_polling_transmit() calls */
volatile uint32_t g_w5k_bytes = 0;   /* bytes clocked, header included */
volatile uint32_t g_w5k_sels  = 0;   /* wizchip_select() calls (bus acquires) */
}

static void w5k_flush_write(void) {
  if (s_frame_len == 0 || !g_spi_handle) { s_frame_len = 0; return; }
  spi_transaction_t t = {};
  t.length = (size_t)s_frame_len * 8;
  t.tx_buffer = s_frame_tx;
  g_w5k_txns++; g_w5k_bytes += s_frame_len;
  spi_device_polling_transmit(g_spi_handle, &t);
  s_frame_len = 0;
}

/* Clock out whatever header has accumulated and read len bytes in the same
 * transaction. */
static void w5k_flush_read(uint8_t* out, uint16_t len) {
  if (!g_spi_handle || len == 0) return;
  uint16_t total = s_frame_len + len;
  if (total > W5K_FRAME_MAX) {
    /* Unreachable: the largest real frame is 3 + Sn_TxMAX(2048) = 2051. Fail
     * loudly rather than hand a caller's unaligned buffer to DMA. */
    ESP_LOGE(TAG, "W5500 frame %u exceeds buffer — read dropped", total);
    s_frame_len = 0;
    memset(out, 0, len);
    return;
  }
  memset(&s_frame_tx[s_frame_len], 0xFF, len);
  spi_transaction_t t = {};
  t.length = (size_t)total * 8;
  t.rxlength = (size_t)total * 8;
  t.tx_buffer = s_frame_tx;
  t.rx_buffer = s_frame_rx;
  g_w5k_txns++; g_w5k_bytes += total;
  spi_device_polling_transmit(g_spi_handle, &t);
  memcpy(out, &s_frame_rx[s_frame_len], len);
  s_frame_len = 0;
}

/*
 * The bus lock is taken ONCE, at init, and never released.
 *
 * spi_device_acquire_bus()/release_bus() were being called around every
 * register access — about 60 times per NTP reply — and each pair costs a few
 * microseconds of driver bookkeeping (semaphore, device selection, hardware
 * reconfiguration check), which measured as a double-digit fraction of the
 * whole reply path. Holding the lock is legal here because the W5500 is the
 * ONLY device on SPI2_HOST (the display lives on SPI3, see config.cpp) and the
 * NTP task is its sole owner by design (see the comment in app_main.cpp's
 * ntp_task explaining why housekeeping shares that task rather than running
 * concurrently). CS is still toggled per access, so the framing the W5500 sees
 * is unchanged.
 */
static bool s_bus_held = false;

static void w5k_bus_hold(void) {
  if (g_spi_handle && !s_bus_held &&
      spi_device_acquire_bus(g_spi_handle, portMAX_DELAY) == ESP_OK) {
    s_bus_held = true;
  }
}

/*
 * Bespoke single-transaction accessors for the NTP reply path (see
 * w5500_fast.h). Separate buffers from the shim's, so a fast access can never
 * be issued in the middle of a library frame that is still accumulating.
 */
static WORD_ALIGNED_ATTR uint8_t s_fast_tx[3 + 64 + 4];
static WORD_ALIGNED_ATTR uint8_t s_fast_rx[3 + 64 + 4];
/* The peripheral's own data buffer is 64 bytes and that is the transfer unit
 * below, so a whole access — 3-byte header plus payload — must fit in it. The
 * largest real access is the 56-byte RX drain (3 + 8 + 48 = 59). */
#define W5K_FAST_MAX 61

/*
 * The reply path drives SPI2 directly instead of going through
 * spi_device_polling_transmit().
 *
 * Measured with the bus pre-acquired and every access already collapsed to one
 * transaction, a transaction still cost ~17 us of pure overhead against ~0.4
 * us/byte of clocking — and 25 of them per reply is the entire remaining budget.
 * None of that overhead is DMA (disabling DMA changed nothing, measurably): it
 * is the driver re-deriving per-transaction state that never varies here. Every
 * call re-runs line-mode, dummy-bit, MISO-delay, command/address-length and
 * keep-CS configuration, rebuilds DMA descriptors and resets the DMA channel,
 * then memcpy's a config struct — for a transfer whose only variables are the
 * bit length and the bytes.
 *
 * So: let the driver configure the peripheral once (it already does, during
 * wizchip_init), then for the hot path write the length, load the peripheral's
 * data buffer, and start. Everything else is left exactly as the driver set it,
 * which is why the library path can keep using the driver on the same device.
 *
 * Safe here because the NTP task owns this bus by design (see ntp_task in
 * app_main.cpp) and because spi_device_acquire_bus(), which is held for the
 * lifetime of the device, disables the driver's interrupt — so no driver
 * transaction can be in flight underneath us. The DMA links are explicitly
 * stopped and the DMA FIFOs reset before each transfer, so a preceding
 * driver-issued DMA transaction cannot leave the peripheral sourcing data from
 * anywhere but the buffer we just filled.
 */
/*
 * Set to 0 to route the reply path back through spi_device_polling_transmit()
 * without touching anything else — the escape hatch if a future IDF changes what
 * the driver leaves configured in the peripheral.
 *
 * Measured cost of turning it off, everything else held fixed (60-sample runs
 * from a GPS-locked reference, medians): served t3-t2 279 -> 300 us, and the SPI
 * spans rx_ready 9.3 -> 17.2, recvfrom 76.8 -> 102.5, stage 54.8 -> 74.4,
 * stamp_send 16.5 -> 27.2 us. The ~20 us end-to-end is the smaller half of why
 * this is kept: stamp_send IS the window between t3 being written and the frame
 * being handed to the wire, and 16 us instead of 27 us is an accuracy win as
 * much as a latency one.
 */
#define W5K_DIRECT_SPI 1

static spi_dev_t* s_hw = nullptr;
static uint32_t s_cs_mask = 0;      /* nonzero only for a CS pin below 32 */

static inline void IRAM_ATTR w5k_cs_low(void) {
  if (s_cs_mask) GPIO.out_w1tc = s_cs_mask;
  else if (g_cs_pin >= 0) gpio_set_level((gpio_num_t)g_cs_pin, 0);
}
static inline void IRAM_ATTR w5k_cs_high(void) {
  if (s_cs_mask) GPIO.out_w1ts = s_cs_mask;
  else if (g_cs_pin >= 0) gpio_set_level((gpio_num_t)g_cs_pin, 1);
}

/* Clock `total` bytes out of s_fast_tx; if `want_rx`, capture them into
 * s_fast_rx. Full duplex, which is what the shim already relies on. */
static void IRAM_ATTR w5k_fifo_xfer(uint16_t total, bool want_rx) {
  const size_t bits = (size_t)total * 8;
  g_w5k_txns++; g_w5k_bytes += total; g_w5k_sels++;

#if !W5K_DIRECT_SPI
  spi_transaction_t t = {};
  t.length = bits;
  t.tx_buffer = s_fast_tx;
  if (want_rx) { t.rxlength = bits; t.rx_buffer = s_fast_rx; }
  spi_device_polling_transmit(g_spi_handle, &t);
  return;
#else
  spi_dev_t* hw = s_hw;

  /* Make sure nothing is still driving the peripheral from DMA. */
  hw->dma_out_link.start = 0;
  hw->dma_in_link.start = 0;
  hw->dma_conf.val |= SPI_LL_DMA_FIFO_RST_MASK;
  hw->dma_conf.val &= ~SPI_LL_DMA_FIFO_RST_MASK;

  hw->user.usr_mosi_highpart = 0;
  hw->user.usr_miso_highpart = 0;
  spi_ll_enable_mosi(hw, 1);
  spi_ll_enable_miso(hw, want_rx ? 1 : 0);
  spi_ll_set_mosi_bitlen(hw, bits);
  spi_ll_set_miso_bitlen(hw, bits);
  spi_ll_write_buffer(hw, s_fast_tx, bits);

  spi_ll_clear_int_stat(hw);
  spi_ll_user_start(hw);
  while (!spi_ll_usr_is_done(hw)) { }

  if (want_rx) spi_ll_read_buffer(hw, s_fast_rx, bits);
#endif
}

extern "C" void IRAM_ATTR w5k_xfer_wr(uint32_t addrsel, const uint8_t* buf, uint16_t len) {
  if (!g_spi_handle || len == 0 || len > W5K_FAST_MAX) return;
  s_fast_tx[0] = (uint8_t)((addrsel & 0x00FF0000) >> 16);
  s_fast_tx[1] = (uint8_t)((addrsel & 0x0000FF00) >> 8);
  /* Keep the block select (bits 7:3), set RWB=1 (write), OM=00 (VDM). */
  s_fast_tx[2] = (uint8_t)((addrsel & 0x000000F8) | 0x04);
  memcpy(&s_fast_tx[3], buf, len);
  w5k_cs_low();
  w5k_fifo_xfer((uint16_t)(len + 3), false);
  w5k_cs_high();
}

extern "C" void IRAM_ATTR w5k_xfer_rd(uint32_t addrsel, uint8_t* buf, uint16_t len) {
  if (!g_spi_handle || len == 0 || len > W5K_FAST_MAX) {
    if (buf && len) memset(buf, 0, len);
    return;
  }
  s_fast_tx[0] = (uint8_t)((addrsel & 0x00FF0000) >> 16);
  s_fast_tx[1] = (uint8_t)((addrsel & 0x0000FF00) >> 8);
  /* RWB=0 (read), OM=00 (VDM). */
  s_fast_tx[2] = (uint8_t)(addrsel & 0x000000F8);
  memset(&s_fast_tx[3], 0xFF, len);
  w5k_cs_low();
  w5k_fifo_xfer((uint16_t)(len + 3), true);
  w5k_cs_high();
  memcpy(buf, &s_fast_rx[3], len);
}

static void wizchip_select(void) {
  g_w5k_sels++;
  if (g_cs_pin >= 0) {
    gpio_set_level((gpio_num_t)g_cs_pin, 0);
  }
  s_frame_len = 0;
}

static void wizchip_deselect(void) {
  w5k_flush_write();                 /* anything still buffered goes now */
  if (g_cs_pin >= 0) {
    gpio_set_level((gpio_num_t)g_cs_pin, 1);
  }
}

static uint8_t wizchip_read(void) {
  uint8_t rx_data = 0;
  w5k_flush_read(&rx_data, 1);
  return rx_data;
}

static void wizchip_write(uint8_t wb) {
  if (s_frame_len < W5K_FRAME_MAX) {
    s_frame_tx[s_frame_len++] = wb;
  } else {
    w5k_flush_write();
    s_frame_tx[s_frame_len++] = wb;
  }
}

static void wizchip_readburst(uint8_t* pBuf, uint16_t len) {
  if (pBuf && len > 0) w5k_flush_read(pBuf, len);
}

static void wizchip_writeburst(uint8_t* pBuf, uint16_t len) {
  if (!pBuf || len == 0) return;
  if (s_frame_len + len > W5K_FRAME_MAX) {
    w5k_flush_write();
    if (len > W5K_FRAME_MAX) {          /* oversized: straight to the wire */
      spi_transaction_t t = {};
      t.length = (size_t)len * 8;
      t.tx_buffer = pBuf;
      g_w5k_txns++; g_w5k_bytes += len;
      if (g_spi_handle) spi_device_polling_transmit(g_spi_handle, &t);
      return;
    }
  }
  memcpy(&s_frame_tx[s_frame_len], pBuf, len);
  s_frame_len += len;
  /*
   * CONTROL TEST / correctness guard: flush this accessor's bytes immediately
   * rather than accumulating until CS is deselected. CS is a manual GPIO held
   * low for the whole frame, so per the W5500 datasheet's variable-length data
   * mode (the data phase is delimited by CS, not by transaction boundaries)
   * splitting a frame across transactions is legal. Deferring writes until
   * deselect made a direct wiz_send_data() a no-op — Sn_TX_WR and Sn_TX_FSR
   * both showed zero change — while the library's own send path still worked.
   */
  w5k_flush_write();
}


W5500Eth::W5500Eth()
  : eth_netif(nullptr), linkUp(false), useDhcp(false), intPin(-1), rstPin(-1),
    expectedMac{}, haveStatic(false), staticIp4{}, staticGw4{}, staticSn4{},
    chipResetCount(0), chipResetCb(nullptr), chipResetCbArg(nullptr) {
}

W5500Eth::~W5500Eth() {
  stop();
}

esp_err_t W5500Eth::begin(spi_host_device_t spiHost, int mosiPin, int misoPin, int sclkPin, int csPin, int intPin_, int rstPin_, int clockHz) {
  ESP_LOGI(TAG, "Initializing W5500 Ethernet...");
  
  intPin = intPin_;
  rstPin = rstPin_;
  g_rst_pin = rstPin_;
  g_cs_pin = csPin;
  
  gpio_config_t io_conf = {};
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask = (1ULL << csPin);
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)csPin, 1);
  /* Direct CS toggling on the reply path needs GPIO_OUT, which only covers
   * pins 0-31; anything higher falls back to gpio_set_level(). */
  s_cs_mask = (csPin >= 0 && csPin < 32) ? (1u << csPin) : 0u;

  if (rstPin >= 0) {
    io_conf.pin_bit_mask = (1ULL << rstPin);
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)rstPin, 1);
  }
  
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = mosiPin;
  buscfg.miso_io_num = misoPin;
  buscfg.sclk_io_num = sclkPin;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;
  buscfg.max_transfer_sz = 4092;
  
  esp_err_t ret = spi_bus_initialize(spiHost, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
    return ret;
  }
  
  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = clockHz;  // honor caller's requested SPI clock
  devcfg.mode = 0;
  devcfg.spics_io_num = -1;
  devcfg.queue_size = 7;
  devcfg.command_bits = 0;
  devcfg.address_bits = 0;
  
  ret = spi_bus_add_device(spiHost, &devcfg, &g_spi_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
    spi_bus_free(spiHost);
    return ret;
  }
  
  w5k_bus_hold();

  if (rstPin >= 0) {
    ESP_LOGI(TAG, "Resetting W5500...");
    gpio_set_level((gpio_num_t)rstPin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level((gpio_num_t)rstPin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
  } else {
    ESP_LOGW(TAG, "No reset pin configured - recommend connecting RST pin");
  }
  
  reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
  reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
  reg_wizchip_spiburst_cbfunc(wizchip_readburst, wizchip_writeburst);
  
  uint8_t memsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
  if (wizchip_init(memsize, memsize) < 0) {
    ESP_LOGE(TAG, "W5500 initialization failed");
    spi_bus_remove_device(g_spi_handle);
    spi_bus_free(spiHost);
    g_spi_handle = nullptr;
    return ESP_FAIL;
  }
  
  uint8_t version = 0;
  for (int i = 0; i < 3; ++i) {
    version = getVERSIONR();
    if (version == 0x04) break;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  ESP_LOGI(TAG, "W5500 version: 0x%02x (expected 0x04)", version);

  /* Only now arm the direct-register reply path: wizchip_init() and the version
   * reads above have gone through the driver, so the peripheral's per-transaction
   * configuration (line mode, dummy bits, MISO delay, cmd/addr lengths, keep-CS)
   * is established and the fast path can inherit it. */
  s_hw = SPI_LL_GET_HW(spiHost);
  ESP_LOGI(TAG, "W5500 reply path on direct SPI%d FIFO access", (int)spiHost + 1);
  
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  mac[0] = 0x02;
  memcpy(expectedMac, mac, 6);

  wiz_NetInfo netinfo = {};
  memcpy(netinfo.mac, mac, 6);
  netinfo.dhcp = NETINFO_DHCP;
  wizchip_setnetinfo(&netinfo);
  
  wiz_PhyConf phyconf = {};
  phyconf.by = PHY_CONFBY_SW;
  phyconf.mode = PHY_MODE_AUTONEGO;
  phyconf.speed = PHY_SPEED_100;
  phyconf.duplex = PHY_DUPLEX_FULL;
  wizphy_setphyconf(&phyconf);
  
  uint8_t phycfgr = getPHYCFGR();
  ESP_LOGI(TAG, "PHYCFGR: 0x%02x (Link:%d Speed:%d Duplex:%d)", 
           phycfgr, 
           (phycfgr & 0x01) ? 1 : 0,
           (phycfgr & 0x02) ? 100 : 10,
           (phycfgr & 0x04) ? 1 : 0);
  
  ESP_LOGI(TAG, "W5500 initialized successfully");
  return ESP_OK;
}

esp_err_t W5500Eth::start(bool use_static_ip,
                         const char* static_ip,
                         const char* static_gw,
                         const char* static_netmask) {
  ESP_LOGI(TAG, "Starting Ethernet...");

  wiz_NetInfo netinfo = {};
  getSHAR(netinfo.mac);

  // Apply static config before waiting for link — the W5500 registers don't
  // need a live PHY, so a boot with the cable unplugged still ends up usable.
  if (use_static_ip && static_ip && static_gw && static_netmask) {
    uint8_t ip[4], gw[4], sn[4];
    if (parse_ip4(static_ip, ip) && parse_ip4(static_gw, gw) && parse_ip4(static_netmask, sn)) {
      memcpy(netinfo.ip, ip, 4);
      memcpy(netinfo.gw, gw, 4);
      memcpy(netinfo.sn, sn, 4);
      netinfo.dns[0] = netinfo.dns[1] = netinfo.dns[2] = netinfo.dns[3] = 0;
      netinfo.dhcp = NETINFO_STATIC;
      wizchip_setnetinfo(&netinfo);
      wizchip_getnetinfo(&netinfo);
      haveStatic = true;
      memcpy(staticIp4, ip, 4);
      memcpy(staticGw4, gw, 4);
      memcpy(staticSn4, sn, 4);
      ESP_LOGI(TAG, "Static IP: %d.%d.%d.%d  GW: %d.%d.%d.%d  SN: %d.%d.%d.%d",
               netinfo.ip[0], netinfo.ip[1], netinfo.ip[2], netinfo.ip[3],
               netinfo.gw[0], netinfo.gw[1], netinfo.gw[2], netinfo.gw[3],
               netinfo.sn[0], netinfo.sn[1], netinfo.sn[2], netinfo.sn[3]);
    } else {
      ESP_LOGE(TAG, "Invalid static IP/gw/netmask, falling back to DHCP");
      use_static_ip = false;
    }
  }

  useDhcp = !use_static_ip;

  int retry = 0;
  while (retry < 50) {
    if (wizphy_getphylink() == PHY_LINK_ON) {
      linkUp = true;
      ESP_LOGI(TAG, "Ethernet link is up");
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    retry++;
  }

  if (!linkUp) {
    // Not fatal: loop() watches the PHY and kicks off DHCP when a cable shows up.
    ESP_LOGW(TAG, "No Ethernet link detected; will configure when link comes up");
    return ESP_ERR_TIMEOUT;
  }

  if (useDhcp) {
    ESP_LOGI(TAG, "Starting DHCP on W5500...");
    DHCP_init(DHCP_SOCKET_NUM, s_dhcp_buf);
    reg_dhcp_cbfunc(nullptr, nullptr, nullptr);

    uint32_t elapsedMs = 0;
    uint32_t tickAccumMs = 0;
    uint8_t dhcpState = DHCP_RUNNING;
    const uint32_t timeoutMs = 30000;

    while (elapsedMs < timeoutMs) {
      vTaskDelay(pdMS_TO_TICKS(100));
      elapsedMs += 100;
      tickAccumMs += 100;
      if (tickAccumMs >= 1000) {
        DHCP_time_handler();
        tickAccumMs -= 1000;
      }

      dhcpState = DHCP_run();
      if (dhcpState == DHCP_IP_ASSIGN ||
          dhcpState == DHCP_IP_CHANGED ||
          dhcpState == DHCP_IP_LEASED) {
        break;
      }
      if (dhcpState == DHCP_FAILED ||
          dhcpState == DHCP_STOPPED) {
        break;
      }
    }

    if (dhcpState == DHCP_IP_ASSIGN ||
        dhcpState == DHCP_IP_CHANGED ||
        dhcpState == DHCP_IP_LEASED) {
      wizchip_getnetinfo(&netinfo);
      ESP_LOGI(TAG, "DHCP acquired IP: %d.%d.%d.%d  GW: %d.%d.%d.%d  SN: %d.%d.%d.%d",
               netinfo.ip[0], netinfo.ip[1], netinfo.ip[2], netinfo.ip[3],
               netinfo.gw[0], netinfo.gw[1], netinfo.gw[2], netinfo.gw[3],
               netinfo.sn[0], netinfo.sn[1], netinfo.sn[2], netinfo.sn[3]);
    } else {
      ESP_LOGW(TAG, "DHCP failed (state=%d), falling back to static IP; DHCP keeps retrying in background", dhcpState);
      uint8_t ip[4] = {192, 168, 1, 2};
      uint8_t sn[4] = {255, 255, 255, 0};
      uint8_t gw[4] = {192, 168, 1, 1};
      uint8_t dns[4] = {0, 0, 0, 0};
      memcpy(netinfo.ip, ip, 4);
      memcpy(netinfo.sn, sn, 4);
      memcpy(netinfo.gw, gw, 4);
      memcpy(netinfo.dns, dns, 4);
      netinfo.dhcp = NETINFO_STATIC;
      wizchip_setnetinfo(&netinfo);
      wizchip_getnetinfo(&netinfo);
    }
  }

  ESP_LOGI(TAG, "MAC: %02x:%02x:%02x:%02x:%02x:%02x",
           netinfo.mac[0], netinfo.mac[1], netinfo.mac[2],
           netinfo.mac[3], netinfo.mac[4], netinfo.mac[5]);

  return ESP_OK;
}

esp_err_t W5500Eth::stop() {
  if (g_spi_handle) {
    if (s_bus_held) { spi_device_release_bus(g_spi_handle); s_bus_held = false; }
    spi_bus_remove_device(g_spi_handle);
    g_spi_handle = nullptr;
  }
  linkUp = false;
  return ESP_OK;
}

void W5500Eth::loop() {
  // Health watchdog. An unattended NTP server must self-recover from a wedged
  // W5500 or a dropped link: the CPU and display keep running even when the
  // chip dies, so nothing else would notice. This only ever acts when the
  // network is already unusable and never touches the timekeeping path.
  static int64_t lastCheckUs = 0;
  static int64_t unhealthySinceUs = 0;
  int64_t now = esp_timer_get_time();
  if (now - lastCheckUs < 1000000) return;   // ~1 Hz is plenty
  lastCheckUs = now;

  bool chipOk = (getVERSIONR() == 0x04);      // SPI sanity — catches a wedged W5500
  bool phyOk  = chipOk && (wizphy_getphylink() == PHY_LINK_ON);
  bool healthy = chipOk && phyOk;

  // A W5500 that spontaneously resets (power glitch) still answers SPI and
  // reads VERSIONR=4, but its register file is back to defaults — SHAR
  // included. Left alone, the next DHCP_init() would see SHAR=0 and adopt
  // ioLibrary's temporary MAC (00:08:dc:00:00:00), so the router hands out a
  // brand-new lease and the server silently moves to a different IP without
  // ever rebooting (2026-07-24 outage). Compare SHAR against the MAC we
  // programmed and rebuild the chip config in place when it's been lost.
  if (chipOk) {
    uint8_t shar[6];
    getSHAR(shar);
    if (memcmp(shar, expectedMac, 6) != 0) {
      getSHAR(shar);   // reread — never act on a single glitched SPI transfer
      if (memcmp(shar, expectedMac, 6) != 0) {
        ESP_LOGE(TAG, "W5500 register loss (SHAR=%02x:%02x:%02x:%02x:%02x:%02x) — reinitializing chip in place",
                 shar[0], shar[1], shar[2], shar[3], shar[4], shar[5]);
        chipResetCount++;
        reinitChip();
        linkUp = false;   // PHY renegotiates; the link-up transition below restarts DHCP
        if (chipResetCb) chipResetCb(chipResetCbArg);
        return;
      }
    }
  }

  if (healthy != linkUp) {
    ESP_LOGI(TAG, "Ethernet link %s (chipOk=%d phyOk=%d)", healthy ? "up" : "down", chipOk, phyOk);
    if (healthy && useDhcp) {
      // Cable replugged (possibly into a different network) — restart the DHCP
      // state machine so we reacquire instead of squatting on a stale lease.
      ESP_LOGI(TAG, "Link restored, restarting DHCP");
      DHCP_init(DHCP_SOCKET_NUM, s_dhcp_buf);
    }
    linkUp = healthy;
  }

  // Service the DHCP client at ~1 Hz. Renewal at T1, rebinding, and
  // reacquisition after NAK all happen inside DHCP_run() — it just has to
  // keep being called for the lifetime of the lease, not only at boot.
  if (useDhcp && linkUp) {
    DHCP_time_handler();
    uint8_t st = DHCP_run();
    if (st == DHCP_IP_ASSIGN || st == DHCP_IP_CHANGED) {
      wiz_NetInfo ni = {};
      wizchip_getnetinfo(&ni);
      ESP_LOGI(TAG, "DHCP %s: %d.%d.%d.%d  GW: %d.%d.%d.%d  lease: %us",
               st == DHCP_IP_CHANGED ? "renewed (IP changed)" : "acquired",
               ni.ip[0], ni.ip[1], ni.ip[2], ni.ip[3],
               ni.gw[0], ni.gw[1], ni.gw[2], ni.gw[3],
               (unsigned)getDHCPLeasetime());
    }
  }

  // Reboot only for a wedged chip (dead SPI). A merely unplugged cable must
  // not restart the clock — timekeeping is still valid and the link recovery
  // above handles the replug.
  if (chipOk) {
    unhealthySinceUs = 0;
  } else if (unhealthySinceUs == 0) {
    unhealthySinceUs = now;
  } else if (now - unhealthySinceUs > 60000000) {   // 60s sustained outage
    ESP_LOGE(TAG, "W5500 unresponsive >60s — restarting to recover");
    esp_restart();
  }
}

void W5500Eth::reinitChip() {
  // Start from clean silicon when the reset line is wired, mirroring begin().
  if (rstPin >= 0) {
    gpio_set_level((gpio_num_t)rstPin, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level((gpio_num_t)rstPin, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  uint8_t memsize[8] = {2, 2, 2, 2, 2, 2, 2, 2};
  wizchip_init(memsize, memsize);

  wiz_NetInfo netinfo = {};
  memcpy(netinfo.mac, expectedMac, 6);
  if (haveStatic) {
    memcpy(netinfo.ip, staticIp4, 4);
    memcpy(netinfo.gw, staticGw4, 4);
    memcpy(netinfo.sn, staticSn4, 4);
    netinfo.dhcp = NETINFO_STATIC;
  } else {
    netinfo.dhcp = NETINFO_DHCP;
  }
  wizchip_setnetinfo(&netinfo);

  wiz_PhyConf phyconf = {};
  phyconf.by = PHY_CONFBY_SW;
  phyconf.mode = PHY_MODE_AUTONEGO;
  phyconf.speed = PHY_SPEED_100;
  phyconf.duplex = PHY_DUPLEX_FULL;
  wizphy_setphyconf(&phyconf);
}

bool W5500Eth::isLinkUp() const {
  return linkUp;
}

uint8_t W5500Eth::readVersion() const {
  return getVERSIONR();   // 0x04 when healthy; anything else means the SPI link is wedged
}

esp_err_t W5500Eth::getMacAddr(uint8_t mac[6]) const {
  getSHAR(mac);
  return ESP_OK;
}

bool W5500Eth::getIpAddr(uint32_t& ip) const {
  uint8_t ip_arr[4];
  getSIPR(ip_arr);
  ip = (ip_arr[0] << 24) | (ip_arr[1] << 16) | (ip_arr[2] << 8) | ip_arr[3];
  return ip != 0;
}

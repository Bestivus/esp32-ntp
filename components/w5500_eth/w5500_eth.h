#pragma once
// SPDX-License-Identifier: Unlicense

#include <stdint.h>
#include "esp_err.h"
#include "esp_netif.h"
#include "driver/spi_master.h"

class W5500Eth {
public:
  W5500Eth();
  ~W5500Eth();
  
  esp_err_t begin(spi_host_device_t spiHost, int mosiPin, int misoPin, int sclkPin, int csPin, int intPin, int rstPin, int clockHz);
  esp_err_t start(bool use_static_ip = false,
                  const char* static_ip = nullptr,
                  const char* static_gw = nullptr,
                  const char* static_netmask = nullptr);
  esp_err_t stop();
  void loop();
  bool isLinkUp() const;
  uint8_t readVersion() const;   // W5500 VERSIONR (0x04 = healthy); for diagnostics
  esp_err_t getMacAddr(uint8_t mac[6]) const;
  bool getIpAddr(uint32_t& ip) const;
  uint32_t getChipResetCount() const { return chipResetCount; }
  // Invoked after an in-place chip re-init so socket owners can reopen —
  // a W5500 register loss closes every hardware socket.
  void onChipReset(void (*cb)(void*), void* arg) { chipResetCb = cb; chipResetCbArg = arg; }

private:
  void reinitChip();

  esp_netif_t* eth_netif;
  bool linkUp;
  bool useDhcp;
  int intPin;
  int rstPin;
  uint8_t expectedMac[6];
  bool haveStatic;
  uint8_t staticIp4[4];
  uint8_t staticGw4[4];
  uint8_t staticSn4[4];
  uint32_t chipResetCount;
  void (*chipResetCb)(void*);
  void* chipResetCbArg;
};

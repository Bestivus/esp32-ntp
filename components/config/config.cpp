// SPDX-License-Identifier: Unlicense

#include "config.h"
#include "sdkconfig.h"
#include <time.h>

#ifndef CONFIG_APP_USE_PRESYNC_GLYPH
#define CONFIG_APP_USE_PRESYNC_GLYPH 0
#endif

namespace Config {

static const char* kNtpServer = "192.168.0.1";
static const int kNtpPort = 123;
static const int kNtpServerPort = 123; // for serving NTP

// Display on VSPI (SPI3_HOST)
static const spi_host_device_t kSpiHost = SPI3_HOST;  // VSPI for display
static const int kCsPin = 5;           // MAX7219 display CS pin
static const int kMaxDevices = 4;
static const int kSpiClockHz = 10 * 1000 * 1000;
static const int kSpiMosiPin = 23;     // VSPI MOSI
static const int kSpiMisoPin = -1;     // Display doesn't need MISO
static const int kSpiSclkPin = 18;     // VSPI SCLK

static const char* kTimezone = CONFIG_APP_TZ;

// GPS / PPS
static const int kGpsUartPort = 2;   // UART2
static const int kGpsUartBaud = 9600;
static const int kGpsUartTxPin = 17; // not used by GPS NEO-6M
static const int kGpsUartRxPin = 16; // GPS TX -> ESP RX
static const int kPpsGpioPin = 19;   // PPS input (back to original pin!)
// PPS offset calibration (microseconds, can be positive or negative)
static const int64_t kPpsCalibrationUs = 0;  // MCPWM hardware capture — no ISR latency to compensate

// W5500 Ethernet on HSPI (SPI2_HOST) - separate bus from display
static const spi_host_device_t kW5500SpiHost = SPI2_HOST;  // HSPI for W5500
static const int kW5500CsPin = 25;     // W5500 CS pin
static const int kW5500MosiPin = 33;   // HSPI MOSI
static const int kW5500MisoPin = 35;   // HSPI MISO (GPIO 35 is input-only, perfect for MISO)
static const int kW5500SclkPin = 32;   // HSPI SCLK
static const int kW5500IntPin = 34;    // W5500 INT pin (GPIO 34 is input-only, perfect for INT)
static const int kW5500RstPin = 26;    // W5500 RST pin

const char* getNtpServer() { return kNtpServer; }
int getNtpPort() { return kNtpPort; }
int getNtpServerPort() { return kNtpServerPort; }

bool getUseDisplay() { 
#ifdef CONFIG_APP_USE_DISPLAY
	return CONFIG_APP_USE_DISPLAY;
#else
	return false;
#endif
}
bool getUsePreSyncGlyph() { return CONFIG_APP_USE_PRESYNC_GLYPH; }
spi_host_device_t getSpiHost() { return kSpiHost; }
int getCsPin() { return kCsPin; }
int getMaxDevices() { return kMaxDevices; }
int getSpiClockHz() { return kSpiClockHz; }
int getSpiMosiPin() { return kSpiMosiPin; }
int getSpiMisoPin() { return kSpiMisoPin; }
int getSpiSclkPin() { return kSpiSclkPin; }

const char* getTimezone() { return kTimezone; }
void applyTimezone() { setenv("TZ", kTimezone, 1); tzset(); }

int getGpsUartPort() { return kGpsUartPort; }
int getGpsUartBaud() { return kGpsUartBaud; }
int getGpsUartTxPin() { return kGpsUartTxPin; }
int getGpsUartRxPin() { return kGpsUartRxPin; }
int getPpsGpioPin() { return kPpsGpioPin; }
int64_t getPpsCalibrationUs() { return kPpsCalibrationUs; }

// Served-time calibration, microseconds, SUBTRACTED from both t2 and t3.
//
// t2 is latched when INTn asserts, which is after the W5500 has stored the
// entire frame, so it is systematically late by the chip's receive latency —
// and NTP's offset formula turns that straight into a positive bias. Measured
// against a GPS stratum-1 reference on the same switch (no path asymmetry):
// +15 us median before compensation. Subtracting it from both timestamps
// shifts the reported clock without touching the round-trip delay.
//
// RE-DERIVED after the reply path was cut from ~2.1 ms to ~0.4 ms. The old 15 us
// was measured when the transmit path still stamped t3 well before the frame
// actually left, and that residual lateness partly cancelled the subtraction.
// With t3 now landing within ~5 us of egress the subtraction stopped being
// cancelled and over-corrected: served offset sat at -6..-12 us median against
// the GPS reference across three 60-sample runs. 7 us re-centres it.
int getServeCalibrationUs() { return 7; }

spi_host_device_t getW5500SpiHost() { return kW5500SpiHost; }
int getW5500CsPin() { return kW5500CsPin; }
int getW5500MosiPin() { return kW5500MosiPin; }
int getW5500MisoPin() { return kW5500MisoPin; }
int getW5500SclkPin() { return kW5500SclkPin; }
int getW5500IntPin() { return kW5500IntPin; }
// SPI clock for the W5500. 20 MHz is the long-proven value on this wiring.
// CEILING: reads are full duplex (the coalescing shim clocks the header out
// while the payload comes in), and full-duplex transfers routed through the
// GPIO matrix — which these are, since the SPI pins are arbitrary config
// values rather than IO_MUX pins — top out at 26.6 MHz. Past that, READS
// corrupt silently, which surfaces as wrong Sn_TX_FSR/Sn_TX_WR values and
// therefore malformed packets rather than an obvious failure. Do not exceed
// 26 MHz without moving SPI to the IO_MUX pins.
// TRIED AND REVERTED: 80 MHz / 3 = 26.67 MHz. The divider is integral, so that
// is the only step above 20 MHz that exists (anything between rounds back down
// to 80/4). At 26.67 MHz this board does not come up at all — no NTP, no
// /metrics, no ICMP — so the ceiling above is optimistic for this wiring rather
// than merely marginal. It stays at 20 MHz. Byte clocking is only ~100 us of the
// reply path anyway, so the most this could ever have returned is ~25 us.
int getW5500SpiHz() { return 20000000; }
int getW5500RstPin() { return kW5500RstPin; }

bool getNetworkWiznet() {
#ifdef CONFIG_APP_NETWORK_WIZNET
  return true;
#else
  return false;
#endif
}
bool getNetworkWifi() {
#ifdef CONFIG_APP_NETWORK_WIFI
  return true;
#else
  return false;
#endif
}

const char* getWifiSsid() {
#ifdef CONFIG_APP_NETWORK_WIFI
  return CONFIG_APP_WIFI_SSID;
#else
  return "";
#endif
}
const char* getWifiPassword() {
#ifdef CONFIG_APP_NETWORK_WIFI
  return CONFIG_APP_WIFI_PASSWORD;
#else
  return "";
#endif
}

bool getUseStaticIp() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return true;
#else
  return false;
#endif
}
const char* getStaticIp() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return CONFIG_APP_STATIC_IP_ADDR;
#else
  return "192.168.1.100";
#endif
}
const char* getStaticGw() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return CONFIG_APP_STATIC_GW;
#else
  return "192.168.1.1";
#endif
}
const char* getStaticNetmask() {
#ifdef CONFIG_APP_USE_STATIC_IP
  return CONFIG_APP_STATIC_NETMASK;
#else
  return "255.255.255.0";
#endif
}

bool getMqttEnable() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return true;
#else
  return false;
#endif
}
const char* getMqttBrokerIp() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return CONFIG_APP_MQTT_BROKER_IP;
#else
  return "";
#endif
}
int getMqttBrokerPort() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return CONFIG_APP_MQTT_BROKER_PORT;
#else
  return 1883;
#endif
}
const char* getMqttUsername() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return CONFIG_APP_MQTT_USERNAME;
#else
  return "";
#endif
}
const char* getMqttPassword() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return CONFIG_APP_MQTT_PASSWORD;
#else
  return "";
#endif
}
const char* getMqttClientId() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return CONFIG_APP_MQTT_CLIENT_ID;
#else
  return "esp32-ntp";
#endif
}
const char* getMqttNodeId() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return CONFIG_APP_MQTT_NODE_ID;
#else
  return "esp32ntp";
#endif
}
int getMqttPublishIntervalS() {
#ifdef CONFIG_APP_MQTT_ENABLE
  return CONFIG_APP_MQTT_PUBLISH_INTERVAL_S;
#else
  return 30;
#endif
}

}
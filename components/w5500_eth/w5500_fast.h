#pragma once
// SPDX-License-Identifier: Unlicense
/*
 * One W5500 access == one SPI transaction.
 *
 * A W5500 frame is [addr_hi][addr_lo][control][data...] and, in variable-length
 * data mode, the data phase is delimited by CS rather than by transaction
 * boundaries. The ioLibrary reaches the same wire format but does so through
 * per-byte callbacks: the coalescing shim in w5500_eth.cpp has to flush the
 * three header bytes as their own transaction before it can read, so every
 * register read costs two transactions and every access costs a CS pair plus
 * driver bookkeeping. Measured at 160 MHz with the bus pre-acquired, a
 * transaction costs ~15 us of fixed overhead against ~0.4 us/byte of actual
 * clocking at 20 MHz, so the transaction COUNT — not SPI bandwidth — is what
 * the reply latency is made of.
 *
 * These two functions assemble the header and payload into one buffer and issue
 * exactly one spi_device_polling_transmit() with CS held around it. They are
 * for the NTP reply path only; everything else keeps using the ioLibrary.
 *
 * `addrsel` follows the ioLibrary's encoding: (offset << 8) | (block << 3).
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void w5k_xfer_rd(uint32_t addrsel, uint8_t* buf, uint16_t len);
void w5k_xfer_wr(uint32_t addrsel, const uint8_t* buf, uint16_t len);

#ifdef __cplusplus
}
#endif

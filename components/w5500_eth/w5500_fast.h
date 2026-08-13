#pragma once
// SPDX-License-Identifier: Unlicense
/*
 * One W5500 access == one SPI transaction, with zero driver overhead.
 *
 * A W5500 frame is [addr_hi][addr_lo][control][data...] and, in variable-length
 * data mode, the data phase is delimited by CS rather than by transaction
 * boundaries. Measured at 160 MHz with the bus pre-acquired, a driver-managed
 * transaction costs ~15 us of fixed overhead against ~0.4 us/byte of actual
 * clocking at 20 MHz, so the transaction COUNT — not SPI bandwidth — is what
 * the reply latency is made of. On this FIFO path at 240 MHz the fixed
 * overhead is ~3-4 us per transaction (rx_ready span 6.2 us for one 5-byte
 * access, of which 2 us is clocking; 2026-08-13 bench).
 *
 * These two functions assemble the header and payload into one buffer and
 * clock it through the SPI peripheral's FIFO directly (see w5k_fifo_xfer in
 * w5500_eth.cpp). They are for the NTP reply path only; everything else goes
 * through the driver-managed w5500_bus_rd/w5500_bus_wr in w5500_drv.h.
 *
 * `addrsel` packs the access as (offset << 8) | (block << 3).
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

// SPDX-License-Identifier: Unlicense

#include "w5k_udp_wrapper.h"
#include "esp_timer.h"
#include "wizchip_conf.h"
#include "socket.h"

int w5k_udp_open(uint8_t socket_num, uint16_t port) {
  return socket(socket_num, Sn_MR_UDP, port, 0) == socket_num ? 0 : -1;
}

int w5k_close(uint8_t socket_num) {
  return close(socket_num);
}

int32_t w5k_recvfrom(uint8_t socket_num, uint8_t* buf, uint16_t len, uint8_t* from_ip, uint16_t* from_port) {
  return recvfrom(socket_num, buf, len, from_ip, from_port);
}

int32_t w5k_sendto(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port) {
  return sendto(socket_num, (uint8_t*)buf, len, (uint8_t*)to_ip, to_port);
}

int w5k_set_nonblock(uint8_t socket_num) {
  uint8_t mode = SOCK_IO_NONBLOCK;
  return ctlsocket(socket_num, CS_SET_IOMODE, &mode);
}

int32_t w5k_rx_ready(uint8_t socket_num) {
  return getSn_RX_RSR(socket_num);
}

void w5k_enable_rx_irq(uint8_t socket_num) {
  // Route this socket's interrupts to the INTn pin, and enable only RECV.
  setSIMR(getSIMR() | (uint8_t)(1 << socket_num));
  setSn_IMR(socket_num, Sn_IR_RECV);
}

/*
 * Additionally assert INTn on send completion.
 *
 * The W5500 has NO timestamping hardware of any kind, but INTn is already
 * routed to an MCPWM capture channel for packet arrival, and SENDOK drives the
 * same pin. Enabling it means transmit completion is latched in hardware on the
 * same 80 MHz counter as arrival and as the GPS pulse — so the request-to-egress
 * turnaround becomes a hardware measurement instead of a software estimate.
 *
 * Note INTn is level-asserted: RECV must already be cleared before SEND, or
 * SENDOK produces no falling edge to latch. The caller checks that the capture
 * counter advanced exactly once and discards the sample otherwise.
 */
void w5k_enable_sendok_irq(uint8_t socket_num) {
  setSIMR(getSIMR() | (uint8_t)(1 << socket_num));
  setSn_IMR(socket_num, (uint8_t)(Sn_IR_RECV | Sn_IR_SENDOK));
}

void w5k_clear_rx_irq(uint8_t socket_num) {
  setSn_IR(socket_num, Sn_IR_RECV);
}

int w5k_sendto_nb(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port) {
  setSn_DIPR(socket_num, (uint8_t*)to_ip);
  setSn_DPORT(socket_num, to_port);
  uint16_t maxsz = getSn_TxMAX(socket_num);
  if (len > maxsz) len = maxsz;
  if (getSn_TX_FSR(socket_num) < len) return -1;
  wiz_send_data(socket_num, (uint8_t*)buf, len);
  setSn_CR(socket_num, Sn_CR_SEND);
  while (getSn_CR(socket_num));
  return 0;
}

/* Diagnostic snapshot of the transmit pointer trio. */
void w5k_tx_ptrs(uint8_t socket_num, uint16_t* rd, uint16_t* wr, uint16_t* fsr) {
  if (rd)  *rd  = getSn_TX_RD(socket_num);
  if (wr)  *wr  = getSn_TX_WR(socket_num);
  if (fsr) *fsr = getSn_TX_FSR(socket_num);
}

/*
 * Late-stamped send, in two steps, WITHOUT splitting wiz_send_data().
 *
 * The obvious split (write 40 bytes, stamp t3, write 8 more, SEND) does not
 * work on this chip: wiz_send_data() is a read-modify-write of Sn_TX_WR, and
 * the advance from the first call does not survive to the second, so the tail
 * overwrites the head and SEND emits only 8 bytes. Instead the whole datagram
 * is staged in ONE wiz_send_data() (which advances Sn_TX_WR exactly once and
 * is the pattern the library is actually used with), and the transmit
 * timestamp is then patched in place by writing 8 bytes straight to its offset
 * in the socket's TX buffer. That write does not touch Sn_TX_WR at all, so
 * there is no pointer arithmetic to lose — and only those 8 bytes plus the
 * SEND command sit between stamping t3 and the packet leaving.
 */
int w5k_send_stage(uint8_t socket_num, const uint8_t* buf, uint16_t len,
                   const uint8_t* to_ip, uint16_t to_port, uint16_t* stamp_off,
                   int* wr_delta) {
  /*
   * Byte-for-byte the proven w5k_sendto_nb() sequence, stopping short of
   * Sn_CR_SEND so the caller can patch the transmit timestamp in place first.
   * Deviating from this sequence (adding an Sn_SR gate and a free-space wait
   * loop) left wiz_send_data() doing nothing at all — Sn_TX_WR and Sn_TX_FSR
   * both unchanged — so it is mirrored exactly rather than improved upon.
   */
  setSn_DIPR(socket_num, (uint8_t*)to_ip);
  setSn_DPORT(socket_num, to_port);
  uint16_t maxsz = getSn_TxMAX(socket_num);
  if (len > maxsz) len = maxsz;
  if (getSn_TX_FSR(socket_num) < len) return -1;
  uint16_t wr0 = getSn_TX_WR(socket_num);
  if (stamp_off) *stamp_off = wr0;
  wiz_send_data(socket_num, (uint8_t*)buf, len);
  /*
   * Deliberately do NOT verify by reading Sn_TX_WR back. That read returns a
   * stale value here (it reports no advance even when the write succeeded), and
   * gating on it is what made earlier attempts refuse to send at all. The
   * library never reads it back either — it trusts wiz_send_data and uses the
   * pointer sampled BEFORE the write, which is exactly what stamp_off carries.
   */
  if (wr_delta) *wr_delta = (int)(uint16_t)(getSn_TX_WR(socket_num) - wr0);
  return 0;
}

/*
 * Overwrite `len` bytes at TX-buffer offset `off`, then transmit.
 *
 * Split in two so the caller can time ONLY the pre-egress work. SENDOK asserts
 * after the frame is already on the wire, so including that wait in the t3
 * pre-correction overestimates the latency before egress and biases every
 * served timestamp into the future (measured +43 us against a same-switch
 * GPS reference before this split).
 */
int w5k_send_stamp_and_fire(uint8_t socket_num, uint16_t off,
                            const uint8_t* stamp, uint16_t len) {
  uint32_t addrsel = ((uint32_t)off << 8) + (WIZCHIP_TXBUF_BLOCK(socket_num) << 3);
  WIZCHIP_WRITE_BUF(addrsel, (uint8_t*)stamp, len);
  setSn_CR(socket_num, Sn_CR_SEND);
  while (getSn_CR(socket_num));
  return 0;
}

/* Reap the completion. Call after timing has stopped. */
int w5k_send_reap(uint8_t socket_num) {
  /* A fixed iteration count is a latency cliff: 20000 register reads is ~180 ms
   * of spinning. Bound it in time instead. */
  int64_t deadline = esp_timer_get_time() + 2000;
  while (esp_timer_get_time() < deadline) {
    uint8_t ir = getSn_IR(socket_num);
    if (ir & Sn_IR_SENDOK) { setSn_IR(socket_num, Sn_IR_SENDOK); return 0; }
    if (ir & Sn_IR_TIMEOUT) { setSn_IR(socket_num, Sn_IR_TIMEOUT); return -1; }
  }
  return -1;
}

int w5k_sendto_poll(uint8_t socket_num) {
  uint8_t ir = getSn_IR(socket_num);
  if (ir & Sn_IR_SENDOK) {
    setSn_IR(socket_num, Sn_IR_SENDOK);
    return 1;
  }
  if (ir & Sn_IR_TIMEOUT) {
    setSn_IR(socket_num, Sn_IR_TIMEOUT);
    return -1;
  }
  return 0;
}

int w5k_arp_prime(uint8_t socket_num, const uint8_t* ip) {
  /* Send a 1-byte dummy to port 9 (discard protocol) to trigger
     W5500 ARP resolution.  Block until SENDOK so the caller knows
     the ARP cache is warm before stamping t3.

     A spoofed/unroutable source must not stall the main loop (which also
     services PPS disciplining and DHCP) for the chip's default ~1.8s ARP
     retry budget, so shrink RTR/RCR for the prime: 40ms/try x 2 tries
     ~= 80ms worst case, then restore.  A live LAN host answers ARP in
     well under 40ms. */
  uint16_t rtr = getRTR();
  uint8_t rcr = getRCR();
  setRTR(400);   /* 40ms per try (unit 100us) */
  setRCR(1);     /* 1 retry -> ~80ms to TIMEOUT */

  uint8_t dummy = 0;
  setSn_DIPR(socket_num, (uint8_t*)ip);
  setSn_DPORT(socket_num, 9);           /* RFC 863 discard */
  /* Clear stale completion flags first: otherwise the poll below can see a
   * previous send's SENDOK and return before ARP has actually resolved. */
  setSn_IR(socket_num, (uint8_t)(Sn_IR_SENDOK | Sn_IR_TIMEOUT));
  wiz_send_data(socket_num, &dummy, 1);
  setSn_CR(socket_num, Sn_CR_SEND);
  while (getSn_CR(socket_num));

  int ret = -1;
  for (int i = 0; i < 20000; i++) {     /* backstop; TIMEOUT IR fires first */
    uint8_t ir = getSn_IR(socket_num);
    if (ir & Sn_IR_SENDOK) {
      setSn_IR(socket_num, Sn_IR_SENDOK);
      ret = 0;
      break;
    }
    if (ir & Sn_IR_TIMEOUT) {
      setSn_IR(socket_num, Sn_IR_TIMEOUT);
      break;
    }
  }

  setRTR(rtr);
  setRCR(rcr);
  return ret;
}


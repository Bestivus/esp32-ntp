// SPDX-License-Identifier: Unlicense

#include "w5k_udp_wrapper.h"
#include "esp_timer.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "w5500_fast.h"
#include <string.h>
#include "esp_attr.h"

/*
 * ---------------------------------------------------------------------------
 * Fast reply path.
 *
 * Everything below w5k_fast_* bypasses the ioLibrary and talks to the chip
 * through w5k_xfer_rd/w5k_xfer_wr, which cost exactly one SPI transaction per
 * access. The library's own path is kept for DHCP, the stats TCP socket, and
 * socket setup, where latency is irrelevant.
 *
 * What the library spends transactions on, and why none of it is needed here:
 *  - Sn_RX_RSR / Sn_TX_FSR are read in a do{}while(v != v1) loop because the
 *    library fetches each 16-bit register as two single-byte accesses, which
 *    can tear. A 2-byte burst inside one transaction is atomic on the wire, so
 *    one read suffices.
 *  - recvfrom() drains the 8-byte PACKET-INFO header and the payload in two
 *    wiz_recv_data() calls, each of which re-reads Sn_RX_RD, rewrites it, issues
 *    Sn_CR_RECV and polls Sn_CR. Both live in the same contiguous ring, so one
 *    burst reads the header and payload together and one RECV closes it out.
 *  - Sn_DIPR (0x0C-0x0F) and Sn_DPORT (0x10-0x11) are contiguous: one 6-byte
 *    write replaces two accesses.
 *  - Sn_TX_FSR (0x20), Sn_TX_RD (0x22) and Sn_TX_WR (0x24) are contiguous too:
 *    one 6-byte read replaces two double-read loops.
 *  - getSn_TxMAX() is a constant 2048 for this memsize configuration.
 *
 * The W5500 masks a socket buffer pointer to the allocated buffer size and
 * auto-increments across the wrap in VDM, which is exactly what the library
 * relies on, so a burst that crosses the ring boundary needs no splitting here
 * either.
 * ---------------------------------------------------------------------------
 */
#define W5K_CREG(off)     ((uint32_t)(off) << 8)   /* common register block 0 */
#define SN_SREG(sn, off)  (((uint32_t)(off) << 8) + ((uint32_t)(1u + 4u * (sn)) << 3))
#define SN_TXBUF(sn, off) (((uint32_t)(off) << 8) + ((uint32_t)(2u + 4u * (sn)) << 3))
#define SN_RXBUF(sn, off) (((uint32_t)(off) << 8) + ((uint32_t)(3u + 4u * (sn)) << 3))

#define OFF_RTR        0x0019   /* 0x19-0x1A retry time, 0x1B retry count */
#define OFF_SN_CR      0x0001
#define OFF_SN_IR      0x0002
#define OFF_SN_DIPR    0x000C   /* 0x0C..0x0F IP, 0x10..0x11 port: contiguous */
#define OFF_SN_TX_FSR  0x0020   /* 0x20 FSR, 0x22 RD, 0x24 WR: contiguous     */
#define OFF_SN_TX_WR   0x0024
#define OFF_SN_RX_RSR  0x0026
#define OFF_SN_RX_RD   0x0028

/* memsize is {2,...} KB for every socket (see W5500Eth::begin). */
#define SN_TX_MAX 2048

/* Attribution counters: how many register reads each spin actually costs. */
volatile uint32_t g_w5k_reap_polls  = 0;
volatile uint32_t g_w5k_prime_polls = 0;

int w5k_udp_open(uint8_t socket_num, uint16_t port) {
  return socket(socket_num, Sn_MR_UDP, port, 0) == socket_num ? 0 : -1;
}

int w5k_close(uint8_t socket_num) {
  return close(socket_num);
}

/*
 * Drain one datagram in six transactions: Sn_RX_RD, one burst covering the
 * PACKET-INFO header and the payload together, the Sn_RX_RD advance, Sn_CR_RECV
 * and one Sn_CR acceptance poll. `rsr` is the byte count the caller already read
 * with w5k_rx_ready(), passed in rather than re-read.
 */
IRAM_ATTR int32_t w5k_recvfrom(uint8_t socket_num, uint8_t* buf, uint16_t len,
                     uint8_t* from_ip, uint16_t* from_port, uint16_t rsr) {
  /* One access must fit the SPI peripheral's 64-byte data buffer once the
   * 3-byte W5500 header is included, so the header-plus-payload burst is capped
   * at 58: the 8-byte PACKET-INFO header plus 50 payload bytes. A 48-byte NTP
   * request fits with room to spare; anything longer is truncated to 50 bytes
   * here, which still exceeds everything this server parses (the first 48). */
  uint8_t frame[58];
  if (rsr < 8) return 0;

  uint8_t rdb[2];
  w5k_xfer_rd(SN_SREG(socket_num, OFF_SN_RX_RD), rdb, 2);
  uint16_t rd = (uint16_t)(((uint16_t)rdb[0] << 8) | rdb[1]);

  uint16_t want = rsr;
  if (len > (uint16_t)(sizeof(frame) - 8)) len = (uint16_t)(sizeof(frame) - 8);
  if (want > (uint16_t)(8 + len)) want = (uint16_t)(8 + len);
  w5k_xfer_rd(SN_RXBUF(socket_num, rd), frame, want);

  uint16_t plen  = (uint16_t)(((uint16_t)frame[6] << 8) | frame[7]);
  uint16_t avail = (uint16_t)(want - 8);
  uint16_t copy  = plen < avail ? plen : avail;
  if (from_ip)   memcpy(from_ip, frame, 4);
  if (from_port) *from_port = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
  if (copy)      memcpy(buf, &frame[8], copy);

  /* Advance past the WHOLE datagram even when it did not fit the caller's
   * buffer, so the next read still lands on a PACKET-INFO header. A header
   * claiming more than the socket holds can only be corruption; resynchronise
   * on Sn_RX_RSR rather than desynchronising the ring. */
  uint16_t consumed = (uint16_t)(8 + plen);
  if (consumed > rsr) consumed = rsr;
  rd = (uint16_t)(rd + consumed);
  uint8_t wrb[2] = { (uint8_t)(rd >> 8), (uint8_t)rd };
  w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_RX_RD), wrb, 2);

  uint8_t cmd = Sn_CR_RECV;
  w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_CR), &cmd, 1);
  /* Sn_CR self-clears when the command is accepted. Wait for that, because a
   * later Sn_CR write (the SEND below) would otherwise be dropped. */
  for (int i = 0; i < 200; i++) {
    uint8_t cr = 0;
    w5k_xfer_rd(SN_SREG(socket_num, OFF_SN_CR), &cr, 1);
    if (cr == 0) break;
  }
  return (int32_t)copy;
}

int32_t w5k_sendto(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port) {
  return sendto(socket_num, (uint8_t*)buf, len, (uint8_t*)to_ip, to_port);
}

int w5k_set_nonblock(uint8_t socket_num) {
  uint8_t mode = SOCK_IO_NONBLOCK;
  return ctlsocket(socket_num, CS_SET_IOMODE, &mode);
}

IRAM_ATTR int32_t w5k_rx_ready(uint8_t socket_num) {
  /* One 2-byte burst: atomic on the wire, so the library's do{}while(v != v1)
   * double read (four single-byte accesses) is unnecessary. */
  uint8_t b[2];
  w5k_xfer_rd(SN_SREG(socket_num, OFF_SN_RX_RSR), b, 2);
  return (int32_t)(uint16_t)(((uint16_t)b[0] << 8) | b[1]);
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

IRAM_ATTR void w5k_clear_rx_irq(uint8_t socket_num) {
  uint8_t v = Sn_IR_RECV;
  w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_IR), &v, 1);
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
IRAM_ATTR int w5k_send_stage(uint8_t socket_num, const uint8_t* buf, uint16_t len,
                   const uint8_t* to_ip, uint16_t to_port, uint16_t* stamp_off,
                   int* wr_delta) {
  /*
   * Same wire sequence as the proven w5k_sendto_nb(), stopping short of
   * Sn_CR_SEND so the caller can patch the transmit timestamp in place — but in
   * four transactions instead of a dozen:
   *   1. Sn_DIPR+Sn_DPORT as one 6-byte write (the registers are contiguous)
   *   2. Sn_TX_FSR+Sn_TX_RD+Sn_TX_WR as one 6-byte read (likewise contiguous,
   *      and atomic within one transaction, so no double-read loop is needed)
   *   3. the frame into the TX ring at Sn_TX_WR
   *   4. the Sn_TX_WR advance
   * getSn_TxMAX() is gone: memsize is 2 KB per socket and never changes.
   *
   * Sn_TX_WR is written here rather than read back afterwards. Reading it back
   * immediately returns a stale value on this chip, and an earlier verification
   * gate built on that read broke the send path outright, so wr_delta is left
   * unreported.
   */
  if (len > SN_TX_MAX) len = SN_TX_MAX;

  uint8_t dst[6];
  dst[0] = to_ip[0]; dst[1] = to_ip[1]; dst[2] = to_ip[2]; dst[3] = to_ip[3];
  dst[4] = (uint8_t)(to_port >> 8); dst[5] = (uint8_t)to_port;
  w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_DIPR), dst, 6);

  uint8_t p[6];
  w5k_xfer_rd(SN_SREG(socket_num, OFF_SN_TX_FSR), p, 6);
  uint16_t fsr = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
  uint16_t wr0 = (uint16_t)(((uint16_t)p[4] << 8) | p[5]);
  if (fsr < len) return -1;

  w5k_xfer_wr(SN_TXBUF(socket_num, wr0), buf, len);
  uint16_t nwr = (uint16_t)(wr0 + len);
  uint8_t wrb[2] = { (uint8_t)(nwr >> 8), (uint8_t)nwr };
  w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_TX_WR), wrb, 2);

  if (stamp_off) *stamp_off = wr0;
  if (wr_delta) *wr_delta = -1;   /* not read back: see above */
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
/*
 * Patch the transmit timestamp in place, then fire.
 *
 * EXACTLY two transactions, back to back, with nothing between them: the 8-byte
 * write into the TX ring, then the Sn_CR_SEND byte. This is the whole point of
 * the split — it is the window between t3 being computed and the frame leaving,
 * and it must not grow. The Sn_CR acceptance poll the library does after SEND is
 * deliberately omitted: it only confirms the chip took the command, and the next
 * Sn_CR write is the following packet's RECV, milliseconds away.
 */
IRAM_ATTR int w5k_send_stamp_and_fire(uint8_t socket_num, uint16_t off,
                            const uint8_t* stamp, uint16_t len) {
  w5k_xfer_wr(SN_TXBUF(socket_num, off), stamp, len);
  uint8_t cmd = Sn_CR_SEND;
  w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_CR), &cmd, 1);
  return 0;
}

/* Reap the completion. Call after timing has stopped. */
IRAM_ATTR int w5k_send_reap(uint8_t socket_num) {
  /* A fixed iteration count is a latency cliff: 20000 register reads is ~180 ms
   * of spinning. Bound it in time instead. */
  int64_t deadline = esp_timer_get_time() + 2000;
  while (esp_timer_get_time() < deadline) {
    uint8_t ir = 0;
    g_w5k_reap_polls++;
    w5k_xfer_rd(SN_SREG(socket_num, OFF_SN_IR), &ir, 1);
    if (ir & Sn_IR_SENDOK) {
      uint8_t c = Sn_IR_SENDOK;
      w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_IR), &c, 1);
      return 0;
    }
    if (ir & Sn_IR_TIMEOUT) {
      uint8_t c = Sn_IR_TIMEOUT;
      w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_IR), &c, 1);
      return -1;
    }
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

uint32_t g_w5k_primes = 0;

int w5k_arp_prime(uint8_t socket_num, const uint8_t* ip) {
  /* Send a 1-byte dummy to port 9 (discard protocol) to trigger
     W5500 ARP resolution.  Block until SENDOK so the caller knows
     the ARP cache is warm before stamping t3.

     A spoofed/unroutable source must not stall the main loop (which also
     services PPS disciplining and DHCP) for the chip's default ~1.8s ARP
     retry budget, so shrink RTR/RCR for the prime: 40ms/try x 2 tries
     ~= 80ms worst case, then restore.  A live LAN host answers ARP in
     well under 40ms.

     On the fast accessors throughout: RTR (0x0019-0x001A) and RCR (0x001B) are
     contiguous in the common register block, so saving and restoring the retry
     budget is one 3-byte read and two 3-byte writes rather than six accesses.
     The staging is shared with the reply path. */
  uint8_t saved[3];
  w5k_xfer_rd(W5K_CREG(OFF_RTR), saved, 3);
  const uint8_t tight[3] = { 0x01, 0x90, 0x01 };  /* RTR=400 (40ms), RCR=1 */
  w5k_xfer_wr(W5K_CREG(OFF_RTR), tight, 3);

  /* Clear stale completion flags first: otherwise the poll below can see a
   * previous send's SENDOK and return before ARP has actually resolved. */
  uint8_t clr = (uint8_t)(Sn_IR_SENDOK | Sn_IR_TIMEOUT);
  w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_IR), &clr, 1);

  uint8_t dummy = 0;
  uint16_t off = 0;
  int ret = -1;
  if (w5k_send_stage(socket_num, &dummy, 1, ip, 9, &off, NULL) == 0) {
    uint8_t cmd = Sn_CR_SEND;
    w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_CR), &cmd, 1);
    /* Time-bounded rather than iteration-bounded: the chip's own TIMEOUT fires
     * first at ~80ms, this is only a backstop against a wedged interface. */
    const int64_t deadline = esp_timer_get_time() + 120000;
    while (esp_timer_get_time() < deadline) {
      uint8_t ir = 0;
      g_w5k_prime_polls++;
      w5k_xfer_rd(SN_SREG(socket_num, OFF_SN_IR), &ir, 1);
      if (ir & Sn_IR_SENDOK) {
        uint8_t c = Sn_IR_SENDOK;
        w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_IR), &c, 1);
        ret = 0;
        break;
      }
      if (ir & Sn_IR_TIMEOUT) {
        uint8_t c = Sn_IR_TIMEOUT;
        w5k_xfer_wr(SN_SREG(socket_num, OFF_SN_IR), &c, 1);
        break;
      }
    }
  }

  w5k_xfer_wr(W5K_CREG(OFF_RTR), saved, 3);
  g_w5k_primes++;
  return ret;
}


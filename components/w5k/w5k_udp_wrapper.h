#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int w5k_udp_open(uint8_t socket_num, uint16_t port);
int w5k_close(uint8_t socket_num);
/* `rsr` is the Sn_RX_RSR value the caller already obtained from
 * w5k_rx_ready(); passing it in avoids re-reading the register. */
int32_t w5k_recvfrom(uint8_t socket_num, uint8_t* buf, uint16_t len,
                     uint8_t* from_ip, uint16_t* from_port, uint16_t rsr);
int32_t w5k_sendto(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port);

/* Split send, so the transmit timestamp can be written as late as physically
 * possible. w5k_send_begin() sets the destination and clocks out everything
 * before the t3 field; w5k_send_commit() clocks out t3 and issues SEND. The
 * W5500's TX write pointer advances across the two calls, so the packet is
 * contiguous. This shrinks the window between "t3 is computed" and "the chip
 * is told to transmit" from a whole 48-byte SPI transaction to 8 bytes. */
void w5k_tx_ptrs(uint8_t socket_num, uint16_t* rd, uint16_t* wr, uint16_t* fsr);
int w5k_send_stage(uint8_t socket_num, const uint8_t* buf, uint16_t len,
                   const uint8_t* to_ip, uint16_t to_port, uint16_t* stamp_off,
                   int* wr_delta);
int w5k_send_stamp_and_fire(uint8_t socket_num, uint16_t off,
                            const uint8_t* stamp, uint16_t len);
int w5k_send_reap(uint8_t socket_num);
int w5k_send_begin(uint8_t socket_num, const uint8_t* head, uint16_t headlen,
                   const uint8_t* to_ip, uint16_t to_port);
int w5k_send_commit(uint8_t socket_num, const uint8_t* tail, uint16_t taillen);
int w5k_set_nonblock(uint8_t socket_num);

// Cheap "bytes waiting in RX buffer" check (reads Sn_RX_RSR only, no payload
// read). Lets the caller timestamp packet arrival before the SPI payload read.
int32_t w5k_rx_ready(uint8_t socket_num);

// Enable the W5500 RECV interrupt for this socket so the INTn pin asserts on
// packet arrival (drives a GPIO ISR for hardware-accurate RX timestamping).
void w5k_enable_rx_irq(uint8_t socket_num);
void w5k_enable_sendok_irq(uint8_t socket_num);
// Acknowledge/clear the RECV interrupt so INTn de-asserts and can fire again.
void w5k_clear_rx_irq(uint8_t socket_num);

// Non-blocking UDP send: initiate send (returns 0 on success, -1 on error)
int w5k_sendto_nb(uint8_t socket_num, const uint8_t* buf, uint16_t len, const uint8_t* to_ip, uint16_t to_port);
// Poll for send completion: returns 1=done, 0=pending, -1=timeout
int w5k_sendto_poll(uint8_t socket_num);

// Prime ARP cache for destination IP by sending a 1-byte dummy.
// Blocks until ARP resolves or times out.  Returns 0=ok, -1=timeout.
int w5k_arp_prime(uint8_t socket_num, const uint8_t* ip);

#ifdef __cplusplus
}
#endif



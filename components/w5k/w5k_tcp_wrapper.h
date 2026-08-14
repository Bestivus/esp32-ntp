#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Socket status values callers switch on (W5500 Sn_SR encodings).
enum {
  W5K_SOCK_CLOSED      = 0x00,
  W5K_SOCK_LISTEN      = 0x14,
  W5K_SOCK_ESTABLISHED = 0x17,
  W5K_SOCK_CLOSE_WAIT  = 0x1C,
};

// Open a TCP listen socket on the given port
int w5k_tcp_listen(uint8_t socket_num, uint16_t port);

// Open a TCP socket and issue a non-blocking connect to a remote peer.
// Returns 0 once the connect has been issued (caller polls w5k_tcp_status()
// for W5K_SOCK_ESTABLISHED), or a negative value if the socket could not
// even be opened/connect could not be issued.
int w5k_tcp_connect(uint8_t socket_num, const uint8_t ip[4], uint16_t port);

// Get socket status register value
uint8_t w5k_tcp_status(uint8_t socket_num);

// Non-blocking receive; returns bytes read or 0 if nothing available
int32_t w5k_tcp_recv(uint8_t socket_num, uint8_t* buf, uint16_t len);

// Send data on an established connection. Returns len once staged, 0 while
// the previous chunk is still in flight or the TX buffer lacks room (retry
// after yielding), <0 when the connection is gone.
int32_t w5k_tcp_send(uint8_t socket_num, const uint8_t* buf, uint16_t len);

// Graceful disconnect (sends FIN)
int w5k_tcp_disconnect(uint8_t socket_num);

// Hard close and release socket
int w5k_tcp_close(uint8_t socket_num);

#ifdef __cplusplus
}
#endif

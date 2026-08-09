#pragma once
// SPDX-License-Identifier: Unlicense
#include <stdint.h>
#include "wizchip_conf.h"
#include "socket.h"

#ifdef __cplusplus
extern "C" {
#endif

// Re-export socket status values so users don't depend on raw ioLibrary names.
enum {
  W5K_SOCK_CLOSED      = SOCK_CLOSED,
  W5K_SOCK_LISTEN      = SOCK_LISTEN,
  W5K_SOCK_ESTABLISHED = SOCK_ESTABLISHED,
  W5K_SOCK_CLOSE_WAIT  = SOCK_CLOSE_WAIT,
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

// Send data on an established connection
int32_t w5k_tcp_send(uint8_t socket_num, const uint8_t* buf, uint16_t len);

// Graceful disconnect (sends FIN)
int w5k_tcp_disconnect(uint8_t socket_num);

// Hard close and release socket
int w5k_tcp_close(uint8_t socket_num);

#ifdef __cplusplus
}
#endif


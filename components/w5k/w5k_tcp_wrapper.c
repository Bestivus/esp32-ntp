// SPDX-License-Identifier: Unlicense

#include "w5k_tcp_wrapper.h"
#include "wizchip_conf.h"
#include "socket.h"

int w5k_tcp_listen(uint8_t socket_num, uint16_t port) {
  close(socket_num);
  if (socket(socket_num, Sn_MR_TCP, port, 0) != socket_num) {
    return -1;
  }
  if (listen(socket_num) != SOCK_OK) {
    close(socket_num);
    return -2;
  }
  return 0;
}

int w5k_tcp_connect(uint8_t socket_num, const uint8_t ip[4], uint16_t port) {
  close(socket_num);
  // Ephemeral local port. Vary it on every call, not just per socket number:
  // reusing the exact same 4-tuple across reconnect attempts can collide
  // with a still-lingering connection-tracking entry for the PREVIOUS
  // attempt in an intermediate stateful firewall, which silently drops the
  // new SYN until that stale entry finally expires -- observed on real
  // hardware as several fully-silent connect cycles in a row, then one that
  // got through immediately, repeating.
  static uint16_t s_ephemeral = 0;
  uint16_t myport = (uint16_t)(20000 + socket_num + (uint16_t)(s_ephemeral++ % 1000));
  if (socket(socket_num, Sn_MR_TCP, myport, 0) != socket_num) {
    return -1;
  }
  // Must be set BEFORE connect(): connect_IO_6() in socket.c only returns
  // immediately with SOCK_BUSY when the non-blocking IO bit is already set;
  // otherwise it spins in place until the TCP handshake completes or the
  // chip's own retry timer expires.
  uint8_t mode = SOCK_IO_NONBLOCK;
  ctlsocket(socket_num, CS_SET_IOMODE, &mode);
  int8_t rc = connect(socket_num, (uint8_t*)ip, port);
  if (rc != SOCK_OK && rc != SOCK_BUSY) {
    close(socket_num);
    return -2;
  }
  return 0;
}

uint8_t w5k_tcp_status(uint8_t socket_num) {
  return getSn_SR(socket_num);
}

int32_t w5k_tcp_recv(uint8_t socket_num, uint8_t* buf, uint16_t len) {
  int32_t size = getSn_RX_RSR(socket_num);
  if (size <= 0) return 0;
  if (size > len) size = len;
  return recv(socket_num, buf, (uint16_t)size);
}

int32_t w5k_tcp_send(uint8_t socket_num, const uint8_t* buf, uint16_t len) {
  return send(socket_num, (uint8_t*)buf, len);
}

int w5k_tcp_disconnect(uint8_t socket_num) {
  if (getSn_SR(socket_num) != SOCK_CLOSED) {
    setSn_CR(socket_num, Sn_CR_DISCON);
    while (getSn_CR(socket_num));
  }
  return 0;
}

int w5k_tcp_close(uint8_t socket_num) {
  return close(socket_num);
}


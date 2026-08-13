// SPDX-License-Identifier: Unlicense
/*
 * First-party DHCP client. Wire behavior mirrors the ioLibrary client this
 * network already accepted: full 548-byte frames, client-id + hostname in
 * every message. The lease reservation is keyed on chaddr (the chip MAC).
 */
#include "w5500_dhcp.h"
#include "w5500_drv.h"
#include "esp_log.h"
#include "esp_random.h"
#include <string.h>

static const char* TAG = "W5500DHCP";

#define DHCP_SERVER_PORT 67
#define DHCP_CLIENT_PORT 68
#define DHCP_MSG_SIZE    548          /* 236 fixed + 312 options, zero-padded */

#define BOOTP_REQUEST 1
#define BOOTP_REPLY   2

#define DHCP_DISCOVER 1
#define DHCP_OFFER    2
#define DHCP_REQUEST  3
#define DHCP_DECLINE  4
#define DHCP_ACK      5
#define DHCP_NAK      6

/* option codes */
#define OPT_SUBNET_MASK  1
#define OPT_ROUTER       3
#define OPT_HOSTNAME     12
#define OPT_REQUESTED_IP 50
#define OPT_LEASE_TIME   51
#define OPT_MSG_TYPE     53
#define OPT_SERVER_ID    54
#define OPT_PARAM_LIST   55
#define OPT_T1           58
#define OPT_T2           59
#define OPT_CLIENT_ID    61
#define OPT_END          255

static const uint8_t MAGIC_COOKIE[4] = { 99, 130, 83, 99 };
static const uint8_t BCAST_IP[4] = { 255, 255, 255, 255 };
static const char HOSTNAME[] = "esp32-ntp";

/* per-phase retry budget: resend after 4 s, three sends per phase */
#define PHASE_TIMEOUT_S 4
#define PHASE_RETRIES   3
/* renew/rebind REQUEST pacing */
#define RENEW_RESEND_S  8

typedef enum {
  ST_INIT = 0,
  ST_SELECTING,    /* DISCOVER sent, waiting for OFFER  */
  ST_REQUESTING,   /* REQUEST sent, waiting for ACK/NAK */
  ST_LEASED,
  ST_RENEWING,     /* past T1, unicast REQUEST to server */
  ST_REBINDING,    /* past T2, broadcast REQUEST         */
} phase_t;

static uint8_t  s_sn = 0;
static phase_t  s_phase = ST_INIT;
static uint8_t  s_mac[6];
static uint32_t s_xid;
static uint32_t s_ticks = 0;          /* seconds, from tick_1s */
static uint32_t s_phase_sent_at = 0;  /* s_ticks when the last frame went out */
static uint8_t  s_retries = 0;

static uint8_t  s_offered_ip[4];
static uint8_t  s_server_id[4];
static uint8_t  s_lease_ip[4];        /* currently bound lease, 0 if none */
static uint32_t s_lease_secs = 0;
static uint32_t s_t1 = 0, s_t2 = 0;   /* absolute, in s_ticks time */
static uint32_t s_lease_end = 0;
static uint32_t s_bound_at = 0;

static uint32_t s_acks = 0, s_naks = 0, s_timeouts = 0, s_renews = 0;

static uint8_t s_msg[DHCP_MSG_SIZE];

uint32_t w5500_dhcp_lease_seconds(void) { return s_lease_secs; }
uint32_t w5500_dhcp_acks_total(void)     { return s_acks; }
uint32_t w5500_dhcp_naks_total(void)     { return s_naks; }
uint32_t w5500_dhcp_timeouts_total(void) { return s_timeouts; }
uint32_t w5500_dhcp_renews_total(void)   { return s_renews; }

void w5500_dhcp_tick_1s(void) { s_ticks++; }

void w5500_dhcp_init(uint8_t sn) {
  s_sn = sn;
  w5500_get_mac(s_mac);
  s_xid = esp_random();
  s_phase = ST_INIT;
  s_retries = 0;
  /* RFC 2131 INIT: discover from 0.0.0.0, so a link bounce onto a different
   * subnet reacquires instead of squatting on the stale lease. */
  static const uint8_t zero[4] = {0, 0, 0, 0};
  w5500_bus_wr(W5500_CREG(W5500_SIPR), zero, 4);
  w5500_bus_wr(W5500_CREG(W5500_GAR),  zero, 4);
  memset(s_lease_ip, 0, 4);
  w5500_udp_open(s_sn, DHCP_CLIENT_PORT);
}

/* Returns the next option index into s_msg. */
static int build_header(uint8_t msg_type) {
  memset(s_msg, 0, sizeof(s_msg));
  s_msg[0] = BOOTP_REQUEST;
  s_msg[1] = 1;                        /* htype: ethernet */
  s_msg[2] = 6;                        /* hlen */
  s_msg[4] = (uint8_t)(s_xid >> 24);
  s_msg[5] = (uint8_t)(s_xid >> 16);
  s_msg[6] = (uint8_t)(s_xid >> 8);
  s_msg[7] = (uint8_t)(s_xid);
  /* Broadcast flag whenever SIPR may still be zero (a unicast reply would be
   * dropped on the way in); only a renewal is safe to ask unicast. */
  bool renewing = (msg_type == DHCP_REQUEST && s_phase == ST_RENEWING);
  bool have_ip = (s_phase == ST_RENEWING || s_phase == ST_REBINDING);
  if (!renewing) s_msg[10] = 0x80;
  if (have_ip) memcpy(&s_msg[12], s_lease_ip, 4);   /* ciaddr */
  memcpy(&s_msg[28], s_mac, 6);        /* chaddr */
  memcpy(&s_msg[236], MAGIC_COOKIE, 4);

  int k = 240;
  s_msg[k++] = OPT_MSG_TYPE; s_msg[k++] = 1; s_msg[k++] = msg_type;
  s_msg[k++] = OPT_CLIENT_ID; s_msg[k++] = 7; s_msg[k++] = 1;
  memcpy(&s_msg[k], s_mac, 6); k += 6;
  s_msg[k++] = OPT_HOSTNAME; s_msg[k++] = (uint8_t)strlen(HOSTNAME);
  memcpy(&s_msg[k], HOSTNAME, strlen(HOSTNAME)); k += strlen(HOSTNAME);
  return k;
}

static void finish_and_send(int k, const uint8_t dst_ip[4]) {
  s_msg[k++] = OPT_PARAM_LIST; s_msg[k++] = 6;
  s_msg[k++] = OPT_SUBNET_MASK; s_msg[k++] = OPT_ROUTER;
  s_msg[k++] = OPT_LEASE_TIME;  s_msg[k++] = OPT_SERVER_ID;
  s_msg[k++] = OPT_T1;          s_msg[k++] = OPT_T2;
  s_msg[k++] = OPT_END;
  bool unicast = memcmp(dst_ip, BCAST_IP, 4) != 0;
  uint8_t saved[3];
  if (unicast) {
    w5500_bus_rd(W5500_CREG(W5500_RTR), saved, 3);
    const uint8_t tight[3] = { 0x01, 0x90, 0x01 };
    w5500_bus_wr(W5500_CREG(W5500_RTR), tight, 3);
  }
  int32_t r = w5500_udp_sendto(s_sn, s_msg, DHCP_MSG_SIZE,
                               dst_ip, DHCP_SERVER_PORT);
  if (unicast) w5500_bus_wr(W5500_CREG(W5500_RTR), saved, 3);
  if (r != DHCP_MSG_SIZE) {
    /* Expected for a unicast renew with the server unreachable (ARP timeout);
     * the phase timer resends or falls through to rebind. */
    ESP_LOGW(TAG, "send failed (rc=%d) in phase %d", (int)r, (int)s_phase);
  }
  s_phase_sent_at = s_ticks;
}

static void send_discover(void) {
  int k = build_header(DHCP_DISCOVER);
  finish_and_send(k, BCAST_IP);
}

static void send_request(void) {
  int k = build_header(DHCP_REQUEST);
  if (s_phase == ST_REQUESTING) {      /* answering an OFFER */
    s_msg[k++] = OPT_REQUESTED_IP; s_msg[k++] = 4;
    memcpy(&s_msg[k], s_offered_ip, 4); k += 4;
    s_msg[k++] = OPT_SERVER_ID; s_msg[k++] = 4;
    memcpy(&s_msg[k], s_server_id, 4); k += 4;
  }
  finish_and_send(k, s_phase == ST_RENEWING ? s_server_id : BCAST_IP);
}

typedef struct {
  uint8_t type;
  uint8_t yiaddr[4];
  uint8_t server_id[4];
  uint8_t mask[4];
  uint8_t router[4];
  uint32_t lease, t1, t2;
  bool has_mask, has_router, has_server;
} reply_t;

static int parse_reply(const uint8_t* p, uint16_t n, reply_t* out) {
  if (n < 244) return -1;
  if (p[0] != BOOTP_REPLY) return -1;
  uint32_t xid = ((uint32_t)p[4] << 24) | ((uint32_t)p[5] << 16) |
                 ((uint32_t)p[6] << 8) | p[7];
  if (xid != s_xid) return -1;
  if (memcmp(&p[28], s_mac, 6) != 0) return -1;
  if (memcmp(&p[236], MAGIC_COOKIE, 4) != 0) return -1;

  memset(out, 0, sizeof(*out));
  memcpy(out->yiaddr, &p[16], 4);

  uint16_t i = 240;
  while (i + 1 < n) {
    uint8_t code = p[i];
    if (code == OPT_END) break;
    if (code == 0) { i++; continue; }
    uint8_t len = p[i + 1];
    if ((uint32_t)i + 2 + len > n) break;
    const uint8_t* v = &p[i + 2];
    switch (code) {
      case OPT_MSG_TYPE:    if (len >= 1) out->type = v[0]; break;
      case OPT_SERVER_ID:   if (len >= 4) { memcpy(out->server_id, v, 4); out->has_server = true; } break;
      case OPT_SUBNET_MASK: if (len >= 4) { memcpy(out->mask, v, 4); out->has_mask = true; } break;
      case OPT_ROUTER:      if (len >= 4) { memcpy(out->router, v, 4); out->has_router = true; } break;
      case OPT_LEASE_TIME:  if (len >= 4) out->lease = ((uint32_t)v[0] << 24) | ((uint32_t)v[1] << 16) | ((uint32_t)v[2] << 8) | v[3]; break;
      case OPT_T1:          if (len >= 4) out->t1 = ((uint32_t)v[0] << 24) | ((uint32_t)v[1] << 16) | ((uint32_t)v[2] << 8) | v[3]; break;
      case OPT_T2:          if (len >= 4) out->t2 = ((uint32_t)v[0] << 24) | ((uint32_t)v[1] << 16) | ((uint32_t)v[2] << 8) | v[3]; break;
      default: break;
    }
    i = (uint16_t)(i + 2 + len);
  }
  return out->type ? 0 : -1;
}

static w5500_dhcp_state_t bind_lease(const reply_t* r) {
  bool changed = (memcmp(s_lease_ip, r->yiaddr, 4) != 0) &&
                 (s_lease_ip[0] | s_lease_ip[1] | s_lease_ip[2] | s_lease_ip[3]);
  bool first = !(s_lease_ip[0] | s_lease_ip[1] | s_lease_ip[2] | s_lease_ip[3]);

  memcpy(s_lease_ip, r->yiaddr, 4);
  if (r->has_server) memcpy(s_server_id, r->server_id, 4);

  uint8_t mask[4] = { 255, 255, 255, 0 };
  uint8_t gw[4] = { 0, 0, 0, 0 };
  if (r->has_mask) memcpy(mask, r->mask, 4);
  if (r->has_router) memcpy(gw, r->router, 4);
  w5500_set_ipconf(s_lease_ip, gw, mask);

  s_lease_secs = r->lease ? r->lease : 3600;
  s_bound_at = s_ticks;
  if (s_lease_secs == 0xFFFFFFFF) {
    s_t1 = s_t2 = s_lease_end = 0xFFFFFFFF;
  } else {
    s_t1 = s_bound_at + (r->t1 ? r->t1 : s_lease_secs / 2);
    s_t2 = s_bound_at + (r->t2 ? r->t2 : (uint32_t)(s_lease_secs / 8ull * 7ull));
    s_lease_end = s_bound_at + s_lease_secs;
  }
  s_phase = ST_LEASED;
  s_retries = 0;
  s_acks++;
  return changed ? W5500_DHCP_CHANGED
                 : (first ? W5500_DHCP_ASSIGNED : W5500_DHCP_LEASED);
}

static void restart_discovery(void) {
  s_xid++;
  s_phase = ST_INIT;
  s_retries = 0;
}

static void drop_lease(void) {
  static const uint8_t zero[4] = {0, 0, 0, 0};
  w5500_bus_wr(W5500_CREG(W5500_SIPR), zero, 4);
  w5500_bus_wr(W5500_CREG(W5500_GAR),  zero, 4);
  memset(s_lease_ip, 0, 4);
}

/* ARP-probe an offered address before binding it: a 1-byte send resolves ARP,
 * so SENDOK means another host already answers for it. Tight RTR/RCR like
 * w5k_arp_prime so a free address costs ~80 ms, not the chip's ~1.8 s. */
static bool lease_ip_in_use(const uint8_t ip[4]) {
  uint8_t saved[3];
  w5500_bus_rd(W5500_CREG(W5500_RTR), saved, 3);
  const uint8_t tight[3] = { 0x01, 0x90, 0x01 };
  w5500_bus_wr(W5500_CREG(W5500_RTR), tight, 3);
  uint8_t dummy = 0;
  int32_t r = w5500_udp_sendto(s_sn, &dummy, 1, ip, 7);
  w5500_bus_wr(W5500_CREG(W5500_RTR), saved, 3);
  return r == 1;
}

static void send_decline(const uint8_t ip[4]) {
  int k = build_header(DHCP_DECLINE);
  s_msg[k++] = OPT_REQUESTED_IP; s_msg[k++] = 4;
  memcpy(&s_msg[k], ip, 4); k += 4;
  s_msg[k++] = OPT_SERVER_ID; s_msg[k++] = 4;
  memcpy(&s_msg[k], s_server_id, 4); k += 4;
  s_msg[k++] = OPT_END;
  w5500_udp_sendto(s_sn, s_msg, DHCP_MSG_SIZE, BCAST_IP, DHCP_SERVER_PORT);
}

w5500_dhcp_state_t w5500_dhcp_run(void) {
  if (w5500_sock_status(s_sn) != W5500_SR_UDP) {
    if (w5500_udp_open(s_sn, DHCP_CLIENT_PORT) != 0) return W5500_DHCP_RUNNING;
  }

  reply_t r;
  bool got_offer = false, got_ack = false, got_nak = false;
  reply_t offer = {0}, ack = {0};
  for (;;) {
    uint8_t from_ip[4]; uint16_t from_port;
    int32_t n = w5500_udp_recvfrom(s_sn, s_msg, sizeof(s_msg), from_ip, &from_port);
    if (n <= 0) break;
    if (from_port != DHCP_SERVER_PORT) continue;
    if (parse_reply(s_msg, (uint16_t)n, &r) != 0) continue;
    if (r.type == DHCP_OFFER) { offer = r; got_offer = true; }
    else if (r.type == DHCP_ACK) { ack = r; got_ack = true; }
    else if (r.type == DHCP_NAK) { got_nak = true; }
  }

  switch (s_phase) {
    case ST_INIT:
      send_discover();
      s_phase = ST_SELECTING;
      return W5500_DHCP_RUNNING;

    case ST_SELECTING:
      if (got_offer) {
        memcpy(s_offered_ip, offer.yiaddr, 4);
        memcpy(s_server_id, offer.server_id, 4);
        s_phase = ST_REQUESTING;
        s_retries = 0;
        send_request();
        return W5500_DHCP_RUNNING;
      }
      if (s_ticks - s_phase_sent_at >= PHASE_TIMEOUT_S) {
        s_timeouts++;
        if (++s_retries >= PHASE_RETRIES) { restart_discovery(); return W5500_DHCP_FAILED; }
        send_discover();
      }
      return W5500_DHCP_RUNNING;

    case ST_REQUESTING:
      if (got_ack) {
        /* Probe only a fresh address; renewals of the address we already
         * hold would ARP our own IP and prove nothing. */
        if (memcmp(ack.yiaddr, s_lease_ip, 4) != 0 && lease_ip_in_use(ack.yiaddr)) {
          ESP_LOGW(TAG, "offered %d.%d.%d.%d already in use — declining",
                   ack.yiaddr[0], ack.yiaddr[1], ack.yiaddr[2], ack.yiaddr[3]);
          send_decline(ack.yiaddr);
          restart_discovery();
          return W5500_DHCP_RUNNING;
        }
        return bind_lease(&ack);
      }
      if (got_nak) { s_naks++; drop_lease(); restart_discovery(); return W5500_DHCP_RUNNING; }
      if (s_ticks - s_phase_sent_at >= PHASE_TIMEOUT_S) {
        s_timeouts++;
        if (++s_retries >= PHASE_RETRIES) { restart_discovery(); return W5500_DHCP_FAILED; }
        send_request();
      }
      return W5500_DHCP_RUNNING;

    case ST_LEASED:
      if (s_ticks >= s_t2) {
        s_xid++;
        s_phase = ST_REBINDING;
        send_request();
      } else if (s_ticks >= s_t1) {
        s_xid++;
        s_phase = ST_RENEWING;
        s_renews++;
        send_request();
      }
      return W5500_DHCP_LEASED;

    case ST_RENEWING:
    case ST_REBINDING:
      if (got_ack) return bind_lease(&ack);
      if (got_nak) { s_naks++; drop_lease(); restart_discovery(); return W5500_DHCP_RUNNING; }
      if (s_ticks >= s_lease_end) {
        /* Keep the address programmed while rediscovering: the server must
         * keep serving, and the reservation makes a conflict implausible. */
        s_timeouts++;
        ESP_LOGW(TAG, "lease expired without renewal, restarting discovery");
        restart_discovery();
        return W5500_DHCP_RUNNING;
      }
      if (s_phase == ST_RENEWING && s_ticks >= s_t2) {
        s_phase = ST_REBINDING;
        send_request();
      } else if (s_ticks - s_phase_sent_at >= RENEW_RESEND_S) {
        s_timeouts++;
        if (s_phase == ST_RENEWING) s_renews++;
        send_request();
      }
      return W5500_DHCP_LEASED;
  }
  return W5500_DHCP_RUNNING;
}

#pragma once
// SPDX-License-Identifier: Unlicense
/*
 * Poll-driven DHCP client on one UDP socket (port 68). Call tick_1s() at
 * ~1 Hz and run() as often as convenient; renew/rebind/NAK handling and
 * post-failure restart all live inside run().
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  W5500_DHCP_RUNNING = 0,   /* no lease yet                              */
  W5500_DHCP_ASSIGNED,      /* first lease bound this cycle (once)       */
  W5500_DHCP_CHANGED,       /* lease bound with a different IP (once)    */
  W5500_DHCP_LEASED,        /* steady state                              */
  W5500_DHCP_FAILED,        /* retry budget exhausted (once, then retry) */
} w5500_dhcp_state_t;

/* Reads SHAR for chaddr, so the MAC must already be programmed. Safe to call
 * again after a chip reset or link bounce. */
void w5500_dhcp_init(uint8_t sn);
w5500_dhcp_state_t w5500_dhcp_run(void);
void w5500_dhcp_tick_1s(void);
uint32_t w5500_dhcp_lease_seconds(void);

uint32_t w5500_dhcp_acks_total(void);
uint32_t w5500_dhcp_naks_total(void);
uint32_t w5500_dhcp_timeouts_total(void);
uint32_t w5500_dhcp_renews_total(void);
/* Restarts that reclaimed the cached lease instead of rediscovering. */
uint32_t w5500_dhcp_reclaims_total(void);

/* Drop the cached lease so the next start runs a full discovery. */
void w5500_dhcp_forget(void);

#ifdef __cplusplus
}
#endif

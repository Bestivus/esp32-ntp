#pragma once
// SPDX-License-Identifier: Unlicense
/*
 * Reply-path attribution.
 *
 * The pre-existing ntp_tx_turnaround_us is INTn-capture to INTn-capture: it
 * starts at the RECV edge and ends at the SENDOK edge, which the W5500 only
 * asserts once the frame has already left the wire. It therefore structurally
 * contains the chip's egress latency and cannot be driven towards zero, so it
 * is useless for judging software work. These spans are the honest picture:
 * every one is delimited by esp_cpu_get_cycle_count() (a single instruction,
 * 6.25 ns at 160 MHz) inside the pinned NTP task, so reading them costs
 * essentially nothing on the path being measured.
 *
 * NTP_PROF_INT_TO_SEND is the headline number: hardware arrival edge to the
 * instant SEND is accepted by the chip.
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
  NTP_PROF_ISR_TO_LOOP = 0, /* INTn ISR stamp -> entry of NtpServer::loop()   */
  NTP_PROF_RX_READY,        /* the Sn_RX_RSR arrival probe                    */
  NTP_PROF_RX_STAMP,        /* capture/ISR-stamp snapshot                     */
  NTP_PROF_RECVFROM,        /* payload drain + RECV command + IR ack          */
  NTP_PROF_BUILD,           /* response assembly, t2 conversion, rate limit   */
  NTP_PROF_ARP,             /* w5k_arp_prime() when it runs                   */
  NTP_PROF_STAGE,           /* w5k_send_stage(): DIPR/DPORT/FSR/WR + 48 bytes */
  NTP_PROF_T3,              /* t3 computation between staging and the stamp   */
  NTP_PROF_STAMP_SEND,      /* 8-byte patch + SEND  (accuracy-critical)       */
  NTP_PROF_REAP,            /* SENDOK spin                                    */
  NTP_PROF_POST,            /* bookkeeping after the reap                     */
  NTP_PROF_INT_TO_SEND,     /* arrival edge -> SEND accepted  (the headline)  */
  NTP_PROF_TOTAL,           /* whole loop() body for a served packet          */
  NTP_PROF_COUNT
};

const char* ntp_prof_name(int i);
double      ntp_prof_ewma_us(int i);
double      ntp_prof_max_us(int i);
double      ntp_prof_min_us(int i);
uint32_t    ntp_prof_samples(void);

/* Per-packet SPI accounting up to the SEND, for the bytes/txns budget check. */
double ntp_prof_txns(void);
double ntp_prof_bytes(void);
double ntp_prof_sels(void);
double ntp_prof_reap_polls(void);
double   ntp_prof_prime_polls(void);
uint32_t ntp_prof_primes(void);   /* ARP primes actually performed */

#ifdef __cplusplus
}
#endif

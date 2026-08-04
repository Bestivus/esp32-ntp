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
 * 4.17 ns at 240 MHz) inside the pinned NTP task, so reading them costs
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
  NTP_PROF_HDR,             /* response header fields (stratum, refid, ...)   */
  NTP_PROF_T2CONV,          /* t2: capture->NTP, cross-check, calibration     */
  NTP_PROF_KOD,             /* rate-limit buckets                             */
  NTP_PROF_ARP,             /* w5k_arp_prime() when it runs BEFORE the stamp  */
  NTP_PROF_STAGE,           /* w5k_send_stage(): DIPR/DPORT/FSR/WR + 48 bytes */
  NTP_PROF_T3,              /* t3 computation between staging and the stamp   */
  NTP_PROF_STAMP_SEND,      /* 8-byte patch + SEND  (accuracy-critical)       */
  NTP_PROF_REAP,            /* SENDOK spin                                    */
  NTP_PROF_POST,            /* bookkeeping after the reap                     */
  NTP_PROF_INT_TO_SEND,     /* arrival edge -> SEND accepted  (the headline)  */
  /*
   * The same interval, but only over replies that did NOT have to resolve ARP
   * first. A pre-send prime is an extra frame plus its wire round trip, and it
   * fires whenever consecutive requests come from different peers, because the
   * W5500 holds exactly one destination MAC per socket. Averaging the two
   * together hides both: this is the software floor, INT_TO_SEND is what a
   * client population actually sees.
   */
  NTP_PROF_INT_TO_SEND_NP,
  NTP_PROF_TOTAL,           /* whole loop() body for a served packet          */
  NTP_PROF_COUNT
};

const char* ntp_prof_name(int i);
double      ntp_prof_ewma_us(int i);
double      ntp_prof_max_us(int i);
double      ntp_prof_min_us(int i);
uint32_t    ntp_prof_samples(void);

/* Distribution of INT_TO_SEND: a 5% EWMA cannot show a bimodal path. */
#define NTP_PROF_BUCKETS 8
uint32_t ntp_prof_bucket(int i);
uint32_t ntp_prof_bucket_edge_us(int i);   /* upper edge; 0 means +infinity */

/*
 * Per-reply SPI accounting.
 *
 * SCOPE MATTERS HERE and got reported wrongly once: the totals span the whole
 * loop() body, so they include the ARP prime when one runs — and the prime's
 * Sn_IR poll loop spins for as long as the ARP takes on the wire, which means
 * its transaction count GOES UP as SPI gets faster. Reading the total as "the
 * cost of serving a reply" therefore overstates it and moves the wrong way under
 * optimisation. ntp_prof_txns_serve() excludes the prime and is the number that
 * reflects the reply path itself.
 */
double ntp_prof_txns(void);         /* whole loop body, prime included */
double ntp_prof_txns_serve(void);   /* reply path only, prime excluded */
double ntp_prof_prime_txns(void);   /* transactions inside w5k_arp_prime */
double ntp_prof_bytes(void);
double ntp_prof_bytes_serve(void);
double ntp_prof_sels(void);
double ntp_prof_reap_polls(void);
double   ntp_prof_prime_polls(void);
uint32_t ntp_prof_primes(void);   /* ARP primes actually performed */

#ifdef __cplusplus
}
#endif

#pragma once
// SPDX-License-Identifier: Unlicense
//
// Minimal MQTT 3.1.1 wire-format codec: exactly the packet types this
// project's non-blocking publisher sends and receives -- CONNECT (optional
// username/password, a retained LWT), PUBLISH at QoS 0, PINGREQ, and parsing
// CONNACK. No subscribe/unsubscribe, no QoS 1/2, no packet-identifier
// tracking, no session resumption -- none of that is used, so none of it
// exists here. Hand-rolled rather than vendored so mqtt_publish has zero
// external dependencies and isn't coupled to any driver/library churn.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum MqttPacketType {
  CONNECT = 1, CONNACK, PUBLISH, PUBACK, PUBREC, PUBREL,
  PUBCOMP, SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK,
  PINGREQ, PINGRESP, DISCONNECT
};

typedef struct {
  char* cstring;   // always NUL-terminated text on this project's usage; never binary
} MQTTString;
#define MQTTString_initializer { NULL }

typedef struct {
  MQTTString topicName;
  MQTTString message;
  unsigned char qos;
  unsigned char retained;
} MQTTPacket_willOptions;

typedef struct {
  unsigned char MQTTVersion;
  MQTTString clientID;
  unsigned short keepAliveInterval;
  unsigned char cleansession;
  unsigned char willFlag;
  MQTTPacket_willOptions will;
  MQTTString username;   // cstring == NULL means "no username flag"
  MQTTString password;   // cstring == NULL means "no password flag"
} MQTTPacket_connectData;
#define MQTTPacket_connectData_initializer \
  { 4, MQTTString_initializer, 60, 1, 0, \
    { MQTTString_initializer, MQTTString_initializer, 0, 0 }, \
    MQTTString_initializer, MQTTString_initializer }

// Returns bytes written, or -1 if buf is too small.
int MQTTSerialize_connect(unsigned char* buf, int buflen, MQTTPacket_connectData* options);
int MQTTSerialize_publish(unsigned char* buf, int buflen, unsigned char dup, int qos,
                           unsigned char retained, unsigned short packetid,
                           MQTTString topicName, unsigned char* payload, int payloadlen);
int MQTTSerialize_pingreq(unsigned char* buf, int buflen);

// Returns 1 on success, 0 if buf is too short to have been a real CONNACK.
int MQTTDeserialize_connack(unsigned char* sessionPresent, unsigned char* connack_rc,
                             unsigned char* buf, int buflen);

// getfn contract: return -1 on transport error, 0 for "nothing yet, call
// again", or the number of bytes actually written to buf (1..len).
typedef struct {
  int (*getfn)(void* sck, unsigned char* buf, int len);
  void* sck;
  // Incremental-read progress. Callers only ever need to zero `state` (e.g.
  // via `transport = {}` or an explicit `transport.state = 0`) to reset to a
  // clean slate -- readnb() re-derives the rest from state==0 itself, and
  // resets state back to 0 on its own the moment a packet completes or
  // errors, so the transport is always ready for the next packet without the
  // caller tracking anything beyond "did I just reset this."
  int state;            // 0 = read header byte, 1 = decode remaining length, 2 = read payload
  unsigned char header;  // fixed header byte, valid once state > 0
  int multiplier;         // remaining-length varint decode progress
  int rem_len;             // decoded remaining length (payload byte count)
  int have;                 // payload bytes read so far
} MQTTTransport;

// Non-blocking incremental packet read, one interior `getfn` call at a time.
// Returns the packet type (enum MqttPacketType) once a full packet has
// landed in buf (buf holds only the variable-header+payload portion, matching
// what MQTTDeserialize_connack() expects), 0 if not yet complete, or -1 on a
// transport error or a packet too large for buf.
int MQTTPacket_readnb(unsigned char* buf, int buflen, MQTTTransport* trp);

#ifdef __cplusplus
}
#endif

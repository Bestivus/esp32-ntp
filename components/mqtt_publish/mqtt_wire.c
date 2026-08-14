// SPDX-License-Identifier: Unlicense

#include "mqtt_wire.h"
#include <string.h>

// MQTT 3.1.1 remaining-length varint: 7 bits of value per byte, MSB is the
// continuation flag, up to 4 bytes (values above 2^28-1 don't occur here --
// every packet this project sends or parses is well under 268 million bytes).
static int encodeRemainingLength(unsigned char* buf, int length) {
  int n = 0;
  do {
    unsigned char d = (unsigned char)(length % 128);
    length /= 128;
    if (length > 0) d |= 0x80;
    buf[n++] = d;
  } while (length > 0);
  return n;
}

static int remainingLengthBytes(int length) {
  if (length < 128) return 1;
  if (length < 16384) return 2;
  if (length < 2097152) return 3;
  return 4;
}

// 2-byte big-endian length prefix + raw bytes, no NUL on the wire.
static int cstringLen(const char* s) {
  return 2 + (s ? (int)strlen(s) : 0);
}
static int writeCString(unsigned char* buf, const char* s) {
  int len = s ? (int)strlen(s) : 0;
  buf[0] = (unsigned char)(len >> 8);
  buf[1] = (unsigned char)(len & 0xFF);
  if (len) memcpy(buf + 2, s, len);
  return len + 2;
}

int MQTTSerialize_connect(unsigned char* buf, int buflen, MQTTPacket_connectData* options) {
  // Variable header: "MQTT" (2+4) + protocol level (1) + connect flags (1) +
  // keepalive (2) = 10 bytes, fixed regardless of what's enabled below.
  int payloadLen = cstringLen(options->clientID.cstring);
  unsigned char connectFlags = (unsigned char)(options->cleansession ? 0x02 : 0x00);
  if (options->willFlag) {
    connectFlags |= 0x04;
    connectFlags = (unsigned char)(connectFlags | ((options->will.qos & 0x03) << 3));
    if (options->will.retained) connectFlags |= 0x20;
    payloadLen += cstringLen(options->will.topicName.cstring);
    payloadLen += cstringLen(options->will.message.cstring);
  }
  if (options->username.cstring) { connectFlags |= 0x80; payloadLen += cstringLen(options->username.cstring); }
  if (options->password.cstring) { connectFlags |= 0x40; payloadLen += cstringLen(options->password.cstring); }

  int remLen = 10 + payloadLen;
  int total = 1 + remainingLengthBytes(remLen) + remLen;
  if (total > buflen) return -1;

  unsigned char* p = buf;
  *p++ = 0x10;  // CONNECT, fixed-header flags always 0 for this packet type
  p += encodeRemainingLength(p, remLen);
  p += writeCString(p, "MQTT");
  *p++ = 4;  // protocol level: MQTT 3.1.1
  *p++ = connectFlags;
  *p++ = (unsigned char)(options->keepAliveInterval >> 8);
  *p++ = (unsigned char)(options->keepAliveInterval & 0xFF);
  p += writeCString(p, options->clientID.cstring);
  if (options->willFlag) {
    p += writeCString(p, options->will.topicName.cstring);
    p += writeCString(p, options->will.message.cstring);
  }
  if (options->username.cstring) p += writeCString(p, options->username.cstring);
  if (options->password.cstring) p += writeCString(p, options->password.cstring);
  return (int)(p - buf);
}

int MQTTSerialize_publish(unsigned char* buf, int buflen, unsigned char dup, int qos,
                           unsigned char retained, unsigned short packetid,
                           MQTTString topicName, unsigned char* payload, int payloadlen) {
  int varHeaderLen = cstringLen(topicName.cstring) + (qos > 0 ? 2 : 0);
  int remLen = varHeaderLen + payloadlen;
  int total = 1 + remainingLengthBytes(remLen) + remLen;
  if (total > buflen) return -1;

  unsigned char* p = buf;
  *p++ = (unsigned char)(0x30 | ((dup & 1) << 3) | ((qos & 3) << 1) | (retained & 1));
  p += encodeRemainingLength(p, remLen);
  p += writeCString(p, topicName.cstring);
  if (qos > 0) {
    *p++ = (unsigned char)(packetid >> 8);
    *p++ = (unsigned char)(packetid & 0xFF);
  }
  if (payloadlen > 0) memcpy(p, payload, (size_t)payloadlen);
  p += payloadlen;
  return (int)(p - buf);
}

int MQTTSerialize_pingreq(unsigned char* buf, int buflen) {
  if (buflen < 2) return -1;
  buf[0] = 0xC0;
  buf[1] = 0x00;
  return 2;
}

int MQTTDeserialize_connack(unsigned char* sessionPresent, unsigned char* connack_rc,
                             unsigned char* buf, int buflen) {
  if (buflen < 2) return 0;
  *sessionPresent = (unsigned char)(buf[0] & 0x01);
  *connack_rc = buf[1];
  return 1;
}

int MQTTPacket_readnb(unsigned char* buf, int buflen, MQTTTransport* trp) {
  if (trp->state == 0) {
    trp->multiplier = 1;
    trp->rem_len = 0;
    trp->have = 0;
    unsigned char b;
    int n = trp->getfn(trp->sck, &b, 1);
    if (n < 0) return -1;
    if (n == 0) return 0;
    trp->header = b;
    trp->state = 1;
  }

  if (trp->state == 1) {
    for (;;) {
      unsigned char b;
      int n = trp->getfn(trp->sck, &b, 1);
      if (n < 0) { trp->state = 0; return -1; }
      if (n == 0) return 0;   // partial remaining-length; resume here next call
      trp->rem_len += (b & 0x7F) * trp->multiplier;
      if ((b & 0x80) == 0) break;
      trp->multiplier *= 128;
      if (trp->multiplier > 128 * 128 * 128) { trp->state = 0; return -1; }  // malformed (>4 bytes)
    }
    if (trp->rem_len > buflen) { trp->state = 0; return -1; }  // wouldn't fit -- treat as an error
    trp->state = 2;
  }

  while (trp->have < trp->rem_len) {
    int want = trp->rem_len - trp->have;
    int n = trp->getfn(trp->sck, buf + trp->have, want);
    if (n < 0) { trp->state = 0; return -1; }
    if (n == 0) return 0;   // partial payload; resume here next call
    trp->have += n;
  }

  int type = (trp->header >> 4) & 0x0F;
  trp->state = 0;   // ready for the next packet without the caller doing anything
  return type;
}

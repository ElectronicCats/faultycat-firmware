#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// services/host_proto/frame_proto — shared SOF/LEN/CRC framing layer
// used by both emfi_proto (CDC0) and crowbar_proto (CDC1), which speak
// byte-identical wire formats:
//   [0]    SOF
//   [1]    CMD (host->device) or CMD|0x80 (device->host reply)
//   [2..3] LEN (little-endian)
//   [4..]  PAYLOAD
//   [4+LEN..] CRC-16/CCITT (poly 0x1021, init 0xFFFF), LE, over CMD+LEN+PAYLOAD
//
// `sof` and `interbyte_ms` are passed in per call rather than baked
// into this module so the two protocols stay independently
// configurable even though their values currently match.

#define FRAME_PROTO_MAX_PAYLOAD 512u

typedef enum {
    FRAME_S_SOF = 0,
    FRAME_S_CMD,
    FRAME_S_LEN_LO,
    FRAME_S_LEN_HI,
    FRAME_S_PAYLOAD,
    FRAME_S_CRC_LO,
    FRAME_S_CRC_HI,
} frame_parse_state_t;

typedef struct {
    frame_parse_state_t state;
    uint8_t cmd;
    uint16_t len;
    uint16_t payload_pos;
    uint8_t payload[FRAME_PROTO_MAX_PAYLOAD];
    uint16_t crc_recv;
    uint32_t last_byte_ms;

    bool frame_ready;
    uint8_t frame_cmd;
    uint16_t frame_len;
    uint8_t frame_payload[FRAME_PROTO_MAX_PAYLOAD];
} frame_parser_t;

void frame_proto_init(frame_parser_t* p);

// Feed one byte into the parser. Returns true iff a complete,
// CRC-valid frame was just assembled (then `p->frame_cmd/frame_len/
// frame_payload` hold it until the next call). On parse error (bad
// SOF, LEN overflow, CRC mismatch, inter-byte timeout) the state is
// reset.
bool frame_proto_feed(frame_parser_t* p, uint8_t byte, uint32_t now_ms, uint8_t sof,
                      uint32_t interbyte_ms);

// Write a full frame (SOF+CMD+LEN+PAYLOAD+CRC) into `out`. Returns 0
// if it doesn't fit or `len` exceeds FRAME_PROTO_MAX_PAYLOAD.
size_t frame_proto_write(uint8_t* out, size_t cap, uint8_t sof, uint8_t cmd_reply,
                         const uint8_t* payload, uint16_t len);

uint16_t frame_proto_crc16(const uint8_t* data, size_t len);

void frame_proto_pack_u32_le(uint8_t* p, uint32_t v);
uint32_t frame_proto_unpack_u32_le(const uint8_t* p);
uint16_t frame_proto_unpack_u16_le(const uint8_t* p);

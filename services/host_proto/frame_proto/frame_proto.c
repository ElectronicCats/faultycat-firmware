#include "frame_proto.h"

#include <string.h>

uint16_t frame_proto_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void frame_proto_init(frame_parser_t* p) {
    memset(p, 0, sizeof(*p));
}

static void reset_parser(frame_parser_t* p) {
    p->state       = FRAME_S_SOF;
    p->cmd         = 0;
    p->len         = 0;
    p->payload_pos = 0;
    p->crc_recv    = 0;
}

bool frame_proto_feed(frame_parser_t* p, uint8_t byte, uint32_t now_ms, uint8_t sof,
                      uint32_t interbyte_ms) {
    if (p->state != FRAME_S_SOF && (now_ms - p->last_byte_ms) > interbyte_ms) {
        reset_parser(p);
    }
    p->last_byte_ms = now_ms;

    switch (p->state) {
        case FRAME_S_SOF:
            if (byte == sof)
                p->state = FRAME_S_CMD;
            return false;
        case FRAME_S_CMD:
            p->cmd   = byte;
            p->state = FRAME_S_LEN_LO;
            return false;
        case FRAME_S_LEN_LO:
            p->len   = byte;
            p->state = FRAME_S_LEN_HI;
            return false;
        case FRAME_S_LEN_HI:
            p->len |= (uint16_t)byte << 8;
            if (p->len > FRAME_PROTO_MAX_PAYLOAD) {
                reset_parser(p);
                return false;
            }
            p->payload_pos = 0;
            p->state       = (p->len == 0) ? FRAME_S_CRC_LO : FRAME_S_PAYLOAD;
            return false;
        case FRAME_S_PAYLOAD:
            p->payload[p->payload_pos++] = byte;
            if (p->payload_pos >= p->len)
                p->state = FRAME_S_CRC_LO;
            return false;
        case FRAME_S_CRC_LO:
            p->crc_recv = byte;
            p->state    = FRAME_S_CRC_HI;
            return false;
        case FRAME_S_CRC_HI: {
            p->crc_recv |= (uint16_t)byte << 8;
            uint8_t hdr[3] = {p->cmd, (uint8_t)(p->len & 0xFFu), (uint8_t)((p->len >> 8) & 0xFFu)};
            uint16_t calc  = frame_proto_crc16(hdr, 3);
            for (uint16_t i = 0; i < p->len; i++) {
                uint16_t crc = calc;
                crc ^= (uint16_t)p->payload[i] << 8;
                for (int b = 0; b < 8; b++) {
                    crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
                }
                calc = crc;
            }
            bool ok = (calc == p->crc_recv);
            if (ok) {
                p->frame_cmd = p->cmd;
                p->frame_len = p->len;
                memcpy(p->frame_payload, p->payload, p->len);
                p->frame_ready = true;
            }
            reset_parser(p);
            return ok;
        }
    }
    return false;
}

size_t frame_proto_write(uint8_t* out, size_t cap, uint8_t sof, uint8_t cmd_reply,
                         const uint8_t* payload, uint16_t len) {
    if (len > FRAME_PROTO_MAX_PAYLOAD)
        return 0;
    size_t needed = 1u + 1u + 2u + (size_t)len + 2u;
    if (cap < needed)
        return 0;
    out[0] = sof;
    out[1] = cmd_reply;
    out[2] = (uint8_t)(len & 0xFFu);
    out[3] = (uint8_t)((len >> 8) & 0xFFu);
    if (len)
        memcpy(&out[4], payload, len);
    uint16_t crc     = frame_proto_crc16(&out[1], 3u + (size_t)len);
    out[4 + len]     = (uint8_t)(crc & 0xFFu);
    out[4 + len + 1] = (uint8_t)((crc >> 8) & 0xFFu);
    return needed;
}

void frame_proto_pack_u32_le(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)(v);
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

uint32_t frame_proto_unpack_u32_le(const uint8_t* p) {
    return ((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

uint16_t frame_proto_unpack_u16_le(const uint8_t* p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

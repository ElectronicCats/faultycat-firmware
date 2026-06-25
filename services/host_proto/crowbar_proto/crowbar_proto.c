#include "crowbar_proto.h"

#include <string.h>

#include "campaign_proto.h"
#include "crowbar_campaign.h"
#include "crowbar_pio.h"
#include "firmware_version.h"
#include "frame_proto.h"

// Parser state ---------------------------------------------------------------

static frame_parser_t s_parser;

uint16_t crowbar_proto_crc16(const uint8_t* data, size_t len) {
    return frame_proto_crc16(data, len);
}

void crowbar_proto_init(void) {
    frame_proto_init(&s_parser);
}

bool crowbar_proto_feed(uint8_t byte, uint32_t now_ms) {
    return frame_proto_feed(&s_parser, byte, now_ms, CROWBAR_PROTO_SOF, CROWBAR_PROTO_INTERBYTE_MS);
}

// Writer ---------------------------------------------------------------------

static size_t write_frame(uint8_t* out, size_t cap, uint8_t cmd_reply, const uint8_t* payload,
                          uint16_t len) {
    return frame_proto_write(out, cap, CROWBAR_PROTO_SOF, cmd_reply, payload, len);
}

// Payload packing helpers ----------------------------------------------------

static void pack_u32_le(uint8_t* p, uint32_t v) {
    frame_proto_pack_u32_le(p, v);
}

static uint32_t unpack_u32_le(const uint8_t* p) {
    return frame_proto_unpack_u32_le(p);
}

// Dispatch -------------------------------------------------------------------
//
// Wire layout for CONFIGURE payload (10 bytes, LE):
//   [0]    trigger    (crowbar_trig_t)
//   [1]    output     (crowbar_out_t — 1=LP, 2=HP)
//   [2..5] delay_us
//   [6..9] width_ns
//
// Wire layout for STATUS reply payload (15 bytes):
//   [0]      state                  (crowbar_state_t)
//   [1]      err                    (crowbar_err_t)
//   [2..5]   last_fire_at_ms
//   [6..9]   pulse_width_ns_actual
//   [10..13] delay_us_actual
//   [14]     output                 (crowbar_out_t)

#define CONFIGURE_PAYLOAD_LEN 10u
#define STATUS_REPLY_LEN      15u
#define FIRE_PAYLOAD_LEN      4u

size_t crowbar_proto_dispatch(uint8_t* reply, size_t reply_cap) {
    if (!s_parser.frame_ready || !reply)
        return 0;
    s_parser.frame_ready = false;

    const uint8_t s_frame_cmd            = s_parser.frame_cmd;
    const uint16_t s_frame_len           = s_parser.frame_len;
    const uint8_t* const s_frame_payload = s_parser.frame_payload;

    uint8_t rpl[CROWBAR_PROTO_MAX_PAYLOAD];
    uint16_t rpl_len = 0;
    uint8_t err      = CROWBAR_ERR_NONE;

    switch (s_frame_cmd) {
        case CROWBAR_CMD_PING: {
            // F11 release: see emfi_proto.c for the rationale. Family
            // byte stays '5' to keep the same shape across the two CDC
            // protocols; only the family byte distinguishes them.
            static const uint8_t pong[] = {
                'F',
                '5',
                (uint8_t)FW_VERSION_MAJOR,
                (uint8_t)FW_VERSION_MINOR,
                (uint8_t)FW_VERSION_PATCH,
                (uint8_t)FW_VERSION_TWEAK,
            };
            memcpy(rpl, pong, sizeof(pong));
            rpl_len = (uint16_t)sizeof(pong);
            break;
        }
        case CROWBAR_CMD_CONFIGURE: {
            if (s_frame_len < CONFIGURE_PAYLOAD_LEN) {
                err     = CROWBAR_ERR_BAD_CONFIG;
                rpl[0]  = err;
                rpl_len = 1;
                break;
            }
            crowbar_config_t c = {
                .trigger  = (crowbar_trig_t)s_frame_payload[0],
                .output   = (crowbar_out_t)s_frame_payload[1],
                .delay_us = unpack_u32_le(&s_frame_payload[2]),
                .width_ns = unpack_u32_le(&s_frame_payload[6]),
            };
            if (!crowbar_campaign_configure(&c))
                err = CROWBAR_ERR_BAD_CONFIG;
            rpl[0]  = err;
            rpl_len = 1;
            break;
        }
        case CROWBAR_CMD_ARM: {
            if (!crowbar_campaign_arm())
                err = CROWBAR_ERR_BAD_CONFIG;
            rpl[0]  = err;
            rpl_len = 1;
            break;
        }
        case CROWBAR_CMD_FIRE: {
            if (s_frame_len < FIRE_PAYLOAD_LEN) {
                err     = CROWBAR_ERR_BAD_CONFIG;
                rpl[0]  = err;
                rpl_len = 1;
                break;
            }
            uint32_t to = unpack_u32_le(s_frame_payload);
            if (!crowbar_campaign_fire(to))
                err = CROWBAR_ERR_INTERNAL;
            rpl[0]  = err;
            rpl_len = 1;
            break;
        }
        case CROWBAR_CMD_DISARM: {
            crowbar_campaign_disarm();
            rpl[0]  = CROWBAR_ERR_NONE;
            rpl_len = 1;
            break;
        }
        case CROWBAR_CMD_STATUS: {
            crowbar_status_t s;
            crowbar_campaign_get_status(&s);
            rpl[0] = (uint8_t)s.state;
            rpl[1] = (uint8_t)s.err;
            pack_u32_le(&rpl[2], s.last_fire_at_ms);
            pack_u32_le(&rpl[6], s.pulse_width_ns_actual);
            pack_u32_le(&rpl[10], s.delay_us_actual);
            rpl[14] = (uint8_t)s.output;
            rpl_len = STATUS_REPLY_LEN;
            break;
        }
        // F9-4 — campaign opcodes. Engine forced to CROWBAR since
        // we arrived on CDC1; identical wire format to emfi_proto.
        case CAMPAIGN_CMD_CONFIG: {
            campaign_config_t cfg;
            if (!campaign_proto_decode_config(s_frame_payload, s_frame_len, CAMPAIGN_ENGINE_CROWBAR,
                                              &cfg)) {
                rpl[0] = CAMPAIGN_PROTO_ERR_BAD_LEN;
            } else {
                rpl[0] = campaign_proto_apply_config(&cfg);
            }
            rpl_len = 1;
            break;
        }
        case CAMPAIGN_CMD_START: {
            rpl[0]  = campaign_manager_start() ? CAMPAIGN_PROTO_OK : CAMPAIGN_PROTO_ERR_REJECTED;
            rpl_len = 1;
            break;
        }
        case CAMPAIGN_CMD_STOP: {
            campaign_manager_stop();
            rpl[0]  = CAMPAIGN_PROTO_OK;
            rpl_len = 1;
            break;
        }
        case CAMPAIGN_CMD_STATUS: {
            rpl_len = (uint16_t)campaign_proto_serialize_status(rpl, sizeof(rpl));
            break;
        }
        case CAMPAIGN_CMD_DRAIN: {
            uint8_t max_count = (s_frame_len >= 1u) ? s_frame_payload[0] : 1u;
            static uint8_t drain_buf[1u + (CAMPAIGN_DRAIN_MAX_COUNT * 28u)];
            size_t drain_len =
                campaign_proto_serialize_drain(drain_buf, sizeof(drain_buf), max_count);
            return write_frame(reply, reply_cap, (uint8_t)(s_frame_cmd | 0x80u), drain_buf,
                               (uint16_t)drain_len);
        }

        default:
            err     = CROWBAR_ERR_BAD_CONFIG;
            rpl[0]  = err;
            rpl_len = 1;
            break;
    }

    return write_frame(reply, reply_cap, (uint8_t)(s_frame_cmd | 0x80u), rpl, rpl_len);
}

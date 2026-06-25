#include "emfi_proto.h"

#include <string.h>

#include "campaign_proto.h"
#include "emfi_campaign.h"
#include "firmware_version.h"
#include "frame_proto.h"

// Parser state ---------------------------------------------------------------

static frame_parser_t s_parser;

uint16_t emfi_proto_crc16(const uint8_t* data, size_t len) {
    return frame_proto_crc16(data, len);
}

void emfi_proto_init(void) {
    frame_proto_init(&s_parser);
}

bool emfi_proto_feed(uint8_t byte, uint32_t now_ms) {
    return frame_proto_feed(&s_parser, byte, now_ms, EMFI_PROTO_SOF, EMFI_PROTO_INTERBYTE_MS);
}

// Writer ---------------------------------------------------------------------

static size_t write_frame(uint8_t* out, size_t cap, uint8_t cmd_reply, const uint8_t* payload,
                          uint16_t len) {
    return frame_proto_write(out, cap, EMFI_PROTO_SOF, cmd_reply, payload, len);
}

// Payload packing helpers ----------------------------------------------------

static void pack_u32_le(uint8_t* p, uint32_t v) {
    frame_proto_pack_u32_le(p, v);
}

static uint32_t unpack_u32_le(const uint8_t* p) {
    return frame_proto_unpack_u32_le(p);
}

static uint16_t unpack_u16_le(const uint8_t* p) {
    return frame_proto_unpack_u16_le(p);
}

// Dispatch -------------------------------------------------------------------

size_t emfi_proto_dispatch(uint8_t* reply, size_t reply_cap) {
    if (!s_parser.frame_ready || !reply)
        return 0;
    s_parser.frame_ready = false;

    const uint8_t s_frame_cmd            = s_parser.frame_cmd;
    const uint16_t s_frame_len           = s_parser.frame_len;
    const uint8_t* const s_frame_payload = s_parser.frame_payload;

    uint8_t rpl[64];
    uint16_t rpl_len = 0;
    uint8_t err      = EMFI_ERR_NONE;

    switch (s_frame_cmd) {
        case EMFI_CMD_PING: {
            // F11 release: PING reply now carries the firmware version
            // so the host can fail-closed on a CLI/firmware mismatch.
            // Layout: 'F', family ('4' for emfi), MAJ, MIN, PATCH, TWEAK.
            // Older firmware replied with 4 bytes (the trailing 0,0
            // were placeholder); the host treats a 4-byte reply as a
            // pre-versioning firmware and refuses to connect.
            static const uint8_t pong[] = {
                'F',
                '4',
                (uint8_t)FW_VERSION_MAJOR,
                (uint8_t)FW_VERSION_MINOR,
                (uint8_t)FW_VERSION_PATCH,
                (uint8_t)FW_VERSION_TWEAK,
            };
            memcpy(rpl, pong, sizeof(pong));
            rpl_len = (uint16_t)sizeof(pong);
            break;
        }
        case EMFI_CMD_CONFIGURE: {
            if (s_frame_len < 1u + 4u + 4u + 4u) {
                err = EMFI_ERR_BAD_CONFIG;
                break;
            }
            emfi_config_t c = {
                .trigger           = (emfi_trig_t)s_frame_payload[0],
                .delay_us          = unpack_u32_le(&s_frame_payload[1]),
                .width_us          = unpack_u32_le(&s_frame_payload[5]),
                .charge_timeout_ms = unpack_u32_le(&s_frame_payload[9]),
            };
            if (!emfi_campaign_configure(&c))
                err = EMFI_ERR_BAD_CONFIG;
            rpl[0]  = err;
            rpl_len = 1;
            break;
        }
        case EMFI_CMD_ARM: {
            if (!emfi_campaign_arm())
                err = EMFI_ERR_BAD_CONFIG;
            rpl[0]  = err;
            rpl_len = 1;
            break;
        }
        case EMFI_CMD_FIRE: {
            if (s_frame_len < 4u) {
                err     = EMFI_ERR_BAD_CONFIG;
                rpl[0]  = err;
                rpl_len = 1;
                break;
            }
            uint32_t to = unpack_u32_le(s_frame_payload);
            if (!emfi_campaign_fire(to))
                err = EMFI_ERR_INTERNAL;
            rpl[0]  = err;
            rpl_len = 1;
            break;
        }
        case EMFI_CMD_DISARM: {
            emfi_campaign_disarm();
            rpl[0]  = EMFI_ERR_NONE;
            rpl_len = 1;
            break;
        }
        case EMFI_CMD_STATUS: {
            emfi_status_t s;
            emfi_campaign_get_status(&s);
            rpl[0] = (uint8_t)s.state;
            rpl[1] = (uint8_t)s.err;
            pack_u32_le(&rpl[2], s.last_fire_at_ms);
            pack_u32_le(&rpl[6], s.capture_fill);
            pack_u32_le(&rpl[10], s.pulse_width_us_actual);
            pack_u32_le(&rpl[14], s.delay_us_actual);
            rpl_len = 18;
            break;
        }
        case EMFI_CMD_CAPTURE: {
            if (s_frame_len < 4u) {
                err     = EMFI_ERR_BAD_CONFIG;
                rpl[0]  = err;
                rpl_len = 1;
                break;
            }
            uint16_t off = unpack_u16_le(&s_frame_payload[0]);
            uint16_t len = unpack_u16_le(&s_frame_payload[2]);
            // Reject rather than clamp — host must be able to
            // distinguish "got everything requested" from a truncation.
            if (len > 512u) {
                err     = EMFI_ERR_BAD_CONFIG;
                rpl[0]  = err;
                rpl_len = 1;
                break;
            }
            if ((uint32_t)off + len > 8192u) {
                err     = EMFI_ERR_BAD_CONFIG;
                rpl[0]  = err;
                rpl_len = 1;
                break;
            }
            const uint8_t* buf = emfi_campaign_capture_buffer();
            return write_frame(reply, reply_cap, (uint8_t)(s_frame_cmd | 0x80u), &buf[off], len);
        }
        // F9-4 — campaign opcodes. Engine forced to EMFI since we
        // arrived on CDC0; the wire format itself is engine-agnostic
        // (crowbar_proto uses identical payloads, sets engine =
        // CROWBAR before calling campaign_proto helpers).
        case CAMPAIGN_CMD_CONFIG: {
            campaign_config_t cfg;
            if (!campaign_proto_decode_config(s_frame_payload, s_frame_len, CAMPAIGN_ENGINE_EMFI,
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
            // Build the reply payload in a stack buffer big enough
            // for CAMPAIGN_DRAIN_MAX_COUNT × 28 + 1 hdr.
            static uint8_t drain_buf[1u + (CAMPAIGN_DRAIN_MAX_COUNT * 28u)];
            size_t drain_len =
                campaign_proto_serialize_drain(drain_buf, sizeof(drain_buf), max_count);
            return write_frame(reply, reply_cap, (uint8_t)(s_frame_cmd | 0x80u), drain_buf,
                               (uint16_t)drain_len);
        }
        default:
            err     = EMFI_ERR_BAD_CONFIG;
            rpl[0]  = err;
            rpl_len = 1;
            break;
    }

    return write_frame(reply, reply_cap, (uint8_t)(s_frame_cmd | 0x80u), rpl, rpl_len);
}

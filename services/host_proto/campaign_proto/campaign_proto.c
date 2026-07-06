/*
 * services/host_proto/campaign_proto/campaign_proto.c — shared
 * payload helpers for the CAMPAIGN_* opcode subset that both
 * emfi_proto (CDC0) and crowbar_proto (CDC1) implement.
 */

#include "campaign_proto.h"

#include <string.h>

#include "frame_proto.h"

bool campaign_proto_decode_config(const uint8_t* payload, size_t len, campaign_engine_t engine,
                                  campaign_config_t* out) {
    if (payload == NULL || out == NULL)
        return false;
    if (len != CAMPAIGN_CONFIG_PAYLOAD_LEN)
        return false;

    out->engine      = engine;
    out->delay.start = frame_proto_unpack_u32_le(&payload[0]);
    out->delay.end   = frame_proto_unpack_u32_le(&payload[4]);
    out->delay.step  = frame_proto_unpack_u32_le(&payload[8]);
    out->width.start = frame_proto_unpack_u32_le(&payload[12]);
    out->width.end   = frame_proto_unpack_u32_le(&payload[16]);
    out->width.step  = frame_proto_unpack_u32_le(&payload[20]);
    out->power.start = frame_proto_unpack_u32_le(&payload[24]);
    out->power.end   = frame_proto_unpack_u32_le(&payload[28]);
    out->power.step  = frame_proto_unpack_u32_le(&payload[32]);
    out->settle_ms   = frame_proto_unpack_u32_le(&payload[36]);
    return true;
}

uint8_t campaign_proto_apply_config(const campaign_config_t* cfg) {
    if (cfg == NULL)
        return CAMPAIGN_PROTO_ERR_BAD_LEN;
    if (!campaign_manager_configure(cfg))
        return CAMPAIGN_PROTO_ERR_REJECTED;
    return CAMPAIGN_PROTO_OK;
}

size_t campaign_proto_serialize_status(uint8_t* out, size_t cap) {
    if (out == NULL || cap < CAMPAIGN_STATUS_REPLY_LEN)
        return 0;
    campaign_status_t st;
    campaign_manager_get_status(&st);

    out[0] = (uint8_t)st.state;
    out[1] = (uint8_t)st.err;
    out[2] = 0u; // reserved
    out[3] = 0u;
    frame_proto_pack_u32_le(&out[4], st.step_n);
    frame_proto_pack_u32_le(&out[8], st.total_steps);
    frame_proto_pack_u32_le(&out[12], st.results_pushed);
    frame_proto_pack_u32_le(&out[16], st.results_dropped);
    return CAMPAIGN_STATUS_REPLY_LEN;
}

size_t campaign_proto_serialize_drain(uint8_t* out, size_t cap, uint8_t max_count) {
    if (out == NULL || cap < CAMPAIGN_DRAIN_REPLY_HDR_LEN)
        return 0;
    if (max_count == 0u)
        max_count = 1u;
    if (max_count > CAMPAIGN_DRAIN_MAX_COUNT)
        max_count = CAMPAIGN_DRAIN_MAX_COUNT;

    // Cap by the available reply space — sizeof(campaign_result_t)
    // is 28 B; we need 1 byte hdr + n × 28 B body to fit in `cap`.
    uint8_t fits_in_cap =
        (uint8_t)((cap - CAMPAIGN_DRAIN_REPLY_HDR_LEN) / sizeof(campaign_result_t));
    if (max_count > fits_in_cap)
        max_count = fits_in_cap;
    if (max_count == 0u) {
        out[0] = 0u;
        return CAMPAIGN_DRAIN_REPLY_HDR_LEN;
    }

    campaign_result_t buf[CAMPAIGN_DRAIN_MAX_COUNT];
    size_t n = campaign_manager_drain_results(buf, max_count);

    out[0] = (uint8_t)n;
    if (n > 0u) {
        memcpy(&out[1], buf, n * sizeof(campaign_result_t));
    }
    return CAMPAIGN_DRAIN_REPLY_HDR_LEN + (n * sizeof(campaign_result_t));
}

/*
 * services/sump_ols/sump_ols.c — SUMP/OLS protocol subset over
 * services/logic_analyzer/logic_analyzer.c, streaming flavour.
 *
 * Protocol command/metadata-token values and the little-endian
 * long-command argument layout were confirmed against the live
 * sigrok `libsigrok/src/hardware/openbench-logic-sniffer/` sources
 * (protocol.h, protocol.c, api.c) — see sump_ols.h for the specifics
 * that matter here (CLOCK_RATE, ID reply, CAPTURE_SIZE packing).
 */

#include "sump_ols.h"

#include <string.h>

#include "logic_analyzer.h"

#define CMD_RESET             0x00u
#define CMD_ARM_BASIC_TRIGGER 0x01u
#define CMD_ID                0x02u
#define CMD_METADATA          0x04u
#define CMD_SET_DIVIDER       0x80u
#define CMD_CAPTURE_SIZE      0x81u

// Basic trigger, stage 0. Confirmed against sigrok's
// openbench-logic-sniffer/protocol.h (CMD_SET_TRIGGER_MASK 0xC0,
// CMD_SET_TRIGGER_VALUE 0xC1, CMD_SET_TRIGGER_CONFIG 0xC2) and
// protocol.c's set_trigger(), which forms each stage's opcode as
// `base + stage * 4` — so 0xC0/0xC1 are stage 0 (0xC4/0xC5 stage 1,
// etc.). We act on mask+value; config (0xC2) and higher stages fall
// through to the generic long-command swallow.
#define CMD_SET_TRIGGER_MASK0  0xC0u
#define CMD_SET_TRIGGER_VALUE0 0xC1u

// SUMP long commands (>= 0x80) always carry exactly 4 trailing data
// bytes, even ones we don't act on (SET_FLAGS, advanced-trigger
// select/write, basic-trigger mask/value/config per stage). We must
// still consume those 4 bytes or the next real command desyncs the
// whole stream — see SUMP_OLS_SWALLOW_LONG_ARG_* in the header.
#define SUMP_LONGCMD_THRESHOLD 0x80u

// CLOCK_RATE in sigrok's protocol.h (100 MHz) — divider = CLOCK_RATE /
// samplerate - 1, so samplerate = CLOCK_RATE / (divider + 1) and
// interval_us = 1e6 / samplerate = (divider + 1) / (CLOCK_RATE / 1e6).
#define SUMP_CLOCK_RATE_MHZ 100u

// Throttle for the yield callback during ARM's capture stream — same
// order of magnitude as flashrom_serprog's SP_YIELD_EVERY_BYTES.
#define SUMP_YIELD_EVERY_BYTES 128u

typedef struct {
    sump_ols_callbacks_t cb;
    sump_ols_state_t state;
    uint32_t arg_acc;      // little-endian accumulator for the long
                           // command currently being read.
    uint32_t interval_us;  // from the last CMD_SET_DIVIDER.
    uint32_t n_samples;    // from the last CMD_CAPTURE_SIZE.
    uint8_t trigger_mask;  // stage-0 basic trigger; NUM_PROBES_LONG=8 so
    uint8_t trigger_value; // one byte covers every exposed channel.
                           // mask == 0 (memset default) = match anything.
    bool capturing;        // true for the whole ARM wait+stream duration.
} sump_t;

static sump_t s_sump;

// -----------------------------------------------------------------------------
// Output helpers
// -----------------------------------------------------------------------------

static void emit(uint8_t b) {
    if (s_sump.cb.write_byte)
        s_sump.cb.write_byte(b, s_sump.cb.user);
}

static void emit_n(const uint8_t* p, size_t n) {
    while (n--)
        emit(*p++);
}

static void emit_be32(uint32_t v) {
    emit((uint8_t)((v >> 24) & 0xFFu));
    emit((uint8_t)((v >> 16) & 0xFFu));
    emit((uint8_t)((v >> 8) & 0xFFu));
    emit((uint8_t)(v & 0xFFu));
}

static void yield_if_due(uint32_t i) {
    if (((i + 1u) % SUMP_YIELD_EVERY_BYTES) == 0u && s_sump.cb.yield) {
        s_sump.cb.yield(s_sump.cb.user);
    }
}

// -----------------------------------------------------------------------------
// CMD_METADATA reply — minimal TLV set. NUM_PROBES_LONG (=8,
// LA_CHANNEL_COUNT) is the field that matters most: it tells the host
// unitsize = (8+7)/8 = 1 byte/sample, matching logic_analyzer's raw
// GPIO_IN[7:0] byte-per-sample format. Sigrok's scan() treats metadata
// as optional (falls back to
// a 32-channel generic-SUMP guess if absent), so skipping this would
// silently corrupt every capture's channel decode — not skippable.
// -----------------------------------------------------------------------------

#define METADATA_TOKEN_DEVICE_NAME           0x01u
#define METADATA_TOKEN_NUM_PROBES_LONG       0x20u
#define METADATA_TOKEN_SAMPLE_MEMORY_BYTES   0x21u
#define METADATA_TOKEN_MAX_SAMPLE_RATE_HZ    0x23u
#define METADATA_TOKEN_PROTOCOL_VERSION_LONG 0x24u
#define METADATA_TOKEN_END                   0x00u

static void emit_metadata(void) {
    emit(METADATA_TOKEN_DEVICE_NAME);
    emit_n((const uint8_t*)SUMP_OLS_DEVICE_NAME, sizeof(SUMP_OLS_DEVICE_NAME)); // incl. NUL

    emit(METADATA_TOKEN_NUM_PROBES_LONG);
    emit_be32(LA_CHANNEL_COUNT);

    emit(METADATA_TOKEN_SAMPLE_MEMORY_BYTES);
    emit_be32(SUMP_OLS_MAX_SAMPLES);

    // DMA-timer pacing is limited in practice by sample_interval_us
    // granularity (1us minimum, see logic_analyzer.c's divisor_for) — report
    // 1 MHz as the ceiling rather than an unachievable peripheral
    // clock rate.
    emit(METADATA_TOKEN_MAX_SAMPLE_RATE_HZ);
    emit_be32(1000000u);

    emit(METADATA_TOKEN_PROTOCOL_VERSION_LONG);
    emit_be32(2u);

    emit(METADATA_TOKEN_END);
}

// -----------------------------------------------------------------------------
// CMD_ARM_BASIC_TRIGGER — synchronous capture + raw stream, same
// ring-drain shape as apps/faultycat_fw/main.c::cmd_la but binary
// (no hex/ASCII framing) and via the yield callback instead of a
// direct usb_composite_task() call.
// -----------------------------------------------------------------------------

static void do_arm(void) {
    if (!la_is_inited())
        return; // can't happen via main.c's mode-switch gate, but cheap to guard.

    uint32_t interval_us =
        (s_sump.interval_us != 0u) ? s_sump.interval_us : SUMP_OLS_DEFAULT_INTERVAL_US;
    uint32_t n = (s_sump.n_samples != 0u) ? s_sump.n_samples : SUMP_OLS_DEFAULT_N_SAMPLES;

    if (!la_start(interval_us))
        return;

    s_sump.capturing   = true;
    const uint8_t* buf = la_buffer();
    uint32_t cursor    = 0u;
    uint32_t streamed  = 0u;

    // Wait-for-trigger: advance the cursor through live samples without
    // emitting until one matches (level match, stage 0). mask == 0
    // matches the very first sample, so "no trigger configured" (the
    // memset default) degenerates to today's immediate start — the
    // streaming loop below just resumes from cursor 0. See
    // docs/UART_LA_TRIGGER_IMPLEMENTATION_PLAN.md.
    for (;;) {
        uint32_t written = la_total();
        if (written - cursor > LA_CAPTURE_BUFFER_BYTES) {
            // DMA lapped the unconsumed pre-trigger region — skip to the
            // oldest sample still in the ring, same best-effort drop the
            // streaming loop does below.
            cursor = written - LA_CAPTURE_BUFFER_BYTES;
        }
        bool matched = false;
        while (cursor < written) {
            uint8_t s = buf[cursor % LA_CAPTURE_BUFFER_BYTES];
            if ((s & s_sump.trigger_mask) == (s_sump.trigger_value & s_sump.trigger_mask)) {
                matched = true; // cursor now points at the triggering sample
                break;
            }
            cursor++;
        }
        if (matched)
            break;
        if (s_sump.cb.yield)
            s_sump.cb.yield(s_sump.cb.user); // keep CDC/USB alive while waiting
    }

    while (streamed < n) {
        uint32_t written   = la_total();
        uint32_t remaining = n - streamed;
        // Cap the live write head to (cursor + remaining), not an
        // absolute sample index: with a trigger the capture starts at
        // the trigger cursor rather than sample 0, so the old `written >
        // n` cap would stop short by exactly the pre-trigger offset.
        if (written - cursor > remaining)
            written = cursor + remaining;
        if (written - cursor > LA_CAPTURE_BUFFER_BYTES) {
            // DMA lapped the cursor — those samples are gone. Skip to
            // the oldest still in the ring; SUMP has no in-band
            // overflow signal so this is silently best-effort, same
            // tradeoff cmd_la's OVERFLOW text just makes visible.
            cursor = written - LA_CAPTURE_BUFFER_BYTES;
        }
        while (cursor < written) {
            emit(buf[cursor % LA_CAPTURE_BUFFER_BYTES]);
            cursor++;
            streamed++;
            yield_if_due(streamed - 1u);
        }
        if (s_sump.cb.yield)
            s_sump.cb.yield(s_sump.cb.user);
    }

    la_stop();
    s_sump.capturing = false;
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

void sump_ols_init(const sump_ols_callbacks_t* cb) {
    memset(&s_sump, 0, sizeof(s_sump));
    if (cb != NULL)
        s_sump.cb = *cb;
    s_sump.state = SUMP_OLS_IDLE;
}

sump_ols_state_t sump_ols_get_state(void) {
    return s_sump.state;
}

bool sump_ols_is_capturing(void) {
    return s_sump.capturing;
}

uint8_t sump_ols_trigger_mask(void) {
    return s_sump.trigger_mask;
}

uint8_t sump_ols_trigger_value(void) {
    return s_sump.trigger_value;
}

void sump_ols_feed_byte(uint8_t b) {
    switch (s_sump.state) {
        case SUMP_OLS_IDLE:
            switch (b) {
                case CMD_RESET:
                    if (s_sump.capturing) {
                        la_stop();
                        s_sump.capturing = false;
                    }
                    return;
                case CMD_ARM_BASIC_TRIGGER:
                    do_arm();
                    return;
                case CMD_ID:
                    emit_n((const uint8_t*)SUMP_OLS_ID_REPLY, 4u);
                    return;
                case CMD_METADATA:
                    emit_metadata();
                    return;
                case CMD_SET_DIVIDER:
                    s_sump.arg_acc = 0u;
                    s_sump.state   = SUMP_OLS_SET_DIVIDER_B0;
                    return;
                case CMD_CAPTURE_SIZE:
                    s_sump.arg_acc = 0u;
                    s_sump.state   = SUMP_OLS_CAPTURE_SIZE_B0;
                    return;
                case CMD_SET_TRIGGER_MASK0:
                    s_sump.arg_acc = 0u;
                    s_sump.state   = SUMP_OLS_TRIGGER_MASK0_B0;
                    return;
                case CMD_SET_TRIGGER_VALUE0:
                    s_sump.arg_acc = 0u;
                    s_sump.state   = SUMP_OLS_TRIGGER_VALUE0_B0;
                    return;
                default:
                    if (b >= SUMP_LONGCMD_THRESHOLD) {
                        // SET_FLAGS / advanced-trigger select+write /
                        // basic-trigger mask+value+config per stage /
                        // any unknown long command: accept and ignore,
                        // but consume the 4 argument bytes so the
                        // stream stays in sync (see header).
                        s_sump.state = SUMP_OLS_SWALLOW_LONG_ARG_1;
                    }
                    // Unknown short command (<0x80): no-op, no bytes
                    // to consume.
                    return;
            }

        case SUMP_OLS_SET_DIVIDER_B0:
            s_sump.arg_acc = (uint32_t)b;
            s_sump.state   = SUMP_OLS_SET_DIVIDER_B1;
            return;
        case SUMP_OLS_SET_DIVIDER_B1:
            s_sump.arg_acc |= (uint32_t)b << 8;
            s_sump.state = SUMP_OLS_SET_DIVIDER_B2;
            return;
        case SUMP_OLS_SET_DIVIDER_B2:
            s_sump.arg_acc |= (uint32_t)b << 16;
            s_sump.state = SUMP_OLS_SET_DIVIDER_B3;
            return;
        case SUMP_OLS_SET_DIVIDER_B3: {
            s_sump.arg_acc |= (uint32_t)b << 24;
            // divider = CLOCK_RATE/samplerate - 1 (host-side, sigrok
            // protocol.c) => interval_us = (divider+1) / clock_MHz.
            uint32_t divider     = s_sump.arg_acc;
            uint32_t interval_us = (divider + 1u) / SUMP_CLOCK_RATE_MHZ;
            s_sump.interval_us   = (interval_us != 0u) ? interval_us : 1u;
            s_sump.state         = SUMP_OLS_IDLE;
            return;
        }

        case SUMP_OLS_CAPTURE_SIZE_B0:
            s_sump.arg_acc = (uint32_t)b;
            s_sump.state   = SUMP_OLS_CAPTURE_SIZE_B1;
            return;
        case SUMP_OLS_CAPTURE_SIZE_B1:
            s_sump.arg_acc |= (uint32_t)b << 8;
            s_sump.state = SUMP_OLS_CAPTURE_SIZE_B2;
            return;
        case SUMP_OLS_CAPTURE_SIZE_B2:
            s_sump.arg_acc |= (uint32_t)b << 16;
            s_sump.state = SUMP_OLS_CAPTURE_SIZE_B3;
            return;
        case SUMP_OLS_CAPTURE_SIZE_B3: {
            s_sump.arg_acc |= (uint32_t)b << 24;
            // arg = WL16(readcount-1) | WL16(delaycount-1) << 16
            // (sigrok protocol.c::ols_prepare_acquisition). readcount
            // is in units of 4 samples; delaycount (pre-trigger) is
            // ignored — every ARM captures post-trigger only.
            uint16_t readcount_minus_1 = (uint16_t)(s_sump.arg_acc & 0xFFFFu);
            uint32_t readcount         = (uint32_t)readcount_minus_1 + 1u;
            s_sump.n_samples           = readcount * 4u;
            s_sump.state               = SUMP_OLS_IDLE;
            return;
        }

        // Stage-0 trigger mask/value: 4-byte little-endian argument
        // (mirrors SET_DIVIDER), but only channels 0-7 are exposed, so
        // just the low byte is retained — the upper 24 bits address
        // probes 8-31 this firmware doesn't have.
        case SUMP_OLS_TRIGGER_MASK0_B0:
            s_sump.arg_acc = (uint32_t)b;
            s_sump.state   = SUMP_OLS_TRIGGER_MASK0_B1;
            return;
        case SUMP_OLS_TRIGGER_MASK0_B1:
            s_sump.arg_acc |= (uint32_t)b << 8;
            s_sump.state = SUMP_OLS_TRIGGER_MASK0_B2;
            return;
        case SUMP_OLS_TRIGGER_MASK0_B2:
            s_sump.arg_acc |= (uint32_t)b << 16;
            s_sump.state = SUMP_OLS_TRIGGER_MASK0_B3;
            return;
        case SUMP_OLS_TRIGGER_MASK0_B3:
            s_sump.arg_acc |= (uint32_t)b << 24;
            s_sump.trigger_mask = (uint8_t)(s_sump.arg_acc & 0xFFu);
            s_sump.state        = SUMP_OLS_IDLE;
            return;

        case SUMP_OLS_TRIGGER_VALUE0_B0:
            s_sump.arg_acc = (uint32_t)b;
            s_sump.state   = SUMP_OLS_TRIGGER_VALUE0_B1;
            return;
        case SUMP_OLS_TRIGGER_VALUE0_B1:
            s_sump.arg_acc |= (uint32_t)b << 8;
            s_sump.state = SUMP_OLS_TRIGGER_VALUE0_B2;
            return;
        case SUMP_OLS_TRIGGER_VALUE0_B2:
            s_sump.arg_acc |= (uint32_t)b << 16;
            s_sump.state = SUMP_OLS_TRIGGER_VALUE0_B3;
            return;
        case SUMP_OLS_TRIGGER_VALUE0_B3:
            s_sump.arg_acc |= (uint32_t)b << 24;
            s_sump.trigger_value = (uint8_t)(s_sump.arg_acc & 0xFFu);
            s_sump.state         = SUMP_OLS_IDLE;
            return;

        case SUMP_OLS_SWALLOW_LONG_ARG_1:
            s_sump.state = SUMP_OLS_SWALLOW_LONG_ARG_2;
            return;
        case SUMP_OLS_SWALLOW_LONG_ARG_2:
            s_sump.state = SUMP_OLS_SWALLOW_LONG_ARG_3;
            return;
        case SUMP_OLS_SWALLOW_LONG_ARG_3:
            s_sump.state = SUMP_OLS_SWALLOW_LONG_ARG_4;
            return;
        case SUMP_OLS_SWALLOW_LONG_ARG_4:
            s_sump.state = SUMP_OLS_IDLE;
            return;
    }
}

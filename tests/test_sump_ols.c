// Unit tests for services/sump_ols — SUMP/OLS protocol subset over
// services/i2c_core/i2c_la (see docs/I2C_LA_DMA_TIMER_PLAN.md §6).
//
// Like test_i2c_la.c, the fake DMA doesn't copy bytes, so these tests
// can't assert captured *values* — only that the protocol shape (exact
// reply bytes for ID/metadata, byte counts for ARM, correct divider
// math) is right. The fake's `transfer_count` is static unless we move
// it ourselves, so the test `yield` callback advances it to simulate
// the DMA finishing the requested sample count — see fix_yield().

#include "unity.h"

#include <string.h>

#include "hal/dma.h"
#include "hal/pio.h"
#include "hal_fake_dma.h"
#include "hal_fake_gpio.h"
#include "hal_fake_pio.h"
#include "hal_fake_time.h"
#include "i2c_la.h"
#include "sump_ols.h"

#define SDA     2u
#define SCL     3u
#define FOREVER 0xFFFFFFFFu

// pio1/SM2 — see services/i2c_core/i2c_la.c.
#define I2C_LA_PIO_INSTANCE 1u
#define I2C_LA_PIO_SM       2u

// -----------------------------------------------------------------------------
// Test fixtures
// -----------------------------------------------------------------------------

#define MAX_WRITES 65536u

static uint8_t s_writes[MAX_WRITES];
static size_t s_writes_len;
static int s_yield_calls;

static int claimed_dma_channel(void) {
    for (int i = 0; i < HAL_FAKE_DMA_CHANNELS; i++) {
        if (hal_fake_dma_channels[i].claimed)
            return i;
    }
    return -1;
}

static void fix_write(uint8_t b, void* u) {
    (void)u;
    if (s_writes_len < MAX_WRITES)
        s_writes[s_writes_len++] = b;
}

// Simulates the DMA ring having finished filling: drives the fake's
// remaining count to 0 (i2c_la_total() == FOREVER) so do_arm()'s drain
// loop sees "everything requested is already available" on its second
// pass, regardless of how large `n` is. Real hardware fills the ring
// progressively; this fake only needs to prove the *count* streamed is
// right, not the timing.
static void fix_yield(void* u) {
    (void)u;
    s_yield_calls++;
    int ch = claimed_dma_channel();
    if (ch >= 0)
        hal_fake_dma_set_transfer_count(ch, 0u);
}

static const sump_ols_callbacks_t TEST_CB = {
    .write_byte = fix_write,
    .yield      = fix_yield,
    .on_exit    = NULL,
    .user       = NULL,
};

// -----------------------------------------------------------------------------
// Trigger-test fixtures. The fake DMA never copies bytes, so trigger-match
// logic needs the ring poked with known samples directly (cast away const,
// same test-only pattern as driving hal_fake_dma_channels[]) and the fake's
// transfer_count driven to "reveal" how many are available — see reveal().
// -----------------------------------------------------------------------------

static void preload_ring(uint32_t offset, const uint8_t* bytes, size_t n) {
    uint8_t* ring = (uint8_t*)i2c_la_buffer();
    for (size_t i = 0; i < n; i++)
        ring[(offset + i) % I2C_LA_CAPTURE_BUFFER_BYTES] = bytes[i];
}

// Make i2c_la_total() report `n` samples available, the way the DMA's
// decrementing transfer_count does on hardware (i2c_la_total() ==
// FOREVER - remaining).
static void reveal(uint32_t n) {
    int ch = claimed_dma_channel();
    if (ch >= 0)
        hal_fake_dma_set_transfer_count(ch, FOREVER - n);
}

// yield that reveals a fixed, bounded sample count each call — enough for
// the wait loop to see the trigger and for streaming to finish, without
// the pathological FOREVER of fix_yield (which would trip do_arm's
// overflow skip-forward and move the cursor off the preloaded bytes).
static uint32_t s_reveal_total;
static void yield_reveal(void* u) {
    (void)u;
    s_yield_calls++;
    reveal(s_reveal_total);
}

static const sump_ols_callbacks_t REVEAL_CB = {
    .write_byte = fix_write,
    .yield      = yield_reveal,
    .on_exit    = NULL,
    .user       = NULL,
};

// yield that keeps only idle samples visible for the first few polls, then
// reveals the matching sample — models a real, drawn-out wait so the test
// can prove do_arm pumps yield() while blocked instead of busy-looping.
static void yield_delayed_trigger(void* u) {
    (void)u;
    s_yield_calls++;
    if (s_yield_calls < 3)
        reveal(2u); // only the two idle samples, no match yet
    else if (s_yield_calls == 3)
        reveal(4u); // start bit at index 2 now visible
    else
        reveal(6u); // enough to finish streaming n=4 from the trigger
}

static const sump_ols_callbacks_t DELAYED_CB = {
    .write_byte = fix_write,
    .yield      = yield_delayed_trigger,
    .on_exit    = NULL,
    .user       = NULL,
};

void setUp(void) {
    i2c_la_deinit();
    hal_fake_gpio_reset();
    hal_fake_dma_reset();
    hal_fake_pio_reset();
    hal_fake_time_reset();
    s_writes_len  = 0;
    s_yield_calls = 0;
    memset(s_writes, 0, sizeof(s_writes));

    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    sump_ols_init(&TEST_CB);
}

void tearDown(void) {
}

static void feed(const uint8_t* bytes, size_t n) {
    for (size_t i = 0; i < n; i++)
        sump_ols_feed_byte(bytes[i]);
}

// -----------------------------------------------------------------------------
// CMD_ID (0x02) — sigrok's ols driver matches this exact 4-byte reply
// (strncmp(buf, "1ALS", 4)) to recognize the device during scan().
// -----------------------------------------------------------------------------

static void test_id_replies_1als(void) {
    uint8_t cmd_id = 0x02u;
    feed(&cmd_id, 1u);
    TEST_ASSERT_EQUAL_UINT32(4u, s_writes_len);
    TEST_ASSERT_EQUAL_MEMORY("1ALS", s_writes, 4u);
}

// -----------------------------------------------------------------------------
// CMD_METADATA (0x04) — minimal TLV: device name, 8 probes, sample
// memory, max rate, protocol version, terminator. NUM_PROBES_LONG=8 is
// the field that matters most (unitsize = (8+7)/8 = 1 byte/sample,
// matching i2c_la's raw GPIO_IN[7:0] byte format).
// -----------------------------------------------------------------------------

static void test_metadata_reports_8_probes_and_max_samples(void) {
    uint8_t cmd_meta = 0x04u;
    feed(&cmd_meta, 1u);

    uint8_t expect[64];
    size_t pos    = 0;
    expect[pos++] = 0x01u; // DEVICE_NAME
    memcpy(&expect[pos], SUMP_OLS_DEVICE_NAME, sizeof(SUMP_OLS_DEVICE_NAME));
    pos += sizeof(SUMP_OLS_DEVICE_NAME);
    expect[pos++] = 0x20u; // NUM_PROBES_LONG
    expect[pos++] = 0x00u;
    expect[pos++] = 0x00u;
    expect[pos++] = 0x00u;
    expect[pos++] = 0x08u;
    expect[pos++] = 0x21u; // SAMPLE_MEMORY_BYTES
    expect[pos++] = (uint8_t)((SUMP_OLS_MAX_SAMPLES >> 24) & 0xFFu);
    expect[pos++] = (uint8_t)((SUMP_OLS_MAX_SAMPLES >> 16) & 0xFFu);
    expect[pos++] = (uint8_t)((SUMP_OLS_MAX_SAMPLES >> 8) & 0xFFu);
    expect[pos++] = (uint8_t)(SUMP_OLS_MAX_SAMPLES & 0xFFu);
    expect[pos++] = 0x23u; // MAX_SAMPLE_RATE_HZ (1 MHz)
    expect[pos++] = 0x00u;
    expect[pos++] = 0x0Fu;
    expect[pos++] = 0x42u;
    expect[pos++] = 0x40u;
    expect[pos++] = 0x24u; // PROTOCOL_VERSION_LONG
    expect[pos++] = 0x00u;
    expect[pos++] = 0x00u;
    expect[pos++] = 0x00u;
    expect[pos++] = 0x02u;
    expect[pos++] = 0x00u; // END

    TEST_ASSERT_EQUAL_UINT32(pos, s_writes_len);
    TEST_ASSERT_EQUAL_MEMORY(expect, s_writes, pos);
}

// -----------------------------------------------------------------------------
// CMD_SET_DIVIDER (0x80) — divider = CLOCK_RATE(100MHz)/samplerate - 1,
// so interval_us = (divider+1)/100. Verified by checking the PIO SM
// clkdiv i2c_la_start() actually programs on the next ARM (2 SM cycles
// per sample — see services/i2c_core/i2c_la.c).
// -----------------------------------------------------------------------------

static void test_set_divider_feeds_interval_into_next_arm(void) {
    // divider = 9999 (0x0000270F, little-endian on the wire) =>
    // interval_us = (9999+1)/100 = 100.
    uint8_t set_divider[] = {0x80u, 0x0Fu, 0x27u, 0x00u, 0x00u};
    feed(set_divider, sizeof(set_divider));

    uint8_t arm = 0x01u;
    feed(&arm, 1u);

    // clk_div = sys_clk(125MHz) * interval_us(100) / 1e6 / 2 = 6250.
    const hal_fake_pio_sm_state_t* sm = &hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM];
    TEST_ASSERT_EQUAL_FLOAT(6250.0f, sm->last_cfg.clk_div);
}

static void test_arm_without_set_divider_uses_default_interval(void) {
    uint8_t arm = 0x01u;
    feed(&arm, 1u);

    // den = 125MHz * 2us / 1e6 / 2 = 125.
    const hal_fake_pio_sm_state_t* sm = &hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM];
    TEST_ASSERT_EQUAL_FLOAT(125.0f, sm->last_cfg.clk_div);
}

// -----------------------------------------------------------------------------
// CMD_CAPTURE_SIZE (0x81) — arg = WL16(readcount-1) | WL16(delaycount-1)
// << 16; readcount is in units of 4 samples; delaycount (pre-trigger)
// is ignored. ARM must stream exactly readcount*4 bytes.
// -----------------------------------------------------------------------------

static void test_capture_size_sets_arm_sample_count(void) {
    // readcount-1 = 4 (=> readcount=5 => n=20), delaycount-1 = 0.
    uint8_t capture_size[] = {0x81u, 0x04u, 0x00u, 0x00u, 0x00u};
    feed(capture_size, sizeof(capture_size));

    uint8_t arm = 0x01u;
    feed(&arm, 1u);

    TEST_ASSERT_EQUAL_UINT32(20u, s_writes_len);
    TEST_ASSERT_TRUE(s_yield_calls > 0);
    TEST_ASSERT_FALSE(sump_ols_is_capturing()); // synchronous: done by the time feed_byte returns
}

static void test_arm_without_capture_size_uses_default_n_samples(void) {
    uint8_t arm = 0x01u;
    feed(&arm, 1u);

    TEST_ASSERT_EQUAL_UINT32(SUMP_OLS_DEFAULT_N_SAMPLES, s_writes_len);
}

static void test_arm_stops_the_dma_after_streaming(void) {
    int ch      = claimed_dma_channel();
    uint8_t arm = 0x01u;
    feed(&arm, 1u);
    TEST_ASSERT_EQUAL_UINT32(1u, hal_fake_dma_channels[ch].abort_calls);
}

// -----------------------------------------------------------------------------
// Unknown/ignored long commands (SET_FLAGS, trigger mask/value/config,
// ...) must still consume exactly 4 argument bytes, or the next real
// command desyncs. Send one, then CMD_ID, and confirm the reply is
// still a clean "1ALS" (not corrupted by a misaligned byte stream).
// -----------------------------------------------------------------------------

static void test_unknown_long_command_swallows_4_bytes_then_resyncs(void) {
    uint8_t set_flags[] = {0x82u, 0xAAu, 0xBBu, 0xCCu, 0xDDu}; // SET_FLAGS, ignored
    feed(set_flags, sizeof(set_flags));

    uint8_t cmd_id = 0x02u;
    feed(&cmd_id, 1u);

    TEST_ASSERT_EQUAL_UINT32(4u, s_writes_len);
    TEST_ASSERT_EQUAL_MEMORY("1ALS", s_writes, 4u);
}

static void test_unknown_short_command_is_ignored(void) {
    uint8_t nop = 0x7Fu; // not a defined SUMP command, < 0x80 (short)
    feed(&nop, 1u);

    uint8_t cmd_id = 0x02u;
    feed(&cmd_id, 1u);

    TEST_ASSERT_EQUAL_UINT32(4u, s_writes_len);
    TEST_ASSERT_EQUAL_MEMORY("1ALS", s_writes, 4u);
}

// -----------------------------------------------------------------------------
// CMD_RESET (0x00) — safe no-op when idle (the only reachable case
// through feed_byte: ARM runs to completion synchronously, so RESET
// can never observe sump_ols_is_capturing() == true from outside it).
// -----------------------------------------------------------------------------

static void test_reset_when_idle_is_safe(void) {
    uint8_t reset = 0x00u;
    feed(&reset, 1u);
    TEST_ASSERT_EQUAL_UINT32(0u, s_writes_len);
    TEST_ASSERT_FALSE(sump_ols_is_capturing());
}

// -----------------------------------------------------------------------------
// Stage-0 basic trigger (CMD_SET_TRIGGER_MASK 0xC0 / VALUE 0xC1). PulseView
// sends these when the user arms a capture with a trigger condition; with
// none configured it sends neither, so trigger_mask stays 0 (match-anything)
// and ARM starts immediately — see UART_LA_TRIGGER_IMPLEMENTATION_PLAN.md.
// -----------------------------------------------------------------------------

// Regression guard for the backward-compat claim: no SET_TRIGGER_* sent, so
// mask/value keep their zero default and the capture streams from sample 0
// exactly like before triggering existed.
static void test_no_trigger_configured_starts_immediately(void) {
    sump_ols_init(&REVEAL_CB);

    uint8_t pattern[20];
    for (uint8_t i = 0; i < 20u; i++)
        pattern[i] = (uint8_t)(0xA0u + i);
    preload_ring(0u, pattern, sizeof(pattern));
    s_reveal_total = 20u;

    TEST_ASSERT_EQUAL_UINT8(0u, sump_ols_trigger_mask()); // default: match anything

    uint8_t capture_size[] = {0x81u, 0x04u, 0x00u, 0x00u, 0x00u}; // readcount 5 => n=20
    feed(capture_size, sizeof(capture_size));
    uint8_t arm = 0x01u;
    feed(&arm, 1u);

    TEST_ASSERT_EQUAL_UINT32(20u, s_writes_len);
    TEST_ASSERT_EQUAL_MEMORY(pattern, s_writes, 20u); // started at sample 0
}

// Mask/value are 4-byte little-endian args (like SET_DIVIDER); only the low
// byte is kept (channels 0-7). Non-zero upper bytes confirm both the byte
// order and that they're discarded.
static void test_trigger_mask_value_parsed_from_wire(void) {
    uint8_t set_mask[]  = {0xC0u, 0x81u, 0x00u, 0x00u, 0x00u};
    uint8_t set_value[] = {0xC1u, 0x01u, 0xFFu, 0xFFu, 0xFFu};
    feed(set_mask, sizeof(set_mask));
    feed(set_value, sizeof(set_value));

    TEST_ASSERT_EQUAL_UINT8(0x81u, sump_ols_trigger_mask());
    TEST_ASSERT_EQUAL_UINT8(0x01u, sump_ols_trigger_value());
    // Stream stays in sync afterward — the next real command still parses.
    uint8_t cmd_id = 0x02u;
    feed(&cmd_id, 1u);
    TEST_ASSERT_EQUAL_UINT32(4u, s_writes_len);
    TEST_ASSERT_EQUAL_MEMORY("1ALS", s_writes, 4u);
}

// Idle-high RX line (bit 0 = 1) then a start bit (bit 0 = 0): a mask/value
// selecting "bit 0 low" must skip the idle samples and begin the stream at
// the triggering sample, not sample 0.
static void test_arm_waits_for_trigger_before_streaming(void) {
    sump_ols_init(&REVEAL_CB);

    uint8_t samples[] = {0x01u, 0x01u, 0x00u, 0xAAu, 0xBBu, 0xCCu};
    preload_ring(0u, samples, sizeof(samples));
    s_reveal_total = 6u;

    uint8_t set_mask[]  = {0xC0u, 0x01u, 0x00u, 0x00u, 0x00u}; // watch bit 0
    uint8_t set_value[] = {0xC1u, 0x00u, 0x00u, 0x00u, 0x00u}; // fire when it's 0
    feed(set_mask, sizeof(set_mask));
    feed(set_value, sizeof(set_value));

    uint8_t capture_size[] = {0x81u, 0x00u, 0x00u, 0x00u, 0x00u}; // readcount 1 => n=4
    feed(capture_size, sizeof(capture_size));
    uint8_t arm = 0x01u;
    feed(&arm, 1u);

    uint8_t expect[] = {0x00u, 0xAAu, 0xBBu, 0xCCu}; // from index 2, not 0
    TEST_ASSERT_EQUAL_UINT32(4u, s_writes_len);
    TEST_ASSERT_EQUAL_MEMORY(expect, s_writes, 4u);
}

// A real trigger wait can span many samples; do_arm must keep calling
// yield() (which pumps tud_task/USB) while blocked, not busy-loop. Reveal
// only idle samples for the first polls, then the match, and confirm both
// that yield fired repeatedly and that the capture unblocks and completes.
static void test_arm_polls_yield_while_waiting_for_trigger(void) {
    sump_ols_init(&DELAYED_CB);

    uint8_t samples[] = {0x01u, 0x01u, 0x00u, 0xAAu, 0xBBu, 0xCCu};
    preload_ring(0u, samples, sizeof(samples));

    uint8_t set_mask[]  = {0xC0u, 0x01u, 0x00u, 0x00u, 0x00u};
    uint8_t set_value[] = {0xC1u, 0x00u, 0x00u, 0x00u, 0x00u};
    feed(set_mask, sizeof(set_mask));
    feed(set_value, sizeof(set_value));

    uint8_t capture_size[] = {0x81u, 0x00u, 0x00u, 0x00u, 0x00u}; // n=4
    feed(capture_size, sizeof(capture_size));
    uint8_t arm = 0x01u;
    feed(&arm, 1u);

    TEST_ASSERT_TRUE(s_yield_calls >= 3);       // polled while blocked waiting
    TEST_ASSERT_EQUAL_UINT32(4u, s_writes_len); // then unblocked and finished
    TEST_ASSERT_EQUAL_UINT8(0x00u, s_writes[0]); // started at the start bit
    TEST_ASSERT_FALSE(sump_ols_is_capturing());
}

// -----------------------------------------------------------------------------
// Runner
// -----------------------------------------------------------------------------

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_id_replies_1als);
    RUN_TEST(test_metadata_reports_8_probes_and_max_samples);

    RUN_TEST(test_set_divider_feeds_interval_into_next_arm);
    RUN_TEST(test_arm_without_set_divider_uses_default_interval);

    RUN_TEST(test_capture_size_sets_arm_sample_count);
    RUN_TEST(test_arm_without_capture_size_uses_default_n_samples);
    RUN_TEST(test_arm_stops_the_dma_after_streaming);

    RUN_TEST(test_unknown_long_command_swallows_4_bytes_then_resyncs);
    RUN_TEST(test_unknown_short_command_is_ignored);

    RUN_TEST(test_reset_when_idle_is_safe);

    RUN_TEST(test_no_trigger_configured_starts_immediately);
    RUN_TEST(test_trigger_mask_value_parsed_from_wire);
    RUN_TEST(test_arm_waits_for_trigger_before_streaming);
    RUN_TEST(test_arm_polls_yield_while_waiting_for_trigger);

    return UNITY_END();
}

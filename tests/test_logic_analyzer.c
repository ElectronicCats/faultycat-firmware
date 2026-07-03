// Unit tests for services/logic_analyzer — passive, protocol-agnostic
// digital logic analyzer (PIO-paced, DMA-drained continuous streamer;
// see docs/I2C_LA_DMA_TIMER_PLAN.md §3/§4 and its postmortem).
//
// The fake DMA does not copy bytes, so these tests verify the DMA channel
// is configured for a continuous ring-mode transfer sourced from the PIO
// RX FIFO, that the PIO SM is configured/enabled correctly, and that
// la_total() reflects the simulated DMA progress. On-wire
// reconstruction of a real transaction is a hardware test (§5), not here.

#include "unity.h"

#include "hal/dma.h"
#include "hal/pio.h"
#include "hal_fake_dma.h"
#include "hal_fake_gpio.h"
#include "hal_fake_pio.h"
#include "hal_fake_time.h"
#include "logic_analyzer.h"

// pio1/SM2 — see services/logic_analyzer/logic_analyzer.c.
#define LA_PIO_INSTANCE 1u
#define LA_PIO_SM       2u

// transfer_count the driver loads for a never-ending transfer; total
// captured == this minus the live remaining count.
#define FOREVER 0xFFFFFFFFu

void setUp(void) {
    // Deinit first (a prior test may have left a capture running, and
    // deinit aborts the DMA), THEN reset the fakes so per-test counters
    // like abort_calls start at zero.
    la_deinit(); // safe even if not inited — resets s_inited for the next test
    hal_fake_gpio_reset();
    hal_fake_dma_reset();
    hal_fake_pio_reset();
    hal_fake_time_reset();
}

void tearDown(void) {
}

// Index of the single DMA channel la_init claimed, or -1.
static int claimed_dma_channel(void) {
    for (int i = 0; i < HAL_FAKE_DMA_CHANNELS; i++) {
        if (hal_fake_dma_channels[i].claimed)
            return i;
    }
    return -1;
}

static bool pio_sm_claimed(void) {
    return hal_fake_pio_insts[LA_PIO_INSTANCE].sm[LA_PIO_SM].claimed;
}

// Simulate the DMA having written `n` samples since start by setting the
// decrementing transfer_count the way the hardware would.
static void simulate_written(int ch, uint32_t n) {
    hal_fake_dma_set_transfer_count(ch, FOREVER - n);
}

// The fake DMA doesn't copy bytes, so trigger-match tests need the ring
// poked with known samples directly — same cast-away-const pattern
// test_sump_ols.c's preload_ring() uses.
static void preload_ring(uint32_t offset, const uint8_t* bytes, size_t n) {
    uint8_t* ring = (uint8_t*)la_buffer();
    for (size_t i = 0; i < n; i++)
        ring[(offset + i) % LA_CAPTURE_BUFFER_BYTES] = bytes[i];
}

// -----------------------------------------------------------------------------
// init / deinit
// -----------------------------------------------------------------------------

static void test_init_configures_channels_as_inputs(void) {
    TEST_ASSERT_TRUE(la_init());
    // Every captured channel (GP0..GP7) is left a plain input.
    for (uint8_t ch = 0u; ch < LA_CHANNEL_COUNT; ch++) {
        TEST_ASSERT_TRUE(hal_fake_gpio_states[ch].initialized);
        TEST_ASSERT_EQUAL_INT(HAL_GPIO_DIR_IN, hal_fake_gpio_states[ch].dir);
    }
}

static void test_init_claims_one_dma_channel_and_pio_sm(void) {
    TEST_ASSERT_TRUE(la_init());
    TEST_ASSERT_TRUE(la_is_inited());
    TEST_ASSERT_NOT_EQUAL(-1, claimed_dma_channel());
    TEST_ASSERT_TRUE(pio_sm_claimed());
    TEST_ASSERT_TRUE(hal_fake_pio_insts[LA_PIO_INSTANCE].program.loaded);
    TEST_ASSERT_EQUAL_UINT32(2u, hal_fake_pio_insts[LA_PIO_INSTANCE].program.length);
}

static void test_init_double_init_rejects(void) {
    TEST_ASSERT_TRUE(la_init());
    TEST_ASSERT_FALSE(la_init()); // already inited
}

static void test_deinit_releases_dma_and_pio_sm(void) {
    TEST_ASSERT_TRUE(la_init());
    la_deinit();
    TEST_ASSERT_FALSE(la_is_inited());
    TEST_ASSERT_EQUAL_INT(-1, claimed_dma_channel());
    TEST_ASSERT_FALSE(pio_sm_claimed());
}

static void test_deinit_stops_running_capture(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(la_start(1u));
    la_deinit();
    TEST_ASSERT_FALSE(la_is_running());
    // The channel was aborted and the SM disabled as part of teardown.
    TEST_ASSERT_EQUAL_UINT32(1u, hal_fake_dma_channels[ch].abort_calls);
    TEST_ASSERT_FALSE(hal_fake_pio_insts[LA_PIO_INSTANCE].sm[LA_PIO_SM].enabled);
}

static void test_deinit_allows_reinit(void) {
    TEST_ASSERT_TRUE(la_init());
    la_deinit();
    TEST_ASSERT_TRUE(la_init());
}

// -----------------------------------------------------------------------------
// start — guards and lifecycle
// -----------------------------------------------------------------------------

static void test_start_without_init_returns_false(void) {
    TEST_ASSERT_FALSE(la_start(1u));
    TEST_ASSERT_FALSE(la_is_running());
}

static void test_start_sets_running_then_stop_clears_it(void) {
    TEST_ASSERT_TRUE(la_init());
    TEST_ASSERT_FALSE(la_is_running());
    TEST_ASSERT_TRUE(la_start(1u));
    TEST_ASSERT_TRUE(la_is_running());
    TEST_ASSERT_TRUE(hal_fake_pio_insts[LA_PIO_INSTANCE].sm[LA_PIO_SM].enabled);
    la_stop();
    TEST_ASSERT_FALSE(la_is_running());
    TEST_ASSERT_FALSE(hal_fake_pio_insts[LA_PIO_INSTANCE].sm[LA_PIO_SM].enabled);
}

static void test_start_rejects_double_start(void) {
    TEST_ASSERT_TRUE(la_init());
    TEST_ASSERT_TRUE(la_start(1u));
    TEST_ASSERT_FALSE(la_start(1u)); // already running
}

static void test_stop_aborts_dma(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(la_start(1u));
    la_stop();
    TEST_ASSERT_EQUAL_UINT32(1u, hal_fake_dma_channels[ch].abort_calls);
}

// -----------------------------------------------------------------------------
// start — channel/PIO configuration
// -----------------------------------------------------------------------------

static void test_start_configures_ring_mode_pio_dma(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();

    TEST_ASSERT_TRUE(la_start(1u));

    hal_pio_inst_t* pio           = hal_pio_instance(LA_PIO_INSTANCE);
    const hal_fake_dma_state_t* s = &hal_fake_dma_channels[ch];
    TEST_ASSERT_EQUAL_UINT(HAL_DMA_SIZE_8, s->cfg.size);
    TEST_ASSERT_FALSE(s->cfg.read_increment);
    TEST_ASSERT_TRUE(s->cfg.write_increment);
    // Ring mode on the write side, wrapping the LA_CAPTURE_BUFFER_BYTES buffer.
    TEST_ASSERT_EQUAL_UINT32(15u, s->cfg.ring_bits);
    TEST_ASSERT_TRUE(s->cfg.ring_on_write);
    TEST_ASSERT_EQUAL_UINT((hal_dma_dreq_t)hal_pio_sm_rx_dreq(pio, LA_PIO_SM), s->cfg.dreq);
    TEST_ASSERT_EQUAL_PTR(hal_pio_sm_rxfifo_register(pio, LA_PIO_SM), s->src);
    // Never-ending transfer.
    TEST_ASSERT_EQUAL_UINT32(FOREVER, s->transfer_count);

    const hal_fake_pio_sm_state_t* sm = &hal_fake_pio_insts[LA_PIO_INSTANCE].sm[LA_PIO_SM];
    TEST_ASSERT_EQUAL_UINT32(0u, sm->last_cfg.in_pin_base);
    TEST_ASSERT_EQUAL_UINT32(8u, sm->last_cfg.in_pin_count);
    TEST_ASSERT_FALSE(sm->last_cfg.in_shift_right);
    TEST_ASSERT_TRUE(sm->enabled);
}

static void test_start_paces_pio_clkdiv_from_interval(void) {
    TEST_ASSERT_TRUE(la_init());

    TEST_ASSERT_TRUE(la_start(10u));

    // 2 SM cycles/sample; clk_div = sys_clk(125 MHz) * 10us / 1e6 / 2 = 625.
    const hal_fake_pio_sm_state_t* sm = &hal_fake_pio_insts[LA_PIO_INSTANCE].sm[LA_PIO_SM];
    TEST_ASSERT_EQUAL_FLOAT(625.0f, sm->last_cfg.clk_div);
}

// -----------------------------------------------------------------------------
// total — reflects DMA progress
// -----------------------------------------------------------------------------

static void test_total_is_zero_before_any_samples(void) {
    TEST_ASSERT_TRUE(la_init());
    TEST_ASSERT_TRUE(la_start(1u));
    // Fresh transfer: transfer_count still == FOREVER, nothing written.
    TEST_ASSERT_EQUAL_UINT32(0u, la_total());
}

static void test_total_reflects_partial_progress(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(la_start(1u));
    simulate_written(ch, 1234u);
    TEST_ASSERT_EQUAL_UINT32(1234u, la_total());
}

static void test_total_counts_past_buffer_size(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(la_start(1u));
    // Continuous capture: total is monotonic past the ring buffer size.
    simulate_written(ch, LA_CAPTURE_BUFFER_BYTES + 5000u);
    TEST_ASSERT_EQUAL_UINT32(LA_CAPTURE_BUFFER_BYTES + 5000u, la_total());
}

static void test_total_is_zero_after_deinit(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(la_start(1u));
    simulate_written(ch, 500u);
    la_deinit();
    TEST_ASSERT_EQUAL_UINT32(0u, la_total());
}

// -----------------------------------------------------------------------------
// la_wait_for_trigger / la_apply_pretrigger
// -----------------------------------------------------------------------------

// mask == 0 must match as soon as any sample exists — the degenerate
// case that makes `trig=` omitted byte-for-byte equivalent to no
// trigger. Regression guard for that backward-compat claim.
static void test_wait_for_trigger_mask_zero_matches_immediately(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(la_start(1u));
    simulate_written(ch, 1u);

    uint32_t cursor = la_wait_for_trigger(0u, 0u, NULL, NULL, 0u);
    TEST_ASSERT_EQUAL_UINT32(0u, cursor);
}

// Idle-high samples (bit 0 = 1) followed by a low sample (bit 0 = 0): a
// mask/value selecting "bit 0 low" must skip the idle samples and return
// the cursor of the triggering sample, not sample 0.
static void test_wait_for_trigger_matches_first_low_sample(void) {
    TEST_ASSERT_TRUE(la_init());
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(la_start(1u));

    uint8_t samples[] = {0x01u, 0x01u, 0x00u, 0xAAu};
    preload_ring(0u, samples, sizeof(samples));
    simulate_written(ch, sizeof(samples));

    uint32_t cursor = la_wait_for_trigger(0x01u, 0x00u, NULL, NULL, 0u);
    TEST_ASSERT_EQUAL_UINT32(2u, cursor);
}

static uint32_t s_timeout_yield_calls;
static void timeout_yield_cb(void* u) {
    (void)u;
    s_timeout_yield_calls++;
    hal_fake_time_advance_ms(10u);
}

// No matching sample ever arrives: the wait must give up once
// timeout_ms elapses (rather than hang) and must have kept pumping
// yield() while blocked, the same busy-loop/USB-starvation guard
// test_arm_polls_yield_while_waiting_for_trigger checks for do_arm().
static void test_wait_for_trigger_times_out(void) {
    TEST_ASSERT_TRUE(la_init());
    TEST_ASSERT_TRUE(la_start(1u));
    // No simulate_written call: la_total() stays 0, so no sample is
    // ever visible to match against.
    s_timeout_yield_calls = 0u;

    uint32_t cursor = la_wait_for_trigger(0x01u, 0x00u, timeout_yield_cb, NULL, 50u);
    TEST_ASSERT_EQUAL_UINT32(LA_NO_TRIGGER_MATCH, cursor);
    TEST_ASSERT_TRUE(s_timeout_yield_calls >= 1u);
}

// cursor near 0 (no history to serve) clamps to 0 instead of
// underflowing; a small n caps the pretrigger at n/8 rather than the
// full LA_PRETRIGGER_MIN — same edge cases do_arm() handles inline in
// services/sump_ols/sump_ols.c, covered once here instead of never.
static void test_apply_pretrigger_floors_and_caps(void) {
    TEST_ASSERT_EQUAL_UINT32(0u, la_apply_pretrigger(5u, 1000u)); // 5 < LA_PRETRIGGER_MIN
    TEST_ASSERT_EQUAL_UINT32(1000u - LA_PRETRIGGER_MIN,
                             la_apply_pretrigger(1000u, 1000u)); // full floor available
    TEST_ASSERT_EQUAL_UINT32(40u - (32u / 8u), la_apply_pretrigger(40u, 32u)); // n/8 cap
}

// -----------------------------------------------------------------------------
// Runner
// -----------------------------------------------------------------------------

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_configures_channels_as_inputs);
    RUN_TEST(test_init_claims_one_dma_channel_and_pio_sm);
    RUN_TEST(test_init_double_init_rejects);
    RUN_TEST(test_deinit_releases_dma_and_pio_sm);
    RUN_TEST(test_deinit_stops_running_capture);
    RUN_TEST(test_deinit_allows_reinit);

    RUN_TEST(test_start_without_init_returns_false);
    RUN_TEST(test_start_sets_running_then_stop_clears_it);
    RUN_TEST(test_start_rejects_double_start);
    RUN_TEST(test_stop_aborts_dma);

    RUN_TEST(test_start_configures_ring_mode_pio_dma);
    RUN_TEST(test_start_paces_pio_clkdiv_from_interval);

    RUN_TEST(test_total_is_zero_before_any_samples);
    RUN_TEST(test_total_reflects_partial_progress);
    RUN_TEST(test_total_counts_past_buffer_size);
    RUN_TEST(test_total_is_zero_after_deinit);

    RUN_TEST(test_wait_for_trigger_mask_zero_matches_immediately);
    RUN_TEST(test_wait_for_trigger_matches_first_low_sample);
    RUN_TEST(test_wait_for_trigger_times_out);
    RUN_TEST(test_apply_pretrigger_floors_and_caps);

    return UNITY_END();
}

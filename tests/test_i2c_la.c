// Unit tests for services/i2c_core/i2c_la — passive I2C logic analyzer
// (PIO-paced, DMA-drained continuous streamer; see
// docs/I2C_LA_DMA_TIMER_PLAN.md §3/§4 and its postmortem).
//
// The fake DMA does not copy bytes, so these tests verify the DMA channel
// is configured for a continuous ring-mode transfer sourced from the PIO
// RX FIFO, that the PIO SM is configured/enabled correctly, and that
// i2c_la_total() reflects the simulated DMA progress. On-wire
// reconstruction of a real transaction is a hardware test (§5), not here.

#include "unity.h"

#include "hal/dma.h"
#include "hal/pio.h"
#include "hal_fake_dma.h"
#include "hal_fake_gpio.h"
#include "hal_fake_pio.h"
#include "hal_fake_time.h"
#include "i2c_la.h"

#define SDA 2u
#define SCL 3u

// pio1/SM2 — see services/i2c_core/i2c_la.c.
#define I2C_LA_PIO_INSTANCE 1u
#define I2C_LA_PIO_SM       2u

// transfer_count the driver loads for a never-ending transfer; total
// captured == this minus the live remaining count.
#define FOREVER 0xFFFFFFFFu

void setUp(void) {
    // Deinit first (a prior test may have left a capture running, and
    // deinit aborts the DMA), THEN reset the fakes so per-test counters
    // like abort_calls start at zero.
    i2c_la_deinit(); // safe even if not inited — resets s_inited for the next test
    hal_fake_gpio_reset();
    hal_fake_dma_reset();
    hal_fake_pio_reset();
    hal_fake_time_reset();
}

void tearDown(void) {
}

// Index of the single DMA channel i2c_la_init claimed, or -1.
static int claimed_dma_channel(void) {
    for (int i = 0; i < HAL_FAKE_DMA_CHANNELS; i++) {
        if (hal_fake_dma_channels[i].claimed)
            return i;
    }
    return -1;
}

static bool pio_sm_claimed(void) {
    return hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM].claimed;
}

// Simulate the DMA having written `n` samples since start by setting the
// decrementing transfer_count the way the hardware would.
static void simulate_written(int ch, uint32_t n) {
    hal_fake_dma_set_transfer_count(ch, FOREVER - n);
}

// -----------------------------------------------------------------------------
// init / deinit
// -----------------------------------------------------------------------------

static void test_init_rejects_equal_pins(void) {
    TEST_ASSERT_FALSE(i2c_la_init(SDA, SDA));
    TEST_ASSERT_FALSE(i2c_la_is_inited());
}

static void test_init_rejects_out_of_range_pin(void) {
    TEST_ASSERT_FALSE(i2c_la_init(30u, SCL));
    TEST_ASSERT_FALSE(i2c_la_init(SDA, 30u));
    TEST_ASSERT_FALSE(i2c_la_is_inited());
    // A rejected init must not leak a DMA channel or PIO SM.
    TEST_ASSERT_EQUAL_INT(-1, claimed_dma_channel());
    TEST_ASSERT_FALSE(pio_sm_claimed());
}

static void test_init_claims_one_dma_channel_and_pio_sm(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_TRUE(i2c_la_is_inited());
    TEST_ASSERT_NOT_EQUAL(-1, claimed_dma_channel());
    TEST_ASSERT_TRUE(pio_sm_claimed());
    TEST_ASSERT_TRUE(hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].program.loaded);
    TEST_ASSERT_EQUAL_UINT32(2u, hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].program.length);
}

static void test_init_double_init_rejects(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_FALSE(i2c_la_init(SDA, SCL)); // already inited
}

static void test_deinit_releases_dma_and_pio_sm(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    i2c_la_deinit();
    TEST_ASSERT_FALSE(i2c_la_is_inited());
    TEST_ASSERT_EQUAL_INT(-1, claimed_dma_channel());
    TEST_ASSERT_FALSE(pio_sm_claimed());
}

static void test_deinit_stops_running_capture(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    i2c_la_deinit();
    TEST_ASSERT_FALSE(i2c_la_is_running());
    // The channel was aborted and the SM disabled as part of teardown.
    TEST_ASSERT_EQUAL_UINT32(1u, hal_fake_dma_channels[ch].abort_calls);
    TEST_ASSERT_FALSE(hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM].enabled);
}

static void test_deinit_allows_reinit(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    i2c_la_deinit();
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
}

// -----------------------------------------------------------------------------
// start — guards and lifecycle
// -----------------------------------------------------------------------------

static void test_start_without_init_returns_false(void) {
    TEST_ASSERT_FALSE(i2c_la_start(1u));
    TEST_ASSERT_FALSE(i2c_la_is_running());
}

static void test_start_sets_running_then_stop_clears_it(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_FALSE(i2c_la_is_running());
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    TEST_ASSERT_TRUE(i2c_la_is_running());
    TEST_ASSERT_TRUE(hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM].enabled);
    i2c_la_stop();
    TEST_ASSERT_FALSE(i2c_la_is_running());
    TEST_ASSERT_FALSE(hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM].enabled);
}

static void test_start_rejects_double_start(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    TEST_ASSERT_FALSE(i2c_la_start(1u)); // already running
}

static void test_stop_aborts_dma(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    i2c_la_stop();
    TEST_ASSERT_EQUAL_UINT32(1u, hal_fake_dma_channels[ch].abort_calls);
}

// -----------------------------------------------------------------------------
// start — channel/PIO configuration
// -----------------------------------------------------------------------------

static void test_start_configures_ring_mode_pio_dma(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    int ch = claimed_dma_channel();

    TEST_ASSERT_TRUE(i2c_la_start(1u));

    hal_pio_inst_t* pio           = hal_pio_instance(I2C_LA_PIO_INSTANCE);
    const hal_fake_dma_state_t* s = &hal_fake_dma_channels[ch];
    TEST_ASSERT_EQUAL_UINT(HAL_DMA_SIZE_8, s->cfg.size);
    TEST_ASSERT_FALSE(s->cfg.read_increment);
    TEST_ASSERT_TRUE(s->cfg.write_increment);
    // Ring mode on the write side, wrapping the 8192-byte buffer.
    TEST_ASSERT_EQUAL_UINT32(13u, s->cfg.ring_bits);
    TEST_ASSERT_TRUE(s->cfg.ring_on_write);
    TEST_ASSERT_EQUAL_UINT((hal_dma_dreq_t)hal_pio_sm_rx_dreq(pio, I2C_LA_PIO_SM), s->cfg.dreq);
    TEST_ASSERT_EQUAL_PTR(hal_pio_sm_rxfifo_register(pio, I2C_LA_PIO_SM), s->src);
    // Never-ending transfer.
    TEST_ASSERT_EQUAL_UINT32(FOREVER, s->transfer_count);

    const hal_fake_pio_sm_state_t* sm = &hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM];
    TEST_ASSERT_EQUAL_UINT32(0u, sm->last_cfg.in_pin_base);
    TEST_ASSERT_EQUAL_UINT32(8u, sm->last_cfg.in_pin_count);
    TEST_ASSERT_FALSE(sm->last_cfg.in_shift_right);
    TEST_ASSERT_TRUE(sm->enabled);
}

static void test_start_paces_pio_clkdiv_from_interval(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));

    TEST_ASSERT_TRUE(i2c_la_start(10u));

    // 2 SM cycles/sample; clk_div = sys_clk(125 MHz) * 10us / 1e6 / 2 = 625.
    const hal_fake_pio_sm_state_t* sm = &hal_fake_pio_insts[I2C_LA_PIO_INSTANCE].sm[I2C_LA_PIO_SM];
    TEST_ASSERT_EQUAL_FLOAT(625.0f, sm->last_cfg.clk_div);
}

// -----------------------------------------------------------------------------
// total — reflects DMA progress
// -----------------------------------------------------------------------------

static void test_total_is_zero_before_any_samples(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    // Fresh transfer: transfer_count still == FOREVER, nothing written.
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_total());
}

static void test_total_reflects_partial_progress(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    simulate_written(ch, 1234u);
    TEST_ASSERT_EQUAL_UINT32(1234u, i2c_la_total());
}

static void test_total_counts_past_buffer_size(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    // Continuous capture: total is monotonic past the ring buffer size.
    simulate_written(ch, I2C_LA_CAPTURE_BUFFER_BYTES + 5000u);
    TEST_ASSERT_EQUAL_UINT32(I2C_LA_CAPTURE_BUFFER_BYTES + 5000u, i2c_la_total());
}

static void test_total_is_zero_after_deinit(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    int ch = claimed_dma_channel();
    TEST_ASSERT_TRUE(i2c_la_start(1u));
    simulate_written(ch, 500u);
    i2c_la_deinit();
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_total());
}

// -----------------------------------------------------------------------------
// Runner
// -----------------------------------------------------------------------------

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_rejects_equal_pins);
    RUN_TEST(test_init_rejects_out_of_range_pin);
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

    return UNITY_END();
}

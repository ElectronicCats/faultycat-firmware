// Unit tests for services/i2c_core/i2c_la — passive I2C logic analyzer
// sampler (see docs/I2C_LOGIC_ANALYZER_PLAN.md).

#include "unity.h"

#include "hal_fake_gpio.h"
#include "hal_fake_time.h"
#include "i2c_la.h"

#define SDA 2u
#define SCL 3u

void setUp(void) {
    hal_fake_gpio_reset();
    hal_fake_time_reset();
    i2c_la_deinit(); // safe even if not inited — resets s_inited for the next test
}

void tearDown(void) {
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
}

static void test_init_succeeds_and_double_init_rejects(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_TRUE(i2c_la_is_inited());
    TEST_ASSERT_FALSE(i2c_la_init(SDA, SCL)); // already inited
}

static void test_deinit_allows_reinit(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    i2c_la_deinit();
    TEST_ASSERT_FALSE(i2c_la_is_inited());
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
}

// -----------------------------------------------------------------------------
// capture — argument validation
// -----------------------------------------------------------------------------

static void test_capture_without_init_returns_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_capture(1u, 10u, 100u));
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_count());
}

static void test_capture_rejects_zero_max_samples(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_capture(1u, 0u, 100u));
}

static void test_capture_rejects_zero_max_ms(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_capture(1u, 10u, 0u));
}

// -----------------------------------------------------------------------------
// capture — sample bit layout
// -----------------------------------------------------------------------------

static void test_capture_encodes_sda_scl_bits(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));

    hal_gpio_put(SDA, true);
    hal_gpio_put(SCL, false);
    TEST_ASSERT_EQUAL_UINT32(1u, i2c_la_capture(1u, 1u, 1000u));
    TEST_ASSERT_EQUAL_UINT8(I2C_LA_SAMPLE_SDA_BIT, i2c_la_buffer()[0]);

    hal_gpio_put(SDA, false);
    hal_gpio_put(SCL, true);
    TEST_ASSERT_EQUAL_UINT32(1u, i2c_la_capture(1u, 1u, 1000u));
    TEST_ASSERT_EQUAL_UINT8(I2C_LA_SAMPLE_SCL_BIT, i2c_la_buffer()[0]);

    hal_gpio_put(SDA, true);
    hal_gpio_put(SCL, true);
    TEST_ASSERT_EQUAL_UINT32(1u, i2c_la_capture(1u, 1u, 1000u));
    TEST_ASSERT_EQUAL_UINT8(I2C_LA_SAMPLE_SDA_BIT | I2C_LA_SAMPLE_SCL_BIT, i2c_la_buffer()[0]);
}

// -----------------------------------------------------------------------------
// capture — stop conditions
// -----------------------------------------------------------------------------

static void test_capture_stops_at_max_samples_before_max_ms(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    // 5 samples * 10us = 50us elapsed, far under the 1s budget.
    TEST_ASSERT_EQUAL_UINT32(5u, i2c_la_capture(10u, 5u, 1000u));
    TEST_ASSERT_EQUAL_UINT32(5u, i2c_la_count());
}

static void test_capture_stops_at_max_ms_before_max_samples(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    // 1ms per sample; the 3ms budget is checked before each sample, so the
    // 4th check (t=3ms) trips and the capture stops at 3 samples even
    // though max_samples allows for 100.
    TEST_ASSERT_EQUAL_UINT32(3u, i2c_la_capture(1000u, 100u, 3u));
    TEST_ASSERT_EQUAL_UINT32(3u, i2c_la_count());
}

static void test_capture_caps_max_samples_to_buffer_size(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    uint32_t n = i2c_la_capture(1u, I2C_LA_CAPTURE_BUFFER_BYTES + 100u, 1000000u);
    TEST_ASSERT_EQUAL_UINT32(I2C_LA_CAPTURE_BUFFER_BYTES, n);
}

static void test_capture_resets_count_on_rejected_call(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));
    TEST_ASSERT_EQUAL_UINT32(5u, i2c_la_capture(10u, 5u, 1000u));
    // A subsequent rejected call clears the previous count.
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_capture(10u, 0u, 1000u));
    TEST_ASSERT_EQUAL_UINT32(0u, i2c_la_count());
}

// -----------------------------------------------------------------------------
// capture — reconstructs a scripted transaction
// -----------------------------------------------------------------------------

static void test_capture_reconstructs_scripted_transaction(void) {
    TEST_ASSERT_TRUE(i2c_la_init(SDA, SCL));

    // START (SDA falls while SCL high) + one data bit '1' + ACK (slave
    // pulls SDA low) sampled one bit pair per sample.
    static const bool sda_script[] = {true, false, true, true, false, true};
    static const bool scl_script[] = {true, true, true, false, true, true};
    const size_t n                 = sizeof(sda_script) / sizeof(sda_script[0]);

    hal_fake_gpio_input_script_load(SDA, sda_script, n);
    hal_fake_gpio_input_script_load(SCL, scl_script, n);

    TEST_ASSERT_EQUAL_UINT32((uint32_t)n, i2c_la_capture(1u, (uint32_t)n, 1000u));

    const uint8_t* buf = i2c_la_buffer();
    for (size_t i = 0; i < n; i++) {
        uint8_t expected = 0u;
        if (sda_script[i])
            expected |= I2C_LA_SAMPLE_SDA_BIT;
        if (scl_script[i])
            expected |= I2C_LA_SAMPLE_SCL_BIT;
        TEST_ASSERT_EQUAL_UINT8(expected, buf[i]);
    }
}

// -----------------------------------------------------------------------------
// Runner
// -----------------------------------------------------------------------------

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_init_rejects_equal_pins);
    RUN_TEST(test_init_rejects_out_of_range_pin);
    RUN_TEST(test_init_succeeds_and_double_init_rejects);
    RUN_TEST(test_deinit_allows_reinit);

    RUN_TEST(test_capture_without_init_returns_zero);
    RUN_TEST(test_capture_rejects_zero_max_samples);
    RUN_TEST(test_capture_rejects_zero_max_ms);

    RUN_TEST(test_capture_encodes_sda_scl_bits);

    RUN_TEST(test_capture_stops_at_max_samples_before_max_ms);
    RUN_TEST(test_capture_stops_at_max_ms_before_max_samples);
    RUN_TEST(test_capture_caps_max_samples_to_buffer_size);
    RUN_TEST(test_capture_resets_count_on_rejected_call);

    RUN_TEST(test_capture_reconstructs_scripted_transaction);

    return UNITY_END();
}

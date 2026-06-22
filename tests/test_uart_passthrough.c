// Unit tests for services/uart_passthrough — CDC3 <-> scanner-header
// UART0 byte bridge, plus its claim on the shared swd_bus_lock.

#include "unity.h"

#include "swd_bus_lock.h"
#include "uart_passthrough.h"

#include "hal_fake_uart.h"

#include <string.h>

static uint8_t s_rx_capture[256];
static size_t s_rx_capture_len;

static void capture_write_byte(uint8_t b, void* user) {
    (void)user;
    s_rx_capture[s_rx_capture_len++] = b;
}

static const uart_passthrough_callbacks_t TEST_CALLBACKS = {
    .write_byte = capture_write_byte,
    .user       = NULL,
};

static const hal_uart_config_t DEFAULT_CFG = {
    .baudrate  = 115200u,
    .data_bits = 8u,
    .stop_bits = 1u,
    .parity    = HAL_UART_PARITY_NONE,
};

void setUp(void) {
    swd_bus_lock_init();
    hal_fake_uart_reset();
    s_rx_capture_len = 0;
}

void tearDown(void) {
    // uart_passthrough keeps its enabled/config state in static
    // globals (no reset hook by design — main.c never needs one).
    // Tests must tear down explicitly so each case starts clean.
    uart_passthrough_disable();
}

// -----------------------------------------------------------------------------
// Enable / disable + bus ownership
// -----------------------------------------------------------------------------

static void test_enable_claims_bus_and_inits_hal(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    TEST_ASSERT_TRUE(uart_passthrough_is_enabled());
    TEST_ASSERT_EQUAL(SWD_BUS_OWNER_UART_PASSTHRU, swd_bus_owner());
    TEST_ASSERT_EQUAL(1u, hal_fake_uart_state.init_calls);
}

static void test_enable_fails_when_bus_held_elsewhere(void) {
    TEST_ASSERT_TRUE(swd_bus_try_acquire(SWD_BUS_OWNER_SCANNER));
    TEST_ASSERT_FALSE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    TEST_ASSERT_FALSE(uart_passthrough_is_enabled());
    TEST_ASSERT_EQUAL(0u, hal_fake_uart_state.init_calls);
}

static void test_enable_twice_fails(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    TEST_ASSERT_FALSE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    TEST_ASSERT_EQUAL(1u, hal_fake_uart_state.init_calls);
}

static void test_disable_releases_bus_and_deinits_hal(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    uart_passthrough_disable();
    TEST_ASSERT_FALSE(uart_passthrough_is_enabled());
    TEST_ASSERT_EQUAL(SWD_BUS_OWNER_IDLE, swd_bus_owner());
    TEST_ASSERT_EQUAL(1u, hal_fake_uart_state.deinit_calls);
}

static void test_disable_when_not_enabled_is_safe(void) {
    uart_passthrough_disable();
    TEST_ASSERT_EQUAL(0u, hal_fake_uart_state.deinit_calls);
}

static void test_disable_then_another_owner_can_acquire(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    uart_passthrough_disable();
    TEST_ASSERT_TRUE(swd_bus_try_acquire(SWD_BUS_OWNER_SCANNER));
}

// -----------------------------------------------------------------------------
// Live reconfigure
// -----------------------------------------------------------------------------

static void test_set_baud_updates_config_when_enabled(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    uart_passthrough_set_baud(9600u);
    TEST_ASSERT_EQUAL_UINT32(9600u, uart_passthrough_get_config().baudrate);
    TEST_ASSERT_EQUAL(1u, hal_fake_uart_state.set_config_calls);
}

static void test_set_parity_updates_config_when_enabled(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    uart_passthrough_set_parity(HAL_UART_PARITY_EVEN);
    TEST_ASSERT_EQUAL(HAL_UART_PARITY_EVEN, uart_passthrough_get_config().parity);
}

static void test_set_stop_bits_updates_config_when_enabled(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    uart_passthrough_set_stop_bits(2u);
    TEST_ASSERT_EQUAL_UINT8(2u, uart_passthrough_get_config().stop_bits);
}

static void test_setters_are_no_op_when_disabled(void) {
    uart_passthrough_set_baud(9600u);
    uart_passthrough_set_parity(HAL_UART_PARITY_ODD);
    uart_passthrough_set_stop_bits(2u);
    TEST_ASSERT_EQUAL(0u, hal_fake_uart_state.set_config_calls);
}

// -----------------------------------------------------------------------------
// Pump — byte bridging in both directions
// -----------------------------------------------------------------------------

static void test_pump_writes_host_bytes_to_uart(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    const uint8_t host_bytes[] = {0x41, 0x42, 0x43};
    uart_passthrough_pump(host_bytes, sizeof(host_bytes));
    TEST_ASSERT_EQUAL(sizeof(host_bytes), hal_fake_uart_state.tx_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(host_bytes, hal_fake_uart_state.tx_buf, sizeof(host_bytes));
}

static void test_pump_drains_uart_rx_to_host(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    const uint8_t target_bytes[] = {0x55, 0x66};
    hal_fake_uart_inject_rx(target_bytes, sizeof(target_bytes));
    uart_passthrough_pump(NULL, 0);
    TEST_ASSERT_EQUAL(sizeof(target_bytes), s_rx_capture_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(target_bytes, s_rx_capture, sizeof(target_bytes));
}

static void test_pump_is_no_op_when_disabled(void) {
    const uint8_t host_bytes[] = {0x41};
    uart_passthrough_pump(host_bytes, sizeof(host_bytes));
    TEST_ASSERT_EQUAL(0u, hal_fake_uart_state.tx_len);
}

// -----------------------------------------------------------------------------
// TX retry — host→target bytes that don't fit the UART's hardware FIFO
// in one pump must be retried, not dropped (see hal/src/rp2040/uart.c's
// PL011 FIFO and services/uart_passthrough/uart_passthrough.c's staging
// buffer fix).
// -----------------------------------------------------------------------------

static void test_pump_retries_bytes_that_overflow_fifo(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    // Simulate a 2-byte FIFO: only the first 2 of 5 bytes go out on
    // this pump.
    hal_fake_uart_state.tx_room_per_call = 2u;
    const uint8_t host_bytes[]           = {0x01, 0x02, 0x03, 0x04, 0x05};
    uart_passthrough_pump(host_bytes, sizeof(host_bytes));
    TEST_ASSERT_EQUAL(2u, hal_fake_uart_state.tx_len);
    TEST_ASSERT_EQUAL_UINT32(0u, uart_passthrough_get_tx_dropped());

    // FIFO drains (simulated as "unlimited room" again) on the next
    // pump — the staged 3 bytes go out with no new host bytes, in order.
    hal_fake_uart_state.tx_room_per_call = 0u;
    uart_passthrough_pump(NULL, 0);
    TEST_ASSERT_EQUAL(5u, hal_fake_uart_state.tx_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(host_bytes, hal_fake_uart_state.tx_buf, sizeof(host_bytes));
    TEST_ASSERT_EQUAL_UINT32(0u, uart_passthrough_get_tx_dropped());
}

static void test_pump_preserves_order_across_staged_and_new_bytes(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    hal_fake_uart_state.tx_room_per_call = 1u;
    const uint8_t first[]                = {0xAA, 0xBB};
    uart_passthrough_pump(first, sizeof(first)); // 0xAA out, 0xBB staged

    const uint8_t second[] = {0xCC, 0xDD};
    // FIFO still saturated — second batch must queue behind 0xBB, not
    // jump ahead of it.
    uart_passthrough_pump(second, sizeof(second));

    hal_fake_uart_state.tx_room_per_call = 0u;
    uart_passthrough_pump(NULL, 0);

    const uint8_t expected[] = {0xAA, 0xBB, 0xCC, 0xDD};
    TEST_ASSERT_EQUAL(sizeof(expected), hal_fake_uart_state.tx_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, hal_fake_uart_state.tx_buf, sizeof(expected));
}

static void test_pump_counts_drops_when_staging_buffer_also_full(void) {
    TEST_ASSERT_TRUE(uart_passthrough_enable(DEFAULT_CFG, &TEST_CALLBACKS));
    // FIFO completely stalled — every byte must land in the staging
    // buffer (256 B cap). A 300-byte burst overflows it by 44 bytes.
    hal_fake_uart_state.tx_blocked = true;
    static uint8_t big[300];
    memset(big, 0x5A, sizeof(big));
    uart_passthrough_pump(big, sizeof(big));
    TEST_ASSERT_EQUAL(0u, hal_fake_uart_state.tx_len);
    TEST_ASSERT_EQUAL_UINT32(44u, uart_passthrough_get_tx_dropped());

    // FIFO recovers — the 256 staged bytes drain in order; the 44
    // dropped bytes are gone for good (the documented, honest cost of
    // exceeding the bridge's backpressure budget).
    hal_fake_uart_state.tx_blocked = false;
    uart_passthrough_pump(NULL, 0);
    TEST_ASSERT_EQUAL(256u, hal_fake_uart_state.tx_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(big, hal_fake_uart_state.tx_buf, 256u);
    TEST_ASSERT_EQUAL_UINT32(44u, uart_passthrough_get_tx_dropped());
}

// -----------------------------------------------------------------------------
// Runner
// -----------------------------------------------------------------------------

int main(void) {
    UNITY_BEGIN();

    RUN_TEST(test_enable_claims_bus_and_inits_hal);
    RUN_TEST(test_enable_fails_when_bus_held_elsewhere);
    RUN_TEST(test_enable_twice_fails);
    RUN_TEST(test_disable_releases_bus_and_deinits_hal);
    RUN_TEST(test_disable_when_not_enabled_is_safe);
    RUN_TEST(test_disable_then_another_owner_can_acquire);

    RUN_TEST(test_set_baud_updates_config_when_enabled);
    RUN_TEST(test_set_parity_updates_config_when_enabled);
    RUN_TEST(test_set_stop_bits_updates_config_when_enabled);
    RUN_TEST(test_setters_are_no_op_when_disabled);

    RUN_TEST(test_pump_writes_host_bytes_to_uart);
    RUN_TEST(test_pump_drains_uart_rx_to_host);
    RUN_TEST(test_pump_is_no_op_when_disabled);

    RUN_TEST(test_pump_retries_bytes_that_overflow_fifo);
    RUN_TEST(test_pump_preserves_order_across_staged_and_new_bytes);
    RUN_TEST(test_pump_counts_drops_when_staging_buffer_also_full);

    return UNITY_END();
}

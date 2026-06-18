#include "uart_passthrough.h"

#include "swd_bus_lock.h"

static bool s_enabled;
static hal_uart_config_t s_cfg;
static uart_passthrough_callbacks_t s_cb;

bool uart_passthrough_enable(hal_uart_config_t cfg, const uart_passthrough_callbacks_t* cb) {
    if (s_enabled)
        return false; // already running — caller must disable first
    if (!swd_bus_try_acquire(SWD_BUS_OWNER_UART_PASSTHRU))
        return false;

    hal_uart_init(cfg);
    s_cfg     = cfg;
    s_cb      = *cb;
    s_enabled = true;
    return true;
}

void uart_passthrough_disable(void) {
    if (!s_enabled)
        return;
    hal_uart_deinit();
    swd_bus_release(SWD_BUS_OWNER_UART_PASSTHRU);
    s_enabled = false;
}

bool uart_passthrough_is_enabled(void) {
    return s_enabled;
}

void uart_passthrough_set_baud(uint32_t baudrate) {
    if (!s_enabled)
        return;
    s_cfg.baudrate = baudrate;
    hal_uart_set_config(s_cfg);
}

void uart_passthrough_set_parity(hal_uart_parity_t parity) {
    if (!s_enabled)
        return;
    s_cfg.parity = parity;
    hal_uart_set_config(s_cfg);
}

void uart_passthrough_set_stop_bits(uint8_t stop_bits) {
    if (!s_enabled)
        return;
    s_cfg.stop_bits = stop_bits;
    hal_uart_set_config(s_cfg);
}

hal_uart_config_t uart_passthrough_get_config(void) {
    return s_cfg;
}

void uart_passthrough_pump(const uint8_t* from_host, size_t from_host_len) {
    if (!s_enabled)
        return;

    if (from_host_len > 0)
        hal_uart_write(from_host, from_host_len);

    uint8_t rx_chunk[64];
    while (hal_uart_rx_available()) {
        size_t n = hal_uart_read(rx_chunk, sizeof(rx_chunk));
        if (n == 0)
            break;
        for (size_t i = 0; i < n; i++) {
            s_cb.write_byte(rx_chunk[i], s_cb.user);
        }
    }
}

#include "uart_passthrough.h"

#include <string.h>

#include "hal/uart_debug.h"
#include "swd_bus_lock.h"

static bool s_enabled;
static hal_uart_config_t s_cfg;
static uart_passthrough_callbacks_t s_cb;

// Host→target staging buffer. hal_uart_write() is non-blocking and
// returns however many bytes actually fit in the (32-byte) PL011 TX
// FIFO; the remainder used to be silently dropped. Stage it here and
// retry on the next pump instead — at any baud fast enough to drain
// the FIFO between 1 ms main-loop ticks this empties immediately, so
// the steady-state cost is one extra memcpy check.
#define TX_PENDING_CAP 256u
static uint8_t s_tx_pending[TX_PENDING_CAP];
static size_t s_tx_pending_len;
static uint32_t s_tx_dropped; // bytes lost because even the staging buffer was full

bool uart_passthrough_enable(hal_uart_config_t cfg, const uart_passthrough_callbacks_t* cb) {
    if (s_enabled)
        return false; // already running — caller must disable first
    if (!swd_bus_try_acquire(SWD_BUS_OWNER_UART_PASSTHRU))
        return false;

    hal_uart_init(cfg);
    hal_uart_debug_init(cfg.baudrate);
    s_cfg            = cfg;
    s_cb             = *cb;
    s_enabled        = true;
    s_tx_pending_len = 0u;
    s_tx_dropped     = 0u;
    return true;
}

void uart_passthrough_disable(void) {
    if (!s_enabled)
        return;
    hal_uart_deinit();
    hal_uart_debug_deinit();
    swd_bus_release(SWD_BUS_OWNER_UART_PASSTHRU);
    s_enabled        = false;
    s_tx_pending_len = 0u;
}

bool uart_passthrough_is_enabled(void) {
    return s_enabled;
}

void uart_passthrough_set_baud(uint32_t baudrate) {
    if (!s_enabled)
        return;
    s_cfg.baudrate = baudrate;
    hal_uart_set_config(s_cfg);
    hal_uart_debug_set_baud(baudrate);
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

uint32_t uart_passthrough_get_tx_dropped(void) {
    return s_tx_dropped;
}

// Append as much of `data` as fits into the staging buffer; whatever
// doesn't fit is counted as dropped. Only reached when the FIFO is
// staying full across multiple pumps (sustained host throughput above
// what the configured baud can drain).
static void tx_stage(const uint8_t* data, size_t len) {
    size_t room = TX_PENDING_CAP - s_tx_pending_len;
    size_t n    = (len < room) ? len : room;
    memcpy(s_tx_pending + s_tx_pending_len, data, n);
    s_tx_pending_len += n;
    if (n < len) {
        s_tx_dropped += (uint32_t)(len - n);
    }
}

// Push whatever is staged out to the UART first, preserving byte order
// against anything staged in an earlier pump. Shifts any leftover to
// the front of the buffer so the next stage() call appends cleanly.
static void tx_flush_pending(void) {
    if (s_tx_pending_len == 0u)
        return;
    size_t n = hal_uart_write(s_tx_pending, s_tx_pending_len);
    if (n == s_tx_pending_len) {
        s_tx_pending_len = 0u;
        return;
    }
    memmove(s_tx_pending, s_tx_pending + n, s_tx_pending_len - n);
    s_tx_pending_len -= n;
}

void uart_passthrough_pump(const uint8_t* from_host, size_t from_host_len) {
    if (!s_enabled)
        return;

    tx_flush_pending();

    if (from_host_len > 0) {
        if (s_tx_pending_len == 0u) {
            size_t n = hal_uart_write(from_host, from_host_len);
            if (n < from_host_len) {
                tx_stage(from_host + n, from_host_len - n);
            }
        } else {
            // FIFO is still backed up from a previous pump — stage
            // behind what's already pending instead of writing out of
            // order.
            tx_stage(from_host, from_host_len);
        }
    }

    uint8_t rx_chunk[64];
    while (hal_uart_rx_available()) {
        size_t n = hal_uart_read(rx_chunk, sizeof(rx_chunk));
        if (n == 0)
            break;
        for (size_t i = 0; i < n; i++) {
            s_cb.write_byte(rx_chunk[i], s_cb.user);
        }
        // Mirror to the debug UART (UART1/GP4) for an external
        // logic analyzer or second terminal. Best-effort: if the
        // debug TX FIFO can't keep up, those bytes are simply lost —
        // unlike the host path, there's no retry-staging for a tap.
        hal_uart_debug_write(rx_chunk, n);
    }
}

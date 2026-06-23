#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/uart.h"

#define HAL_FAKE_UART_TX_CAP 256
#define HAL_FAKE_UART_RX_CAP 256

typedef struct {
    bool initialized;
    hal_uart_config_t last_config;
    uint32_t init_calls;
    uint32_t deinit_calls;
    uint32_t set_config_calls;

    uint8_t tx_buf[HAL_FAKE_UART_TX_CAP]; // bytes the service wrote out
    size_t tx_len;

    // Caps how many bytes a single hal_uart_write() call accepts,
    // independent of the remaining tx_buf capacity — simulates a
    // hardware TX FIFO that's smaller than what the caller hands it.
    // 0 = unlimited (default).
    size_t tx_room_per_call;

    // When true, hal_uart_write() accepts 0 bytes regardless of
    // tx_room_per_call — simulates a fully stalled TX FIFO (e.g. the
    // line is disconnected / target not draining).
    bool tx_blocked;

    uint8_t rx_buf[HAL_FAKE_UART_RX_CAP]; // bytes a test injects as "received"
    size_t rx_len;
    size_t rx_pos;
} hal_fake_uart_state_t;

extern hal_fake_uart_state_t hal_fake_uart_state;

void hal_fake_uart_reset(void);

// Test seam: queue bytes for the next hal_uart_read calls to return.
void hal_fake_uart_inject_rx(const uint8_t* data, size_t len);

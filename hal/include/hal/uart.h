#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// HAL — UART
//
// Portable access to a single hardware UART instance, non-blocking.
// On RP2040 this wraps UART0 fixed to its native GP0 (TX) / GP1 (RX)
// pins — see services/uart_passthrough for the policy layer that
// claims the scanner-header bus before calling into this HAL.
//
// Scope: init/deinit, live baud/parity/stopbits reconfigure,
// non-blocking byte read/write. No flow control (CTS/RTS), no IRQs —
// the passthrough service polls every main-loop tick.

typedef enum {
    HAL_UART_PARITY_NONE = 0,
    HAL_UART_PARITY_EVEN = 1,
    HAL_UART_PARITY_ODD  = 2,
} hal_uart_parity_t;

typedef struct {
    uint32_t baudrate;
    uint8_t data_bits; // 5..8
    uint8_t stop_bits; // 1..2
    hal_uart_parity_t parity;
} hal_uart_config_t;

// Claim UART0 + its native TX/RX pins and apply `cfg`. Safe to call
// again without an intervening deinit (re-applies pin function +
// config), but callers should prefer `hal_uart_set_config` for a
// live reconfigure since it avoids re-touching the GPIO mux.
void hal_uart_init(hal_uart_config_t cfg);

// Release the UART. Leaves the TX/RX pins at whatever level they were
// last driving — callers that care about pin state after release
// should reconfigure them via hal/gpio.h.
void hal_uart_deinit(void);

// Apply a new baud/format without re-touching the GPIO mux. Must be
// called after hal_uart_init.
void hal_uart_set_config(hal_uart_config_t cfg);

// Non-blocking write. Returns the number of bytes actually queued
// into the TX FIFO; stops early if the FIFO fills up.
size_t hal_uart_write(const uint8_t* data, size_t len);

// Non-blocking read. Returns the number of bytes actually read (0 if
// the RX FIFO is empty).
size_t hal_uart_read(uint8_t* data, size_t cap);

// True if at least one byte is waiting in the RX FIFO.
bool hal_uart_rx_available(void);

// --- RX diagnostics (temporary — remove once the RX path is proven) --------
// Snapshot of what the RP2040 UART0 RX side is actually doing, so the host
// can tell "no byte ever reached the FIFO" from "bytes arrived but were
// garbled" from "bytes read but never left over USB".
typedef struct {
    uint32_t func_tx;     // gpio_get_function(GP0) — expect 2 (GPIO_FUNC_UART)
    uint32_t func_rx;     // gpio_get_function(GP1) — expect 2 (GPIO_FUNC_UART)
    uint32_t rx_level;    // GP1 pad level NOW — a UART RX line MUST idle at 1
    uint32_t tx_level;    // GP0 pad level NOW — idles at 1 too
    uint32_t cr;          // PL011 CR — expect EN(0x1)|TXE(0x100)|RXE(0x200)=0x301
    uint32_t lcr_h;       // PL011 LCR_H — word length / FIFO enable / parity
    uint32_t fr;          // PL011 FR — RXFE(0x10) set when RX FIFO empty
    uint32_t bytes_read;  // total bytes pulled out of the RX FIFO
    uint32_t err_framing; // DR framing errors (FE) — baud/format/idle wrong
    uint32_t err_parity;  // DR parity errors (PE)
    uint32_t err_break;   // DR break errors (BE) — line stuck low / inverted
    uint32_t err_overrun; // DR overrun errors (OE) — FIFO not drained fast
} hal_uart_rx_diag_t;

void hal_uart_get_rx_diag(hal_uart_rx_diag_t* out);

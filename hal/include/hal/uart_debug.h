#pragma once

#include <stddef.h>
#include <stdint.h>

// HAL — debug UART
//
// A second, TX-only UART used to mirror traffic for external
// debugging (logic analyzer / second terminal) without disturbing
// the primary hal/uart.h instance. On RP2040 this wraps UART1 on
// GP4 — UART1's native TX, shared with scanner CH4 and protected by
// the same swd_bus_lock that guards UART0/scanner CH0-1. No RX pin
// is claimed since this is an outgoing mirror only.

// Claim UART1 TX (GP4) and apply `baudrate` (8N1, fixed format —
// this is a debug tap, not a configurable link).
void hal_uart_debug_init(uint32_t baudrate);

// Release UART1 TX.
void hal_uart_debug_deinit(void);

// Live baud change. Must be called after hal_uart_debug_init.
void hal_uart_debug_set_baud(uint32_t baudrate);

// Non-blocking write, same contract as hal_uart_write: returns the
// number of bytes actually queued, stops early if the TX FIFO fills.
size_t hal_uart_debug_write(const uint8_t* data, size_t len);

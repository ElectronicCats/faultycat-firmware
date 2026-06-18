#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/uart.h"

// services/uart_passthrough — bridges raw bytes between a CDC (the
// host's "Target UART" port, CDC3 in apps/faultycat_fw/main.c) and
// the RP2040's UART0 peripheral, fixed to the scanner-header CH0
// (TX) / CH1 (RX) pins (see hal/src/rp2040/uart.c).
//
// Unlike services/buspirate_compat and services/flashrom_serprog,
// there is no framed protocol to parse here — it's a dumb byte pipe.
// Control (baud/parity/stop-bits) happens out-of-band via shell
// commands on CDC2; this service only owns enable/disable/pump.
//
// Claims services/swd_bus_lock's SWD_BUS_OWNER_UART_PASSTHRU around
// the session since CH0/CH1 are shared with swd_core/jtag_core/
// pinout_scanner/campaign_manager. `uart_passthrough_enable` fails if
// the bus is held by another owner, mirroring the contract callers
// already get from `swd_bus_try_acquire`.

typedef struct {
    void (*write_byte)(uint8_t b, void* user); // → host CDC (TX path)
    void* user;
} uart_passthrough_callbacks_t;

// Claim the scanner-header bus, init the HAL UART with `cfg`, and
// stash `cb` for `uart_passthrough_pump`'s UART→host direction.
// Returns false (no-op) if the bus is already held by another owner.
bool uart_passthrough_enable(hal_uart_config_t cfg, const uart_passthrough_callbacks_t* cb);

// Deinit the HAL UART and release the bus. Safe to call when already
// disabled.
void uart_passthrough_disable(void);

bool uart_passthrough_is_enabled(void);

// Live reconfigure. No-op if not currently enabled.
void uart_passthrough_set_baud(uint32_t baudrate);
void uart_passthrough_set_parity(hal_uart_parity_t parity);
void uart_passthrough_set_stop_bits(uint8_t stop_bits);

// Current config (only meaningful while enabled).
hal_uart_config_t uart_passthrough_get_config(void);

// Drains up to `len` bytes of host→target traffic (the caller reads
// these from CDC3 and hands them here) into the UART TX FIFO, then
// drains whatever's waiting in the UART RX FIFO out through
// `write_byte`. No-op if not enabled. Call once per main-loop tick.
void uart_passthrough_pump(const uint8_t* from_host, size_t from_host_len);

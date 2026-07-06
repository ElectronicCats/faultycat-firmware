#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/gpio.h"

// services/i2c_core — I2C bit-bang master (CPU-driven), analogous to
// services/jtag_core but for the I2C protocol.
//
// Per docs/I2C_SCANNER_PLAN.md: no hardware_i2c peripheral (pico-sdk
// ties SDA/SCL to fixed pin pairs; the scanner header needs SDA/SCL on
// any of GP0..GP7). Open-drain is emulated over hal/gpio:
//   - "drive low"   -> reconfigure pin as OUTPUT, put(false).
//   - "release/high"-> reconfigure pin as INPUT with internal pull-up.
// This mirrors real open-drain wired-AND behavior: any device (master
// or slave) can pull the line low, and it only goes high when every
// party releases it. It also means a slave can hold SCL low past our
// release point (clock stretching) and we observe that by reading the
// pin back before assuming the bus actually went high.
//
// Protocol decisions (resolving the "Pendiente de decisión" in the
// plan doc):
//   1. Timing: i2c_init takes freq_khz and derives a half-bit-period
//      busy-wait from it via hal_busy_wait_us. No PIO, no DMA — same
//      bit-bang-by-CPU philosophy as jtag_core.
//   2. Clock stretching: every time we release SCL we poll it back
//      (bounded by I2C_CLOCK_STRETCH_TIMEOUT_US) and only proceed once
//      it reads high, or the timeout elapses. We don't hang forever —
//      a slave wedged low would otherwise lock up the whole shell.
//   3. NACK mid-transfer: i2c_write_byte returns whether the slave
//      ACKed: callers (i2c_probe_address, future multi-byte transfers)
//      decide whether to continue or issue i2c_stop. i2c_core itself
//      never aborts a transfer on NACK — that's the caller's policy,
//      same division of responsibility as jtag_core leaving TAP
//      navigation decisions to its callers.
//
// 100% host-testable: links against tests/hal_fake/gpio_fake.c, using
// hal_fake_gpio_input_script_load on the SDA pin to script a
// slave's ACK/NACK bits and read-byte values bit by bit.

#define I2C_SCAN_ADDR_MIN 0x08u // first address in the standard 7-bit scan range
#define I2C_SCAN_ADDR_MAX 0x77u // last address in the standard 7-bit scan range

// How long we wait for a slave to release a stretched SCL before we
// give up and treat the bus as stuck. 1ms is generous for bit-banged
// I2C in the tens-to-hundreds of kHz range.
#define I2C_CLOCK_STRETCH_TIMEOUT_US 1000u

// Initialize the bit-bang master on the given SDA/SCL pins at
// `freq_khz` (e.g. 100 for standard mode). Both pins start released
// (idle high via internal pull-up), matching the I2C bus-idle state.
//
// Returns false if sda/scl are out of range, equal, or already
// inited. freq_khz == 0 falls back to 100 (standard mode). Cross-
// service ownership (vs swd_phy / jtag_core sharing the same header)
// is enforced at the shell/swd_bus_lock level, NOT here.
bool i2c_init(uint8_t sda, uint8_t scl, uint32_t freq_khz);

// Tear down: release both pins back to plain GPIO inputs with pulls
// disabled. Safe to call repeatedly.
void i2c_deinit(void);

// True iff i2c_init succeeded since the last deinit.
bool i2c_is_inited(void);

// Drive a START condition (SDA falls while SCL is high). Bus must be
// idle (both lines released) on entry. Undefined if !i2c_is_inited().
void i2c_start(void);

// Drive a STOP condition (SDA rises while SCL is high), returning the
// bus to idle. Undefined if !i2c_is_inited().
void i2c_stop(void);

// Shift out `byte` MSB-first, then sample the slave's ACK/NACK bit.
// Returns true if the slave pulled SDA low (ACK), false on NACK or on
// a clock-stretch timeout. Undefined if !i2c_is_inited().
bool i2c_write_byte(uint8_t byte);

// Shift in one byte MSB-first, then drive the master's reply bit:
// `ack = true` pulls SDA low (more bytes follow), `ack = false`
// releases it (NACK, last byte of the transfer). Undefined if
// !i2c_is_inited().
uint8_t i2c_read_byte(bool ack);

// Probe a single 7-bit address: START, write (addr << 1 | 0) i.e. a
// write-direction address byte, sample ACK/NACK, STOP. Returns true
// iff the slave ACKed. This is the single building block both
// i2c_bus_scan and services/pinout_scanner's i2c sweep use.
bool i2c_probe_address(uint8_t addr);

// Probe every address in [I2C_SCAN_ADDR_MIN, I2C_SCAN_ADDR_MAX] and
// write the ones that ACKed into `found` (caller-owned, capacity
// `max`). Returns the number of addresses written into `found`,
// capped at `max`. Undefined if !i2c_is_inited().
size_t i2c_bus_scan(uint8_t* found, size_t max);

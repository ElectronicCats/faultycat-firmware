#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/gpio.h"

// services/i2c_core/i2c_la — passive I2C logic analyzer (CPU sampler).
//
// Sibling of i2c_core, not part of it (see docs/I2C_LOGIC_ANALYZER_PLAN.md
// §"Componentes nuevos"/1): the bit-bang master and this passive sampler
// have independent lifecycles and must not share static state. i2c_la
// never drives SDA/SCL — it only reads them, so it can observe either an
// i2c_core transfer (loop back to back) or any third-party device on the
// scanner header.
//
// Same constraints as i2c_core (per the plan doc):
//   - No PIO: SDA/SCL are an arbitrary (not necessarily adjacent) pair
//     over GP0..GP7, and hal_pio_sm_cfg_t needs a contiguous pin window.
//   - No async timer: hal/time only exposes busy-wait + monotonic clocks,
//     so fixed-interval sampling is a blocking busy-wait loop, same as
//     i2c_core's half_delay().
//   - Blocking, non-cooperative by design: yielding mid-capture (e.g. for
//     tud_task) would jitter the sample interval. max_ms bounds the
//     worst-case stall instead.
//
// 100% host-testable: links against tests/hal_fake/gpio_fake.c, using
// hal_fake_gpio_input_script_load on SDA/SCL to script a transaction
// (START/address/ACK/byte) bit by bit and verify i2c_la_buffer()
// reconstructs the same level sequence.

// Capture buffer size — same order of magnitude as
// EMFI_CAPTURE_BUFFER_BYTES (services/glitch_engine/emfi/emfi_capture.h).
// One byte per sample; with a fixed sample interval the timestamp of each
// sample is implicit (index * sample_interval_us), so no separate
// timestamp storage is needed.
#define I2C_LA_CAPTURE_BUFFER_BYTES 8192u

// Per-sample bit layout: bit0 = SDA level, bit1 = SCL level. All other
// bits are always 0.
#define I2C_LA_SAMPLE_SDA_BIT 0x01u
#define I2C_LA_SAMPLE_SCL_BIT 0x02u

// Initialize the passive sampler on the given SDA/SCL pins. Both pins are
// configured as plain inputs with no pull resistors touched — the bus
// already has pulls from i2c_core or an external device, and the sampler
// must not affect bus levels.
//
// Returns false if sda/scl are out of range, equal, or already inited.
bool i2c_la_init(uint8_t sda, uint8_t scl);

// Tear down: release both pins back to plain GPIO inputs with pulls
// disabled. Safe to call repeatedly.
void i2c_la_deinit(void);

// True iff i2c_la_init succeeded since the last deinit.
bool i2c_la_is_inited(void);

// Blocking capture: sample SDA/SCL every `sample_interval_us` until either
// `max_samples` is reached or `max_ms` has elapsed, whichever comes
// first. `max_samples` is capped at I2C_LA_CAPTURE_BUFFER_BYTES;
// `sample_interval_us == 0` is treated as 1. Returns the number of
// samples actually captured (also available via i2c_la_count()).
//
// Blocking on purpose — see header comment above. Callers (the shell
// command) must keep max_ms low enough to bound the USB stall.
//
// Returns 0 without sampling if !i2c_la_is_inited(), max_samples == 0, or
// max_ms == 0.
uint32_t i2c_la_capture(uint32_t sample_interval_us, uint32_t max_samples, uint32_t max_ms);

// Pointer to the capture buffer, valid until the next i2c_la_capture()
// call or i2c_la_deinit(). Only the first i2c_la_count() bytes are
// meaningful.
const uint8_t* i2c_la_buffer(void);

// Number of samples captured by the most recent i2c_la_capture() call.
uint32_t i2c_la_count(void);

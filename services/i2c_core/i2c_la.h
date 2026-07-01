#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/gpio.h"

// services/i2c_core/i2c_la — passive I2C logic analyzer (DMA streamer).
//
// Sibling of i2c_core, not part of it (see docs/I2C_LOGIC_ANALYZER_PLAN.md
// §"Componentes nuevos"/1): the bit-bang master and this passive sampler
// have independent lifecycles and must not share static state. i2c_la
// never drives SDA/SCL — it only reads them, so it can observe either an
// i2c_core transfer (loop back to back) or any third-party device on the
// scanner header.
//
// Sampling path (see docs/I2C_LA_DMA_TIMER_PLAN.md):
//   - PIO (pio1/SM2) runs a 2-instruction loop (`in pins, 8` / `push`)
//     that snapshots GP0..GP7 into its RX FIFO every clk_div cycles; a
//     DMA channel drains that FIFO into the buffer with no CPU per
//     sample — same ring pattern as emfi_capture.c. An earlier design
//     paced a DMA channel reading straight from SIO->GPIO_IN via a DMA
//     timer; that doesn't work on RP2040 (SIO isn't reachable by the DMA
//     bus master), hence PIO instead.
//   - Continuous streaming, not a fixed one-shot: i2c_la_start arms a
//     ring-mode DMA that runs in the background and wraps the buffer
//     forever. The caller drains it as it fills (read cursor vs.
//     i2c_la_total()), so a capture can exceed the buffer size — the
//     buffer is a sliding window, not the capture limit. i2c_la_stop
//     ends it. Nothing blocks: the CPU is free between drains.
//
// Host-testable: links against tests/hal_fake (gpio_fake.c + dma_fake.c).
// The fake DMA does not copy bytes, so tests verify the channel/timer are
// configured correctly (ring_bits, dreq, src == hal_gpio_in_register) and
// that i2c_la_total() reflects the simulated DMA progress; on-wire
// reconstruction is a hardware test.

// Ring buffer size. One byte per sample; with a fixed sample interval the
// timestamp of each sample is implicit (index * sample_interval_us), so no
// separate timestamp storage is needed. In continuous mode this is the
// sliding-window size, not the total capture length — the DMA wraps it and
// the caller must drain faster than I2C_LA_CAPTURE_BUFFER_BYTES samples
// accumulate or it lags. 32KB (up from 8KB) quadruples the drain-time
// budget at the fastest supported sample rate (1us/sample: 32768us = 33ms
// before the DMA laps the cursor, vs. 8ms at the old size) — the real
// overflow risk at high sample rates is USB CDC drain throughput, not
// total capture length (the streaming loop in cmd_i2c_la/cmd_uart_la/
// sump_ols.c::do_arm handles n_samples well beyond this size already).
// 32768 (2^15) is also the hardware ceiling: the RP2040 DMA ring-wrap
// field is 4 bits wide (0-15), so this is the largest ring the silicon
// can express — see I2C_LA_CAPTURE_RING_BITS in i2c_la.c. RP2040 has
// 256KB SRAM total; this and emfi_capture's 8KB ring are the only large
// static buffers, leaving plenty of headroom. MUST stay a power of two
// (ring-mode wrap masks the low bits).
#define I2C_LA_CAPTURE_BUFFER_BYTES 32768u

// Per-sample byte: a raw snapshot of GPIO_IN[7:0] (GP0..GP7) copied
// verbatim by DMA. The SDA level of a sample is bit (1u << sda) and SCL is
// bit (1u << scl) — the bit position is the pin number, not a fixed 0/1
// layout. Per-pin extraction happens at dump time, not during capture, so
// SDA/SCL must live in GP0..GP7 for this byte-wide capture.
#define I2C_LA_SAMPLE_BIT(pin) ((uint8_t)(1u << (pin)))

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

// Start continuous capture: arm a ring-mode DMA that copies GPIO_IN[7:0]
// into the buffer every `sample_interval_us`, wrapping forever, with no
// CPU per sample. Non-blocking — returns immediately while the DMA runs in
// the background. `sample_interval_us == 0` is treated as 1.
//
// The caller drains the ring as it fills: read i2c_la_buffer()[k %
// I2C_LA_CAPTURE_BUFFER_BYTES] for k in [read_cursor, i2c_la_total()), and
// advance the cursor. If (i2c_la_total() - read_cursor) ever exceeds
// I2C_LA_CAPTURE_BUFFER_BYTES the DMA has lapped the cursor and those
// samples are lost — skip the cursor forward and flag an overflow.
//
// Returns false if !i2c_la_is_inited() or a capture is already running.
bool i2c_la_start(uint32_t sample_interval_us);

// Stop the running capture (abort the DMA). Safe to call when not running.
// i2c_la_total() stays readable afterward until the next start or deinit.
void i2c_la_stop(void);

// True iff a capture is currently running (i2c_la_start without a matching
// stop/deinit).
bool i2c_la_is_running(void);

// Total samples the DMA has written since the last i2c_la_start —
// monotonic, counts past I2C_LA_CAPTURE_BUFFER_BYTES (the buffer wraps).
// The live write offset within the ring is i2c_la_total() %
// I2C_LA_CAPTURE_BUFFER_BYTES. Returns 0 if no capture has produced
// samples yet, or after deinit.
uint32_t i2c_la_total(void);

// Pointer to the ring buffer base, valid until i2c_la_deinit(). Index it
// modulo I2C_LA_CAPTURE_BUFFER_BYTES against an i2c_la_total()-derived
// cursor (see i2c_la_start).
const uint8_t* i2c_la_buffer(void);

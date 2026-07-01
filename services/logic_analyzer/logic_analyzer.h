#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal/gpio.h"

// services/logic_analyzer — passive, protocol-agnostic digital logic
// analyzer (PIO+DMA streamer).
//
// This is a raw sampler: it snapshots GP0..GP7 (LA_CHANNEL_COUNT
// channels) verbatim and streams the bytes out with no interpretation.
// It knows nothing about I2C, UART, SPI, or any protocol — all decoding
// happens host-side in PulseView/sigrok (via services/sump_ols). The
// same capture therefore works for any digital signal you wire onto
// GP0..GP7; a "protocol" is just a documented pin mapping plus the
// decoder the operator picks in PulseView (e.g. I2C: GP0=SDA/GP1=SCL;
// UART: GP0=RX; SPI: GP0..GP3; GPIO: any).
//
// It never drives any pin — it only reads them, so it can observe an
// i2c_core transfer (loop back to back), a UART/SPI bus, or any
// third-party device on the scanner header without affecting bus levels.
//
// Sampling path (see docs/I2C_LA_DMA_TIMER_PLAN.md):
//   - PIO (pio1/SM2) runs a 2-instruction loop (`in pins, 8` / `push`)
//     that snapshots GP0..GP7 into its RX FIFO every clk_div cycles; a
//     DMA channel drains that FIFO into the buffer with no CPU per
//     sample — same ring pattern as emfi_capture.c. An earlier design
//     paced a DMA channel reading straight from SIO->GPIO_IN via a DMA
//     timer; that doesn't work on RP2040 (SIO isn't reachable by the DMA
//     bus master), hence PIO instead.
//   - Continuous streaming, not a fixed one-shot: la_start arms a
//     ring-mode DMA that runs in the background and wraps the buffer
//     forever. The caller drains it as it fills (read cursor vs.
//     la_total()), so a capture can exceed the buffer size — the
//     buffer is a sliding window, not the capture limit. la_stop
//     ends it. Nothing blocks: the CPU is free between drains.
//
// Host-testable: links against tests/hal_fake (gpio_fake.c + dma_fake.c).
// The fake DMA does not copy bytes, so tests verify the channel/timer are
// configured correctly (ring_bits, dreq, src == hal_gpio_in_register) and
// that la_total() reflects the simulated DMA progress; on-wire
// reconstruction is a hardware test.

// Number of captured channels — one byte per sample covers GP0..GP7.
// This is the single source of truth for the channel count; sump_ols
// reports it to PulseView as NUM_PROBES_LONG.
#define LA_CHANNEL_COUNT 8u

// Ring buffer size. One byte per sample; with a fixed sample interval the
// timestamp of each sample is implicit (index * sample_interval_us), so no
// separate timestamp storage is needed. In continuous mode this is the
// sliding-window size, not the total capture length — the DMA wraps it and
// the caller must drain faster than LA_CAPTURE_BUFFER_BYTES samples
// accumulate or it lags. 32KB (up from 8KB) quadruples the drain-time
// budget at the fastest supported sample rate (1us/sample: 32768us = 33ms
// before the DMA laps the cursor, vs. 8ms at the old size) — the real
// overflow risk at high sample rates is USB CDC drain throughput, not
// total capture length (the streaming loop in cmd_la/sump_ols.c::do_arm
// handles n_samples well beyond this size already).
// 32768 (2^15) is also the hardware ceiling: the RP2040 DMA ring-wrap
// field is 4 bits wide (0-15), so this is the largest ring the silicon
// can express — see LA_CAPTURE_RING_BITS in logic_analyzer.c. RP2040 has
// 256KB SRAM total; this and emfi_capture's 8KB ring are the only large
// static buffers, leaving plenty of headroom. MUST stay a power of two
// (ring-mode wrap masks the low bits).
#define LA_CAPTURE_BUFFER_BYTES 32768u

// Per-sample byte: a raw snapshot of GPIO_IN[7:0] (GP0..GP7) copied
// verbatim by DMA. The level of channel `pin` in a sample is bit
// (1u << pin) — the bit position is the pin number, not a fixed layout.
// Per-channel extraction happens host-side (PulseView), not during
// capture, so the signals of interest must live in GP0..GP7 for this
// byte-wide capture.
#define LA_SAMPLE_BIT(pin) ((uint8_t)(1u << (pin)))

// Initialize the passive sampler: configure GP0..GP7 as plain inputs
// with no pull resistors touched — the bus already has its own pulls
// (from i2c_core or an external device), and the sampler must not affect
// bus levels.
//
// Returns false if already inited or if PIO/DMA resources can't be
// claimed.
bool la_init(void);

// Tear down: release GP0..GP7 back to plain GPIO inputs with pulls
// disabled. Safe to call repeatedly.
void la_deinit(void);

// True iff la_init succeeded since the last deinit.
bool la_is_inited(void);

// Start continuous capture: arm a ring-mode DMA that copies GPIO_IN[7:0]
// into the buffer every `sample_interval_us`, wrapping forever, with no
// CPU per sample. Non-blocking — returns immediately while the DMA runs in
// the background. `sample_interval_us == 0` is treated as 1.
//
// The caller drains the ring as it fills: read la_buffer()[k %
// LA_CAPTURE_BUFFER_BYTES] for k in [read_cursor, la_total()), and
// advance the cursor. If (la_total() - read_cursor) ever exceeds
// LA_CAPTURE_BUFFER_BYTES the DMA has lapped the cursor and those
// samples are lost — skip the cursor forward and flag an overflow.
//
// Returns false if !la_is_inited() or a capture is already running.
bool la_start(uint32_t sample_interval_us);

// Stop the running capture (abort the DMA). Safe to call when not running.
// la_total() stays readable afterward until the next start or deinit.
void la_stop(void);

// True iff a capture is currently running (la_start without a matching
// stop/deinit).
bool la_is_running(void);

// Total samples the DMA has written since the last la_start —
// monotonic, counts past LA_CAPTURE_BUFFER_BYTES (the buffer wraps).
// The live write offset within the ring is la_total() %
// LA_CAPTURE_BUFFER_BYTES. Returns 0 if no capture has produced
// samples yet, or after deinit.
uint32_t la_total(void);

// Pointer to the ring buffer base, valid until la_deinit(). Index it
// modulo LA_CAPTURE_BUFFER_BYTES against a la_total()-derived cursor
// (see la_start).
const uint8_t* la_buffer(void);

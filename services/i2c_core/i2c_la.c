/*
 * services/i2c_core/i2c_la.c — passive I2C logic analyzer (DMA streamer).
 *
 * See i2c_la.h for the design rationale. Samples are paced by a DMA timer
 * (no CPU per sample, no PIO) straight from SIO->GPIO_IN into a ring
 * buffer, mirroring the lifecycle of
 * services/glitch_engine/emfi/emfi_capture.c (ADC + DMA in ring mode) but
 * with a GPIO source and a DMA-timer DREQ instead of the ADC. The transfer
 * runs continuously and the caller streams it out as it fills, so a
 * capture is not bounded by the buffer size. Everything routes through
 * hal/dma + hal/gpio + hal/time so it links against tests/hal_fake on the
 * host.
 */

#include "i2c_la.h"

#include "hal/dma.h"
#include "hal/gpio.h"
#include "hal/time.h"

// RP2040 default system clock. The DMA timer paces its DREQ at
// sys_clk * (num/den) Hz; with num fixed at 1 the sample rate is
// sys_clk / den. The firmware never reprograms clk_sys (no set_sys_clock
// call anywhere), so 125 MHz is exact here — §5 of the DMA-timer adenda
// (docs/I2C_LA_DMA_TIMER_PLAN.md) validates the real interval against this
// assumption on hardware.
#define I2C_LA_SYS_CLK_HZ 125000000u

// Ring-mode wrap width: 2^13 = 8192 = I2C_LA_CAPTURE_BUFFER_BYTES. Must
// track the buffer size exactly (the build asserts it below).
#define I2C_LA_CAPTURE_RING_BITS 13u

// transfer_count for a never-ending transfer: the DMA decrements from this
// on every sample, so (0xFFFFFFFF - remaining) is the running total.
#define I2C_LA_FOREVER 0xFFFFFFFFu

static uint8_t s_sda;
static uint8_t s_scl;
static bool s_inited           = false;
static bool s_running          = false;
static hal_dma_channel_t s_dma = -1;
static hal_dma_timer_t s_timer = -1;

// RP2040 DMA ring mode wraps the WRITE address on the low RING_BITS bits,
// so the buffer MUST be aligned to its own size or wraps overrun into
// adjacent memory — empirically wedges the TinyUSB stack after a few
// seconds of continuous capture (see emfi_capture.c:12-19).
static uint8_t s_buffer[I2C_LA_CAPTURE_BUFFER_BYTES]
    __attribute__((aligned(I2C_LA_CAPTURE_BUFFER_BYTES)));

_Static_assert((1u << I2C_LA_CAPTURE_RING_BITS) == I2C_LA_CAPTURE_BUFFER_BYTES,
               "ring bits must match buffer size");

// Denominator for hal_dma_timer_set_fraction at a fixed numerator of 1, so
// the paced rate is sys_clk / den ≈ one sample every sample_interval_us.
// num=1 guarantees the SDK's num<=den constraint; den is clamped into the
// uint16_t range the timer accepts (≈ up to 524 µs/sample at 125 MHz).
static uint16_t divisor_for(uint32_t sample_interval_us) {
    if (sample_interval_us == 0u)
        sample_interval_us = 1u;
    uint64_t den = ((uint64_t)I2C_LA_SYS_CLK_HZ * sample_interval_us) / 1000000u;
    if (den < 1u)
        den = 1u;
    if (den > 0xFFFFu)
        den = 0xFFFFu;
    return (uint16_t)den;
}

bool i2c_la_init(uint8_t sda, uint8_t scl) {
    if (s_inited)
        return false;
    if (sda >= 30u || scl >= 30u || sda == scl)
        return false;

    s_dma = hal_dma_claim_unused();
    if (s_dma < 0)
        return false;
    s_timer = hal_dma_timer_claim();
    if (s_timer < 0) {
        hal_dma_unclaim(s_dma);
        s_dma = -1;
        return false;
    }

    s_sda = sda;
    s_scl = scl;
    hal_gpio_init(s_sda, HAL_GPIO_DIR_IN);
    hal_gpio_init(s_scl, HAL_GPIO_DIR_IN);

    s_running = false;
    s_inited  = true;
    return true;
}

void i2c_la_deinit(void) {
    if (!s_inited)
        return;
    hal_dma_abort(s_dma);
    hal_dma_unclaim(s_dma);
    hal_dma_timer_unclaim(s_timer);
    s_dma     = -1;
    s_timer   = -1;
    s_running = false;

    hal_gpio_init(s_sda, HAL_GPIO_DIR_IN);
    hal_gpio_init(s_scl, HAL_GPIO_DIR_IN);
    hal_gpio_set_pulls(s_sda, false, false);
    hal_gpio_set_pulls(s_scl, false, false);
    s_inited = false;
}

bool i2c_la_is_inited(void) {
    return s_inited;
}

bool i2c_la_start(uint32_t sample_interval_us) {
    if (!s_inited || s_running)
        return false;

    hal_dma_timer_set_fraction(s_timer, 1u, divisor_for(sample_interval_us));

    // Ring mode, byte-wide: each sample is the low byte of GPIO_IN
    // (GP0..GP7). src is the raw GPIO_IN register, not auto-incremented;
    // dst walks s_buffer and wraps every I2C_LA_CAPTURE_BUFFER_BYTES. The
    // transfer never ends on its own (I2C_LA_FOREVER) — i2c_la_stop aborts
    // it, and i2c_la_total() derives progress from the decrementing count.
    hal_dma_cfg_t cfg = {
        .size            = HAL_DMA_SIZE_8,
        .read_increment  = false,
        .write_increment = true,
        .ring_bits       = I2C_LA_CAPTURE_RING_BITS,
        .ring_on_write   = true,
        .dreq            = hal_dma_timer_dreq(s_timer),
    };
    hal_dma_configure(s_dma, &cfg, s_buffer, hal_gpio_in_register(), I2C_LA_FOREVER, true);

    s_running = true;
    return true;
}

void i2c_la_stop(void) {
    if (!s_running)
        return;
    hal_dma_abort(s_dma);
    s_running = false;
}

bool i2c_la_is_running(void) {
    return s_running;
}

uint32_t i2c_la_total(void) {
    if (!s_inited || s_dma < 0)
        return 0u;
    uint32_t remaining = hal_dma_transfer_count(s_dma);
    if (remaining == I2C_LA_FOREVER)
        return 0u;
    return I2C_LA_FOREVER - remaining;
}

const uint8_t* i2c_la_buffer(void) {
    return s_buffer;
}

/*
 * services/i2c_core/i2c_la.c — passive I2C logic analyzer (PIO+DMA streamer).
 *
 * See i2c_la.h for the design rationale. Samples are paced by a PIO state
 * machine (pio1/SM2) running a 2-instruction loop (`in pins, 8` / `push`)
 * that snapshots GP0..GP7 into its RX FIFO every clk_div cycles; a DMA
 * channel drains that FIFO into a ring buffer with no CPU per sample,
 * mirroring the lifecycle of services/glitch_engine/emfi/emfi_capture.c
 * (ADC + DMA in ring mode) but with a PIO RX FIFO source instead of the
 * ADC.
 *
 * This supersedes an earlier design that paced a DMA channel reading
 * straight from SIO->GPIO_IN via a DMA timer. That never worked on real
 * hardware: SIO is wired to each CPU core's private single-cycle I/O
 * port, not to the AHB-Lite bus fabric the DMA controller's bus master
 * sits on, so DMA cannot address SIO_BASE at all — zero samples ever
 * arrived, independent of pacing (see docs/I2C_LA_DMA_TIMER_PLAN.md
 * postmortem). The PIO RX FIFO is on the bus fabric and has a real DREQ,
 * which is why this works. `IN PINS` reads pad inputs directly without
 * needing hal_pio_gpio_init (that's only required for PIO-driven
 * outputs), so i2c_la still never drives SDA/SCL.
 *
 * The transfer runs continuously and the caller streams it out as it
 * fills, so a capture is not bounded by the buffer size. Everything
 * routes through hal/dma + hal/pio + hal/gpio so it links against
 * tests/hal_fake on the host.
 */

#include "i2c_la.h"

#include "hal/dma.h"
#include "hal/gpio.h"
#include "hal/pio.h"

// RP2040 default system clock. The firmware never reprograms clk_sys (no
// set_sys_clock call anywhere), so 125 MHz is exact here.
#define I2C_LA_SYS_CLK_HZ 125000000u

// pio1/SM2 — first free slot in the range swd_phy.c reserves for future
// UART/SPI work (SM1/SM3 stay reserved; SM0 is swd_phy). pio0 is fully
// claimed by the EMFI/crowbar glitch engines (frozen at F4-1/F5-2).
#define I2C_LA_PIO_INSTANCE 1u
#define I2C_LA_PIO_SM       2u

// PIO program: sample GP0..GP7 into the RX FIFO every 2 SM-clock cycles,
// forever. in_shift_right=false (left shift) means a fresh `in pins, 8`
// on a freshly-pushed (zeroed) ISR deposits the byte at ISR[7:0] — the
// low byte of the 32-bit FIFO word, which is exactly what an 8-bit-wide
// DMA read from the FIFO register picks up on this little-endian core.
// `push` opcode (0x8020) reused verbatim from services/swd_core/swd_phy.c.
#define I2C_LA_PIO_PROG_LEN 2u
static const uint16_t s_pio_prog[I2C_LA_PIO_PROG_LEN] = {
    /* 0 */ 0x4008u, // in pins, 8         <-- wrap_target
    /* 1 */ 0x8020u, // push               <-- wrap
};
#define I2C_LA_PIO_WRAP_TARGET       0u
#define I2C_LA_PIO_WRAP_END          1u
#define I2C_LA_PIO_CYCLES_PER_SAMPLE 2u

// Ring-mode wrap width: 2^15 = 32768 = I2C_LA_CAPTURE_BUFFER_BYTES. Must
// track the buffer size exactly (the build asserts it below). 15 is the
// hardware ceiling: the RP2040 DMA CTRL_TRIG.RING_SIZE field is only 4
// bits wide (values 0-15, i.e. up to a 32768-byte ring — see pico-sdk's
// channel_config_set_ring() doc comment). 16 silently overflows that
// field: size_bits << RING_SIZE_LSB lands exactly on the adjacent
// RING_SEL bit, so RING_SIZE reads back as 0, which the SDK defines as
// "no wrapping" — the DMA then free-runs linearly past the end of
// s_buffer[] into adjacent RAM instead of wrapping, which is what
// wedges the TinyUSB stack (see the comment on s_buffer below).
#define I2C_LA_CAPTURE_RING_BITS 15u

// transfer_count for a never-ending transfer: the DMA decrements from this
// on every sample, so (0xFFFFFFFF - remaining) is the running total.
#define I2C_LA_FOREVER 0xFFFFFFFFu

static uint8_t s_sda;
static uint8_t s_scl;
static bool s_inited           = false;
static bool s_running          = false;
static hal_dma_channel_t s_dma = -1;
static hal_pio_inst_t* s_pio   = NULL;
static uint32_t s_pio_off      = 0u;

// RP2040 DMA ring mode wraps the WRITE address on the low RING_BITS bits,
// so the buffer MUST be aligned to its own size or wraps overrun into
// adjacent memory — empirically wedges the TinyUSB stack after a few
// seconds of continuous capture (see emfi_capture.c:12-19).
static uint8_t s_buffer[I2C_LA_CAPTURE_BUFFER_BYTES]
    __attribute__((aligned(I2C_LA_CAPTURE_BUFFER_BYTES)));

_Static_assert((1u << I2C_LA_CAPTURE_RING_BITS) == I2C_LA_CAPTURE_BUFFER_BYTES,
               "ring bits must match buffer size");

// PIO SM clock divider for a given sample interval: 2 SM-clock cycles
// (in + push) per sample, so clk_div = sample_interval_us * sys_clk_hz /
// 1e6 / cycles_per_sample. Integer divider only (no fractional part),
// same rounding caveat as the DMA-timer design this replaces — clamped
// into the 16-bit range the SM clock divider accepts.
static uint32_t divisor_for(uint32_t sample_interval_us) {
    if (sample_interval_us == 0u)
        sample_interval_us = 1u;
    uint64_t div = ((uint64_t)I2C_LA_SYS_CLK_HZ * sample_interval_us) /
                   (1000000u * I2C_LA_PIO_CYCLES_PER_SAMPLE);
    if (div < 1u)
        div = 1u;
    if (div > 0xFFFFu)
        div = 0xFFFFu;
    return (uint32_t)div;
}

bool i2c_la_init(uint8_t sda, uint8_t scl) {
    if (s_inited)
        return false;
    if (sda >= 30u || scl >= 30u || sda == scl)
        return false;

    s_dma = hal_dma_claim_unused();
    if (s_dma < 0)
        return false;

    s_pio = hal_pio_instance(I2C_LA_PIO_INSTANCE);
    if (!s_pio || !hal_pio_claim_sm(s_pio, I2C_LA_PIO_SM)) {
        hal_dma_unclaim(s_dma);
        s_dma = -1;
        return false;
    }
    hal_pio_program_t prog = {
        .instructions = s_pio_prog, .length = I2C_LA_PIO_PROG_LEN, .origin = -1};
    if (!hal_pio_add_program(s_pio, &prog, &s_pio_off)) {
        hal_pio_unclaim_sm(s_pio, I2C_LA_PIO_SM);
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
    s_dma = -1;

    hal_pio_sm_set_enabled(s_pio, I2C_LA_PIO_SM, false);
    hal_pio_program_t prog = {
        .instructions = s_pio_prog, .length = I2C_LA_PIO_PROG_LEN, .origin = -1};
    hal_pio_remove_program(s_pio, &prog, s_pio_off);
    hal_pio_unclaim_sm(s_pio, I2C_LA_PIO_SM);
    s_pio     = NULL;
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

    // in_pin_base=0/in_pin_count=8 covers GP0..GP7 byte-wide, same range
    // I2C_LA_SAMPLE_BIT() assumes. No sideset, no out/set pins — i2c_la
    // never drives SDA/SCL.
    hal_pio_sm_cfg_t cfg = {
        .in_pin_base    = 0u,
        .in_pin_count   = 8u,
        .wrap_target    = I2C_LA_PIO_WRAP_TARGET,
        .wrap_end       = I2C_LA_PIO_WRAP_END,
        .in_shift_right = false,
        .clk_div        = (float)divisor_for(sample_interval_us),
    };
    hal_pio_sm_configure(s_pio, I2C_LA_PIO_SM, s_pio_off, &cfg);
    hal_pio_sm_clear_fifos(s_pio, I2C_LA_PIO_SM);
    hal_pio_sm_set_enabled(s_pio, I2C_LA_PIO_SM, true);

    // Ring mode, byte-wide: each sample is the low byte of the PIO RX
    // FIFO word (GP0..GP7, see I2C_LA_PIO program comment above). src is
    // the FIFO register, not auto-incremented; dst walks s_buffer and
    // wraps every I2C_LA_CAPTURE_BUFFER_BYTES. The transfer never ends on
    // its own (I2C_LA_FOREVER) — i2c_la_stop aborts it, and i2c_la_total()
    // derives progress from the decrementing count.
    hal_dma_cfg_t dma_cfg = {
        .size            = HAL_DMA_SIZE_8,
        .read_increment  = false,
        .write_increment = true,
        .ring_bits       = I2C_LA_CAPTURE_RING_BITS,
        .ring_on_write   = true,
        .dreq            = (hal_dma_dreq_t)hal_pio_sm_rx_dreq(s_pio, I2C_LA_PIO_SM),
    };
    hal_dma_configure(s_dma, &dma_cfg, s_buffer, hal_pio_sm_rxfifo_register(s_pio, I2C_LA_PIO_SM),
                      I2C_LA_FOREVER, true);

    s_running = true;
    return true;
}

void i2c_la_stop(void) {
    if (!s_running)
        return;
    hal_dma_abort(s_dma);
    hal_pio_sm_set_enabled(s_pio, I2C_LA_PIO_SM, false);
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

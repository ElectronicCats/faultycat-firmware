#pragma once

#include <stdbool.h>
#include <stdint.h>

// HAL — DMA
//
// Portable access to the RP2040 DMA controller. Minimal surface for
// the ADC capture ring used by services/glitch_engine/emfi/ and
// (later) services/glitch_engine/crowbar/. The RP2040 has 12 channels;
// claim_unused walks the pool, nothing else is abstracted.

typedef int hal_dma_channel_t; // -1 == invalid

typedef enum {
    HAL_DMA_SIZE_8  = 0,
    HAL_DMA_SIZE_16 = 1,
    HAL_DMA_SIZE_32 = 2,
} hal_dma_xfer_size_t;

typedef enum {
    HAL_DMA_DREQ_PIO0_0 = 0x00,
    HAL_DMA_DREQ_PIO0_1 = 0x01,
    HAL_DMA_DREQ_PIO0_2 = 0x02,
    HAL_DMA_DREQ_PIO0_3 = 0x03,
    HAL_DMA_DREQ_PIO1_0 = 0x08,
    HAL_DMA_DREQ_PIO1_1 = 0x09,
    HAL_DMA_DREQ_PIO1_2 = 0x0A,
    HAL_DMA_DREQ_PIO1_3 = 0x0B,
    HAL_DMA_DREQ_ADC    = 0x24,
    HAL_DMA_DREQ_TIMER0 = 0x3B,
    HAL_DMA_DREQ_TIMER1 = 0x3C,
    HAL_DMA_DREQ_TIMER2 = 0x3D,
    HAL_DMA_DREQ_TIMER3 = 0x3E,
    HAL_DMA_DREQ_FORCE  = 0x3F,
} hal_dma_dreq_t;

typedef struct {
    hal_dma_xfer_size_t size;
    bool read_increment;
    bool write_increment;
    // Ring-mode: when non-zero, write/read wraps every
    // (1 << ring_bits) bytes. 0 disables ring mode.
    uint32_t ring_bits;
    bool ring_on_write; // true = ring dst, false = ring src
    hal_dma_dreq_t dreq;
} hal_dma_cfg_t;

// Claim the first unused channel. Returns -1 if all 12 are busy.
hal_dma_channel_t hal_dma_claim_unused(void);
void hal_dma_unclaim(hal_dma_channel_t ch);

// Program the channel. If `start` is true the transfer begins
// immediately; otherwise call hal_dma_start explicitly.
void hal_dma_configure(hal_dma_channel_t ch, const hal_dma_cfg_t* cfg, volatile void* dst,
                       const volatile void* src, uint32_t transfer_count, bool start);

void hal_dma_start(hal_dma_channel_t ch);
void hal_dma_abort(hal_dma_channel_t ch);
bool hal_dma_is_busy(hal_dma_channel_t ch);
uint32_t hal_dma_transfer_count(hal_dma_channel_t ch);

// DMA timers — paced DREQ sources. The RP2040 has 4, shared across the
// whole board. Each timer raises its DREQ at sys_clk * (num/den) Hz,
// letting a DMA channel transfer at a fixed rate with no CPU work (used
// by the I2C logic analyzer to sample GPIO_IN). claim_unused walks the
// pool; nothing else is abstracted.

typedef int hal_dma_timer_t; // -1 == invalid

// Claim one of the 4 DMA timers. Returns -1 if all 4 are busy.
hal_dma_timer_t hal_dma_timer_claim(void);
void            hal_dma_timer_unclaim(hal_dma_timer_t t);

// Pace the timer's DREQ at sys_clk * (numerator/denominator) Hz.
// Caller computes numerator/denominator for the desired sample rate
// (sys_clk is normally 125 MHz on RP2040). numerator must be <= denominator.
void hal_dma_timer_set_fraction(hal_dma_timer_t t, uint16_t numerator,
                                uint16_t denominator);

// Returns the HAL_DMA_DREQ_TIMERn value for a claimed timer, to plug
// straight into hal_dma_cfg_t.dreq.
hal_dma_dreq_t hal_dma_timer_dreq(hal_dma_timer_t t);

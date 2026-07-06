#pragma once

#include <stdbool.h>
#include <stdint.h>

// services/glitch_engine/pio_glitch_prog — shared PIO program compiler for
// the EMFI and crowbar glitch engines (services/glitch_engine/emfi/emfi_pio.c
// and services/glitch_engine/crowbar/crowbar_pio.c). Both compile the same
// delay/trigger/pulse/IRQ layout:
//
// [pindir]  SET PINDIRS, 1             ; crowbar only — see with_pindir_setup
// [0]       PULL block                 ; pull delay_ticks into OSR
// [1]       OUT Y, 32                  ; Y = delay_ticks
// [2..N]    trigger block (0..3 instrs); compiled from the trig value
// [N+1]     JMP Y-- self               ; delay loop
// [N+2]     PULL block                 ; pull pulse_width_ticks
// [N+3]     OUT Y, 32                  ; Y = pulse_width_ticks
// [N+4]     SET pins=1                 ; rising edge of pulse
// [N+5]     JMP Y-- self               ; hold high
// [N+6]     SET pins=0                 ; falling edge
// [N+7]     IRQ n                      ; signal done to CPU (n differs so
//                                      ; the two engines don't share a flag)
//
// Header-only (static inline): each caller's translation unit gets its own
// copy with internal linkage, so emfi_pio.c and crowbar_pio.c can each embed
// it without a shared .c file colliding when both static libraries link into
// the same firmware image.

// ---------------------------------------------------------------------------
// PIO instruction encodings (RP2040 datasheet §3.4)
// ---------------------------------------------------------------------------
#define PIO_OP_PULL_BLOCK      0x80A0u
#define PIO_OP_OUT_Y_32        0x6040u
#define PIO_OP_WAIT_0_PIN0     0x2020u
#define PIO_OP_WAIT_1_PIN0     0x20A0u
#define PIO_OP_SET_PIN_HIGH    0xE001u
#define PIO_OP_SET_PIN_LOW     0xE000u
#define PIO_OP_SET_PINDIRS_OUT 0xE081u
#define PIO_OP_IRQ(n)          ((uint16_t)(0xC000u | ((n) & 0x7u)))

static inline uint16_t pio_glitch_op_jmp_y_dec(uint8_t addr) {
    return (uint16_t)(0x0080u | (addr & 0x1Fu));
}

// Trigger polarity values shared by emfi_trig_t and crowbar_trig_t — both
// enums use this same numbering (see either header's doc-comment).
#define PIO_GLITCH_TRIG_IMMEDIATE     0u
#define PIO_GLITCH_TRIG_EXT_RISING    1u
#define PIO_GLITCH_TRIG_EXT_FALLING   2u
#define PIO_GLITCH_TRIG_EXT_PULSE_POS 3u
#define PIO_GLITCH_TRIG_EXT_PULSE_NEG 4u

// Appends the WAIT sequence for `trig` to `out` and returns how many
// instructions were written (0..3).
static inline uint32_t pio_glitch_compile_trigger_block(uint16_t* out, uint8_t trig) {
    switch (trig) {
        case PIO_GLITCH_TRIG_IMMEDIATE:
            return 0;
        case PIO_GLITCH_TRIG_EXT_RISING:
            out[0] = PIO_OP_WAIT_0_PIN0;
            out[1] = PIO_OP_WAIT_1_PIN0;
            return 2;
        case PIO_GLITCH_TRIG_EXT_FALLING:
            out[0] = PIO_OP_WAIT_1_PIN0;
            out[1] = PIO_OP_WAIT_0_PIN0;
            return 2;
        case PIO_GLITCH_TRIG_EXT_PULSE_POS:
            out[0] = PIO_OP_WAIT_0_PIN0;
            out[1] = PIO_OP_WAIT_1_PIN0;
            out[2] = PIO_OP_WAIT_0_PIN0;
            return 3;
        case PIO_GLITCH_TRIG_EXT_PULSE_NEG:
            // Inverse of PULSE_POS: HIGH-idle, source dips LOW and comes
            // back HIGH. Trailing rising edge is the trigger event. See
            // emfi_trig_t / crowbar_trig_t doc-comments for the full
            // per-option contract.
            out[0] = PIO_OP_WAIT_1_PIN0;
            out[1] = PIO_OP_WAIT_0_PIN0;
            out[2] = PIO_OP_WAIT_1_PIN0;
            return 3;
        default:
            return 0;
    }
}

// Builds the shared delay/trigger/pulse/IRQ program into `prog` and returns
// its length. `irq_op` is the encoded IRQ instruction to raise on
// completion. `with_pindir_setup` prepends a SET PINDIRS,1 instruction
// (crowbar's gate pin needs it embedded in-program — see crowbar_pio.c's
// build_program comment for why; EMFI's GP14 doesn't).
static inline uint32_t pio_glitch_build_program(uint16_t* prog, uint8_t trig, uint16_t irq_op,
                                                 bool with_pindir_setup) {
    uint32_t len = 0;
    if (with_pindir_setup)
        prog[len++] = PIO_OP_SET_PINDIRS_OUT;
    prog[len++] = PIO_OP_PULL_BLOCK;
    prog[len++] = PIO_OP_OUT_Y_32;
    len += pio_glitch_compile_trigger_block(&prog[len], trig);
    uint8_t delay_loop_addr = (uint8_t)len;
    prog[len++]             = pio_glitch_op_jmp_y_dec(delay_loop_addr);
    prog[len++]             = PIO_OP_PULL_BLOCK;
    prog[len++]             = PIO_OP_OUT_Y_32;
    prog[len++]             = PIO_OP_SET_PIN_HIGH;
    uint8_t hold_loop_addr  = (uint8_t)len;
    prog[len++]             = pio_glitch_op_jmp_y_dec(hold_loop_addr);
    prog[len++]             = PIO_OP_SET_PIN_LOW;
    prog[len++]             = irq_op;
    return len;
}

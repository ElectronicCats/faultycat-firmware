#include "emfi_pio.h"

#include "board_v2.h"
#include "emfi_pulse.h"
#include "hal/pio.h"
#include "pio_glitch_prog.h"

// ---------------------------------------------------------------------------
// Clock — 125 MHz / 1.0 = 125 MHz PIO clock. 1 instr = 8 ns nominal,
// so 1 µs = 125 ticks.
// ---------------------------------------------------------------------------
#define EMFI_PIO_CLK_DIV      1.0f
#define EMFI_PIO_TICKS_PER_US 125u

// Program layout: the shared delay/trigger/pulse/IRQ compiler in
// pio_glitch_prog.h, with no leading pindir setup and IRQ 0 (crowbar
// uses IRQ 1 — see that engine's pio_glitch_build_program call).

static uint16_t s_prog[24];
static uint32_t s_prog_len;
static hal_pio_inst_t* s_pio = NULL;
static uint32_t s_sm         = 0;
static uint32_t s_offset     = 0;
static bool s_claimed        = false;
static bool s_loaded         = false;

static uint32_t s_delay_ticks;
static uint32_t s_width_ticks;
static uint32_t s_repeat = 1u;

static void build_program(const emfi_pio_params_t* p) {
    s_prog_len = pio_glitch_build_program(s_prog, p->trigger, PIO_OP_IRQ(0), false);
}

bool emfi_pio_init(void) {
    s_pio = hal_pio_instance(0);
    if (!s_pio)
        return false;
    if (!hal_pio_claim_sm(s_pio, 0))
        return false;
    s_sm      = 0;
    s_claimed = true;
    s_loaded  = false;
    return true;
}

void emfi_pio_deinit(void) {
    if (!s_claimed)
        return;
    // Detach the driver BEFORE unclaiming the SM — otherwise the
    // driver would stay marked attached while the PIO path is gone,
    // leaving the CPU fire path permanently refused.
    emfi_pulse_detach_pio();
    if (s_loaded) {
        hal_pio_program_t prog = {.instructions = s_prog, .length = s_prog_len, .origin = -1};
        hal_pio_remove_program(s_pio, &prog, s_offset);
        s_loaded = false;
    }
    hal_pio_sm_set_enabled(s_pio, s_sm, false);
    hal_pio_unclaim_sm(s_pio, s_sm);
    s_claimed = false;
    s_pio     = NULL;
}

bool emfi_pio_load(const emfi_pio_params_t* p) {
    if (!s_claimed || !p)
        return false;
    if (p->width_us < EMFI_PULSE_MIN_WIDTH_US || p->width_us > EMFI_PULSE_MAX_WIDTH_US)
        return false;

    build_program(p);
    if (s_loaded) {
        hal_pio_program_t old = {.instructions = s_prog, .length = s_prog_len, .origin = -1};
        hal_pio_remove_program(s_pio, &old, s_offset);
        s_loaded = false;
    }

    hal_pio_program_t prog = {.instructions = s_prog, .length = s_prog_len, .origin = -1};
    if (!hal_pio_add_program(s_pio, &prog, &s_offset))
        return false;
    s_loaded = true;

    // Attach GP14 to PIO and bind GP8 as in-pin for trigger waits.
    hal_pio_gpio_init(s_pio, BOARD_GP_HV_PULSE);
    hal_pio_set_consecutive_pindirs(s_pio, s_sm, BOARD_GP_HV_PULSE, 1, true);

    if (!emfi_pulse_attach_pio(s_pio, s_sm)) {
        hal_pio_remove_program(s_pio, &prog, s_offset);
        s_loaded = false;
        return false;
    }

    hal_pio_sm_cfg_t cfg = {
        .set_pin_base      = BOARD_GP_HV_PULSE,
        .set_pin_count     = 1,
        .sideset_pin_base  = 0,
        .sideset_pin_count = 0,
        .in_pin_base       = BOARD_GP_EXT_TRIGGER,
        .in_pin_count      = 1,
        .clk_div           = EMFI_PIO_CLK_DIV,
    };
    hal_pio_sm_configure(s_pio, s_sm, s_offset, &cfg);
    hal_pio_sm_clear_fifos(s_pio, s_sm);
    hal_pio_irq_clear(s_pio, 0);

    // Cache tick counts; pushed to TX FIFO in emfi_pio_start so the
    // program reads delay first, then width, in that order.
    s_delay_ticks = p->delay_us * EMFI_PIO_TICKS_PER_US;
    s_width_ticks = p->width_us * EMFI_PIO_TICKS_PER_US;
    s_repeat      = (p->repeat > 0u) ? p->repeat : 1u;
    return true;
}

bool emfi_pio_start(void) {
    if (!s_claimed || !s_loaded)
        return false;
    // FIFO order must match pio_glitch_build_program: [repeat-1, delay, width].
    hal_pio_sm_put_blocking(s_pio, s_sm, s_repeat - 1u);
    hal_pio_sm_put_blocking(s_pio, s_sm, s_delay_ticks);
    hal_pio_sm_put_blocking(s_pio, s_sm, s_width_ticks);
    hal_pio_sm_set_enabled(s_pio, s_sm, true);
    return true;
}

bool emfi_pio_is_done(void) {
    if (!s_claimed)
        return false;
    return hal_pio_irq_get(s_pio, 0);
}

void emfi_pio_clear_done(void) {
    if (!s_claimed)
        return;
    hal_pio_irq_clear(s_pio, 0);
    hal_pio_sm_set_enabled(s_pio, s_sm, false);
}

uint32_t emfi_pio_ticks_per_us(void) {
    return EMFI_PIO_TICKS_PER_US;
}

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// services/sump_ols — SUMP/OLS ("Openbench Logic Sniffer") protocol,
// streaming flavour, layered on top of services/i2c_core/i2c_la.c.
//
// Why this exists: PulseView/sigrok ship the "ols" driver out of the
// box, which speaks the classic SUMP serial protocol with no
// configuration needed beyond picking a serial port — see
// docs/I2C_LA_DMA_TIMER_PLAN.md §6. Implementing this subset lets
// PulseView drive `i2c_la` directly instead of needing a bespoke
// sigrok driver or Python client.
//
// Mode-switch shape is identical to buspirate_compat / flashrom_serprog
// (apps/faultycat_fw/main.c): a text command on the CDC2 shell
// (`i2c la sump enter <sda> <scl>`) calls i2c_la_init() and flips
// SHELL_MODE_SUMP; every subsequent byte is fed here instead of the
// line parser. Like serprog, SUMP has no in-band escape byte back to
// text mode, so leaving the mode relies on host DTR-drop detection
// (main.c's disconnect handler) calling sump_ols_on_exit().
//
// Protocol values below were confirmed against the live sigrok
// `libsigrok/src/hardware/openbench-logic-sniffer/{protocol.h,
// protocol.c,api.c}` sources (CLOCK_RATE = 100 MHz, ID reply "1ALS",
// metadata token bytes, little-endian long-command arguments,
// CMD_CAPTURE_SIZE's readcount/4 packing) rather than guessed from the
// datasheet — getting any of these wrong fails PulseView's scan
// silently (no error, just "device not found" or garbled samples).
//
// Scope: enough of SUMP to let PulseView scan the device and run a
// capture with an optional stage-0 level trigger — the subset
// enumerated in I2C_LA_DMA_TIMER_PLAN.md §6 plus basic triggering (see
// docs/UART_LA_TRIGGER_IMPLEMENTATION_PLAN.md). Stage-0 trigger
// mask/value (CMD_SET_TRIGGER_MASK/VALUE, 0xC0/0xC1) are parsed and
// used to delay the capture start until the first matching sample;
// stage-0 config (0xC2) and every higher stage (0xC4..0xCE) are still
// accepted-and-ignored (bytes consumed so the stream stays in sync).
// With no trigger configured PulseView sends no SET_TRIGGER_* at all,
// so trigger_mask stays 0 (match-anything) and ARM starts immediately,
// exactly as before.

typedef struct {
    void (*write_byte)(uint8_t b, void* user);
    void (*yield)(void* user);   // called periodically while streaming
                                 // ARM's capture data, to keep tud_task
                                 // / other CDC pumps alive.
    void (*on_exit)(void* user); // SUMP has no in-band exit byte; the
                                 // main loop's DTR-drop disconnect
                                 // handler calls this directly.
    void* user;
} sump_ols_callbacks_t;

void sump_ols_init(const sump_ols_callbacks_t* cb);
void sump_ols_feed_byte(uint8_t b);

// Mirrors flashrom_serprog_get_state()/buspirate_compat — exposed for
// tests and for main.c's disconnect handler to decide whether a
// capture needs aborting.
typedef enum {
    SUMP_OLS_IDLE               = 0,
    SUMP_OLS_SET_DIVIDER_B0     = 1,
    SUMP_OLS_SET_DIVIDER_B1     = 2,
    SUMP_OLS_SET_DIVIDER_B2     = 3,
    SUMP_OLS_SET_DIVIDER_B3     = 4,
    SUMP_OLS_CAPTURE_SIZE_B0    = 5,
    SUMP_OLS_CAPTURE_SIZE_B1    = 6,
    SUMP_OLS_CAPTURE_SIZE_B2    = 7,
    SUMP_OLS_CAPTURE_SIZE_B3    = 8,
    SUMP_OLS_SWALLOW_LONG_ARG_1 = 9,  // generic 4-byte-argument sink for
    SUMP_OLS_SWALLOW_LONG_ARG_2 = 10, // long commands we accept but
    SUMP_OLS_SWALLOW_LONG_ARG_3 = 11, // ignore (SET_FLAGS, stage-0
    SUMP_OLS_SWALLOW_LONG_ARG_4 = 12, // trigger config, higher stages).

    // Stage-0 basic trigger mask/value (CMD_SET_TRIGGER_MASK 0xC0 /
    // CMD_SET_TRIGGER_VALUE 0xC1), 4-byte little-endian argument like
    // SET_DIVIDER — only the low byte is kept (channels 0-7).
    SUMP_OLS_TRIGGER_MASK0_B0  = 13,
    SUMP_OLS_TRIGGER_MASK0_B1  = 14,
    SUMP_OLS_TRIGGER_MASK0_B2  = 15,
    SUMP_OLS_TRIGGER_MASK0_B3  = 16,
    SUMP_OLS_TRIGGER_VALUE0_B0 = 17,
    SUMP_OLS_TRIGGER_VALUE0_B1 = 18,
    SUMP_OLS_TRIGGER_VALUE0_B2 = 19,
    SUMP_OLS_TRIGGER_VALUE0_B3 = 20,
} sump_ols_state_t;

sump_ols_state_t sump_ols_get_state(void);

// True iff an ARM-triggered capture is currently streaming (i.e. a
// sump_ols_feed_byte(CMD_ARM_BASIC_TRIGGER) call hasn't returned yet).
// Always false from outside feed_byte itself — capture runs to
// completion synchronously inside the ARM call, same as cmd_i2c_la.
// Exposed for tests only.
bool sump_ols_is_capturing(void);

// Stage-0 trigger mask/value last parsed from CMD_SET_TRIGGER_MASK/VALUE
// (low byte only — channels 0-7). mask == 0 means "match anything", the
// zero-initialized default that makes ARM start immediately. Exposed for
// tests only.
uint8_t sump_ols_trigger_mask(void);
uint8_t sump_ols_trigger_value(void);

// ID reply sent for CMD_ID (0x02) — the literal 4 bytes the sigrok
// `ols` driver's scan() matches with strncmp(buf, "1ALS", 4) (it also
// accepts "1SLO" — the same bytes byte-swapped — but "1ALS" is the
// canonical OLS reply and what real boards send).
#define SUMP_OLS_ID_REPLY "1ALS"

// Device name reported in metadata token 0x01 (DEVICE_NAME, ASCII +
// NUL). Exposed for tests/docs.
#define SUMP_OLS_DEVICE_NAME "FaultyCat I2C LA"

// Maximum sample count the SUMP CMD_CAPTURE_SIZE encoding can express:
// readcount is a uint16 (max 65536), stored in units of 4 samples →
// 65536 * 4 = 262144.  Reported in CMD_METADATA SAMPLE_MEMORY_BYTES so
// PulseView lets the user select up to this value.  The ring buffer
// (I2C_LA_CAPTURE_BUFFER_BYTES) is just the DMA sliding window; the
// streaming loop in do_arm() handles n_samples well beyond that size as
// long as USB can drain it fast enough.
#define SUMP_OLS_MAX_SAMPLES 262144u

// Fallback sample interval (microseconds) used by CMD_ARM if the host
// never sent CMD_SET_DIVIDER first — shouldn't happen in practice
// (sigrok's ols_prepare_acquisition always sends it), but keeps ARM
// well-defined regardless of call order.
#define SUMP_OLS_DEFAULT_INTERVAL_US 2u

// Fallback total sample count for CMD_ARM if the host never sent
// CMD_CAPTURE_SIZE — see SUMP_OLS_DEFAULT_INTERVAL_US.
#define SUMP_OLS_DEFAULT_N_SAMPLES 2048u

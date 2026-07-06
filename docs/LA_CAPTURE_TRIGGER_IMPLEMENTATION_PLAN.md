# `la capture` — Trigger + Pretrigger Support (raw streaming path)

**Status: implemented.** Shipped in `b12ae8d` (`la_wait_for_trigger` /
`la_apply_pretrigger` in `services/logic_analyzer/`, `trig=<ch>[:<ms>]`
parsing in `cmd_la`, tests in `tests/test_logic_analyzer.c`); documented
as current behavior in `docs/LOGIC_ANALYZER.md`'s "Raw-path trigger"
section. The sections below are the original design plan, kept as-is for
context. Sibling to `UART_LA_TRIGGER_IMPLEMENTATION_PLAN.md`
(the SUMP/PulseView trigger). That plan explicitly scoped trigger support to
the PulseView/SUMP path and listed the raw `la`/`uart la` shell command as
**out of scope**, reasoning that "trigger only makes sense through
PulseView's SUMP path." This plan revisits that call for the renamed
`la capture` command (`faultycmd la capture`, backed by `cmd_la` in
`main.c`) — see "Why this is back in scope" below.

---

## Problem this fixes

`cmd_la` (`main.c:843-877`) starts counting its `n`-sample window the
instant the command runs — no trigger, no pretrigger:

```c
la_start(us);
shell_printf("LA: OK ... n=%lu ...");
la_stream_and_finish(n, binary);   // starts counting toward n right now
```

Two consequences, both observed against a real UART target with
`faultycmd la capture --decode uart --rx 5`:

1. **Nothing captured when the bus is idle.** If no transmission happens
   inside the fixed `n * us` window, the capture is all idle-high and
   `decode_uart` finds no frames.
2. **Garbage + `framing_error` when something is captured.** The window
   opens at an arbitrary point in the stream, so the first `1->0`
   transition `decode_uart` finds may be a mid-byte data bit, not a real
   start bit preceded by idle. Bit sampling desyncs from there and cascades
   wrong bytes/framing errors through the rest of the capture.

This is the exact failure mode `SUMP_OLS_PRETRIGGER_MIN` was added to
`sump_ols.c::do_arm()` to prevent (see `sump_ols.h:151-161`) — but that fix
only covers captures taken through PulseView (`la sump enter`).

A host-side mitigation (idle-guard in `decode_uart`, `uart_decode.py`) has
already been applied for problem 2 — it requires ~1.5 bit-times of stable
idle-high before accepting a start-bit candidate, which rejects most false
locks. It is **not a full fix**: it can't do anything about problem 1
(nothing to decode if nothing was captured), and it can still misframe a
false start that lands in the first bit-time of the buffer, where no prior
samples exist to check. That residual gap is what this plan closes,
deterministically, in firmware.

### Why this is back in scope

`UART_LA_TRIGGER_IMPLEMENTATION_PLAN.md` excluded the raw command because,
at the time, its role was "quick fixed-length hexdump," and PulseView
already had a working trigger. Since then `la capture`'s own docstring
(`cli.py:1177-1181`) made unbounded continuous streaming from the ring
buffer its whole reason to exist — precisely the mode SUMP *can't* offer
(`SUMP_OLS_MAX_SAMPLES` caps it at 16384 samples). Operators who need a
long/unbounded capture *and* reliable sync currently have no working
option. This plan gives the raw path its own trigger, independent of SUMP.

---

## Non-goals — do not touch the working PulseView path

- **No changes to `sump_ols.c` / `sump_ols.h`.** That flow is in
  production and out of scope for any regression risk here.
- **No changes to `la_start` / `la_stop` / `la_total` / `la_buffer`'s
  existing contract** (`logic_analyzer.h`). They stay exactly as-is;
  this plan only adds a new function alongside them.
- **No change to the wire format** of `la`'s streamed samples (still 1
  byte/sample, hex or binary via `bin`). Only *when* the `n`-sample window
  starts changes — everything downstream of that (`la_stream_and_finish`)
  is reused unmodified.
- `la sump enter` / `cmd_la_sump_enter` in `main.c` is untouched.

---

## Design

Mirror `do_arm()`'s proven two-phase pattern (wait-for-trigger, then
pretrigger + stream), but as a **new, independently testable primitive**
rather than a copy inlined into `main.c` — `cmd_la` lives in `apps/`,
which today has no host-test coverage (unlike `services/*`, see "Tests"
below), so the logic that actually needs verifying belongs in
`services/logic_analyzer/`, not in the shell-command glue.

### New primitive — `services/logic_analyzer/logic_analyzer.{h,c}`

```c
// Blocking wait for a sample matching (sample & mask) == (value & mask).
// mask == 0 matches immediately (degenerates to today's behavior).
// Calls yield(user) periodically while waiting so USB/CDC stays serviced.
// Returns the ring cursor of the matching sample, or LA_NO_TRIGGER_MATCH
// if timeout_ms elapses first (0 = wait forever, only for host tests).
//
// Read-only with respect to la_start/la_total/la_buffer — this is the
// same wait-then-scan loop do_arm() hand-rolls today in sump_ols.c,
// lifted out so it has one implementation and one set of tests instead
// of two. sump_ols.c is NOT changed to call this in this plan (see
// "Out of scope") — it's added for cmd_la's use, and stays available for
// sump_ols.c to adopt later, at zero risk to callers who don't opt in.
uint32_t la_wait_for_trigger(uint8_t mask, uint8_t value,
                              void (*yield)(void* user), void* user,
                              uint32_t timeout_ms);
#define LA_NO_TRIGGER_MATCH UINT32_MAX
```

Internals are `do_arm()`'s phase 1, verbatim (poll `la_total()`/
`la_buffer()`, handle ring-lap-before-match the same way, call `yield`
every iteration). Add a `LA_PRETRIGGER_MIN` constant (mirrors
`SUMP_OLS_PRETRIGGER_MIN` = 32 samples, capped at `n/8`) and a small helper
to compute the pretrigger-adjusted start cursor — `do_arm()`'s phase 2 math
(`sump_ols.c:214-243`), also lifted out so both sides share one correct
implementation of "how far back can we actually serve from the ring."

### `cmd_la` (`main.c`) — new optional trigger argument

Extend the grammar to `la <us> <n> [bin] [trig=<ch>]`, fully backward
compatible — omitting `trig=` is byte-for-byte today's behavior (`mask=0`
matches immediately, same as no trigger configured in SUMP):

```c
static void cmd_la(int argc, char** argv) {
    ...
    uint8_t trigger_mask = 0, trigger_value = 0;
    if (argc >= 5 && !strncmp(argv[argc - 1], "trig=", 5)) {
        int ch = atoi(argv[argc - 1] + 5);
        if (ch < 0 || ch >= LA_CHANNEL_COUNT) {
            shell_print("LA: ERR trig channel out of range (0-7)\n");
            return;
        }
        trigger_mask  = LA_SAMPLE_BIT(ch);
        trigger_value = 0; // wait for the channel to go LOW (idle-high assumed)
    }
    ...
    la_start(us);
    uint32_t cursor = la_wait_for_trigger(trigger_mask, trigger_value,
                                           yield_cb, NULL, LA_TRIGGER_TIMEOUT_MS);
    if (cursor == LA_NO_TRIGGER_MATCH) {
        shell_print("LA: ERR trigger_timeout\n");
        la_stop(); la_deinit(); swd_bus_release(...);
        return;
    }
    cursor = la_apply_pretrigger(cursor, n); // clamps into LA_PRETRIGGER_MIN..n/8
    shell_printf("LA: OK capture ch=GP0..GP7 stream n=%lu interval_us=%lu\n", n, us);
    la_stream_and_finish_from(cursor, n, binary); // la_stream_and_finish, cursor param added
}
```

`la_stream_and_finish` needs one signature change — take a starting
cursor instead of hardcoding `0` — but its body (drain loop, overflow
handling, `TRUNC`/`OVERFLOW` replies) is otherwise untouched.

`LA_TRIGGER_TIMEOUT_MS`: a bounded wait is required here (unlike SUMP,
whose wait is bounded by the *user* closing PulseView / sending
`CMD_FORCE_EXIT`) — a shell command must hand control back to the CLI, or
an unresponsive target hangs the whole CDC shell, not just this command.
Default proposal: reuse the host's `--timeout-s` value by having the CLI
pass it as an extra arg (`trig=<ch>:<timeout_ms>`), falling back to a
firmware-side constant (e.g. 5000 ms) if omitted.

---

## Host / CLI changes

- **`ScannerClient.la()`** (`scanner.py:685-716`): add
  `trigger_ch: int | None = None` and `trigger_timeout_ms: int | None =
  None`, appended to the command string as `trig=<ch>[:<timeout_ms>]`.
  The existing `" ERR "` check in `_capture_la_stream` already turns any
  `LA: ERR trigger_timeout` reply into a `ScannerError` with no code
  change — just needs a test confirming that reply shape is covered
  (today's `_LA_OK_RE` only matches the `OK` summary line; the `ERR`
  branch is generic and already handles unknown `ERR ...` text verbatim).
- **`la capture` CLI** (`cli.py:1161-1303`): add `--trigger/--no-trigger`
  (default **on** when `--decode uart` is given — that's the motivating
  case — default **off** otherwise, so plain GPIO/raw captures keep
  today's immediate-start behavior) and `--trigger-timeout-s` (default:
  same value as `--timeout-s`). When triggering under `--decode uart`,
  the trigger channel defaults to `--rx`; add a `--trigger-ch` override
  for `--decode none`/custom wiring.
- **`decode_uart` idle-guard** (already applied, `uart_decode.py`): no
  further change needed. It remains a defense-in-depth layer for noise/
  glitches once a real trigger + pretrigger removes the *systematic*
  cause of misframing.

---

## Tests

- **`services/logic_analyzer/`** gains host-test coverage for the first
  time on the trigger path (it currently has none — `test_logic_analyzer.c`
  only covers `la_start`/`la_total` bookkeeping against the fake DMA).
  Add, in `tests/test_logic_analyzer.c`, following `test_sump_ols.c`'s
  established pattern of poking bytes into the ring via the same
  cast-away-`const` helper:
  - `test_wait_for_trigger_mask_zero_matches_immediately` — regression
    guard for the backward-compat claim (`trig=` omitted ≡ mask 0).
  - `test_wait_for_trigger_matches_first_low_sample` — preload idle
    samples then a low sample on the target channel, confirm the
    returned cursor points at it.
  - `test_wait_for_trigger_times_out` — preload only idle samples, small
    `timeout_ms`, confirm `LA_NO_TRIGGER_MATCH` and that `yield` was
    called at least once (busy-loop/USB-starvation guard, same intent as
    `test_arm_polls_yield_while_waiting_for_trigger` in the SUMP plan).
  - `test_apply_pretrigger_floors_and_caps` — cursor near 0 (nothing to
    serve, clamps to 0) and a small `n` (pretrigger capped at `n/8`, not
    the full `LA_PRETRIGGER_MIN`) — same edge cases `do_arm()` already
    handles inline, now covered once instead of never.
- **`cmd_la` itself** (`apps/faultycat_fw/main.c`) has no host-test
  harness today (commands under `apps/` aren't linked against
  `tests/hal_fake`, unlike `services/*`) — this plan doesn't add one, to
  stay consistent with the rest of the file. It's covered by the manual
  bring-up check below instead, same as every other shell command in
  `main.c`.

---

## Manual verification (required — `cmd_la` has no automated coverage)

Since `apps/faultycat_fw/main.c` isn't part of the host-test suite, verify
on real hardware before considering this done:

1. Wire a UART target to the scanner header (RX on the channel used for
   `--trigger-ch`/`--rx`), transmission stopped.
2. `faultycmd la capture --decode uart --rx <ch> --trigger` — confirm the
   command blocks (not an empty/garbage capture) until transmission
   starts, then decodes cleanly with zero `framing_error` rows.
3. Start the target's transmission, then run the same command — confirm
   it captures the very first burst, correctly framed.
4. `--trigger-timeout-s` with transmission left off — confirm a clean
   `LA: ERR trigger_timeout` → `ScannerError`, not a CLI hang.
5. Re-run `faultycmd la pulseview` against the same target — confirm it
   behaves identically to before this change (this is the regression
   check for the "non-goals" section: SUMP/PulseView must be unaffected).
6. `faultycmd la capture` with no `--trigger` (plain raw/GPIO capture,
   `--decode none`) — confirm byte-for-byte the same immediate-start
   behavior as before this change.

---

## Out of scope (explicitly deferred)

- **Migrating `sump_ols.c::do_arm()` to call `la_wait_for_trigger`.** The
  new primitive is built to make that possible later (dedup two copies of
  the same loop), but doing it now adds risk to the working PulseView path
  for zero user-facing benefit — a pure refactor, not a fix. Flag as a
  follow-up once `la_wait_for_trigger` has real hardware mileage under
  `cmd_la`.
- **Mask/value generality beyond "one channel goes low."** UART's use case
  only needs a single-channel level trigger. The primitive's signature
  already takes a general mask/value pair (same as SUMP's), so multi-bit
  triggers work if a future protocol needs them — no redesign required,
  just not exercised by `cmd_la`'s `trig=<ch>` shorthand yet.
- **Edge-vs-level trigger distinction.** Same simplification SUMP's plan
  used: on an idle-high signal, "first sample where the bit reads 0"
  behaves like an edge trigger for this use case without needing real
  edge-detection state.

---

## Files touched

| File | Change |
|---|---|
| `faultycat-firmware/services/logic_analyzer/logic_analyzer.h` | New `la_wait_for_trigger`, `la_apply_pretrigger`, `LA_PRETRIGGER_MIN`, `LA_NO_TRIGGER_MATCH` declarations |
| `faultycat-firmware/services/logic_analyzer/logic_analyzer.c` | New primitive implementation (lifted from `do_arm()`'s phase 1/2 logic) |
| `faultycat-firmware/apps/faultycat_fw/main.c` | `cmd_la` parses `trig=<ch>[:<timeout_ms>]`; `la_stream_and_finish` takes a starting cursor; `shell_help()` usage line updated |
| `faultycat-firmware/tests/test_logic_analyzer.c` | New trigger/pretrigger tests |
| `faultycat-TUI/src/faultycmd/protocols/scanner.py` | `ScannerClient.la()` gains `trigger_ch`/`trigger_timeout_ms` |
| `faultycat-TUI/src/faultycmd/core/cli.py` | `la capture` gains `--trigger/--no-trigger`, `--trigger-ch`, `--trigger-timeout-s` |
| `faultycat-firmware/docs/LOGIC_ANALYZER.md` | Document the new trigger option on the raw path |

**Not touched:** `sump_ols.c`, `sump_ols.h`, `tests/test_sump_ols.c`,
`la_start`/`la_stop`/`la_total`/`la_buffer` signatures, the SUMP wire
protocol, `faultycmd la pulseview`.

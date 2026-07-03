# Logic Analyzer — protocol-agnostic digital sampler

**Status: current.** Supersedes the I2C-specific framing of the original
`docs/I2C_LOGIC_ANALYZER_PLAN.md` / `docs/I2C_LA_DMA_TIMER_PLAN.md` (the
sampling-path postmortems there still apply; only the naming and the
"I2C-only" framing changed).

## What it is

A passive, **protocol-agnostic** digital logic analyzer. The firmware
snapshots the 8-channel bank **GP0..GP7** (`LA_CHANNEL_COUNT`) verbatim
and streams the raw bytes out — it never interprets them and never drives
a pin. **All protocol decoding happens host-side in PulseView/sigrok.**

The same capture therefore works for *any* digital signal wired onto
GP0..GP7. A "protocol" is nothing more than:

1. a documented pin mapping (which channel carries which signal), and
2. the decoder the operator selects in PulseView.

Adding a new protocol needs **zero firmware changes** — just wire it up
and pick the decoder.

## Architecture

```
  GP0..GP7 ──▶ services/logic_analyzer   (PIO1/SM2 snapshot loop + DMA ring)
                     │  raw GPIO_IN[7:0], one byte per sample
                     ▼
               services/sump_ols          (SUMP/OLS transport over CDC2)
                     │  classic "ols" serial protocol
                     ▼
               PulseView / sigrok         (protocol decoders: I2C/UART/SPI/…)
```

- **`services/logic_analyzer/logic_analyzer.{c,h}`** — the sampler.
  `la_init()` claims PIO1/SM2 + a DMA channel and configures GP0..GP7 as
  inputs; `la_start(interval_us)` arms a continuous ring-mode DMA;
  `la_total()` / `la_buffer()` let a caller drain the sliding window. See
  the header for the full contract and `docs/I2C_LA_DMA_TIMER_PLAN.md` for
  why the pacing is PIO+DMA (SIO->GPIO_IN is unreachable by the DMA bus
  master on RP2040).
- **`services/sump_ols/`** — a SUMP/OLS protocol subset so PulseView's
  stock `ols` driver can drive captures with no bespoke client. Reports
  `NUM_PROBES_LONG = LA_CHANNEL_COUNT` and device name `"FaultyCat LA"`.
  Optional stage-0 level trigger (see
  `docs/UART_LA_TRIGGER_IMPLEMENTATION_PLAN.md`) — the trigger matches raw
  channel bytes, so it too is protocol-agnostic. PulseView's
  **pre-trigger capture ratio** is honored (SUMP `delaycount`): that
  fraction of the window is served from before the trigger. Serial
  decoders need some of it — a window starting dead on the trigger
  sample gives the UART decoder no idle line/falling edge to sync on —
  so triggered captures always include at least
  `SUMP_OLS_PRETRIGGER_MIN` (32) samples of history even at the default
  0% ratio; set ~10% when you want more context before the trigger.

  SUMP captures are **capture-then-dump and lossless**: ARM lets the DMA
  fill the ring with the requested samples, stops it, then streams the
  frozen buffer — USB throughput can no longer cause silent mid-capture
  sample drops (ring laps), which used to corrupt protocol decode at
  sample rates above what USB FS CDC drains. The dump is sent **newest
  sample first** (the SUMP reverse-order convention; sigrok's ols driver
  un-reverses it — verified on hardware, oldest-first shows a
  time-mirrored capture). The tradeoff is a bounded
  capture: at most `SUMP_OLS_MAX_SAMPLES` (16384, half the ring —
  advertised to PulseView as the sample memory) per capture; for longer
  windows, lower the sample rate. The `la <us> <n> [bin]` shell command
  keeps the unbounded best-effort streaming mode for raw captures.

## Shell commands (CDC2 text shell)

| Command | Purpose |
|---|---|
| `la <us> <n> [bin] [trig=<ch>[:<timeout_ms>]]` | Raw capture of GP0..GP7: `n` samples at `<us>` µs/sample, dumped as hex (or `bin` for raw bytes). |
| `la sump enter` | Enter SUMP/OLS mode for PulseView/sigrok (driver `ols`). Exits on the host sending `CMD_FORCE_EXIT` (0x0F) — see `services/sump_ols/sump_ols.c`. |

Both always capture the full GP0..GP7 bank. There are no per-protocol
commands — wiring + the host-side decoder is what makes it "an I2C
capture" or "a UART capture".

### Raw-path trigger (`trig=<ch>`)

Omitting `trig=` starts the `n`-sample window the instant the command
runs, same as always — fine for an already-toggling bus, but on an idle
line it can capture nothing, or lock onto a mid-byte transition instead
of a real start bit (see
`LA_CAPTURE_TRIGGER_IMPLEMENTATION_PLAN.md`). `trig=<ch>` blocks the
window's start until channel `ch` goes low (idle-high assumed), then
backs the window up by up to `LA_PRETRIGGER_MIN` (32) samples — capped
at `n/8` for small captures — so a decoder has idle line to sync on,
mirroring `SUMP_OLS_PRETRIGGER_MIN`'s reasoning for the PulseView path.
The optional `:<timeout_ms>` bounds the wait (firmware default: 5000ms);
if it elapses first the command replies `LA: ERR trigger_timeout`
instead of hanging the shell.

From `faultycmd`: `la capture --decode uart` enables this automatically
(trigger channel defaults to `--rx`); pass `--trigger`/`--no-trigger`,
`--trigger-ch`, and `--trigger-timeout-s` to control it directly,
including for `--decode none`/custom wiring.

## Pin mapping per protocol (convention, not enforced)

| Protocol | Suggested channels |
|---|---|
| I2C | GP0 = SDA, GP1 = SCL |
| UART | GP0 = RX (add GP1 = TX if needed) |
| SPI | GP0 = CLK, GP1 = MOSI, GP2 = MISO, GP3 = CS |
| Plain GPIO | any of GP0..GP7 |

Any signal of interest must sit within GP0..GP7 for the byte-wide capture
to see it. In PulseView, assign the decoder's inputs to the matching
channel numbers.

## Extending

To support a new bus: document its pin mapping (above), wire it to
GP0..GP7, and use the matching PulseView decoder. No code changes. If a
future need requires more than 8 channels or arbitrary base pins, that is
a firmware change to `LA_CHANNEL_COUNT` / the PIO `in_pin_base` and the
SUMP `NUM_PROBES_LONG` metadata — deliberately out of scope today.

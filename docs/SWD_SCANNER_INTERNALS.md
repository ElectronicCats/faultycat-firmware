# FaultyCat v3 — SWD pinout scanner internals (F8-2)

This document walks through `scan swd`'s control flow end to end: the
shell entry point, the permutation engine, the SWD PHY/DP calls it
drives, and the false-positive guard. For the JTAG/I2C scan siblings
and the service stack diagram see [`JTAG_INTERNALS.md`](JTAG_INTERNALS.md);
for HW pinout caveats (TXS0108EPW) see [`HARDWARE_V2.md`](HARDWARE_V2.md);
for the bus mutex contract see [`MUTEX_INTERNALS.md`](MUTEX_INTERNALS.md).

## 1. There is no hardware multiplexer

FaultyCat v2.x has no mux chip on the scanner header. What exists is:

1. A **logical/software multiplexer**: a channel-index → GPIO lookup
   table (`s_ch_to_gpio` in `pinout_scanner.c`) that lets the scan
   algorithm reason in terms of "channel 0..7" instead of physical
   GPIO numbers.
2. A **TXS0108EPW level shifter** (hardware) between the RP2040 GPIO
   and the 10-pin scanner header (`Conn_01x10`). It adapts voltage and
   auto-senses direction — it does not select/switch channels.

An earlier design with `PIN_MUX0/1/2 = GP1/GP2/GP3` was dropped (see
[`PORTING.md`](PORTING.md)) — those GPIOs are just scanner CH1/CH2/CH3
and no HW mux exists. So "trying every header pin" is done purely
algorithmically: by permutation + PHY re-init, not by switching an
analog mux.

## 2. Files involved

| File | Role |
|---|---|
| [`apps/faultycat_fw/main.c`](../apps/faultycat_fw/main.c) | Shell/CLI: `scan swd` command, progress pump, cross-service bus-busy check |
| [`services/pinout_scanner/pinout_scanner.c`](../services/pinout_scanner/pinout_scanner.c) / `.h` | The state machine: iterates permutations, drives the SWD PHY per candidate |
| [`services/swd_core/swd_phy.c`](../services/swd_core/swd_phy.c) / `.h` | Physical layer: SWD bit-banging over PIO, runtime GPIO selection |
| [`services/swd_core/swd_dp.c`](../services/swd_core/swd_dp.c) / `.h` | Debug Port protocol: `swd_dp_bus_detect`, DPIDR read |
| [`services/swd_bus_lock/swd_bus_lock.c`](../services/swd_bus_lock/swd_bus_lock.c) / `.h` | Service-layer mutex so the scan doesn't race another SWD consumer |
| [`drivers/include/board_v2.h`](../drivers/include/board_v2.h) | Defines `BOARD_GP_SCANNER_CH0..CH7` → GP0..GP7 (the logical-mux map) |
| [`drivers/scanner_io/scanner_io.h`](../drivers/scanner_io/scanner_io.h) | Generic per-channel I/O driver (base abstraction for the header; not called directly by the SWD scan) |

## 3. Step-by-step control flow (`scan swd`)

**1. Shell entry** — `cmd_scan_swd()` in `main.c`
- Calls `shell_bus_busy("SCAN")` first: refuses to start if JTAG,
  direct-SWD shell, I2C, serprog, or UART passthrough currently own
  the scanner header.
- Calls `pinout_scan_swd(&r, scan_yield_progress)`.

**2. Bus mutex acquisition** — `pinout_scanner.c`
- `swd_bus_try_acquire(SWD_BUS_OWNER_SCANNER)` — fails fast
  (`PINOUT_SCAN_SWD_BUS_BUSY`) instead of blocking if another service
  (e.g. a glitch campaign's verify hook) already holds the SWD bus.

**3. The logical multiplexer table** — `pinout_scanner.c`
```c
static const uint8_t s_ch_to_gpio[8] = {CH0, CH1, ..., CH7}; // identity today
```
Indirection layer: today channel N maps to GP N, but a future board
rev could reorder physical routing without touching the algorithm.

**4. Permutation iterator P(8,2) = 56** — `pinout_perm_next()` in
`pinout_scanner.c`
- Generates every distinct ordered pair `(swclk, swdio)` from the 8
  channels, in lexicographic order.
- SWD needs exactly 2 pins (vs. JTAG's 4-tuple, P(8,4) = 1680).

**5. Per-candidate probe**
```
swd_phy_init(swclk, swdio, SWD_PHY_NRST_NONE)   // claim these 2 GPIO as the PHY
swd_phy_set_clk_khz(100)                         // conservative scan clock
swd_dp_bus_detect(&dpidr)                        // line-reset + attempt DPIDR read
```
- No `TARGETSEL` is issued — the goal is to find *any* coherent SWD
  DP, not a preselected multidrop target.
- A candidate is provisional if the ACK is OK and
  `swd_dp_dpidr_is_valid(dpidr)` passes.

**6. False-positive guard (F8-6)**
- Why it exists: the TXS0108EPW can inject noise on the header that
  briefly looks like a valid ACK + plausible DPIDR.
- Mitigation: re-read DPIDR `PINOUT_SCAN_CONFIRM_READS` (= 2) more
  times. Only if all reads agree is the match accepted as stable.

**7. Result**
- Stable match: store `swclk`, `swdio`, `dpidr`; `swd_phy_deinit()`;
  set `found = true`; break (first match wins).
- Otherwise: `swd_phy_deinit()` and continue to the next pair.
- The bus mutex is always released at the end (`swd_bus_release`),
  whether a match was found or the sweep was exhausted.

**8. Cooperative yield** — `scan_yield_progress()` in `main.c`
- Invoked via the `cb` callback before each candidate so the long
  scan doesn't starve TinyUSB or the EMFI/crowbar glitch engines.
- Prints progress on CDC2 every 100 iterations.

**9. Operator-facing output** — `main.c`
```
SCAN: swd MATCH swclk=GPx swdio=GPy
SCAN:   dpidr=0x... targetsel_compat=0x...
```

## 4. Related guards worth knowing

- `pinout_scan_jtag` and `pinout_scan_i2c` in the same file follow the
  identical shape (permutation iterator → init → probe → stability
  re-check → deinit), just with different tuple sizes (4 for JTAG, 2
  for I2C) and different bus primitives (`jtag_core`, `i2c_core`).
- SWD's PIO program emulates open-drain on SWDIO specifically to avoid
  contending with the TXS0108EPW's auto-direction logic — see the
  header comment in `services/swd_core/swd_phy.c` and
  [`JTAG_INTERNALS.md §"Why JTAG works through the TXS0108E and SWD
  doesn't"`](JTAG_INTERNALS.md).

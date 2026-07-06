# FaultyCat v3 — I2C pinout scanner internals (F8-2)

This document walks through `scan i2c`'s control flow and answers a
recurring question: **does the I2C scanner already use all 8 scanner
header pins, like `scan swd` does, or is it limited to a fixed
SDA/SCL pair?** Short answer: it already uses all 8 — no changes are
needed. For the SWD sibling (identical shape, 2-pin tuple) see
[`SWD_SCANNER_INTERNALS.md`](SWD_SCANNER_INTERNALS.md); for the JTAG
sibling (4-pin tuple) and the shared false-positive rationale see
[`JTAG_INTERNALS.md §3`](JTAG_INTERNALS.md); for HW pinout caveats
(TXS0108EPW) see [`HARDWARE_V2.md`](HARDWARE_V2.md).

## 1. No hardware I2C peripheral is used

The RP2040 has two hardware I2C blocks (`i2c0`, `i2c1`), but pico-sdk
ties each one's SDA/SCL to a fixed pair of GPIOs. The scanner needs to
try SDA/SCL on **any** of the 8 scanner-header channels, so
`services/i2c_core` does not touch the hardware peripheral at all —
it reimplements I2C as CPU bit-bang over `hal/gpio`, with open-drain
emulated the same way SWD emulates it over PIO:

```c
// services/i2c_core/i2c_core.h:9-13
// Per docs/I2C_SCANNER_PLAN.md: no hardware_i2c peripheral (pico-sdk
// ties SDA/SCL to fixed pin pairs; the scanner header needs SDA/SCL on
// any of GP0..GP7). Open-drain is emulated over hal/gpio:
//   - "drive low"   -> reconfigure pin as OUTPUT, put(false).
//   - "release/high"-> reconfigure pin as INPUT with internal pull-up.
```

(`I2C_SCANNER_PLAN.md` is referenced by this comment but isn't present
in the repo — the design decision survives only in code comments; this
file now documents it properly.)

`line_drive_low()` / `line_release()` in `i2c_core.c:25-33` reconfigure
the pin direction per bit instead of relying on a dedicated open-drain
GPIO mode, exactly analogous to SWD's open-drain PIO program — both
exist to avoid contending with the TXS0108EPW's auto-direction sniff.

## 2. Files involved

| File | Role |
|---|---|
| [`services/pinout_scanner/pinout_scanner.c`](../services/pinout_scanner/pinout_scanner.c) / `.h` | The state machine: iterates permutations, drives `i2c_core` per candidate |
| [`services/i2c_core/i2c_core.c`](../services/i2c_core/i2c_core.c) / `.h` | Bit-bang I2C over GPIO: `i2c_init`, `i2c_bus_scan`, `i2c_deinit` |
| [`drivers/include/board_v2.h`](../drivers/include/board_v2.h) | Defines `BOARD_GP_SCANNER_CH0..CH7` → GP0..GP7 (the logical-mux map, shared with SWD/JTAG) |

## 3. Step-by-step control flow (`pinout_scan_i2c`)

**1. Bus mutex acquisition** — `pinout_scanner.c:281`
- `swd_bus_try_acquire(SWD_BUS_OWNER_I2C_SCANNER)` — same service-layer
  mutex contract as `pinout_scan_swd`; fails fast with
  `PINOUT_SCAN_I2C_BUS_BUSY` instead of blocking.

**2. The logical multiplexer table** — `pinout_scanner.c:37-40`
```c
static const uint8_t s_ch_to_gpio[PINOUT_SCANNER_CHANNELS] = {
    BOARD_GP_SCANNER_CH0, BOARD_GP_SCANNER_CH1, BOARD_GP_SCANNER_CH2, BOARD_GP_SCANNER_CH3,
    BOARD_GP_SCANNER_CH4, BOARD_GP_SCANNER_CH5, BOARD_GP_SCANNER_CH6, BOARD_GP_SCANNER_CH7,
};
```
The **same table** SWD and JTAG use. There is no separate, smaller
I2C channel map.

**3. Permutation iterator P(8,2) = 56** — `pinout_scanner.c:286-294`
```c
pinout_perm_iter_t it;
pinout_perm_init(&it, PINOUT_SCANNER_I2C_PINS, PINOUT_SCANNER_CHANNELS);
// PINOUT_SCANNER_I2C_PINS = 2u, PINOUT_SCANNER_CHANNELS = 8u
```
Generates every distinct ordered pair `(sda, scl)` from all 8 channels
— identical shape to SWD's `(swclk, swdio)` sweep.

**4. Per-candidate probe** — `pinout_scanner.c:296-302`
```c
uint8_t sda = s_ch_to_gpio[it.indices[0]];
uint8_t scl = s_ch_to_gpio[it.indices[1]];
i2c_init(sda, scl, 100)            // claim these 2 GPIO as bit-bang SDA/SCL, 100 kHz
i2c_bus_scan(addrs, ...)           // probe every 7-bit address 0x00..0x7F (ish) for ACK
```

**5. False-positive guard (F8-6)** — `pinout_scanner.c:258-265, 304-314`
- Why it exists: same rationale as JTAG/SWD — the TXS0108EPW can
  inject a transient low on SDA during the ACK bit window that looks
  like a real device ACK.
- Mitigation: re-run `i2c_bus_scan` `PINOUT_SCAN_CONFIRM_READS` (= 2)
  more times. Only accepted if the **exact same address set** repeats
  every time (`i2c_addr_sets_match`, an exact-length + `memcmp`
  comparison) — random noise rarely ACKs the same addresses twice in a
  row.

**6. Result** — `pinout_scanner.c:315-326`
- Stable match: store `sda`, `scl`, the discovered address list;
  `i2c_deinit()`; set `found = true`; break (first match wins).
- Otherwise: `i2c_deinit()` and continue to the next pair.
- The bus mutex is always released at the end
  (`swd_bus_release(SWD_BUS_OWNER_I2C_SCANNER)`), match or not.

## 4. Conclusion

`scan i2c` already mirrors `scan swd`'s "no hardware mux, pure
permutation + re-init" model end to end:

- Same 8-channel `s_ch_to_gpio` table.
- Same `pinout_perm_next()` iterator (just `PINOUT_SCANNER_I2C_PINS = 2`
  instead of SWD's 2 — both total P(8,2) = 56 candidates).
- Same F8-6 stability-recheck guard against TXS0108EPW noise.
- Same bus-mutex-per-sweep contract via `swd_bus_lock`.

The only structural difference from SWD is at the bottom: SWD drives a
PIO-based PHY (`swd_phy`), I2C drives a CPU bit-bang core (`i2c_core`)
— because pico-sdk's hardware I2C peripheral can't be pointed at
arbitrary GPIO pairs, so bit-banging is the only way to keep SDA/SCL
fully assignable across all 8 header channels.

# Faulty Cat

Faulty Cat is an open-source fault-injection tool for hardware
security research. This repository holds the **firmware** that runs
on the board's RP2040: it exposes a USB composite device that a host
tool drives to fire **EMFI** (electromagnetic) and **Crowbar**
(voltage-glitch) faults at a target, plus an SWD pinout scanner and a
growing set of bus utilities (JTAG, I2C, UART passthrough, BusPirate
and flashrom `serprog` compatibility).

> ## Firmware v3 — rewritten from scratch
>
> This repository ships **firmware v3** for the existing FaultyCat
> v2.x hardware. It is a from-scratch rewrite of the original v2.x
> firmware, not an evolution of it — same board, new stack.

`THIS IS A HIGH-VOLTAGE TOOL. READ THE SAFETY SECTION BELOW BEFORE
ARMING THE BOARD — you can shock yourself on the exposed HV
capacitor.`

## Firmware for different versions

This firmware targets **FaultyCat HW v2.x only** (RP2040 main
microcontroller). v2.1 and v2.2 boards differ only in silkscreen
labels, not nets, so the same firmware build runs on both — see
[`docs/HARDWARE_V2.md`](docs/HARDWARE_V2.md) for the full pinout
reference. There is no v1.x or v3.x firmware variant in this
repository; if you are looking for the board's hardware design
files (KiCad, BOM, schematic), see the
[hardware repository](https://github.com/ElectronicCats/faultycat).

## Understanding what FaultyCat does

Fault injection on FaultyCat lives on two independent axes —
**technique** (how the fault is physically induced) and **mode**
(how many shots, with what parameter strategy):

| Technique | Mechanism | Hardware path |
|---|---|---|
| **EMFI** | Charges a ~250 V cap and dumps it into a coil placed on the target die; the transient B-field flips bits while the cap discharges. | `hv_charger` → 250 V cap → `emfi_pulse` → coil on SMA |
| **Crowbar** | A power MOSFET briefly shorts the target's VCC to GND, violating setup/hold on the target's flip-flops. | `crowbar_mosfet` (LP/HP gate, break-before-make) |

Each technique runs in either **Direct** (single shot) or
**Campaign** (parameter sweep) mode. Full details, including the
wire-protocol parameter matrix, live in
[`docs/GLITCHING.md`](docs/GLITCHING.md).

Beyond glitching, the firmware also exposes (over the scanner USB
CDC) an SWD pinout scanner, and — currently gated as work-in-progress
for the v3.0 release — direct SWD/JTAG verbs, a JTAG pinout scanner,
BusPirate/`serprog` compatibility modes, and a passive I2C logic
analyzer reachable via the SUMP/OLS protocol. See
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the current
status of every service.

## Building the firmware

The fastest way to build the firmware from source is the official
**Raspberry Pi Pico** extension for VS Code — it installs the
toolchain (cmake, ninja, arm-none-eabi-gcc) and the pico-sdk for
you, and runs the cmake configure/build steps from the editor.

1. Install the
   [Raspberry Pi Pico](https://marketplace.visualstudio.com/items?itemName=raspberry-pi.raspberry-pi-pico)
   extension from the VS Code Marketplace.
2. Clone this repository **with submodules**:
   `git clone --recursive git@github.com:ElectronicCats/faultycat-firmware.git`
   (or, after a plain clone, run
   `git submodule update --init --recursive`).
3. Open the cloned folder in VS Code, then run
   **Raspberry Pi Pico: Import Project** from the command palette
   (Ctrl+Shift+P) and point it at this folder.
4. Hit **Compile** in the status bar. The resulting `.uf2` lands
   under `build/.../apps/faultycat_fw/faultycat.uf2`.

Flash that `.uf2` using the steps in **Flashing the firmware** below.

## Flashing the firmware

1. Hold the **BOOTSEL** button on the FaultyCat while plugging in the
   USB cable. The board appears as a mass-storage volume named
   `RPI-RP2`.
2. Drag the `.uf2` onto that volume; the board reboots into the new
   firmware automatically and re-enumerates as `1209:fa17`.
3. Verify the flashed version via the CDC2 diag banner or the
   `version` shell command (`SHELL: VERSION 3.0.0.0`).

Pre-built UF2s for tagged releases are published under
[Releases](https://github.com/ElectronicCats/faultycat-firmware/releases).
See [`docs/RELEASES.md`](docs/RELEASES.md) for the full versioning
scheme and the host/firmware parity gate.

## Safety

This board drives a ~250 V flyback capacitor. Always keep the
plastic shield installed during any arm/fire sequence, never leave
the output SMA open while armed, and trust the firmware's 60-second
auto-disarm as a safety net, not a substitute for disarming
explicitly when done. Full operating procedure and the commit-level
safety gate for any change touching the HV path are in
[`docs/SAFETY.md`](docs/SAFETY.md).

## Documentation

| Doc | Covers |
|---|---|
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Layering, data flow, current build status per service. |
| [`docs/GLITCHING.md`](docs/GLITCHING.md) | EMFI / Crowbar techniques and Direct / Campaign modes. |
| [`docs/HARDWARE_V2.md`](docs/HARDWARE_V2.md) | GPIO → function map, scanner header, known HW quirks. |
| [`docs/SAFETY.md`](docs/SAFETY.md) | HV commit gate and physical operating procedure. |
| [`docs/RELEASES.md`](docs/RELEASES.md) | Versioning scheme, build/release process, flashing guide. |
| [`docs/JTAG_INTERNALS.md`](docs/JTAG_INTERNALS.md) | JTAG core, pinout scanner, BusPirate/serprog wire stack. |
| [`docs/SWD_SCANNER_INTERNALS.md`](docs/SWD_SCANNER_INTERNALS.md) | SWD pinout scanner internals. |
| [`docs/I2C_SCANNER_INTERNALS.md`](docs/I2C_SCANNER_INTERNALS.md) | I2C and SWD scanner internals. |
| [`docs/MUTEX_INTERNALS.md`](docs/MUTEX_INTERNALS.md) | SWD bus arbitration and campaign manager. |
| [`docs/PORTING.md`](docs/PORTING.md) | Porting notes from the legacy v2.x firmware. |

## Host tool

A host tool (`faultycmd`: CLI + TUI) drives this firmware's wire
protocols — EMFI/Crowbar direct and campaign fire, SWD scanning, and
the other CDC2 shell modes. It now lives in its own repository:

https://github.com/ElectronicCats/faultycat-TUI

The wire protocols it consumes (binary framing, opcodes, the CDC2
text-shell grammar) are defined and owned by this firmware repo.

## Hardware Repository

The board design (KiCad, BOM, mechanical files) lives in a separate
repository:

https://github.com/ElectronicCats/faultycat

## How to contribute <img src="https://electroniccats.com/wp-content/uploads/2018/01/fav.png" height="35"><img src="https://raw.githubusercontent.com/gist/ManulMax/2d20af60d709805c55fd784ca7cba4b9/raw/bcfeac7604f674ace63623106eb8bb8471d844a6/github.gif" height="30">
Contributions are welcome!

Please read the document [**Contribution Manual**](https://github.com/ElectronicCats/electroniccats-cla/blob/main/electroniccats-contribution-manual.md) which will show you how to contribute your changes to the project.

✨ Thanks to all our [contributors](https://github.com/ElectronicCats/faultycat-firmware/graphs/contributors)! ✨

See [**_Electronic Cats CLA_**](https://github.com/ElectronicCats/electroniccats-cla/blob/main/electroniccats-cla.md) for more information.

See the [**community code of conduct**](https://github.com/ElectronicCats/electroniccats-cla/blob/main/electroniccats-community-code-of-conduct.md) for a vision of the community we want to build and what we expect from it.

## License

This project FaultyCat is adapted from [ChipSHOUTER PicoEMP](https://github.com/newaetech/chipshouter-picoemp) by [Colin O'Flynn](https://github.com/colinoflynn) is licensed under CC BY-SA 3.0, "FaultyCat" contains modifications such as: porting the project to Kicad, modifying BOM and dimensions is licensed under CC BY-SA 3.0 by ElectronicCats.

Firmware released under a BSD 3-Clause license. See the [LICENSE](LICENSE) file and [LICENSES/](LICENSES/) for third-party attributions.

Electronic Cats invests time and resources in providing this open-source design. Please support Electronic Cats and open-source hardware by purchasing products from Electronic Cats!

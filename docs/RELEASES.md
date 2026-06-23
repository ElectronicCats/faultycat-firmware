# FaultyCat releases — versioning, build, distribution

This document describes how the FaultyCat firmware (RP2040 UF2) is
versioned, built, and shipped. It covers:

  - the version scheme used by the project (`vX.Y.Z.W`),
  - where the version lives in the source tree and how it propagates
    into the running firmware,
  - the wire-protocol fields a host tool can use to check parity with
    the flashed firmware,
  - how to cut a firmware release as a maintainer,
  - how to flash a specific release as an end user.

The host tool (CLI/TUI) that talks to this firmware now lives in a
separate repository and has its own versioning and release process;
it is not covered here beyond the wire-protocol version fields the
firmware exposes for any host tool to consume.

---

## Version scheme

Releases are tagged `vMAJOR.MINOR.PATCH.TWEAK`. Four numeric
segments, mandatory `v` prefix on the git tag. Pre-release variants
append a hyphen-suffix (`v3.0.0.0-rc1`).

| Segment | Bumped when… |
|---|---|
| MAJOR | Incompatible wire-protocol change (PING reply shape, opcode renumbering, CDC interface re-layout). Operators must re-flash and update any host tool together. |
| MINOR | New protocol command that the firmware gained; remains backwards-compatible at the *transport* level (an old host tool can still PING new firmware) but an Exact-match version gate on the host side would refuse the pairing anyway. |
| PATCH | Bug fix or polish that does not add or remove protocol surface. |
| TWEAK | Reserved for hotfixes-of-patch and for distinguishing identical-looking re-spins (e.g. a CI re-run that produced a different binary). |

An Exact-match policy (host tool must agree with firmware on **all
four** segments) is the recommended pairing check for any host tool
talking to this firmware — see "Host-side parity gate" below for the
fields the firmware exposes to support that check. The four-segment
shape gives a release the flexibility to ship a firmware-only patch
without bumping the user-visible MAJOR/MINOR.

The `v` prefix is preserved in everything user-facing (git tag, UF2
filename, GitHub release title, artifact name in CI).

## Where the version lives in the source tree

`CMakeLists.txt`'s `project(faultycat VERSION X.Y.Z.W ...)` is the
single source of truth on the firmware side. The release workflow
(`.github/workflows/release.yml`) and the local helper
(`scripts/get_build.sh`) rewrite this literal on every tag. The
top-level CMakeLists.txt then runs
`configure_file(cmake/firmware_version.h.in ...)` to generate
`firmware_version.h`:

```c
#define FW_VERSION_MAJOR  3u
#define FW_VERSION_MINOR  0u
#define FW_VERSION_PATCH  0u
#define FW_VERSION_TWEAK  0u
#define FW_VERSION_STR    "3.0.0.0"
#define FW_VERSION_BCD    /* packed 0xMMmp for USB bcdDevice */
```

The generated header is exposed as a CMake `INTERFACE` library
(`firmware_version`) that targets link against to pull the include
path in. Linking `firmware_version` is the only thing a new
firmware-side consumer has to do — no copy-pasted literals, no
sdkconfig-style indirection.

## How the firmware advertises its version to the host

The host has three ways to ask the firmware which version it is
running. Each is consumed by a different protocol client:

### PING reply (CDC0 EMFI / CDC1 Crowbar)

The PING handler in `services/host_proto/{emfi,crowbar}_proto/*.c`
embeds the four version bytes directly in the reply payload:

```
SOF  CMD|0x80  LEN_LE     payload (6 bytes)              CRC_LE
0xFA 0x81      0x06 0x00  'F' family MAJ MIN PATCH TWEAK CC CC
```

`family` is `'4'` for emfi_proto and `'5'` for crowbar_proto so a
host that ever sees a stray reply on the wrong CDC can detect the
mismatch. Firmware that predates this change replies with the legacy
4-byte payload (`'F', family, 0, 0`); the host detects the short
reply and surfaces it as a distinct `<pre-versioning>` mismatch
instead of treating the trailing zeros as `0.0.0.0`.

### CDC2 shell `version` command

The text shell on the scanner CDC accepts `version` and replies:

```
SHELL: VERSION 3.0.0.0
```

This is what the `ScannerClient` probes on connect — it never sends
the binary PING because the scanner CDC speaks the line-oriented
shell protocol.

### Diag banner

Every operator that connects to the scanner CDC (`picocom`, `screen`,
or the TUI's diag tail) sees a banner that includes the version
line:

```
========================================
FaultyCat v3 — F8 diag (composite scanner CDC + unified shell)
Firmware version: 3.0.0.0
========================================
```

Useful for an at-a-glance check without typing anything.

### USB bcdDevice

The USB device descriptor's `bcdDevice` field is computed from
`FW_VERSION_BCD` and packed as `0xMMmp` (MAJOR encoded as BCD in the
high byte, MINOR in the high nibble of the low byte, PATCH in the
low nibble). The TWEAK component does not fit in the 16-bit field
and is intentionally dropped from `bcdDevice`; it remains reachable
via PING / `version` / the banner. Operators reading `lsusb -v` see
the first three segments without re-opening any of the CDC ports.

## Host-side parity gate

Any host tool talking to this firmware should compare the firmware's
reported version (via PING, the `version` shell command, or
`bcdDevice`) against its own version and refuse to operate on a
mismatch — see "How the firmware advertises its version to the
host" above for the exact fields available. The recommended policy
is **Exact**: every one of the four segments should match, since a
mismatched pairing in field use is almost always a half-applied
upgrade and can produce silently-wrong results if the wire protocol
drifted. Implementation of that check (escape hatches for
development, error UX, etc.) is the host tool's responsibility and
lives in its own repository.

## Cutting a firmware release (maintainer guide)

The release workflow lives at `.github/workflows/release.yml`. It
fires on every tag matching `v*.*.*.*` (and `v*.*.*.*-*` for
pre-releases).

### `build` (ubuntu-24.04) — UF2

  1. Validates the tag shape and rejects malformed tags up-front so a
     typo doesn't ship a broken release.
  2. Installs the cross-compile toolchain (`gcc-arm-none-eabi`,
     `libnewlib-arm-none-eabi`, `libstdc++-arm-none-eabi-newlib`,
     `cmake`, `ninja`).
  3. Runs `./scripts/get_build.sh ${TAG}` which:
       - rewrites the version literal in the workflow checkout
         (NOT committed back to `main`),
       - re-runs `cmake --preset fw-release` so `firmware_version.h`
         is regenerated with `PROJECT_VERSION_*` matching the tag,
       - builds the firmware (`build/fw-release/apps/faultycat_fw/faultycat.uf2`),
       - copies the UF2 to
         `build/apps/faultycat_fw/faultycat_${TAG}.uf2`.
  4. Uploads `build/apps/faultycat_fw/` as artifact
     `faultycat-release-${TAG}`.

### `release` (ubuntu-24.04) — publish draft

Downloads the build artifact to `./release/`, then publishes a
**draft** GitHub Release with the UF2 attached. `generateReleaseNotes:
true` pulls the changelog from PRs and commits since the previous
tag; an additional "Highlights" section in the body lists the
filtered `feat/fix/perf/refactor/ci` commits since the previous
non-suffix tag.

The release is created as a draft so the maintainer reviews it, edits
the notes if needed, and clicks publish manually.

### To cut `v3.0.1.0`

```bash
git checkout main
# Optional: bump the literal locally for repo consistency — not
# required, the workflow re-bumps it defensively during the build.
sed -i -E 's/^(\s*VERSION\s+)[0-9.]+/\13.0.1.0/' CMakeLists.txt
git commit -am "chore(release): bump to 3.0.1.0"

git tag v3.0.1.0
git push origin main
git push origin v3.0.1.0
```

Watch the **Create Release on Tag** workflow under the Actions tab;
when it goes green a new draft release appears under
[Releases](https://github.com/ElectronicCats/faultycat/releases). Open
the draft, sanity-check the auto-generated notes, then publish.

This project mirrors Minino's pattern of **not** committing the
workflow's defensive bump back to `main`. That means the literal in
`main` reflects the last *manually* committed bump, which may lag the
latest tag. Use the optional `sed` block above to keep it in sync
with each tag.

### Dry-running a release locally

The same script CI runs is callable from the repo root:

```bash
./scripts/get_build.sh v3.0.1.0
ls build/apps/faultycat_fw/
# faultycat_v3.0.1.0.uf2
```

The script does the same in-place version bump CI does, so a
follow-up `git diff` shows exactly what the workflow will write into
its checkout. Revert with:

```bash
git checkout -- CMakeLists.txt
```

if the tree should stay clean.

Pre-release tags work the same way:

```bash
./scripts/get_build.sh v3.1.0.0-rc1
# faultycat_v3.1.0.0-rc1.uf2  ← suffix preserved in the filename
```

## Flashing a release (end-user guide)

Each GitHub Release ships the firmware UF2:

  1. **Flash the firmware.** Hold the BOOTSEL button on the FaultyCat
     while plugging the USB cable. The board appears as a
     mass-storage volume named `RPI-RP2`. Drag
     `faultycat_v3.0.1.0.uf2` onto that volume; the board reboots
     into the new firmware automatically and re-enumerates as
     `1209:fa17`.

  2. **Verify the firmware version**, e.g. via the CDC2 diag banner
     or the `version` shell command:

     ```
     SHELL: VERSION 3.0.1.0
     ```

  3. **Pair with a compatible host tool, if you use one.** See its
     own repository for install instructions. Confirm the host tool
     reports an exact version match against the flashed firmware
     before relying on it — see "Host-side parity gate" above for
     the fields the firmware exposes for that check.

## Related files

| File | Purpose |
|---|---|
| `CMakeLists.txt` | `project(... VERSION ...)` — firmware-side SSoT. |
| `cmake/firmware_version.h.in` | Template `configure_file` fills with `PROJECT_VERSION_*`. |
| `scripts/get_build.sh` | Local + CI build wrapper for the firmware UF2. |
| `.github/workflows/release.yml` | Tag-driven release workflow (`build` + `release` jobs). |
| `services/host_proto/{emfi,crowbar}_proto/*.c` | 6-byte PING reply. |
| `apps/faultycat_fw/main.c` | Diag banner + `version` shell command. |
| `usb/src/usb_descriptors.c` | `bcdDevice` derived from `FW_VERSION_BCD`. |

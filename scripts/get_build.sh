#!/usr/bin/env bash
#
# scripts/get_build.sh — pico-sdk equivalent of Minino's get_build.sh.
#
# Bumps the version literal in CMakeLists.txt to match the requested
# tag, then builds the FaultyCat firmware (RP2040 UF2 + ELF), leaving
# the release artifact under `build/apps/faultycat_fw/` (covered by
# the existing `/**/build/` gitignore entry).
#
# Output layout (under `build/apps/faultycat_fw/`):
#
#   faultycat_vX.Y.Z.W.uf2         drag onto the BOOTSEL volume to flash
#
# Invocation:
#   ./scripts/get_build.sh vX.Y.Z.W
#   ./scripts/get_build.sh vX.Y.Z.W-rc1
#   ./scripts/get_build.sh                 (parses CMakeLists.txt as a fallback)
#
# WHAT THE SCRIPT MODIFIES (intentionally):
#   - CMakeLists.txt                                  project(VERSION X.Y.Z.W)
#
# This is the same defensive bump the CI workflow used to perform inline;
# running it here means a local `./scripts/get_build.sh v3.0.0.5` actually
# burns "v3.0.0.5" into the firmware's `firmware_version.h` (via CMake's
# configure_file from `project(VERSION ...)`) instead of building with
# whatever the working tree was carrying. If you want to keep your tree
# untouched, run the script in a fresh git worktree or revert with
# `git checkout -- CMakeLists.txt` afterwards.
#
# Prereqs the caller MUST install before running this:
#   - cmake, ninja, gcc-arm-none-eabi, libnewlib-arm-none-eabi,
#     libstdc++-arm-none-eabi-newlib  (firmware UF2)
#
# Run from the repo root.

set -euo pipefail

# -----------------------------------------------------------------------------
# Resolve version + tag
# -----------------------------------------------------------------------------

INPUT_ARG="${1:-}"

if [ -z "${INPUT_ARG}" ]; then
    # Fallback: parse the existing `    VERSION X.Y.Z.W` line from
    # CMakeLists.txt and add the `v` prefix back so the output naming
    # is consistent with CI invocations.
    PARSED=$(grep -oE '^[[:space:]]*VERSION[[:space:]]+[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' \
                  CMakeLists.txt \
              | head -1 \
              | awk '{print $2}')
    INPUT_ARG="v${PARSED}"
fi

# Normalise: TAG always carries the leading `v`, VERSION never does.
# Accept both `v3.0.0.0` and `3.0.0.0` inputs.
TAG="${INPUT_ARG}"
[[ "${TAG}" =~ ^v ]] || TAG="v${TAG}"
VERSION="${TAG#v}"
# Pre-release suffix (e.g. `-rc1`) is allowed in tag/filename but not
# in the numeric CMake literal.
VERSION_NUMERIC="${VERSION%%-*}"

if [[ ! "${VERSION_NUMERIC}" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "ERROR: input '${INPUT_ARG}' does not parse as v?MAJOR.MINOR.PATCH.TWEAK[-suffix]" >&2
    echo "Usage: $0 vX.Y.Z.W[-suffix]" >&2
    exit 1
fi

echo "==> Building FaultyCat release ${TAG}"
echo "    Numeric version for code literals: ${VERSION_NUMERIC}"

# -----------------------------------------------------------------------------
# Bump version literal in the source tree
#
# CMake re-reads `project(VERSION ...)` on every configure, so editing
# CMakeLists.txt + re-running `cmake --preset fw-release` is enough to
# propagate ${VERSION_NUMERIC} into firmware_version.h (via
# configure_file), the USB bcdDevice (via FW_VERSION_BCD), the PING
# reply bytes on CDC0/CDC1, and the `SHELL: VERSION ...` reply on CDC2.
# -----------------------------------------------------------------------------

echo "==> Bumping version literal to ${VERSION_NUMERIC}"
sed -i -E \
    "s/^([[:space:]]*VERSION[[:space:]]+)[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+/\\1${VERSION_NUMERIC}/" \
    CMakeLists.txt

echo "    CMakeLists.txt:        $(grep -E '^\s*VERSION\s+[0-9]' CMakeLists.txt | head -1 | xargs)"

# -----------------------------------------------------------------------------
# Output directory under build/ (gitignored)
# -----------------------------------------------------------------------------

OUT_DIR="build/apps/faultycat_fw"
mkdir -p "${OUT_DIR}"

# -----------------------------------------------------------------------------
# Firmware (RP2040 UF2 + ELF)
# -----------------------------------------------------------------------------

echo "==> Configuring firmware (cmake --preset fw-release)"
# Re-configure so PROJECT_VERSION is re-read from the freshly-bumped
# CMakeLists.txt; this is what regenerates firmware_version.h with the
# matching FW_VERSION_STR / FW_VERSION_BCD / FW_VERSION_{MAJOR,...}.
cmake --preset fw-release

echo "==> Building firmware"
cmake --build build/fw-release --parallel

echo "==> Inspecting firmware artifact"
ls -la build/fw-release/apps/faultycat_fw/
# `arm-none-eabi-size` reads section sizes from the ELF — useful CI
# diagnostic even though we no longer ship the ELF as a release asset
# (it just confirmed the image is sane before we copy the UF2).
arm-none-eabi-size build/fw-release/apps/faultycat_fw/faultycat.elf

echo "==> Publishing firmware artifact as ${OUT_DIR}/faultycat_${TAG}.uf2"
cp build/fw-release/apps/faultycat_fw/faultycat.uf2 \
   "${OUT_DIR}/faultycat_${TAG}.uf2"

# -----------------------------------------------------------------------------
# Show what we made
# -----------------------------------------------------------------------------

echo "==> Done. Contents of ${OUT_DIR}/:"
ls -la "${OUT_DIR}/"

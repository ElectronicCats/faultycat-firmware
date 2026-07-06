#!/usr/bin/env bash
# Build the FaultyCat v3 firmware UF2 without the VS Code Pico extension.
#
# Usage: ./build.sh [fw-debug|fw-release] [--clean]
set -euo pipefail

PRESET="${1:-fw-debug}"
BUILD_DIR="build/${PRESET}"

if [[ "${2:-}" == "--clean" || "${1:-}" == "--clean" ]]; then
    rm -rf "${BUILD_DIR}"
fi

cmake --preset "${PRESET}"
cmake --build --preset "${PRESET}" --target faultycat

UF2=$(find "${BUILD_DIR}/apps/faultycat_fw" -name '*.uf2' -print -quit)

if [[ -z "${UF2}" ]]; then
    echo "Build finished but no .uf2 was found under ${BUILD_DIR}/apps/faultycat_fw" >&2
    exit 1
fi

mv "${UF2}" .

echo "Firmware built: $(basename "${UF2}")"

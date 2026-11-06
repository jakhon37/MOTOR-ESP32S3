#!/bin/bash
# Collects what's needed to flash this firmware without PlatformIO into flash_bundle/.
# Run after `pio run`. The steps that use it are in docs/FIRST_DRIVE.md.
set -euo pipefail
cd "$(dirname "$0")/.."

build=.pio/build/esp32-s3-devkitc-1
core=${PLATFORMIO_CORE_DIR:-$HOME/.platformio}

rm -rf flash_bundle
mkdir flash_bundle
cp "$build/bootloader.bin" "$build/partitions.bin" "$build/firmware.bin" tools/wheel_test.py flash_bundle/
cp "$core/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" flash_bundle/
git rev-parse --short HEAD > flash_bundle/COMMIT
git diff --quiet HEAD -- . || echo "uncommitted changes" >> flash_bundle/COMMIT
(cd flash_bundle && sha256sum -- * > SHA256SUMS)
ls -l flash_bundle

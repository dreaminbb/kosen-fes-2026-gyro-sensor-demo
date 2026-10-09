#!/usr/bin/env bash
# 処理の流れ: Arduino-ESP32のMTU交換不具合を補正し、ESP32-C6用BLE Centralをコンパイルする。
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FQBN="${FQBN:-esp32:esp32:XIAO_ESP32C6}"

if ! command -v arduino-cli >/dev/null 2>&1; then
  printf 'arduino-cli is required but was not found in PATH.\n' >&2
  exit 127
fi

python3 "$ROOT_DIR/tools/patch_arduino_ble_mtu.py"
arduino-cli compile --fqbn "$FQBN" "$ROOT_DIR/ble/c6_central"

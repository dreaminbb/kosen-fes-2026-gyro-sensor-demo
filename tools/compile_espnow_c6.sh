#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FQBN="${FQBN:-esp32:esp32:XIAO_ESP32C6}"

if ! command -v arduino-cli >/dev/null 2>&1; then
  printf 'arduino-cli is required but was not found in PATH.\n' >&2
  exit 127
fi

arduino-cli compile --fqbn "$FQBN" "$ROOT_DIR/espnow/c6_receiver"

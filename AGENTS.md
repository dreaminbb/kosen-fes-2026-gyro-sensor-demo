# Repository Guidelines

## Project Structure & Module Organization

- `kosen-fes-2026.ino` contains the Arduino entry points (`setup()` and `loop()`), the six-axis `SensorData` structure, and `sampleData()`. Sampling currently returns placeholder values; hardware initialization and transmission remain unfinished.
- `document/通信方式.md` records communication choices for sending sensor readings to a PC running Godot: USB serial for wired use and Wi-Fi UDP for wireless use. BLE is not planned, despite the existing include.
- `tool.sh` wraps Arduino CLI commands for ESP32-C6.
- `eim_config.toml` configures the local Espressif installer; `compile_flags.txt` is available for compiler tooling. `.pio/` holds local tooling output. No dedicated test or asset directories exist.

## Build, Test, and Development Commands

Install Arduino CLI and the ESP32 board package supporting `esp32:esp32:esp32c6`. Run from the repository root:

- `bash tool.sh compile` — compile the sketch for ESP32-C6.
- `bash tool.sh upload /dev/cu.usbmodemXXXX` — upload to the connected board after compiling; replace the example port with the actual device.
- `bash tool.sh monitor /dev/cu.usbmodemXXXX` — open the serial monitor at 115200 baud.

## Coding Style & Naming Conventions

Use two-space indentation in Arduino C++, braces on the declaration line, PascalCase for types (`SensorData`), camelCase for functions and fields (`sampleData`, `accelX`), and uppercase snake case for constants (`CONNECTION_POLLING_TIME`). Match existing shell formatting. No formatter or linter is configured. Document sensor units and timing assumptions explicitly.

## Testing Guidelines

No automated test framework or coverage threshold is configured. Compile every firmware change, then validate affected behavior on the ESP32-C6. For sensor or transport changes, verify readings, axis ordering, units, and reception in Godot. Record the board, transport, and observed results; identify hardware checks that could not be performed.

## Commit & Pull Request Guidelines

This checkout has no Git metadata, so commit conventions cannot be verified. Use concise, imperative commit subjects, such as `Implement UDP sensor transmission`. PRs should explain the behavior changed, link relevant issues, and report compilation and hardware validation. Update communication notes when transport decisions change. Keep credentials and machine-specific paths out of contributions; `eim_config.toml` is already ignored.

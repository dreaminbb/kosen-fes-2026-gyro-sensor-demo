# Repository Guidelines

## Project Structure & Module Organization

- `kosen-fes-2026.ino` is the XIAO MG24 BLE peripheral and sends sensor samples through GATT notifications.
- `ble/c6_central/c6_central.ino` is the XIAO ESP32-C6 BLE central; Godot reads its USB serial stream through `arduino-cli monitor`.
- `高専祭玉転がし/scripts/ble_monitor.gd` implements the Godot monitor, command controls, telemetry parsing, and 3D preview. `document/通信方式.md` documents the BLE protocol.
- The C6 firmware builds directly with Arduino CLI. No separate test or asset directories are configured.
- `eim_config.toml` configures the local Espressif installer; `compile_flags.txt` is available for compiler tooling. `.pio/` holds local tooling output. No dedicated test or asset directories exist.

## Build, Test, and Development Commands

Install Arduino CLI, the Silicon Labs board package, and the ESP32 board package. Run from the repository root:

- `arduino-cli compile --fqbn "SiliconLabs:silabs:xiao_mg24:protocol_stack=ble_silabs" .` — compile the MG24 peripheral.
- `arduino-cli compile --fqbn esp32:esp32:XIAO_ESP32C6 ble/c6_central` — compile the C6 central.
- `godot --path '高専祭玉転がし'` — run the Godot serial monitor; Arduino CLI must be on `PATH` and the C6 port must be free.

## Coding Style & Naming Conventions

Use two-space indentation in Arduino C++, braces on the declaration line, PascalCase for types (`SensorData`), camelCase for functions and fields (`sampleData`, `accelX`), and uppercase snake case for constants (`CONNECTION_POLLING_TIME`). Match existing shell formatting. No formatter or linter is configured. Document sensor units and timing assumptions explicitly.

## Testing Guidelines

No automated test framework or coverage threshold is configured. Compile every firmware change, then validate affected behavior on the ESP32-C6. For sensor or transport changes, verify readings, axis ordering, units, and reception in Godot. Record the board, transport, and observed results; identify hardware checks that could not be performed.

## Commit & Pull Request Guidelines

Use concise, imperative commit subjects, such as `Add BLE telemetry bridge`. PRs should explain behavior changes and report compilation and hardware validation. Update communication notes when the BLE protocol changes. Keep credentials and machine-specific paths out of contributions; `eim_config.toml` is already ignored.

#!/usr/bin/env python3
"""Work around duplicate NimBLE MTU exchange in Arduino-ESP32 BLEClient."""

from pathlib import Path
import subprocess
import sys


ORIGINAL = """        rc = ble_gattc_exchange_mtu(client->m_conn_id, nullptr, nullptr);
        if (rc != 0) {
"""
OLD_PATCH = """        // Some peers have already started the MTU exchange at connection time.
        // NimBLE reports BLE_HS_EALREADY in that case; the link is still valid.
        if (rc == BLE_HS_EALREADY) {
          rc = 0;
        }
"""
NEW_PATCH = """        // Some peers have already started the MTU exchange at connection time.
        // NimBLE reports BLE_HS_EALREADY in that case; the link is still valid,
        // and there may be no further MTU event to release the connect waiter.
        if (rc == BLE_HS_EALREADY) {
          rc = 0;
          if (client->m_pTaskData != nullptr) {
            BLEUtils::taskRelease(*client->m_pTaskData, 0);
          }
        }
"""
PATCHED = ORIGINAL.replace("        if (rc != 0) {\n", NEW_PATCH + "        if (rc != 0) {\n", 1)
MARKER = "// Some peers have already started the MTU exchange at connection time."


def main() -> int:
    try:
        data_dir = subprocess.check_output(
            ["arduino-cli", "config", "get", "directories.data"], text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"Could not locate Arduino CLI data directory: {error}", file=sys.stderr)
        return 1

    package_dir = Path(data_dir) / "packages/esp32/hardware/esp32"
    candidates = sorted(package_dir.glob("*/libraries/BLE/src/BLEClient.cpp"))
    if not candidates:
        print(f"Arduino-ESP32 BLEClient.cpp not found under {package_dir}", file=sys.stderr)
        return 1

    source = candidates[-1]
    contents = source.read_text()
    if MARKER in contents and "BLEUtils::taskRelease(*client->m_pTaskData, 0);" in contents:
        print(f"BLE MTU workaround already applied: {source}")
        return 0
    if MARKER in contents:
        if OLD_PATCH not in contents:
            print(f"Found an unknown BLE MTU workaround in {source}", file=sys.stderr)
            return 1
        source.write_text(contents.replace(OLD_PATCH, NEW_PATCH, 1))
        print(f"Updated BLE MTU workaround in {source}")
        return 0
    if ORIGINAL not in contents:
        print(
            "Arduino-ESP32 BLEClient.cpp has an unknown layout; refusing to patch it: "
            f"{source}",
            file=sys.stderr,
        )
        return 1

    source.write_text(contents.replace(ORIGINAL, PATCHED, 1))
    print(f"Applied BLE MTU workaround to {source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

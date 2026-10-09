#!/usr/bin/env python3
"""処理の流れ: Arduino CLIからESP32コアのBLEClient.cppを探す。
既知のMTU交換競合だけを安全に補正し、未対応のソース形式なら停止する。
"""

from pathlib import Path
import re
import subprocess
import sys


ORIGINAL = """        rc = ble_gattc_exchange_mtu(client->m_conn_id, nullptr, nullptr);
        if (rc != 0) {
"""
PATCHED = """        // 接続時に相手側がMTU交換を開始済みの場合があるため、重複要求を処理する。
        // BLE_HS_EALREADYは接続成功として扱い、待機中の接続処理を解除する。
        if (rc == BLE_HS_EALREADY) {
          rc = 0;
          if (client->m_pTaskData != nullptr) {
            BLEUtils::taskRelease(*client->m_pTaskData, 0);
          }
        }
"""
PATCHED_ORIGINAL = ORIGINAL.replace(
    "        if (rc != 0) {\n", PATCHED + "        if (rc != 0) {\n", 1
)
LEGACY_PATCH = re.compile(
    r"(?ms)^ {8}(?://[^\n]*\n)+"
    r"^ {8}if \(rc == BLE_HS_EALREADY\) \{.*?^ {8}\}\n"
)


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
    if PATCHED in contents:
        print(f"BLE MTU workaround already applied: {source}")
        return 0
    legacy_match = LEGACY_PATCH.search(contents)
    if legacy_match is not None:
        updated = LEGACY_PATCH.sub(PATCHED, contents, count=1)
        source.write_text(updated)
        print(f"BLE MTU workaround updated: {source}")
        return 0
    if ORIGINAL not in contents:
        print(
            "Arduino-ESP32 BLEClient.cpp has an unknown layout; refusing to patch it: "
            f"{source}",
            file=sys.stderr,
        )
        return 1

    source.write_text(contents.replace(ORIGINAL, PATCHED_ORIGINAL, 1))
    print(f"BLE MTU workaround applied: {source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

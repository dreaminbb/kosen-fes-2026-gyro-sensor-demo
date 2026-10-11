#!/usr/bin/env python3
"""処理の流れ: Arduino CLIからESP32コアのBLEClient.cppを探す。
既知のMTU交換競合と分割mbufの通知欠落を補正する。
未対応のソース形式なら停止する。
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
NOTIFY_ORIGINAL = """          characteristic->second->m_semaphoreReadCharEvt.take();
          characteristic->second->m_value = String((char *)event->notify_rx.om->om_data, event->notify_rx.om->om_len);
          characteristic->second->m_semaphoreReadCharEvt.give();

          if (characteristic->second->m_notifyCallback != nullptr) {
            log_d("Invoking callback for notification on characteristic %s", characteristic->second->toString().c_str());
            characteristic->second->m_notifyCallback(
              characteristic->second, event->notify_rx.om->om_data, event->notify_rx.om->om_len, !event->notify_rx.indication
            );
          }
"""
NOTIFY_PATCHED = """          // Notifications can span several mbufs. om_len covers only the first one.
          const uint16_t notificationLength = OS_MBUF_PKTLEN(event->notify_rx.om);
          uint8_t notificationValue[BLE_ATT_ATTR_MAX_LEN];
          if (notificationLength > sizeof(notificationValue) ||
              os_mbuf_copydata(event->notify_rx.om, 0, notificationLength, notificationValue) != 0) {
            log_e("Could not copy complete BLE notification");
            break;
          }
          characteristic->second->m_semaphoreReadCharEvt.take();
          characteristic->second->m_value = String((char *)notificationValue, notificationLength);
          characteristic->second->m_semaphoreReadCharEvt.give();

          if (characteristic->second->m_notifyCallback != nullptr) {
            log_d("Invoking callback for notification on characteristic %s", characteristic->second->toString().c_str());
            characteristic->second->m_notifyCallback(
              characteristic->second, notificationValue, notificationLength, !event->notify_rx.indication
            );
          }
"""


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
    updated = contents
    if PATCHED not in updated:
        legacy_match = LEGACY_PATCH.search(updated)
        if legacy_match is not None:
            updated = LEGACY_PATCH.sub(PATCHED, updated, count=1)
        elif ORIGINAL in updated:
            updated = updated.replace(ORIGINAL, PATCHED_ORIGINAL, 1)
        else:
            print(
                "Arduino-ESP32 BLEClient.cpp has an unknown MTU layout: "
                f"{source}",
                file=sys.stderr,
            )
            return 1
    if NOTIFY_PATCHED not in updated:
        if NOTIFY_ORIGINAL not in updated:
            print(
                "Arduino-ESP32 BLEClient.cpp has an unknown notification layout: "
                f"{source}",
                file=sys.stderr,
            )
            return 1
        updated = updated.replace(NOTIFY_ORIGINAL, NOTIFY_PATCHED, 1)
    if updated != contents:
        source.write_text(updated)
        print(f"BLE MTU and notification workarounds applied: {source}")
    else:
        print(f"BLE MTU and notification workarounds already applied: {source}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

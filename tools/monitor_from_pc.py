"""Receive sensor data from the ESP32 UDP access point."""

import socket
import time
from datetime import datetime

ESP_IP = "192.168.4.1"
ESP_PORT = 8888
RECEIVE_TIMEOUT_SECONDS = 1.0
HELLO_INTERVAL_SECONDS = 1.0
RECONNECT_AFTER_SECONDS = 3.5


def format_message(message: str) -> str:
    fields = message.split(",")
    if len(fields) == 8 and fields[0] == "DATA":
        try:
            timestamp = int(fields[1])
            accel = ", ".join(f"{float(value):8.3f}" for value in fields[2:5])
            gyro = ", ".join(f"{float(value):8.3f}" for value in fields[5:8])
        except ValueError:
            return f"不正なDATAパケット: {message}"
        return f"DATA t={timestamp:>8} ms | accel=({accel}) | gyro=({gyro})"
    return message


def main() -> None:
    # Bind an ephemeral local port so the ESP32 can send replies back to us.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("0.0.0.0", 0))
        sock.settimeout(RECEIVE_TIMEOUT_SECONDS)
        local_port = sock.getsockname()[1]
        print(f"UDP受信ポート: {local_port}")
        print(f"ESP32: {ESP_IP}:{ESP_PORT}（ESP32のAPにWi-Fi接続してください）")

        next_hello_at = 0.0
        last_packet_at = 0.0
        packet_count = 0

        try:
            while True:
                now = time.monotonic()
                if now >= next_hello_at:
                    try:
                        sock.sendto(b"HELLO", (ESP_IP, ESP_PORT))
                        print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] 接続確認 HELLO を送信")
                    except OSError as error:
                        print(f"UDP送信エラー: {error}")
                    next_hello_at = now + HELLO_INTERVAL_SECONDS

                try:
                    data, address = sock.recvfrom(2048)
                    message = data.decode('utf-8', errors='replace').strip()
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] {address[0]}:{address[1]} 受信: {format_message(message)}")
                    if address == (ESP_IP, ESP_PORT) and message.startswith(("DATA,", "ACK,", "HELLO_ACK")):
                        next_hello_at = float("inf")

                except socket.timeout:
                    print(f"[{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] タイムアウト: 受信パケットなし")

                    if last_packet_at and time.monotonic() - last_packet_at >= RECONNECT_AFTER_SECONDS:
                        # If the ESP32 rebooted, its setup waits for another HELLO.
                        next_hello_at = 0.0
                        last_packet_at = 0.0
                    continue
                except OSError as error:
                    print(f"UDP受信エラー: {error}")
                    continue


                packet_count += 1
                last_packet_at = time.monotonic()

        except KeyboardInterrupt:
            print(f"\n終了します。受信パケット数: {packet_count}")


if __name__ == "__main__":
    main()

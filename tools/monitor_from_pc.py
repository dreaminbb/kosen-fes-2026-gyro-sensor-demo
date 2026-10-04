import socket

esp_ip = "192.168.4.1"
esp_port = 8888


def main():
    while True:
        send_udp_message()

def send_udp_message():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(3)
        try:
            data, address = sock.recvfrom(1024)
            print(f"応答: {data.decode(errors='replace')} ({address[0]}:{address[1]})")

        except socket.timeout:
            print("3秒以内に応答なし。Wi-Fi接続、ESP32のAP起動、UDPポートを確認してください。")



if __name__ == "__main__":
    main()

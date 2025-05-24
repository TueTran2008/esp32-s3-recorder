import socket
import threading

BUFFER_SIZE = 1024
output_clients = []

# Accepts and handles output clients (port 9001)
def output_client_handler(conn, addr):
    print(f"[+] Output client connected: {addr}")
    output_clients.append(conn)
    try:
        while True:
            data = conn.recv(1)  # Keep connection alive
            if not data:
                break
    finally:
        print(f"[-] Output client disconnected: {addr}")
        output_clients.remove(conn)
        conn.close()

# Accepts input from ESP32 A and broadcasts to all output clients
def input_client_handler(conn, addr):
    print(f"[+] Input client connected: {addr}")
    try:
        while True:
            data = conn.recv(BUFFER_SIZE)
            if not data:
                break
            for c in output_clients:
                try:
                    c.sendall(data)
                except:
                    output_clients.remove(c)
    finally:
        print(f"[-] Input client disconnected: {addr}")
        conn.close()

def main():
    # Input server (port 9000)
    input_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    input_server.bind(('0.0.0.0', 8081))
    input_server.listen(1)

    # Output server (port 9001)
    output_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    output_server.bind(('0.0.0.0', 8082))
    output_server.listen(5)

    print("[*] TCP Relay Server running...")
    print("  - Waiting for audio input on port 8081")
    print("  - Accepting listeners on port 8082")

    # Thread to handle output clients
    def accept_output_clients():
        while True:
            conn, addr = output_server.accept()
            threading.Thread(target=output_client_handler, args=(conn, addr), daemon=True).start()

    threading.Thread(target=accept_output_clients, daemon=True).start()

    # Accept one input client (ESP32 A)
    while True:
        conn, addr = input_server.accept()
        threading.Thread(target=input_client_handler, args=(conn, addr), daemon=True).start()

if __name__ == "__main__":
    main()

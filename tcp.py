import socket
import threading

# Configuration
HOST = '0.0.0.0'  # Listen on all interfaces
PORT = 8080       # Server port

# Globals to store the client sockets
sender_socket = None
receiver_socket = None

def handle_client(client_socket, client_type):
    global sender_socket, receiver_socket

    print(f"[INFO] {client_type} connected.")

    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                print(f"[INFO] {client_type} disconnected.")
                break

            if client_type == "Sender" and receiver_socket:
                try:
                    receiver_socket.sendall(data)
                except:
                    print("[ERROR] Failed to send data to Receiver.")
            elif client_type == "Receiver":
                # Typically, Receiver just listens; nothing to send back
                pass

    except Exception as e:
        print(f"[ERROR] Connection error with {client_type}: {e}")

    client_socket.close()
    if client_type == "Sender":
        sender_socket = None
    else:
        receiver_socket = None

def start_server():
    global sender_socket, receiver_socket

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(2)

    print(f"[INFO] Server listening on {HOST}:{PORT}")

    while True:
        client_socket, addr = server.accept()
        print(f"[INFO] Connection from {addr}")

        if not sender_socket:
            sender_socket = client_socket
            threading.Thread(target=handle_client, args=(client_socket, "Sender")).start()
        elif not receiver_socket:
            receiver_socket = client_socket
            threading.Thread(target=handle_client, args=(client_socket, "Receiver")).start()
        else:
            print("[INFO] Too many clients connected. Rejecting.")
            client_socket.close()

if __name__ == "__main__":
    start_server()

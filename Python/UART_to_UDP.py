import serial
import struct
import socket

# --- UART Setup ---
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
print("Listening to UART and broadcasting via UDP...")

# --- UDP Setup ---
UDP_IP = '255.255.255.255'  # Broadcast address
UDP_PORT = 8848
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

buffer = bytearray()

while True:
    data = ser.read(ser.in_waiting or 1)
    if data:
        buffer += data

    # Process in 16-byte chunks
    while len(buffer) >= 16:
        payload = buffer[:16]
        buffer = buffer[16:]

        try:
            floats = struct.unpack('<ffff', payload)
            print(f"Received floats: {floats}")

            # Send as raw float bytes via UDP
            udp_socket.sendto(payload, (UDP_IP, UDP_PORT))

        except struct.error as e:
            print(f"Unpack error: {e}")

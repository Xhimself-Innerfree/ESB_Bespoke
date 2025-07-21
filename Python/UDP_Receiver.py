import socket
import struct

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 8848))

print("Listening for UDP packets on port 8848...")

while True:
    data, addr = sock.recvfrom(1024)
    if len(data) == 16:
        floats = struct.unpack('<ffff', data)
        print(f"From {addr}: {floats}")

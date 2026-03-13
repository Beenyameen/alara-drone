import socket
import time
import subprocess

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while True:
    try:
        ip = subprocess.check_output(["hostname", "-I"]).decode().split()[0]
        sock.sendto(f"PI:{ip}".encode(), ("255.255.255.255", 10000))
    except Exception:
        pass
    time.sleep(1)
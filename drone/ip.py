import socket
import time
import subprocess
import os

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)


def get_local_ip() -> str:
    # Determine the current outbound IPv4 address without relying on interface names.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        probe.close()


def get_wsl_ip() -> str | None:
    try:
        # On Windows, use WSL to retrieve eth0 IPv4 for simulator endpoints.
        output = subprocess.check_output(
            ["wsl", "sh", "-lc", "hostname -I | cut -d' ' -f1"],
            text=True,
        ).strip()
        if output and output != "127.0.0.1":
            return output
    except Exception:
        pass
    return None


def get_default_gateway() -> str | None:
    try:
        output = subprocess.check_output(["ip", "route", "show", "default"], text=True)
        parts = output.strip().split()
        if "via" in parts:
            return parts[parts.index("via") + 1]
    except Exception:
        pass
    return None

while True:
    try:
        # Priority: explicit override > discovered WSL IP > local outbound IP.
        ip = os.getenv("PI_ADVERTISE_IP") or get_wsl_ip() or get_local_ip()
        payload = f"PI:{ip}".encode()

        # Broadcast for LAN peers.
        sock.sendto(payload, ("255.255.255.255", 10000))

        # Direct send improves reliability for WSL mirrored mode.
        gateway = get_default_gateway()
        if gateway:
            sock.sendto(payload, (gateway, 10000))

        # localhost path can work when host loopback integration is enabled.
        sock.sendto(payload, ("127.0.0.1", 10000))
        print(f"Broadcasted IP: {ip}")
    except Exception:
        pass
    time.sleep(1)
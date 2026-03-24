import socket
import time
import struct
from gpiozero import Button
from signal import pause

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

last_timestamp = -1
last_usv = -1

def on_radiation_pulse():
    global last_timestamp
    global last_usv
    current_time = time.time()
    cpm=-1
    print(f"Radiation pulse detected! CPM: {cpm}, USV: {usv}")
    if last_timestamp == -1:
        last_timestamp = current_time
    else:
        delta = current_time - last_timestamp
        cpm = 60.0 / delta
        usv = cpm / 153.8
        last_usv = (0.2*last_usv + 0.8*usv) if last_usv != -1 else usv
        last_timestamp = current_time


    payload = struct.pack("!dd", time.time(), last_usv)
    try:
        sock.sendto(payload, ("<broadcast>", 11000))
    except Exception:
        pass

geiger = Button(17, pull_up=True)
geiger.when_pressed = on_radiation_pulse

pause()
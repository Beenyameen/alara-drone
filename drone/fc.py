import time
import struct
import zmq
from pymavlink import mavutil

print("Init ZMQ")
ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.bind("tcp://0.0.0.0:15000")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

pub = ctx.socket(zmq.PUB)
pub.bind("tcp://0.0.0.0:16000")

rep = ctx.socket(zmq.REP)
rep.bind("tcp://0.0.0.0:15001")

print("Wait Heartbeat")
master = mavutil.mavlink_connection("/dev/serial0", baud=115200)
msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=10.0)
if not msg:
    print("Heartbeat timeout, exiting to restart...")
    import sys
    sys.exit(1)
master.target_system = 1
master.target_component = 1

def send_cmd(cmd, p1, p2=0):
    for _ in range(5):
        master.mav.command_long_send(1, 1, cmd, 0, p1, p2, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            msg = master.recv_match(type="COMMAND_ACK", blocking=False)
            if msg and msg.command == cmd:
                return msg.result == 0
        time.sleep(0.1)
    return False

def set_param(param_id, param_value, param_type):
    for _ in range(5):
        master.mav.param_set_send(1, 1, param_id.encode("utf-8"), param_value, param_type)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            msg = master.recv_match(type="PARAM_VALUE", blocking=False)
            if msg and msg.param_id == param_id:
                return True
        time.sleep(0.1)
    return False

print("Disable Auto-Disarm")
set_param("DISARM_DELAY", 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

print("Set Mode")
send_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1)

print("Active")
cur = [1500, 1500, 1000, 1500]
tgt = [1500, 1500, 1000, 1500]
inc = [1, 1, 4, 2]
last_cmd_time = time.time()

LOOP_RATE = 50
LOOP_PERIOD = 1.0 / LOOP_RATE

while True:
    loop_start = time.time()
    
    try:
        msg = rep.recv(flags=zmq.NOBLOCK)
        if msg == b"TOGGLE_ARM":
            target_state = 0 if master.motors_armed() else 1
            success = send_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, target_state, 21196 if target_state == 0 else 0)
            if success:
                rep.send(b"1" if target_state else b"0")
            else:
                rep.send(b"-1")
        elif msg == b"CHECK_ARM":
            rep.send(b"1" if master.motors_armed() else b"0")
    except zmq.Again:
        pass

    while True:
        try:
            msg = sub.recv(flags=zmq.NOBLOCK)
            if len(msg) == 16:
                r, p, y, t = struct.unpack("<ffff", msg)
                last_cmd_time = time.time()
                print(f"\rRx - Roll: {r:6.3f} | Pitch: {p:6.3f} | Yaw: {y:6.3f} | Throttle: {t:6.3f}          ", end="", flush=True)
                tgt[0] = max(1000, min(2000, 1500 + int(r * 100)))
                tgt[1] = max(1000, min(2000, 1500 + int(p * 100)))
                tgt[2] = max(1000, min(2000, 1000 + int(t * 1000)))
                tgt[3] = max(1000, min(2000, 1500 + int(y * 100)))
        except zmq.Again:
            break

    if time.time() - last_cmd_time > 0.5:
        tgt[0] = 1500
        cur[0] = 1500
        tgt[1] = 1500
        cur[1] = 1500
        tgt[3] = 1500
        cur[3] = 1500
        
    if time.time() - last_cmd_time > 1.0:
        tgt[2] = 1000

    for i in range(4):
        if cur[i] < tgt[i]: cur[i] = min(cur[i] + inc[i], tgt[i])
        elif cur[i] > tgt[i]: cur[i] = max(cur[i] - inc[i], tgt[i])

    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    master.mav.rc_channels_override_send(1, 1, cur[0], cur[1], cur[2], cur[3], 65535, 65535, 65535, 65535)

    while True:
        msg = master.recv_match(blocking=False)
        if not msg:
            break
        if msg.get_type() == "ATTITUDE":
            pub.send(struct.pack("<fff", msg.roll, msg.pitch, msg.yaw))

    elapsed = time.time() - loop_start
    if elapsed < LOOP_PERIOD:
        time.sleep(LOOP_PERIOD - elapsed)

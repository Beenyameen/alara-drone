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

print("Wait Heartbeat")
master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
master.wait_heartbeat()
master.target_system = 1
master.target_component = 1

def send_cmd(cmd, p1):
    while True:
        master.mav.command_long_send(1, 1, cmd, 0, p1, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            msg = master.recv_match(type='COMMAND_ACK', blocking=False)
            if msg and msg.command == cmd and msg.result == 0:
                return
        time.sleep(0.1)

def set_param(param_id, param_value, param_type):
    while True:
        master.mav.param_set_send(1, 1, param_id.encode('utf-8'), param_value, param_type)
        t0 = time.time()
        while time.time() - t0 < 0.5:
            msg = master.recv_match(type='PARAM_VALUE', blocking=False)
            if msg and msg.param_id == param_id:
                return
        time.sleep(0.1)

print("Disable Auto-Disarm")
set_param('DISARM_DELAY', 0.0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

print("Set Mode")
send_cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1)

print("Arm")
send_cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)

print("Active")
cur = [1500, 1500, 1000, 1500]
tgt = [1500, 1500, 1000, 1500]
inc = [5, 5, 20, 10]

while True:
    while True:
        try:
            msg = sub.recv(flags=zmq.NOBLOCK)
            if len(msg) == 16:
                r, p, y, t = struct.unpack('<ffff', msg)
                print(f"\rRx - Roll: {r:6.3f} | Pitch: {p:6.3f} | Yaw: {y:6.3f} | Throttle: {t:6.3f}          ", end='', flush=True)
                tgt[0] = max(1000, min(2000, 1500 + int(r * 100)))
                tgt[1] = max(1000, min(2000, 1500 + int(p * 100)))
                tgt[2] = max(1000, min(2000, 1000 + int(t * 1000)))
                tgt[3] = max(1000, min(2000, 1500 + int(y * 100)))
        except zmq.Again:
            break

    for i in range(4):
        if cur[i] < tgt[i]: cur[i] = min(cur[i] + inc[i], tgt[i])
        elif cur[i] > tgt[i]: cur[i] = max(cur[i] - inc[i], tgt[i])

    master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    master.mav.rc_channels_override_send(1, 1, cur[0], cur[1], cur[2], cur[3], 65535, 65535, 65535, 65535)

    while True:
        msg = master.recv_match(blocking=False)
        if not msg:
            break
        if msg.get_type() == 'ATTITUDE':
            pub.send(struct.pack('<fff', msg.roll, msg.pitch, msg.yaw))

    time.sleep(0.1)

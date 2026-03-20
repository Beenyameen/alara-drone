import time
import struct
import zmq
from pymavlink import mavutil

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.bind("tcp://0.0.0.0:15000")
sub.setsockopt_string(zmq.SUBSCRIBE, "")

pub = ctx.socket(zmq.PUB)
pub.bind("tcp://0.0.0.0:16000")

master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
master.wait_heartbeat()
master.target_system = 1
master.target_component = 1

master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(1)
master.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(2)

cur = [1500, 1500, 1000, 1500]
tgt = [1500, 1500, 1000, 1500]
inc = [5, 5, 20, 10]

while True:
    try:
        msg = sub.recv(flags=zmq.NOBLOCK)
        if len(msg) == 16:
            r, p, y, t = struct.unpack('<ffff', msg)
            tgt[0] = max(1000, min(2000, 1500 + int(r * 100)))
            tgt[1] = max(1000, min(2000, 1500 + int(p * 100)))
            tgt[2] = max(1000, min(2000, 1000 + int(t * 1000)))
            tgt[3] = max(1000, min(2000, 1500 + int(y * 100)))
    except zmq.Again:
        pass

    for i in range(4):
        if cur[i] < tgt[i]: cur[i] = min(cur[i] + inc[i], tgt[i])
        elif cur[i] > tgt[i]: cur[i] = max(cur[i] - inc[i], tgt[i])

    master.mav.rc_channels_override_send(1, 1, cur[0], cur[1], cur[2], cur[3], 0, 0, 0, 0)

    imu = master.recv_match(type='ATTITUDE', blocking=False)
    if imu: pub.send(struct.pack('<fff', imu.roll, imu.pitch, imu.yaw))

    time.sleep(0.1)

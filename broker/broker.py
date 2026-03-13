import socket
import zmq
import rvl

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("0.0.0.0", 10000))

while True:
    data = sock.recvfrom(1024)[0].decode()
    if data.startswith("PI:"):
        ip = data.split(":")[1]
        break

ctx = zmq.Context()
sub = ctx.socket(zmq.SUB)
sub.setsockopt(zmq.SUBSCRIBE, b"")
sub.setsockopt(zmq.RCVHWM, 2)
sub.connect(f"tcp://{ip}:11000")

pub = ctx.socket(zmq.PUB)
pub.setsockopt(zmq.SNDHWM, 2)
pub.bind("tcp://0.0.0.0:12000")

while True:
    m = sub.recv_multipart()
    pub.send_multipart([m[0], m[1], rvl.decompress(m[2])])

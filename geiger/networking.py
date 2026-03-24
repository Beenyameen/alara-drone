import logging
import os
import socket
import struct
import time

import numpy as np
import zmq

from geigercalc import RadMapper


def parse_pose_from_trajectory_msg(msg):
	# trajectory:13001 payload is one or more 4x4 float32 matrices (64 bytes each).
	msg_len = len(msg)
	if msg_len < 64 or (msg_len % 64) != 0:
		return None
	last_mat = np.frombuffer(msg, dtype=np.float32, count=16, offset=msg_len - 64)
	if last_mat.size != 16:
		return None
	return float(last_mat[12]), float(last_mat[13]), float(last_mat[14])


def build_geiger_hosts():
	hosts = []
	single_host = os.getenv("GEIGER_SOURCE_HOST", "").strip()
	if single_host:
		hosts.append(single_host)

	host_list = os.getenv("GEIGER_SOURCE_HOSTS", "").strip()
	if host_list:
		for host in host_list.split(","):
			host = host.strip()
			if host and host not in hosts:
				hosts.append(host)

	for host in ("host.docker.internal", "gateway.docker.internal"):
		if host not in hosts:
			hosts.append(host)

	return hosts


def main():
	logging.basicConfig(
		level=logging.INFO,
		format="[%(levelname)s] %(name)s: %(message)s",
		force=True,
	)
	logger = logging.getLogger("geiger.networking")

	radmapper = RadMapper(room_width=20, room_length=20, room_ceil=5)

	ctx = zmq.Context.instance()
	geiger_hosts = build_geiger_hosts()
	geiger_port = int(os.getenv("GEIGER_SOURCE_PORT", "10350"))
	geiger_sock = None
	geiger_buffer = b""
	last_geiger_connect_try = 0.0
	geiger_host_index = 0

	position_sub = ctx.socket(zmq.SUB)
	position_sub.setsockopt_string(zmq.SUBSCRIBE, "")
	position_sub.connect("tcp://trajectory:13001")

	output_pub = ctx.socket(zmq.PUB)
	output_pub.bind("tcp://0.0.0.0:13075")

	poller = zmq.Poller()
	poller.register(position_sub, zmq.POLLIN)

	logger.info(
		"Geiger service started, waiting for data... geiger_sources=%s port=%d",
		geiger_hosts,
		geiger_port,
	)

	last_publish_time = 0.0
	last_invalid_pose_log_time = 0.0
	while True:
		events = dict(poller.poll(timeout=100))

		now = time.time()
		if geiger_sock is None and now - last_geiger_connect_try >= 1.0:
			last_geiger_connect_try = now
			geiger_host = geiger_hosts[geiger_host_index % len(geiger_hosts)]
			try:
				candidate = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				candidate.settimeout(2.0)
				candidate.connect((geiger_host, geiger_port))
				candidate.setblocking(False)
				geiger_sock = candidate
				geiger_buffer = b""
				logger.info("Connected geiger TCP stream to %s:%d", geiger_host, geiger_port)
			except OSError as exc:
				geiger_host_index += 1
				logger.warning(
					"Failed to connect geiger TCP stream to %s:%d (%s)",
					geiger_host,
					geiger_port,
					exc,
				)

		if geiger_sock is not None:
			try:
				chunk = geiger_sock.recv(4096)
				if not chunk:
					raise ConnectionError("geiger stream closed by peer")
				geiger_buffer += chunk
				while len(geiger_buffer) >= 16:
					packet = geiger_buffer[:16]
					geiger_buffer = geiger_buffer[16:]
					timestamp, usv = struct.unpack("!dd", packet)
					radmapper.report_geiger(timestamp, usv)
					logger.debug(
						"Received geiger data: timestamp=%.3f, usv=%.2f",
						timestamp,
						usv,
					)
			except BlockingIOError:
				pass
			except Exception:
				logger.exception("Error while receiving geiger TCP stream")
				try:
					geiger_sock.close()
				except OSError:
					pass
				geiger_sock = None
				geiger_buffer = b""

		if position_sub in events:
			try:
				data = position_sub.recv()
				xyz = parse_pose_from_trajectory_msg(data)
				if xyz is not None:
					timestamp = time.time()
					x, y, z = xyz
					radmapper.report_position(timestamp, x, y, z)
					logger.debug(
						"Received position data: timestamp=%.3f, x=%.2f, y=%.2f, z=%.2f",
						timestamp,
						x,
						y,
						z,
					)
				elif now - last_invalid_pose_log_time >= 2.0:
					logger.warning(
						"Ignored trajectory payload with invalid length: %d bytes",
						len(data),
					)
					last_invalid_pose_log_time = now
			except Exception:
				logger.exception("Error while receiving position data")

		if now - last_publish_time >= 5.0:
			radmapper._run_batch_prediction(time.time())
			prediction_grid = radmapper.get_prediction_grid()
			output_pub.send(prediction_grid.tobytes())
			logger.info("Published prediction grid (%d bytes) max value: %.2f", prediction_grid.nbytes, np.max(prediction_grid))
			last_publish_time = now

if __name__ == "__main__":
	main()
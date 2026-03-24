import argparse
import logging
import time

import matplotlib.pyplot as plt
import numpy as np
import zmq


def parse_args():
	parser = argparse.ArgumentParser(
		description="Live visualizer for geiger prediction grid from ZMQ PUB."
	)
	parser.add_argument(
		"--endpoint",
		default="tcp://127.0.0.1:13075",
		help="ZMQ PUB endpoint to subscribe to.",
	)
	parser.add_argument("--grid-width", type=int, default=40)
	parser.add_argument("--grid-length", type=int, default=40)
	parser.add_argument("--grid-ceil", type=int, default=10)
	parser.add_argument(
		"--slice-z",
		type=int,
		default=-1,
		help="Z index to show in right panel. -1 means center slice.",
	)
	parser.add_argument(
		"--clip-max",
		type=float,
		default=500.0,
		help="Color max for visualization scaling.",
	)
	parser.add_argument(
		"--fixed-scale",
		action="store_true",
		help="Use fixed color range [0, --clip-max] instead of auto-scaling.",
	)
	parser.add_argument(
		"--min-clip-max",
		type=float,
		default=1.0,
		help="Minimum vmax when auto-scaling.",
	)
	return parser.parse_args()


def decode_grid(payload, expected_shape):
	expected_count = int(np.prod(expected_shape))
	expected_bytes = expected_count * 4
	if len(payload) != expected_bytes:
		return None

	arr = np.frombuffer(payload, dtype=np.float32, count=expected_count)
	return arr.reshape(expected_shape)


def main():
	args = parse_args()
	logging.basicConfig(
		level=logging.INFO,
		format="[%(levelname)s] %(name)s: %(message)s",
		force=True,
	)
	logger = logging.getLogger("geiger.tempvisualizer")

	shape = (args.grid_width, args.grid_length, args.grid_ceil)
	if min(shape) <= 0:
		raise ValueError("grid dimensions must all be > 0")

	slice_z = args.slice_z
	if slice_z < 0:
		slice_z = shape[2] // 2
	slice_z = max(0, min(shape[2] - 1, slice_z))

	ctx = zmq.Context.instance()
	sub = ctx.socket(zmq.SUB)
	sub.setsockopt_string(zmq.SUBSCRIBE, "")
	sub.connect(args.endpoint)

	poller = zmq.Poller()
	poller.register(sub, zmq.POLLIN)

	plt.ion()
	fig, axes = plt.subplots(1, 2, figsize=(12, 5))
	ax_mip, ax_slice = axes

	init_2d = np.zeros((shape[0], shape[1]), dtype=np.float32)
	im_mip = ax_mip.imshow(
		init_2d,
		origin="lower",
		cmap="hot",
		vmin=0.0,
		vmax=args.clip_max,
		interpolation="nearest",
	)
	im_slice = ax_slice.imshow(
		init_2d,
		origin="lower",
		cmap="hot",
		vmin=0.0,
		vmax=args.clip_max,
		interpolation="nearest",
	)
	fig.colorbar(im_mip, ax=ax_mip, fraction=0.046, pad=0.04, label="CPS")
	fig.colorbar(im_slice, ax=ax_slice, fraction=0.046, pad=0.04, label="CPS")

	ax_mip.set_title("Max Projection (Z)")
	ax_slice.set_title(f"Z Slice = {slice_z}")
	ax_mip.set_xlabel("Y")
	ax_mip.set_ylabel("X")
	ax_slice.set_xlabel("Y")
	ax_slice.set_ylabel("X")

	status = fig.text(0.02, 0.01, "Waiting for data...", fontsize=9)

	logger.info(
		"Visualizer started. endpoint=%s shape=%s slice_z=%d",
		args.endpoint,
		shape,
		slice_z,
	)

	frames = 0
	last_fps_time = time.time()
	while True:
		events = dict(poller.poll(timeout=200))
		if sub in events:
			payload = sub.recv()
			grid = decode_grid(payload, shape)
			if grid is None:
				logger.warning(
					"Received payload with unexpected size: %d bytes (expected %d)",
					len(payload),
					int(np.prod(shape)) * 4,
				)
				continue

			mip = np.max(grid, axis=2)
			zslice = grid[:, :, slice_z]

			if not args.fixed_scale:
				frame_max = float(np.max(grid))
				if frame_max > 0.0:
					auto_vmax = float(np.percentile(grid, 99.0))
					auto_vmax = max(args.min_clip_max, auto_vmax)
				else:
					auto_vmax = args.min_clip_max
				im_mip.set_clim(0.0, auto_vmax)
				im_slice.set_clim(0.0, auto_vmax)

			im_mip.set_data(mip)
			im_slice.set_data(zslice)

			frames += 1
			now = time.time()
			elapsed = max(1e-6, now - last_fps_time)
			fps = frames / elapsed
			status.set_text(
				f"frames={frames} | fps={fps:.1f} | max={float(np.max(grid)):.4f}"
			)

			fig.canvas.draw_idle()
			fig.canvas.flush_events()

		plt.pause(0.001)


if __name__ == "__main__":
	main()

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
		"--voxel-percentile",
		type=float,
		default=98.0,
		help="Percentile threshold used to select visible voxels.",
	)
	parser.add_argument(
		"--voxel-max-cubes",
		type=int,
		default=3500,
		help="Maximum number of visible voxels per frame (0 disables cap).",
	)
	parser.add_argument(
		"--voxel-elev",
		type=float,
		default=25.0,
		help="Elevation angle for the 3D voxel camera.",
	)
	parser.add_argument(
		"--voxel-azim",
		type=float,
		default=-60.0,
		help="Azimuth angle for the 3D voxel camera.",
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

	voxel_percentile = max(0.0, min(100.0, args.voxel_percentile))
	if args.voxel_max_cubes < 0:
		raise ValueError("voxel-max-cubes must be >= 0")

	ctx = zmq.Context.instance()
	sub = ctx.socket(zmq.SUB)
	sub.setsockopt_string(zmq.SUBSCRIBE, "")
	sub.connect(args.endpoint)

	poller = zmq.Poller()
	poller.register(sub, zmq.POLLIN)

	plt.ion()
	fig = plt.figure(figsize=(13, 6))
	ax_mip = fig.add_subplot(1, 2, 1)
	ax_voxel = fig.add_subplot(1, 2, 2, projection="3d")

	init_2d = np.zeros((shape[0], shape[1]), dtype=np.float32)
	im_mip = ax_mip.imshow(
		init_2d,
		origin="lower",
		cmap="hot",
		vmin=0.0,
		vmax=args.clip_max,
		interpolation="nearest",
	)
	fig.colorbar(im_mip, ax=ax_mip, fraction=0.046, pad=0.04, label="CPS")

	ax_mip.set_title("Max Projection (Z)")
	ax_voxel.set_title("3D Voxels")
	ax_mip.set_xlabel("Y")
	ax_mip.set_ylabel("X")
	ax_voxel.set_xlabel("X")
	ax_voxel.set_ylabel("Y")
	ax_voxel.set_zlabel("Z")
	ax_voxel.view_init(elev=args.voxel_elev, azim=args.voxel_azim)
	if hasattr(ax_voxel, "set_box_aspect"):
		ax_voxel.set_box_aspect((shape[0], shape[1], shape[2]))

	status = fig.text(0.02, 0.01, "Waiting for data...", fontsize=9)

	logger.info(
		"Visualizer started. endpoint=%s shape=%s voxel_percentile=%.1f",
		args.endpoint,
		shape,
		voxel_percentile,
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
			vmax_display = args.clip_max

			if not args.fixed_scale:
				frame_max = float(np.max(grid))
				if frame_max > 0.0:
					auto_vmax = float(np.percentile(grid, 99.0))
					auto_vmax = max(args.min_clip_max, auto_vmax)
				else:
					auto_vmax = args.min_clip_max
				vmax_display = auto_vmax
				im_mip.set_clim(0.0, auto_vmax)

			im_mip.set_data(mip)

			positive = grid[grid > 0.0]
			if positive.size == 0:
				occupancy = np.zeros(shape, dtype=bool)
				threshold = 0.0
			else:
				threshold = float(np.percentile(positive, voxel_percentile))
				occupancy = grid >= threshold

				if args.voxel_max_cubes > 0 and int(np.count_nonzero(occupancy)) > args.voxel_max_cubes:
					flat = grid.reshape(-1)
					k = max(0, flat.size - args.voxel_max_cubes)
					top_threshold = float(np.partition(flat, k)[k])
					threshold = max(threshold, top_threshold)
					occupancy = grid >= threshold

			norm = np.clip(grid / max(vmax_display, 1e-6), 0.0, 1.0)
			facecolors = plt.cm.inferno(norm)

			ax_voxel.cla()
			ax_voxel.set_title("3D Voxels")
			ax_voxel.set_xlabel("X")
			ax_voxel.set_ylabel("Y")
			ax_voxel.set_zlabel("Z")
			ax_voxel.view_init(elev=args.voxel_elev, azim=args.voxel_azim)
			ax_voxel.set_xlim(0, shape[0])
			ax_voxel.set_ylim(0, shape[1])
			ax_voxel.set_zlim(0, shape[2])
			if hasattr(ax_voxel, "set_box_aspect"):
				ax_voxel.set_box_aspect((shape[0], shape[1], shape[2]))

			if np.any(occupancy):
				ax_voxel.voxels(
					occupancy,
					facecolors=facecolors,
					edgecolor="k",
					linewidth=0.05,
					shade=True,
				)
			else:
				ax_voxel.text2D(0.05, 0.95, "No non-zero voxels", transform=ax_voxel.transAxes)

			frames += 1
			now = time.time()
			elapsed = max(1e-6, now - last_fps_time)
			fps = frames / elapsed
			status.set_text(
				f"frames={frames} | fps={fps:.1f} | max={float(np.max(grid)):.4f} | voxels={int(np.count_nonzero(occupancy))} | th={threshold:.4f}"
			)

			fig.canvas.draw_idle()
			fig.canvas.flush_events()

		plt.pause(0.001)


if __name__ == "__main__":
	main()

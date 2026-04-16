# indoor_drone_sim/radmapper.py
# Helper script and ROS2 node for generating grid map for radiation.

import csv
import logging
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
import time
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


class RadMapper():
    def __init__(
        self,
        room_width,
        room_length,
        room_ceil,
        debug_msg = False,
        voxel_size = 0.5,
        prediction_interval_sec = 10.0,
        prediction_kernel_sigma_m = 1.0,
    ) -> None:
        self.room_width = float(room_width)
        self.room_length = float(room_length)
        self.room_ceil = float(room_ceil)
        self.voxel_size = float(voxel_size)
        self.prediction_interval_sec = float(prediction_interval_sec)
        self.prediction_kernel_sigma_m = float(prediction_kernel_sigma_m)
        if self.voxel_size <= 0.0:
            raise ValueError("voxel_size must be > 0")
        if self.prediction_interval_sec <= 0.0:
            raise ValueError("prediction_interval_sec must be > 0")
        if self.prediction_kernel_sigma_m <= 0.0:
            raise ValueError("prediction_kernel_sigma_m must be > 0")

        self.grid_width = max(1, int(math.ceil(self.room_width / self.voxel_size)))
        self.grid_length = max(1, int(math.ceil(self.room_length / self.voxel_size)))
        self.grid_ceil = max(1, int(math.ceil(self.room_ceil / self.voxel_size)))

        self.debug_msg = debug_msg
        self.logger = logging.getLogger('RadMapper')
        if self.debug_msg and not self.logger.hasHandlers():
            logging.basicConfig(
                level=logging.INFO,
                format='[%(levelname)s] %(name)s: %(message)s',
                force=True,
            )

        self.geiger_gridmap = np.zeros((self.grid_width, self.grid_length, self.grid_ceil), dtype=np.float32)
        self.grid_sum = np.zeros((self.grid_width, self.grid_length, self.grid_ceil), dtype=np.float32)
        self.grid_count = np.zeros((self.grid_width, self.grid_length, self.grid_ceil), dtype=np.int32)
        self.last_prediction_timestamp = None
        self.geigerstore = []
        self.positionstore = []
        self.fig = None
        self.ax_live = None
        self.ax_pred = None
        self.cbar_live = None
        self.cbar_pred = None
        self.mappable_live = None
        self.mappable_pred = None
        self.last_position = (-1, -1, -1)
        self.latest_timestamp = None

    def _world_to_grid(self, x, y, z):
        gx = int(np.floor(float(x) / self.voxel_size))
        gy = int(np.floor(float(y) / self.voxel_size))
        gz = int(np.floor(float(z) / self.voxel_size))
        return gx, gy, gz

    def _in_grid_bounds(self, gx, gy, gz):
        return (
            0 <= gx < self.grid_width
            and 0 <= gy < self.grid_length
            and 0 <= gz < self.grid_ceil
        )

    def _apply_axes_style(self, ax, title_text):
        ax.set_title(title_text)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_xlim(0, self.room_width)
        ax.set_ylim(0, self.room_length)
        ax.set_zlim(0, self.room_ceil)
        if hasattr(ax, 'set_box_aspect'):
            # Keep metric ratio consistent so 1m on each axis renders equally.
            ax.set_box_aspect((self.room_width, self.room_length, self.room_ceil))

    def _build_live_grid(self):
        live_grid = np.zeros_like(self.grid_sum, dtype=np.float32)
        observed = self.grid_count > 0
        if np.any(observed):
            live_grid[observed] = self.grid_sum[observed] / self.grid_count[observed].astype(np.float32)
        return live_grid

    def _draw_grid_panel(self, ax, grid, title_text, draw_drone, cbar_attr, mappable_attr):
        filled = grid > 0.0
        self._apply_axes_style(ax, title_text)

        if not np.any(filled):
            return

        cmap = LinearSegmentedColormap.from_list(
            'geiger_gradient',
            [
                (0.0, 'green'),
                (0.04, 'xkcd:yellowgreen'),
                (0.2, 'yellow'),
                (0.4, 'red'),
                (1.0, 'xkcd:maroon')
            ],
        )
        norm = plt.Normalize(vmin=0.0, vmax=500.0, clip=True)
        alpha_norm = plt.Normalize(vmin=0.0, vmax=500.0, clip=True)

        filled_idx = np.argwhere(filled)
        cps_values = grid[filled]
        filled_xyz = self._grid_indices_to_world(filled_idx)

        # Render as point cloud to avoid voxel shell occluding internal structure.
        colors = cmap(norm(cps_values))
        colors[:, 3] = 0.2 + 0.8 * alpha_norm(cps_values)
        ax.scatter(
            filled_xyz[:, 0],
            filled_xyz[:, 1],
            filled_xyz[:, 2],
            c=colors,
            s=22,
            marker='o',
            depthshade=False,
            zorder=5,
        )

        mappable = getattr(self, mappable_attr)
        if mappable is None:
            mappable = plt.cm.ScalarMappable(norm=norm, cmap=cmap)
            mappable.set_array([])
            setattr(self, mappable_attr, mappable)
        else:
            mappable.set_norm(norm)
            mappable.set_cmap(cmap)

        cbar = getattr(self, cbar_attr)
        if cbar is None:
            cbar = self.fig.colorbar(mappable, ax=ax, pad=0.08, fraction=0.046)
            cbar.set_label('Geiger CPS')
            setattr(self, cbar_attr, cbar)
        else:
            cbar.update_normal(mappable)

        cbar.set_ticks([0.0, 100.0, 200.0, 500.0])
        cbar.set_ticklabels(['0', '100', '200', '500+'])

        if draw_drone:
            drone_x, drone_y, drone_z = self.last_position
            ax.scatter(drone_x, drone_y, drone_z, color='blue', s=100, label='Drone Position', zorder=10)
            ax.legend(loc='upper right')

    def _grid_indices_to_world(self, idx):
        # Map voxel index to center point in world coordinates.
        return (idx.astype(np.float32) + 0.5) * self.voxel_size

    def _run_batch_prediction(self, timestamp):
        observed_mask = self.grid_count > 0
        if not np.any(observed_mask):
            return False

        observed_idx = np.argwhere(observed_mask)
        observed_rates = self.grid_sum[observed_mask] / self.grid_count[observed_mask].astype(np.float32)
        observed_xyz = self._grid_indices_to_world(observed_idx)

        all_idx = np.indices(self.geiger_gridmap.shape, dtype=np.float32)
        all_idx = np.stack(all_idx, axis=-1).reshape(-1, 3)
        all_xyz = self._grid_indices_to_world(all_idx)

        sigma2 = self.prediction_kernel_sigma_m * self.prediction_kernel_sigma_m
        diff = all_xyz[:, None, :] - observed_xyz[None, :, :]
        dist2 = np.sum(diff * diff, axis=2)
        weights = np.exp(-0.5 * dist2 / sigma2).astype(np.float32)

        weighted_sum = np.matmul(weights, observed_rates)
        weight_norm = np.sum(weights, axis=1)
        pred_flat = np.divide(weighted_sum, weight_norm + 1e-6, dtype=np.float32)

        pred_grid = pred_flat.reshape(self.geiger_gridmap.shape)
        # Keep measured voxels exact and only smooth the unobserved region.
        pred_grid[observed_mask] = observed_rates
        self.geiger_gridmap = pred_grid.astype(np.float32)
        self.last_prediction_timestamp = timestamp

        if self.debug_msg:
            self.logger.info(
                "Batch prediction updated: obs_voxels=%d, sigma=%.2fm",
                int(observed_idx.shape[0]),
                self.prediction_kernel_sigma_m,
            )
        return True

    def report_geiger(self, timestamp, geiger_cps):
        self.geigerstore.append((timestamp, geiger_cps, False, -1, -1, -1))
        self.latest_timestamp = timestamp
        if self.debug_msg:
            self.logger.info(f"Received geiger reading: timestamp={timestamp:.3f}, cps={geiger_cps:.2f}")
        # Don't know the actual pos yet due to async logging, will fill in later

    def report_position(self, timestamp, x, y, z, yaw=None):
        if self.debug_msg:
            self.logger.info(f"Received position reading: timestamp={timestamp:.3f}, x={x:.2f}, y={y:.2f}, z={z:.2f}")
        self.latest_timestamp = timestamp
        self.positionstore.append((timestamp, x, y, z))
        grid_updated = False
        self.last_position = (x, y, z)
        # Find any geiger readings that haven't been matched to a position yet, and assign them by interpolation
        for i, (g_timestamp, geiger_cps, matched, _, _, _) in enumerate(self.geigerstore):
            if not matched:
                if g_timestamp <= timestamp:
                    # Find the previous position reading
                    prev_pos = None
                    for j in range(len(self.positionstore)-2, -1, -1):
                        if self.positionstore[j][0] <= g_timestamp:
                            prev_pos = self.positionstore[j]
                            break
                    if prev_pos is not None:
                        # Interpolate position
                        t1, x1, y1, z1 = prev_pos
                        t2, x2, y2, z2 = timestamp, x, y, z
                        ratio = (g_timestamp - t1) / (t2 - t1) if t2 != t1 else 0.0
                        interp_x = x1 + ratio * (x2 - x1)
                        interp_y = y1 + ratio * (y2 - y1)
                        interp_z = z1 + ratio * (z2 - z1)
                        self.geigerstore[i] = (g_timestamp, geiger_cps, True, interp_x, interp_y, interp_z)
                        gx, gy, gz = self._world_to_grid(interp_x, interp_y, interp_z)
                        # Look for the correct geiger grid to assign to
                        if self._in_grid_bounds(gx, gy, gz):
                            self.grid_sum[gx][gy][gz] += geiger_cps
                            self.grid_count[gx][gy][gz] += 1
                            grid_updated = True

        prediction_due = (
            self.last_prediction_timestamp is None
            or (timestamp - self.last_prediction_timestamp) >= self.prediction_interval_sec
        )
        prediction_updated = False
        if grid_updated and prediction_due:
            prediction_updated = self._run_batch_prediction(timestamp)

        if grid_updated and self.fig is not None:
            self._redraw_plot()

    def _redraw_plot(self):
        self.ax_live.cla()
        self.ax_pred.cla()

        timestamp_suffix = 't=N/A'
        if self.latest_timestamp is not None:
            timestamp_suffix = f't={self.latest_timestamp:.3f}s'

        live_grid = self._build_live_grid()
        pred_grid = self.geiger_gridmap

        self._draw_grid_panel(
            self.ax_live,
            live_grid,
            f'Real-time Observed CPS | {timestamp_suffix}',
            True,
            'cbar_live',
            'mappable_live',
        )
        self._draw_grid_panel(
            self.ax_pred,
            pred_grid,
            f'Predicted CPS (batch) | {timestamp_suffix}',
            True,
            'cbar_pred',
            'mappable_pred',
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

    def visualization_matplotlib(self):
        # Visualize the 3D geiger gridmap using matplotlib
        if self.fig is None or not plt.fignum_exists(self.fig.number):
            plt.ion()
            self.fig = plt.figure(figsize=(16, 7))
            self.ax_live = self.fig.add_subplot(121, projection='3d')
            self.ax_pred = self.fig.add_subplot(122, projection='3d')
            self.cbar_live = None
            self.cbar_pred = None
            self.mappable_live = None
            self.mappable_pred = None

        self._redraw_plot()
        plt.show(block=False)


class RadMapperNode(Node):
    def __init__(self) -> None:
        super().__init__('radmapper_node')

        self.declare_parameter('room_width', 8.0)
        self.declare_parameter('room_height', 10.0)
        self.declare_parameter('ceiling_z', 3.0)

        room_width = float(self.get_parameter('room_width').value)
        room_height = float(self.get_parameter('room_height').value)
        ceiling_z = float(self.get_parameter('ceiling_z').value)

        self.mapper = RadMapper(room_width, room_height, ceiling_z)
        self.mapper.visualization_matplotlib()

        self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.create_subscription(Float32, '/geiger_count', self.on_geiger, 10)

        self.get_logger().info(
            'radmapper_node started: subscribing /odom and /geiger_count'
        )

    @staticmethod
    def _stamp_to_float(sec: int, nanosec: int) -> float:
        return float(sec) + float(nanosec) * 1e-9

    def on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp
        timestamp = self._stamp_to_float(stamp.sec, stamp.nanosec)
        if timestamp <= 0.0:
            now = self.get_clock().now().to_msg()
            timestamp = self._stamp_to_float(now.sec, now.nanosec)

        position = msg.pose.pose.position
        self.mapper.report_position(timestamp, float(position.x), float(position.y), float(position.z))

    def on_geiger(self, msg: Float32) -> None:
        now = self.get_clock().now().to_msg()
        timestamp = self._stamp_to_float(now.sec, now.nanosec)
        self.mapper.report_geiger(timestamp, float(msg.data))


def run_ros_node() -> None:
    rclpy.init()
    node = RadMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def run_offline_demo(
    geiger_csv_path: str = 'data/geiger.csv',
    state_csv_path: str = 'data/state.csv',
    room_width: float = 8.0,
    room_length: float = 10.0,
    room_ceil: float = 3.0,
    playback_sleep_sec: float = 0.01,
) -> None:
    logger = logging.getLogger('radmapper_demo')
    if not logger.handlers:
        logging.basicConfig(
            level=logging.INFO,
            format='[%(levelname)s] %(name)s: %(message)s',
            force=True,
        )
    mapper = RadMapper(room_width, room_length, room_ceil, False)
    mapper.visualization_matplotlib()
    plt.show(block=False)
    logger.info("Starting offline demo playback.")
    with open(geiger_csv_path, 'r') as geiger_file:
        geiger_reader = csv.DictReader(geiger_file)
        for row in geiger_reader:
            timestamp = float(row['timestamp'])
            geiger_cps = float(row['geiger_cps'])
            mapper.report_geiger(timestamp, geiger_cps)
    with open(state_csv_path, 'r') as state_file:
        state_reader = csv.DictReader(state_file)
        for row in state_reader:
            timestamp = float(row['timestamp'])
            x = float(row['x'])
            y = float(row['y'])
            z = float(row['z'])
            mapper.report_position(timestamp, x, y, z)
            if playback_sleep_sec > 0.0:
                time.sleep(playback_sleep_sec)
    logger.info("Demo finished.")
    time.sleep(5)


def main():
    run_ros_node()


def demo_main():
    run_offline_demo()


if __name__ == '__main__':
    main()
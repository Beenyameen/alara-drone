# indoor_drone_sim/radmapper.py
# Helper script and ROS2 node for generating grid map for radiation.

import csv
import logging
import numpy as np
import time
import math


class RadMapper:
    def __init__(
        self,
        room_width,
        room_length,
        room_ceil,
        debug_msg=False,
        voxel_size=0.5,
        prediction_interval_sec=10.0,
        prediction_kernel_sigma_m=1.0,
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
        self.offset_x_m = (self.grid_width * self.voxel_size) * 0.5
        self.offset_y_m = (self.grid_length * self.voxel_size) * 0.5
        self.offset_z_m = (self.grid_ceil * self.voxel_size) * 0.5

        self.debug_msg = debug_msg
        self.logger = logging.getLogger("RadMapper")
        if self.debug_msg and not self.logger.hasHandlers():
            logging.basicConfig(
                level=logging.INFO,
                format="[%(levelname)s] %(name)s: %(message)s",
                force=True,
            )

        self.geiger_gridmap = np.zeros(
            (self.grid_width, self.grid_length, self.grid_ceil), dtype=np.float32
        )
        self.grid_sum = np.zeros(
            (self.grid_width, self.grid_length, self.grid_ceil), dtype=np.float32
        )
        self.grid_count = np.zeros(
            (self.grid_width, self.grid_length, self.grid_ceil), dtype=np.int32
        )
        self.last_prediction_timestamp = None
        self.geigerstore = []
        self.positionstore = []
        self.latest_timestamp = None

    def _world_to_grid(self, x, y, z):
        gx = int(np.floor((float(x) + self.offset_x_m) / self.voxel_size))
        gy = int(np.floor((float(y) + self.offset_y_m) / self.voxel_size))
        gz = int(np.floor((float(z) + self.offset_z_m) / self.voxel_size))
        return gx, gy, gz

    def _in_grid_bounds(self, gx, gy, gz):
        return (
            0 <= gx < self.grid_width
            and 0 <= gy < self.grid_length
            and 0 <= gz < self.grid_ceil
        )

    def _grid_indices_to_world(self, idx):
        # Map voxel index to center point in world coordinates.
        points = (idx.astype(np.float32) + 0.5) * self.voxel_size
        points[:, 0] -= self.offset_x_m
        points[:, 1] -= self.offset_y_m
        points[:, 2] -= self.offset_z_m
        return points

    def _run_batch_prediction(self, timestamp):
        observed_mask = self.grid_count > 0
        if not np.any(observed_mask):
            return False

        observed_idx = np.argwhere(observed_mask)
        observed_rates = self.grid_sum[observed_mask] / self.grid_count[
            observed_mask
        ].astype(np.float32)
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

    def report_geiger(self, timestamp, geiger_usv):
        self.geigerstore.append((timestamp, geiger_usv, False, -1, -1, -1))
        self.latest_timestamp = timestamp
        if self.debug_msg:
            self.logger.info(
                f"Received geiger reading: timestamp={timestamp:.3f}, cps={geiger_usv:.2f}"
            )
        # Don't know the actual pos yet due to async logging, will fill in later

    def report_position(self, timestamp, x, y, z, yaw=None):
        if self.debug_msg:
            self.logger.info(
                f"Received position reading: timestamp={timestamp:.3f}, x={x:.2f}, y={y:.2f}, z={z:.2f}"
            )
        self.latest_timestamp = timestamp
        self.positionstore.append((timestamp, x, y, z))
        grid_updated = False
        # Find any geiger readings that haven't been matched to a position yet, and assign them by interpolation
        for i, (g_timestamp, geiger_cps, matched, _, _, _) in enumerate(
            self.geigerstore
        ):
            if not matched:
                if g_timestamp <= timestamp:
                    # Find the previous position reading
                    prev_pos = None
                    for j in range(len(self.positionstore) - 2, -1, -1):
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
                        self.geigerstore[i] = (
                            g_timestamp,
                            geiger_cps,
                            True,
                            interp_x,
                            interp_y,
                            interp_z,
                        )
                        gx, gy, gz = self._world_to_grid(interp_x, interp_y, interp_z)
                        # Look for the correct geiger grid to assign to
                        if self._in_grid_bounds(gx, gy, gz):
                            self.grid_sum[gx][gy][gz] += geiger_cps
                            self.grid_count[gx][gy][gz] += 1
                            grid_updated = True

        # prediction_due = (
        #     self.last_prediction_timestamp is None
        #     or (timestamp - self.last_prediction_timestamp)
        #     >= self.prediction_interval_sec
        # )
        # prediction_updated = False
        # if grid_updated and prediction_due:
        #     prediction_updated = self._run_batch_prediction(timestamp)

    def get_prediction_grid(self):
        return self.geiger_gridmap.astype(np.float32)



if __name__ == "__main__":
    print("This is a helper module for indoor_drone_sim. It provides the RadMapper class for processing geiger counter and position data to build a 3D grid map of radiation levels. It is not meant to be run directly.")

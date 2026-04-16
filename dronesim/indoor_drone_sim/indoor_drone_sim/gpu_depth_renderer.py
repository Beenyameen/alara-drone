from __future__ import annotations

import math
import site
import sys
from pathlib import Path
from typing import Sequence, Tuple

import numpy as np


class GpuDepthRenderer:
    """Offscreen GPU renderer producing RGB and metric depth frames.

    This is an optional backend used by sim_world_node when camera_backend=gpu.
    It relies on pyrender + trimesh availability and a GPU-capable OpenGL/EGL stack.
    """

    def __init__(
        self,
        room_width: float,
        room_height: float,
        floor_z: float,
        ceiling_z: float,
        boxes: Sequence[Tuple[float, float, float, float, float, float]],
        cylinders: Sequence[Tuple[float, float, float, float, float]],
    ) -> None:
        self._ensure_extra_site_packages()
        try:
            import pyrender  # type: ignore
            import trimesh  # type: ignore
        except Exception as exc:  # pragma: no cover - depends on local runtime
            raise RuntimeError(
                'GPU renderer requires pyrender and trimesh; install both to enable camera_backend=gpu. '
                f'Original import error: {exc}'
            ) from exc

        self._pyrender = pyrender
        self._trimesh = trimesh

        self._scene = pyrender.Scene(bg_color=np.array([220, 220, 225, 255], dtype=np.uint8), ambient_light=[0.6, 0.6, 0.62])
        self._renderer = None
        self._renderer_size = (0, 0)
        self._camera_node = None

        self._build_static_scene(room_width, room_height, floor_z, ceiling_z, boxes, cylinders)

    def _ensure_extra_site_packages(self) -> None:
        """Allow ROS system-python nodes to reuse project venv packages when present."""
        py_ver = f'python{sys.version_info.major}.{sys.version_info.minor}'
        candidates = []

        env_venv = Path(sys.prefix)
        candidates.append(env_venv / 'lib' / py_ver / 'site-packages')

        cwd = Path.cwd()
        candidates.append(cwd / '.venv' / 'lib' / py_ver / 'site-packages')
        candidates.append(cwd.parent / '.venv' / 'lib' / py_ver / 'site-packages')

        module_dir = Path(__file__).resolve().parent
        for parent in module_dir.parents:
            candidates.append(parent / '.venv' / 'lib' / py_ver / 'site-packages')

        seen = set()
        for c in candidates:
            if c in seen:
                continue
            seen.add(c)
            if c.exists():
                site.addsitedir(str(c))

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.delete()
            self._renderer = None

    def render(
        self,
        x: float,
        y: float,
        z: float,
        yaw: float,
        width: int,
        height: int,
        hfov_rad: float,
        vfov_rad: float,
        max_range_m: float,
    ) -> Tuple[np.ndarray, np.ndarray]:
        self._ensure_renderer(width, height)

        fx = width / (2.0 * math.tan(0.5 * hfov_rad))
        fy = height / (2.0 * math.tan(0.5 * vfov_rad))
        cx = width * 0.5
        cy = height * 0.5

        camera = self._pyrender.IntrinsicsCamera(fx=fx, fy=fy, cx=cx, cy=cy, znear=0.05, zfar=max_range_m)
        pose = self._camera_pose(x, y, z, yaw)

        if self._camera_node is not None:
            self._scene.remove_node(self._camera_node)
        self._camera_node = self._scene.add(camera, pose=pose)

        color_rgba, depth_m = self._renderer.render(self._scene)

        color_rgb = np.ascontiguousarray(color_rgba[:, :, :3], dtype=np.uint8)
        depth_m = np.asarray(depth_m, dtype=np.float32)

        depth_m = np.where(np.isfinite(depth_m), depth_m, 0.0)
        depth_m = np.clip(depth_m, 0.0, max_range_m)
        depth_mm = np.ascontiguousarray(np.round(depth_m * 1000.0).astype(np.uint16))

        return depth_mm, color_rgb

    def _ensure_renderer(self, width: int, height: int) -> None:
        if self._renderer is not None and self._renderer_size == (width, height):
            return
        if self._renderer is not None:
            self._renderer.delete()
            self._renderer = None

        self._renderer = self._pyrender.OffscreenRenderer(width, height)
        self._renderer_size = (width, height)

    def _build_static_scene(
        self,
        room_width: float,
        room_height: float,
        floor_z: float,
        ceiling_z: float,
        boxes: Sequence[Tuple[float, float, float, float, float, float]],
        cylinders: Sequence[Tuple[float, float, float, float, float]],
    ) -> None:
        pyrender = self._pyrender
        trimesh = self._trimesh

        def add_box_with_material(
            extents: Tuple[float, float, float],
            center: Tuple[float, float, float],
            color: Tuple[int, int, int],
            metallic: float = 0.0,
            roughness: float = 0.5,
        ) -> None:
            mesh = trimesh.creation.box(extents=extents)
            # Use PBR material for richer appearance
            mat = pyrender.MetallicRoughnessMaterial(
                baseColorFactor=np.array([color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, 1.0]),
                metallicFactor=metallic,
                roughnessFactor=roughness,
            )
            pose = np.eye(4, dtype=np.float32)
            pose[:3, 3] = np.array(center, dtype=np.float32)
            self._scene.add(pyrender.Mesh.from_trimesh(mesh, material=mat, smooth=True), pose=pose)

        def add_cylinder_with_material(
            radius: float,
            height: float,
            center: Tuple[float, float, float],
            color: Tuple[int, int, int],
            metallic: float = 0.0,
            roughness: float = 0.5,
        ) -> None:
            mesh = trimesh.creation.cylinder(radius=max(0.02, radius), height=max(0.02, height), sections=32)
            mat = pyrender.MetallicRoughnessMaterial(
                baseColorFactor=np.array([color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, 1.0]),
                metallicFactor=metallic,
                roughnessFactor=roughness,
            )
            pose = np.eye(4, dtype=np.float32)
            pose[:3, 3] = np.array(center, dtype=np.float32)
            self._scene.add(pyrender.Mesh.from_trimesh(mesh, material=mat, smooth=True), pose=pose)

        wall_h = max(0.1, ceiling_z - floor_z)
        # Floor: matte
        add_box_with_material(
            (room_width, room_height, 0.02),
            (0.5 * room_width, 0.5 * room_height, floor_z - 0.01),
            (90, 90, 90),
            metallic=0.0,
            roughness=0.8,
        )
        # Ceiling: slightly glossy
        add_box_with_material(
            (room_width, room_height, 0.02),
            (0.5 * room_width, 0.5 * room_height, ceiling_z + 0.01),
            (190, 195, 205),
            metallic=0.0,
            roughness=0.5,
        )

        # Add grid lines to floor and ceiling
        grid_spacing = 1.0
        grid_line_thickness = 0.005

        # Floor grid: dark lines
        self._add_grid_lines(
            room_width, room_height, floor_z + 0.015,
            grid_spacing, grid_line_thickness,
            (50, 50, 50), 0.0, 0.8
        )

        # Ceiling grid: light lines
        self._add_grid_lines(
            room_width, room_height, ceiling_z - 0.015,
            grid_spacing, grid_line_thickness,
            (220, 220, 225), 0.0, 0.5
        )
        # Walls: matte
        add_box_with_material(
            (room_width, 0.03, wall_h),
            (0.5 * room_width, 0.0, floor_z + 0.5 * wall_h),
            (140, 145, 160),
            metallic=0.0,
            roughness=0.7,
        )
        add_box_with_material(
            (room_width, 0.03, wall_h),
            (0.5 * room_width, room_height, floor_z + 0.5 * wall_h),
            (140, 145, 160),
            metallic=0.0,
            roughness=0.7,
        )
        add_box_with_material(
            (0.03, room_height, wall_h),
            (0.0, 0.5 * room_height, floor_z + 0.5 * wall_h),
            (140, 145, 160),
            metallic=0.0,
            roughness=0.7,
        )
        add_box_with_material(
            (0.03, room_height, wall_h),
            (room_width, 0.5 * room_height, floor_z + 0.5 * wall_h),
            (140, 145, 160),
            metallic=0.0,
            roughness=0.7,
        )

        # Objects
        for x_min, y_min, x_max, y_max, z_bottom, z_top in boxes:
            sx = max(0.02, x_max - x_min)
            sy = max(0.02, y_max - y_min)
            sz = max(0.02, z_top - z_bottom)
            cx = 0.5 * (x_min + x_max)
            cy = 0.5 * (y_min + y_max)
            cz = floor_z + 0.5 * (z_bottom + z_top)
            # Low objects: more vivid, slight metallic sheen
            if z_bottom <= 1.0:
                color = (70, 130, 220)
                metallic = 0.0
                roughness = 0.95
            else:
                color = (170, 174, 182)
                metallic = 0.0
                roughness = 0.9
            add_box_with_material((sx, sy, sz), (cx, cy, cz), color, metallic, roughness)

        for cx, cy, radius, z_bottom, z_top in cylinders:
            h = max(0.02, z_top - z_bottom)
            center = (cx, cy, floor_z + 0.5 * (z_bottom + z_top))
            # Low objects: warm tone with metallic finish
            if z_bottom <= 1.0:
                color = (225, 110, 55)
                metallic = 0.0
                roughness = 0.95
            else:
                color = (176, 178, 182)
                metallic = 0.0
                roughness = 0.92
            add_cylinder_with_material(radius, h, center, color, metallic, roughness)

        # Three-point lighting
        # Key light: main directional light
        key_light = pyrender.DirectionalLight(color=np.array([1.0, 0.98, 0.95]), intensity=5.0)
        key_pose = np.eye(4, dtype=np.float32)
        key_pose[:3, :3] = self._rot_z(math.radians(22.0)) @ self._rot_x(math.radians(-55.0))
        self._scene.add(key_light, pose=key_pose)

        # Fill light: softer opposite direction
        fill_light = pyrender.DirectionalLight(color=np.array([0.85, 0.88, 0.95]), intensity=1.5)
        fill_pose = np.eye(4, dtype=np.float32)
        fill_pose[:3, :3] = self._rot_z(math.radians(200.0)) @ self._rot_x(math.radians(-35.0))
        self._scene.add(fill_light, pose=fill_pose)

        # Rim light: from top to add silhouette
        rim_light = pyrender.DirectionalLight(color=np.array([0.9, 0.92, 1.0]), intensity=2.0)
        rim_pose = np.eye(4, dtype=np.float32)
        rim_pose[:3, :3] = self._rot_z(math.radians(-90.0)) @ self._rot_x(math.radians(-70.0))
        self._scene.add(rim_light, pose=rim_pose)

    def _add_grid_lines(
        self,
        width: float,
        height: float,
        z: float,
        spacing: float,
        thickness: float,
        color: Tuple[int, int, int],
        metallic: float = 0.0,
        roughness: float = 0.8,
    ) -> None:
        """Add grid lines to floor or ceiling."""
        pyrender = self._pyrender
        trimesh = self._trimesh

        mat = pyrender.MetallicRoughnessMaterial(
            baseColorFactor=np.array([color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, 1.0]),
            metallicFactor=metallic,
            roughnessFactor=roughness,
        )

        # Vertical lines (parallel to Y axis)
        x = spacing
        while x < width:
            center_x = x
            line_box = trimesh.creation.box(extents=(thickness, height, thickness))
            pose = np.eye(4, dtype=np.float32)
            pose[:3, 3] = np.array([center_x, 0.5 * height, z], dtype=np.float32)
            self._scene.add(pyrender.Mesh.from_trimesh(line_box, material=mat, smooth=False), pose=pose)
            x += spacing

        # Horizontal lines (parallel to X axis)
        y = spacing
        while y < height:
            center_y = y
            line_box = trimesh.creation.box(extents=(width, thickness, thickness))
            pose = np.eye(4, dtype=np.float32)
            pose[:3, 3] = np.array([0.5 * width, center_y, z], dtype=np.float32)
            self._scene.add(pyrender.Mesh.from_trimesh(line_box, material=mat, smooth=False), pose=pose)
            y += spacing

    def _camera_pose(self, x: float, y: float, z: float, yaw: float) -> np.ndarray:
        # World axes: X forward, Y left, Z up.
        # OpenGL camera axes used by pyrender: +X right, +Y up, looks toward -Z.
        forward = np.array([math.cos(yaw), math.sin(yaw), 0.0], dtype=np.float32)
        left = np.array([-math.sin(yaw), math.cos(yaw), 0.0], dtype=np.float32)
        up = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        right = -left
        back = -forward

        pose = np.eye(4, dtype=np.float32)
        pose[:3, 0] = right
        pose[:3, 1] = up
        pose[:3, 2] = back
        pose[:3, 3] = np.array([x, y, z], dtype=np.float32)
        return pose

    @staticmethod
    def _rot_x(angle: float) -> np.ndarray:
        c = math.cos(angle)
        s = math.sin(angle)
        return np.array(
            [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]],
            dtype=np.float32,
        )

    @staticmethod
    def _rot_z(angle: float) -> np.ndarray:
        c = math.cos(angle)
        s = math.sin(angle)
        return np.array(
            [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )

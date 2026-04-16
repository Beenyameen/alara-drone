#type: ignore
import atexit
import fcntl
import math
import os
import random
import struct
import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Protocol
from cv_bridge import CvBridge
import cv2

import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

try:
    from .gpu_depth_renderer import GpuDepthRenderer as _GpuDepthRenderer
except Exception:
    _GpuDepthRenderer = None

bridge = CvBridge()

@dataclass
class CircleObstacle:
    cx: float
    cy: float
    radius: float


@dataclass
class BoxObstacle:
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    z_bottom: float
    z_top: float


@dataclass
class CylinderObstacle:
    cx: float
    cy: float
    radius: float
    z_bottom: float
    z_top: float


@dataclass
class RayHit:
    distance: float
    x: float
    y: float
    z: float
    nx: float
    ny: float
    nz: float
    material: str


class _RendererProtocol(Protocol):
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
    ) -> tuple[np.ndarray, np.ndarray]:
        ...

    def close(self) -> None:
        ...


class SimWorldNode(Node):
    def __init__(self) -> None:
        super().__init__('sim_world_node')

        self.logger = self.get_logger()

        self.declare_parameter('single_instance', True)
        self._instance_lock_file = None
        if bool(self.get_parameter('single_instance').value):
            self._acquire_instance_lock()



        self.declare_parameter('room_width', 8.0)
        self.declare_parameter('room_height', 10.0)
        self.declare_parameter('depth_rate_hz', 30.0)
        self.declare_parameter('state_rate_hz', 30.0)
        self.declare_parameter('geiger_rate_hz', 30.0)
        self.declare_parameter('depth_max_range', 12.0)
        self.declare_parameter('depth_width', 320)
        self.declare_parameter('depth_height', 240)
        self.declare_parameter('depth_width_gpu', 320)
        self.declare_parameter('depth_height_gpu', 240)
        self.declare_parameter('depth_hfov_deg', 87.0)
        self.declare_parameter('depth_vfov_deg', 58.0)
        self.declare_parameter('initial_x', 4.4)
        self.declare_parameter('initial_y', 0.65)
        self.declare_parameter('initial_z', 0.3)
        self.declare_parameter('initial_yaw', 1.5)
        self.declare_parameter('floor_z', 0.2)
        self.declare_parameter('ceiling_z', 3.0)
        self.declare_parameter('geiger_source_x', 4.0)
        self.declare_parameter('geiger_source_y', 5.0)
        self.declare_parameter('geiger_source_z', 1.2)
        self.declare_parameter('geiger_strength', 120.0)
        self.declare_parameter('geiger_background', 12.0)
        self.declare_parameter('wall_thickness', 0.1)
        self.declare_parameter('obstacle_height', 2.0)
        self.declare_parameter('run_without_rviz', True)
        self.declare_parameter('save_depth_images', False)
        self.declare_parameter('camera_backend', 'gpu')

        self.room_width = float(self.get_parameter('room_width').value)
        self.room_height = float(self.get_parameter('room_height').value)
        self.depth_max_range = float(self.get_parameter('depth_max_range').value)
        self.depth_rate_hz = float(self.get_parameter('depth_rate_hz').value)
        self.depth_hfov = math.radians(float(self.get_parameter('depth_hfov_deg').value))
        self.depth_vfov = math.radians(float(self.get_parameter('depth_vfov_deg').value))
        self.run_without_rviz = bool(self.get_parameter('run_without_rviz').value)
        self.save_depth_images = bool(self.get_parameter('save_depth_images').value)
        self.camera_backend = str(self.get_parameter('camera_backend').value or 'cpu').strip().lower()
        if self.camera_backend not in ('cpu', 'gpu'):
            self.get_logger().warning(
                f"Unsupported camera_backend='{self.camera_backend}', falling back to 'cpu'"
            )
            self.camera_backend = 'cpu'

        self.depth_width = int(self.get_parameter('depth_width').value)
        self.depth_height = int(self.get_parameter('depth_height').value)

        self.x = float(self.get_parameter('initial_x').value)
        self.y = float(self.get_parameter('initial_y').value)
        self.z = float(self.get_parameter('initial_z').value)
        self.yaw = float(self.get_parameter('initial_yaw').value)
        self.floor_z = float(self.get_parameter('floor_z').value)
        self.ceiling_z = float(self.get_parameter('ceiling_z').value)

        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_linear_z = 0.0
        self.cmd_angular_z = 0.0

        self.geiger_source_x = float(self.get_parameter('geiger_source_x').value)
        self.geiger_source_y = float(self.get_parameter('geiger_source_y').value)
        self.geiger_source_z = float(self.get_parameter('geiger_source_z').value)
        self.geiger_strength = float(self.get_parameter('geiger_strength').value)
        self.geiger_background = float(self.get_parameter('geiger_background').value)
        self.wall_thickness = float(self.get_parameter('wall_thickness').value)
        self.obstacle_height = float(self.get_parameter('obstacle_height').value)

        # Medical / Operating Room Layout (8m x 10m)
        self.boxes: List[BoxObstacle] = [
            # Patient Table (Center)
            BoxObstacle(3.2, 4.0, 4.8, 6.0, 0.0, 0.9),
            # C-Arm C-Section (Horizontal part above table)
            BoxObstacle(3.2, 4.5, 4.8, 5.5, 1.8, 2.3),
            # Cabinets (Left Wall)
            BoxObstacle(0.0, 1.0, 0.8, 9.0, 0.0, 2.2),
            # Monitor Cart (Right side)
            BoxObstacle(5.5, 4.5, 6.3, 5.3, 0.0, 1.6),
            # Monitor Displays (Above Cart)
            BoxObstacle(5.5, 4.5, 6.3, 5.3, 1.6, 2.1),
            # Anesthesia Machine (Top Right near head)
            BoxObstacle(5.5, 6.5, 6.5, 7.5, 0.0, 1.5),
            # Lead Shield (Portable, on floor)
            BoxObstacle(2.0, 5.0, 2.1, 6.0, 0.0, 1.8),
            # Instrument Table (Bottom Left)
            BoxObstacle(1.0, 1.0, 2.5, 1.8, 0.0, 1.0),
            # Ceiling Mounted Display Boom 1
            BoxObstacle(4.5, 5.5, 5.0, 6.0, 2.0, 2.5),
            # Ceiling Mounted Display Boom 2
            BoxObstacle(5.5, 3.5, 6.0, 4.0, 1.9, 2.4),
        ]
        self.cylinders: List[CylinderObstacle] = [
            # C-Arm Base / Stand (floor mounted near head)
            CylinderObstacle(4.0, 7.5, 0.4, 0.0, 2.5),
            # IV Pole
            CylinderObstacle(2.8, 6.5, 0.1, 0.0, 1.9),
            # Anesthesia Boom (Ceiling mounted column)
            CylinderObstacle(6.0, 8.0, 0.3, 1.5, 3.0),
            # Surgical Light 1 (Ceiling mounted)
            CylinderObstacle(4.0, 5.0, 0.4, 2.4, 2.6),
            # Surgical Light 2 (Ceiling mounted)
            CylinderObstacle(3.5, 4.5, 0.4, 2.4, 2.6),
        ]

        self._gpu_renderer: Optional[_RendererProtocol] = None
        if self.camera_backend == 'gpu':
            self._init_gpu_renderer()

        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 20)
        self.create_subscription(Bool, '/reset_position', self.reset_position, 10)
        self.cmd_linear_x = float(0.0)
        self.cmd_linear_y = float(0.0)
        self.cmd_linear_z = float(0.0)
        self.cmd_angular_z = float(0.0)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        depth_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.depth_image_pub = self.create_publisher(Image, '/depth/image_raw', depth_qos)
        self.depth_viz_pub = self.create_publisher(Image, '/depth/image_viz', depth_qos)
        self.depth_camera_info_pub = self.create_publisher(CameraInfo, '/depth/camera_info', depth_qos)
        self.depth_points_pub = self.create_publisher(PointCloud2, '/depth/points', depth_qos)
        self.geiger_pub = self.create_publisher(Float32, '/geiger_count', 10)
        geometry_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.geometry_pub = self.create_publisher(MarkerArray, '/world_geometry', geometry_qos)
        self.tf_broadcaster = TransformBroadcaster(self)

        state_period = 1.0 / float(self.get_parameter('state_rate_hz').value)
        depth_period = 1.0 / self.depth_rate_hz
        geiger_period = 1.0 / float(self.get_parameter('geiger_rate_hz').value)

        self.state_logfile = open('data/state.csv', 'w')
        self.geiger_logfile = open('data/geiger.csv', 'w')
        self.state_logfile.write('timestamp,x,y,z,yaw\n')
        self.geiger_logfile.write('timestamp,geiger_cps\n')
        self.last_depth_hash = None


        self.state_timer = self.create_timer(state_period, self.update_state)
        self.depth_timer = self.create_timer(depth_period, self.publish_depth)
        self.geiger_timer = self.create_timer(geiger_period, self.publish_geiger)
        if not self.run_without_rviz:
            self.geometry_timer = self.create_timer(1.0, self.publish_geometry)

        self.last_state_time = self.get_clock().now()
        if not self.run_without_rviz:
            self.publish_geometry()
        self.get_logger().info(
            'sim_world_node started: publishing /odom /tf /depth/image_raw /depth/image_viz /depth/camera_info '
            '/depth/points /geiger_count /world_geometry'
        )

    def _init_gpu_renderer(self) -> None:
        if _GpuDepthRenderer is None:
            self.get_logger().warning('camera_backend=gpu requested but GPU renderer module is unavailable; using cpu')
            self.camera_backend = 'cpu'
            return

        try:
            box_tuples = [
                (b.x_min, b.y_min, b.x_max, b.y_max, b.z_bottom, b.z_top)
                for b in self.boxes
            ]
            cylinder_tuples = [
                (c.cx, c.cy, c.radius, c.z_bottom, c.z_top)
                for c in self.cylinders
            ]
            self._gpu_renderer = _GpuDepthRenderer(
                room_width=self.room_width,
                room_height=self.room_height,
                floor_z=self.floor_z,
                ceiling_z=self.ceiling_z,
                boxes=box_tuples,
                cylinders=cylinder_tuples,
            )
            self.get_logger().info('GPU camera backend enabled')
        except Exception as exc:
            self._gpu_renderer = None
            self.camera_backend = 'cpu'
            self.get_logger().warning(
                f'camera_backend=gpu initialization failed ({exc}); falling back to cpu backend'
            )
        # Successful? Cramp up the res!
        self.depth_width = int(self.get_parameter('depth_width_gpu').value)
        self.depth_height = int(self.get_parameter('depth_height_gpu').value)

    def _acquire_instance_lock(self) -> None:
        lock_path = '/tmp/indoor_drone_sim_sim_world_node.lock'
        self._instance_lock_file = open(lock_path, 'w')
        try:
            fcntl.flock(self._instance_lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._instance_lock_file.seek(0)
            self._instance_lock_file.truncate()
            self._instance_lock_file.write(str(os.getpid()))
            self._instance_lock_file.flush()
            atexit.register(self._release_instance_lock)
        except OSError:
            self.get_logger().error(
                'Detected another running sim_world_node instance. '
                'Stop existing launch sessions before starting a new one.'
            )
            raise RuntimeError('sim_world_node single-instance lock acquisition failed')

    def _release_instance_lock(self) -> None:
        if self._instance_lock_file is None:
            return
        try:
            fcntl.flock(self._instance_lock_file.fileno(), fcntl.LOCK_UN)
        except OSError:
            pass
        try:
            self._instance_lock_file.close()
        except OSError:
            pass
        self._instance_lock_file = None

    def on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_linear_x = float(msg.linear.x)
        self.cmd_linear_y = float(msg.linear.y)
        self.cmd_linear_z = float(msg.linear.z)
        self.cmd_angular_z = float(msg.angular.z)

    def update_state(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_state_time).nanoseconds * 1e-9
        self.last_state_time = now
        if dt <= 0.0:
            return

        self.yaw += self.cmd_angular_z * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        # Apply body-frame planar velocity (x forward, y left) to world-frame position.
        self.x += (self.cmd_linear_x * math.cos(self.yaw) - self.cmd_linear_y * math.sin(self.yaw)) * dt
        self.y += (self.cmd_linear_x * math.sin(self.yaw) + self.cmd_linear_y * math.cos(self.yaw)) * dt
        self.z += self.cmd_linear_z * dt

        self.x = min(max(self.x, 0.2), self.room_width - 0.2)
        self.y = min(max(self.y, 0.2), self.room_height - 0.2)
        self.z = min(max(self.z, self.floor_z), self.ceiling_z)

        self.state_logfile.write(f'{now.to_msg().sec}.{now.to_msg().nanosec:09d},{self.x:.3f},{self.y:.3f},{self.z:.3f},{self.yaw:.3f}\n')
        self.publish_odom_and_tf(now, publish_tf=not self.run_without_rviz)
        if not self.run_without_rviz:
            self.publish_drone_marker(now)

    def publish_odom_and_tf(self, now, publish_tf: bool = True) -> None:
        quat_z = math.sin(self.yaw * 0.5)
        quat_w = math.cos(self.yaw * 0.5)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = self.z
        odom.pose.pose.orientation.z = quat_z
        odom.pose.pose.orientation.w = quat_w
        odom.twist.twist.linear.x = self.cmd_linear_x
        odom.twist.twist.linear.y = self.cmd_linear_y
        odom.twist.twist.linear.z = self.cmd_linear_z
        odom.twist.twist.angular.z = self.cmd_angular_z
        self.odom_pub.publish(odom)

        if publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = self.z
            transform.transform.rotation.z = quat_z
            transform.transform.rotation.w = quat_w
            self.tf_broadcaster.sendTransform(transform)

    def publish_depth(self) -> None:
        stamp = self.last_state_time.to_msg()
        frame_id = 'base_link'

        width = self.depth_width
        height = self.depth_height

        # self.logger.info(f"{width}x{height}")

        half_hfov = 0.5 * self.depth_hfov
        half_vfov = 0.5 * self.depth_vfov
        fx = width / (2.0 * math.tan(half_hfov))
        fy = height / (2.0 * math.tan(half_vfov))
        cx = width * 0.5
        cy = height * 0.5

        depth_mm_arr = np.zeros((height, width), dtype=np.uint16)
        depth_viz_arr = np.zeros((height, width, 3), dtype=np.uint8)
        cloud_points: List[bytes] = []

        sample_step_u = 2
        sample_step_v = 2

        if self.camera_backend == 'gpu' and self._gpu_renderer is not None:
            try:
                depth_mm_arr, depth_viz_arr = self._gpu_renderer.render(
                    x=self.x,
                    y=self.y,
                    z=self.z,
                    yaw=self.yaw,
                    width=width,
                    height=height,
                    hfov_rad=self.depth_hfov,
                    vfov_rad=self.depth_vfov,
                    max_range_m=self.depth_max_range,
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'GPU render failed ({exc}); reverting to CPU raycast for this session'
                )
                self.camera_backend = 'cpu'
                self._gpu_renderer = None

        if self.camera_backend == 'cpu' or self._gpu_renderer is None:
            for v in range(height):
                theta_y = -((v + 0.5) / float(height) - 0.5) * self.depth_vfov

                for u in range(width):
                    theta_x = -((u + 0.5) / float(width) - 0.5) * self.depth_hfov
                    yaw_angle = self.yaw + theta_x
                    hit = self.cast_ray(yaw_angle, theta_y)

                    if hit is None:
                        continue

                    depth_mm_arr[v, u] = int(round(hit.distance * 1000.0))
                    depth_viz_arr[v, u, :] = self.hit_to_rgb(hit)

        if not self.run_without_rviz:
            for v in range(0, height, sample_step_v):
                theta_y = -((v + 0.5) / float(height) - 0.5) * self.depth_vfov
                cos_theta_y = math.cos(theta_y)
                sin_theta_y = math.sin(theta_y)
                for u in range(0, width, sample_step_u):
                    depth_mm = int(depth_mm_arr[v, u])
                    if depth_mm <= 0:
                        continue
                    depth = depth_mm * 0.001
                    theta_x = -((u + 0.5) / float(width) - 0.5) * self.depth_hfov
                    x_cam = depth * cos_theta_y * math.cos(theta_x)
                    y_cam = depth * cos_theta_y * math.sin(theta_x)
                    z_cam = depth * sin_theta_y
                    cloud_points.append(struct.pack('fff', x_cam, y_cam, z_cam))

        depth_image = Image()
        depth_image.header.stamp = stamp
        depth_image.header.frame_id = frame_id
        depth_image.height = height
        depth_image.width = width
        depth_image.encoding = '16UC1'
        depth_image.is_bigendian = False
        depth_image.step = width * 2
        depth_image.data = depth_mm_arr.tobytes(order='C')
        self.depth_image_pub.publish(depth_image)
        if self.save_depth_images:
            depth_image_hash = hash(bytes(depth_image.data))
            if depth_image_hash != self.last_depth_hash:
                self.last_depth_hash = depth_image_hash
                depth_image_pil = bridge.imgmsg_to_cv2(depth_image, desired_encoding='passthrough')
                cv2.imwrite(f'data/depth_{stamp.sec}.{stamp.nanosec:09d}.png', depth_image_pil)



        depth_viz = Image()
        depth_viz.header.stamp = stamp
        depth_viz.header.frame_id = frame_id
        depth_viz.height = height
        depth_viz.width = width
        depth_viz.encoding = 'rgb8'
        depth_viz.is_bigendian = False
        depth_viz.step = width * 3
        depth_viz.data = depth_viz_arr.tobytes(order='C')
        self.depth_viz_pub.publish(depth_viz)

        camera_info = CameraInfo()
        camera_info.header.stamp = stamp
        camera_info.header.frame_id = frame_id
        camera_info.width = width
        camera_info.height = height
        camera_info.distortion_model = 'plumb_bob'
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.depth_camera_info_pub.publish(camera_info)

        point_cloud = PointCloud2()
        point_cloud.header.stamp = stamp
        point_cloud.header.frame_id = frame_id
        point_cloud.height = 1
        point_cloud.width = len(cloud_points)
        point_cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.row_step = point_cloud.point_step * point_cloud.width
        point_cloud.is_dense = False
        point_cloud.data = b''.join(cloud_points)
        self.depth_points_pub.publish(point_cloud)

    def publish_geiger(self) -> None:
        dx = self.x - self.geiger_source_x
        dy = self.y - self.geiger_source_y
        dz = self.z - self.geiger_source_z
        d2 = dx * dx + dy * dy + dz * dz + 0.2
        cps = self.geiger_background + self.geiger_strength / d2
        noisy_cps = max(0.0, random.gauss(cps, math.sqrt(max(cps, 1.0))))

        self.geiger_logfile.write(f'{self.get_clock().now().to_msg().sec}.{self.get_clock().now().to_msg().nanosec:09d},{noisy_cps:.3f}\n')
        msg = Float32()
        msg.data = float(noisy_cps)
        self.geiger_pub.publish(msg)

    def publish_geometry(self) -> None:
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()

        floor = Marker()
        floor.header.stamp = stamp
        floor.header.frame_id = 'odom'
        floor.ns = 'room'
        floor.id = 0
        floor.type = Marker.CUBE
        floor.action = Marker.ADD
        floor.pose.position.x = self.room_width * 0.5
        floor.pose.position.y = self.room_height * 0.5
        floor.pose.position.z = -0.01
        floor.pose.orientation.w = 1.0
        floor.scale.x = self.room_width
        floor.scale.y = self.room_height
        floor.scale.z = 0.02
        floor.color.r = 0.2
        floor.color.g = 0.2
        floor.color.b = 0.2
        floor.color.a = 0.8
        markers.markers.append(floor)

        wall_height = max(self.ceiling_z, self.obstacle_height)
        half_wall_height = wall_height * 0.5
        walls = [
            (self.room_width * 0.5, 0.0, self.room_width, self.wall_thickness),
            (self.room_width * 0.5, self.room_height, self.room_width, self.wall_thickness),
            (0.0, self.room_height * 0.5, self.wall_thickness, self.room_height),
            (self.room_width, self.room_height * 0.5, self.wall_thickness, self.room_height),
        ]
        for idx, (cx, cy, sx, sy) in enumerate(walls, start=1):
            wall = Marker()
            wall.header.stamp = stamp
            wall.header.frame_id = 'odom'
            wall.ns = 'room_walls'
            wall.id = idx
            wall.type = Marker.CUBE
            wall.action = Marker.ADD
            wall.pose.position.x = cx
            wall.pose.position.y = cy
            wall.pose.position.z = half_wall_height
            wall.pose.orientation.w = 1.0
            wall.scale.x = sx
            wall.scale.y = sy
            wall.scale.z = wall_height
            wall.color.r = 0.55
            wall.color.g = 0.55
            wall.color.b = 0.65
            wall.color.a = 0.95
            markers.markers.append(wall)

        marker_id = 100
        for box_obstacle in self.boxes:
            box = Marker()
            box.header.stamp = stamp
            box.header.frame_id = 'odom'
            box.ns = 'box_obstacles'
            box.id = marker_id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = 0.5 * (box_obstacle.x_min + box_obstacle.x_max)
            box.pose.position.y = 0.5 * (box_obstacle.y_min + box_obstacle.y_max)
            box.pose.position.z = self.floor_z + 0.5 * (box_obstacle.z_bottom + box_obstacle.z_top)
            box.pose.orientation.w = 1.0
            box.scale.x = max(0.05, box_obstacle.x_max - box_obstacle.x_min)
            box.scale.y = max(0.05, box_obstacle.y_max - box_obstacle.y_min)
            box.scale.z = max(0.05, box_obstacle.z_top - box_obstacle.z_bottom)
            box.color.r = 0.16
            box.color.g = 0.5
            box.color.b = 0.86
            box.color.a = 0.92
            if box_obstacle.z_bottom > 1.0:
                # Ceiling/Overhead items color
                box.color.r = 0.7
                box.color.g = 0.7
                box.color.b = 0.75
                box.color.a = 0.9
            markers.markers.append(box)
            marker_id += 1

        for cylinder_obstacle in self.cylinders:
            cylinder = Marker()
            cylinder.header.stamp = stamp
            cylinder.header.frame_id = 'odom'
            cylinder.ns = 'cylinder_obstacles'
            cylinder.id = marker_id
            cylinder.type = Marker.CYLINDER
            cylinder.action = Marker.ADD
            cylinder.pose.position.x = cylinder_obstacle.cx
            cylinder.pose.position.y = cylinder_obstacle.cy
            cylinder.pose.position.z = self.floor_z + 0.5 * (cylinder_obstacle.z_bottom + cylinder_obstacle.z_top)
            cylinder.pose.orientation.w = 1.0
            cylinder.scale.x = cylinder_obstacle.radius * 2.0
            cylinder.scale.y = cylinder_obstacle.radius * 2.0
            cylinder.scale.z = max(0.05, cylinder_obstacle.z_top - cylinder_obstacle.z_bottom)
            cylinder.color.r = 0.95
            cylinder.color.g = 0.45
            cylinder.color.b = 0.2
            cylinder.color.a = 0.9
            if cylinder_obstacle.z_bottom > 1.0:
                 # Ceiling/Overhead items color
                cylinder.color.r = 0.7
                cylinder.color.g = 0.7
                cylinder.color.b = 0.75
                cylinder.color.a = 0.9
            markers.markers.append(cylinder)
            marker_id += 1

        source = Marker()
        source.header.stamp = stamp
        source.header.frame_id = 'odom'
        source.ns = 'geiger_source'
        source.id = 300
        source.type = Marker.SPHERE
        source.action = Marker.ADD
        source.pose.position.x = self.geiger_source_x
        source.pose.position.y = self.geiger_source_y
        source.pose.position.z = self.geiger_source_z
        source.pose.orientation.w = 1.0
        source.scale.x = 0.3
        source.scale.y = 0.3
        source.scale.z = 0.3
        source.color.r = 1.0
        source.color.g = 0.0
        source.color.b = 0.2
        source.color.a = 0.95
        markers.markers.append(source)

        source_line = Marker()
        source_line.header.stamp = stamp
        source_line.header.frame_id = 'odom'
        source_line.ns = 'geiger_source'
        source_line.id = 301
        source_line.type = Marker.LINE_LIST
        source_line.action = Marker.ADD
        source_line.scale.x = 0.03
        source_line.color.r = 1.0
        source_line.color.g = 0.25
        source_line.color.b = 0.25
        source_line.color.a = 0.9
        source_line.points = [
            Point(x=self.geiger_source_x, y=self.geiger_source_y, z=0.0),
            Point(x=self.geiger_source_x, y=self.geiger_source_y, z=self.geiger_source_z),
        ]
        markers.markers.append(source_line)

        self.geometry_pub.publish(markers)

    def publish_drone_marker(self, now) -> None:
        stamp = now.to_msg()
        markers = MarkerArray()

        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = 'odom'
        body.ns = 'drone'
        body.id = 400
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = self.x
        body.pose.position.y = self.y
        body.pose.position.z = self.z
        body.pose.orientation.x = 0.0
        body.pose.orientation.y = 0.0
        body.pose.orientation.z = math.sin(self.yaw * 0.5)
        body.pose.orientation.w = math.cos(self.yaw * 0.5)
        body.scale.x = 0.35
        body.scale.y = 0.35
        body.scale.z = 0.12
        body.color.r = 0.1
        body.color.g = 0.95
        body.color.b = 0.5
        body.color.a = 1.0
        markers.markers.append(body)

        heading = Marker()
        heading.header.stamp = stamp
        heading.header.frame_id = 'odom'
        heading.ns = 'drone'
        heading.id = 401
        heading.type = Marker.ARROW
        heading.action = Marker.ADD
        heading.scale.x = 0.04
        heading.scale.y = 0.08
        heading.scale.z = 0.08
        heading.color.r = 0.05
        heading.color.g = 0.85
        heading.color.b = 1.0
        heading.color.a = 0.95
        heading.points = [
            Point(x=self.x, y=self.y, z=self.z + 0.08),
            Point(
                x=self.x + 0.6 * math.cos(self.yaw),
                y=self.y + 0.6 * math.sin(self.yaw),
                z=self.z + 0.08,
            ),
        ]
        markers.markers.append(heading)

        arm1 = Marker()
        arm1.header.stamp = stamp
        arm1.header.frame_id = 'odom'
        arm1.ns = 'drone'
        arm1.id = 410
        arm1.type = Marker.CYLINDER
        arm1.action = Marker.ADD
        arm1.pose.position.x = self.x
        arm1.pose.position.y = self.y
        arm1.pose.position.z = self.z
        arm1_yaw = self.yaw + 45.0 * math.pi / 180.0
        arm1_s = math.sin(arm1_yaw * 0.5)
        arm1_c = math.cos(arm1_yaw * 0.5)
        arm1_k = math.sqrt(0.5)
        arm1.pose.orientation.x = -arm1_s * arm1_k
        arm1.pose.orientation.y = arm1_c * arm1_k
        arm1.pose.orientation.z = arm1_s * arm1_k
        arm1.pose.orientation.w = arm1_c * arm1_k
        arm1.scale.x = 0.05
        arm1.scale.y = 0.05
        arm1.scale.z = 1.0
        arm1.color.r = 0.8
        arm1.color.g = 0.8
        arm1.color.b = 0.8
        arm1.color.a = 0.95
        markers.markers.append(arm1)

        arm2 = Marker()
        arm2.header.stamp = stamp
        arm2.header.frame_id = 'odom'
        arm2.ns = 'drone'
        arm2.id = 411
        arm2.type = Marker.CYLINDER
        arm2.action = Marker.ADD
        arm2.pose.position.x = self.x
        arm2.pose.position.y = self.y
        arm2.pose.position.z = self.z
        arm2_yaw = self.yaw - 45.0 * math.pi / 180.0
        arm2_s = math.sin(arm2_yaw * 0.5)
        arm2_c = math.cos(arm2_yaw * 0.5)
        arm2_k = math.sqrt(0.5)
        arm2.pose.orientation.x = -arm2_s * arm2_k
        arm2.pose.orientation.y = arm2_c * arm2_k
        arm2.pose.orientation.z = arm2_s * arm2_k
        arm2.pose.orientation.w = arm2_c * arm2_k
        arm2.scale.x = 0.05
        arm2.scale.y = 0.05
        arm2.scale.z = 1.0
        arm2.color.r = 0.8
        arm2.color.g = 0.8
        arm2.color.b = 0.8
        arm2.color.a = 0.95
        markers.markers.append(arm2)

        self.geometry_pub.publish(markers)

    def cast_ray(self, yaw_angle: float, pitch_angle: float) -> RayHit | None:
        cos_pitch = math.cos(pitch_angle)
        dx = math.cos(yaw_angle) * cos_pitch
        dy = math.sin(yaw_angle) * cos_pitch
        dz = math.sin(pitch_angle)

        if abs(dx) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9:
            return None

        best: RayHit | None = None

        def consider(hit: RayHit | None) -> None:
            nonlocal best
            if hit is None:
                return
            if hit.distance <= 1e-5 or hit.distance > self.depth_max_range:
                return
            if best is None or hit.distance < best.distance:
                best = hit

        consider(self._intersect_room_planes(dx, dy, dz))

        for box in self.boxes:
            consider(self._intersect_box_3d(dx, dy, dz, box))

        for cylinder in self.cylinders:
            consider(self._intersect_cylinder_3d(dx, dy, dz, cylinder))

        return best

    def hit_to_rgb(self, hit: RayHit) -> tuple[int, int, int]:
        material_colors = {
            'floor': (95.0, 95.0, 95.0),
            'ceiling': (200.0, 205.0, 210.0),
            'wall': (138.0, 142.0, 158.0),
            'box': (65.0, 130.0, 220.0),
            'overhead_box': (168.0, 172.0, 182.0),
            'cylinder': (225.0, 110.0, 55.0),
            'overhead_cylinder': (175.0, 176.0, 180.0),
        }

        base = material_colors.get(hit.material, (140.0, 140.0, 140.0))

        # Mild procedural texture so nearby surfaces look less flat.
        texture = 1.0
        if hit.material in ('floor', 'ceiling'):
            checker = (int(math.floor(hit.x * 2.0)) + int(math.floor(hit.y * 2.0))) & 1
            texture = 0.92 if checker else 1.06
        elif hit.material == 'wall':
            stripe = int(math.floor((hit.z - self.floor_z) * 4.0)) & 1
            texture = 0.95 if stripe else 1.04

        ldx, ldy, ldz = (0.35, -0.25, -0.9)
        lnorm = math.sqrt(ldx * ldx + ldy * ldy + ldz * ldz)
        ldx /= lnorm
        ldy /= lnorm
        ldz /= lnorm

        nx, ny, nz = hit.nx, hit.ny, hit.nz
        ndotl = max(0.0, -(nx * ldx + ny * ldy + nz * ldz))
        ambient = 0.34
        diffuse = 0.66 * ndotl
        dist_falloff = 1.0 / (1.0 + 0.05 * hit.distance * hit.distance)
        shade = (ambient + diffuse) * (0.78 + 0.22 * dist_falloff) * texture

        r = max(0, min(255, int(round(base[0] * shade))))
        g = max(0, min(255, int(round(base[1] * shade))))
        b = max(0, min(255, int(round(base[2] * shade))))
        return (r, g, b)

    def _intersect_room_planes(self, dx: float, dy: float, dz: float) -> RayHit | None:
        best: RayHit | None = None

        def consider(t: float, nx: float, ny: float, nz: float, material: str) -> None:
            nonlocal best
            if t <= 1e-6:
                return
            xh = self.x + dx * t
            yh = self.y + dy * t
            zh = self.z + dz * t
            if not (-1e-6 <= xh <= self.room_width + 1e-6):
                return
            if not (-1e-6 <= yh <= self.room_height + 1e-6):
                return
            if not (self.floor_z - 1e-6 <= zh <= self.ceiling_z + 1e-6):
                return
            hit = RayHit(t, xh, yh, zh, nx, ny, nz, material)
            if best is None or hit.distance < best.distance:
                best = hit

        if abs(dx) > 1e-9:
            t = (0.0 - self.x) / dx
            y_hit = self.y + dy * t
            z_hit = self.z + dz * t
            if 0.0 <= y_hit <= self.room_height and self.floor_z <= z_hit <= self.ceiling_z:
                consider(t, 1.0, 0.0, 0.0, 'wall')

            t = (self.room_width - self.x) / dx
            y_hit = self.y + dy * t
            z_hit = self.z + dz * t
            if 0.0 <= y_hit <= self.room_height and self.floor_z <= z_hit <= self.ceiling_z:
                consider(t, -1.0, 0.0, 0.0, 'wall')

        if abs(dy) > 1e-9:
            t = (0.0 - self.y) / dy
            x_hit = self.x + dx * t
            z_hit = self.z + dz * t
            if 0.0 <= x_hit <= self.room_width and self.floor_z <= z_hit <= self.ceiling_z:
                consider(t, 0.0, 1.0, 0.0, 'wall')

            t = (self.room_height - self.y) / dy
            x_hit = self.x + dx * t
            z_hit = self.z + dz * t
            if 0.0 <= x_hit <= self.room_width and self.floor_z <= z_hit <= self.ceiling_z:
                consider(t, 0.0, -1.0, 0.0, 'wall')

        if abs(dz) > 1e-9:
            t = (self.floor_z - self.z) / dz
            x_hit = self.x + dx * t
            y_hit = self.y + dy * t
            if 0.0 <= x_hit <= self.room_width and 0.0 <= y_hit <= self.room_height:
                consider(t, 0.0, 0.0, 1.0, 'floor')

            t = (self.ceiling_z - self.z) / dz
            x_hit = self.x + dx * t
            y_hit = self.y + dy * t
            if 0.0 <= x_hit <= self.room_width and 0.0 <= y_hit <= self.room_height:
                consider(t, 0.0, 0.0, -1.0, 'ceiling')

        return best

    def _intersect_box_3d(self, dx: float, dy: float, dz: float, box: BoxObstacle) -> RayHit | None:
        z_min = self.floor_z + box.z_bottom
        z_max = self.floor_z + box.z_top

        bounds = [
            (box.x_min, box.x_max, self.x, dx),
            (box.y_min, box.y_max, self.y, dy),
            (z_min, z_max, self.z, dz),
        ]

        t_min = -float('inf')
        t_max = float('inf')
        enter_normal = (0.0, 0.0, 0.0)

        axis_normals = [
            ((-1.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
            ((0.0, -1.0, 0.0), (0.0, 1.0, 0.0)),
            ((0.0, 0.0, -1.0), (0.0, 0.0, 1.0)),
        ]

        for i, (bmin, bmax, origin, d) in enumerate(bounds):
            if abs(d) < 1e-9:
                if origin < bmin or origin > bmax:
                    return None
                continue

            t1 = (bmin - origin) / d
            t2 = (bmax - origin) / d
            n1, n2 = axis_normals[i]
            if t1 > t2:
                t1, t2 = t2, t1
                n1, n2 = n2, n1

            if t1 > t_min:
                t_min = t1
                enter_normal = n1
            t_max = min(t_max, t2)
            if t_min > t_max:
                return None

        t_hit = t_min if t_min > 1e-6 else t_max
        if t_hit <= 1e-6:
            return None

        xh = self.x + dx * t_hit
        yh = self.y + dy * t_hit
        zh = self.z + dz * t_hit
        material = 'overhead_box' if box.z_bottom > 1.0 else 'box'
        nx, ny, nz = enter_normal
        if t_hit == t_max and t_min <= 1e-6:
            nx, ny, nz = -nx, -ny, -nz
        return RayHit(t_hit, xh, yh, zh, nx, ny, nz, material)

    def _intersect_cylinder_3d(self, dx: float, dy: float, dz: float, cylinder: CylinderObstacle) -> RayHit | None:
        z_min = self.floor_z + cylinder.z_bottom
        z_max = self.floor_z + cylinder.z_top
        best: RayHit | None = None

        def consider(hit: RayHit | None) -> None:
            nonlocal best
            if hit is None:
                return
            if hit.distance <= 1e-6:
                return
            if best is None or hit.distance < best.distance:
                best = hit

        ox = self.x - cylinder.cx
        oy = self.y - cylinder.cy
        a = dx * dx + dy * dy
        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - cylinder.radius * cylinder.radius

        if a > 1e-9:
            disc = b * b - 4.0 * a * c
            if disc >= 0.0:
                sqrt_disc = math.sqrt(disc)
                for t in sorted(((-b - sqrt_disc) / (2.0 * a), (-b + sqrt_disc) / (2.0 * a))):
                    if t <= 1e-6:
                        continue
                    zh = self.z + dz * t
                    if not (z_min - 1e-6 <= zh <= z_max + 1e-6):
                        continue
                    xh = self.x + dx * t
                    yh = self.y + dy * t
                    nx = (xh - cylinder.cx) / max(1e-9, cylinder.radius)
                    ny = (yh - cylinder.cy) / max(1e-9, cylinder.radius)
                    material = 'overhead_cylinder' if cylinder.z_bottom > 1.0 else 'cylinder'
                    consider(RayHit(t, xh, yh, zh, nx, ny, 0.0, material))

        if abs(dz) > 1e-9:
            for z_plane, nz in ((z_min, -1.0), (z_max, 1.0)):
                t = (z_plane - self.z) / dz
                if t <= 1e-6:
                    continue
                xh = self.x + dx * t
                yh = self.y + dy * t
                if (xh - cylinder.cx) ** 2 + (yh - cylinder.cy) ** 2 <= cylinder.radius ** 2 + 1e-6:
                    material = 'overhead_cylinder' if cylinder.z_bottom > 1.0 else 'cylinder'
                    consider(RayHit(t, xh, yh, z_plane, 0.0, 0.0, nz, material))

        return best

    def intersect_obstacle_top(
        self,
        z_top: float,
        sin_pitch: float,
        cos_pitch: float,
        dx: float,
        dy: float,
        x_min: float,
        y_min: float,
        x_max: float,
        y_max: float,
    ) -> float | None:
        if abs(sin_pitch) < 1e-9:
            return None
        depth = (z_top - self.z) / sin_pitch
        if not (0.0 < depth <= self.depth_max_range):
            return None
        horizontal = depth * cos_pitch
        x_hit = self.x + horizontal * dx
        y_hit = self.y + horizontal * dy
        if x_min - 1e-6 <= x_hit <= x_max + 1e-6 and y_min - 1e-6 <= y_hit <= y_max + 1e-6:
            return depth
        return None

    def intersect_cylinder_top(
        self,
        z_top: float,
        sin_pitch: float,
        cos_pitch: float,
        dx: float,
        dy: float,
        cx: float,
        cy: float,
        radius: float,
    ) -> float | None:
        if abs(sin_pitch) < 1e-9:
            return None
        depth = (z_top - self.z) / sin_pitch
        if not (0.0 < depth <= self.depth_max_range):
            return None
        horizontal = depth * cos_pitch
        x_hit = self.x + horizontal * dx
        y_hit = self.y + horizontal * dy
        if (x_hit - cx) ** 2 + (y_hit - cy) ** 2 <= radius * radius + 1e-6:
            return depth
        return None

    def intersect_room(self, x: float, y: float, dx: float, dy: float) -> List[float]:
        values: List[float] = []
        values.extend(self.intersect_vertical(x, y, dx, dy, 0.0, 0.0, self.room_height))
        values.extend(self.intersect_vertical(x, y, dx, dy, self.room_width, 0.0, self.room_height))
        values.extend(self.intersect_horizontal(x, y, dx, dy, 0.0, 0.0, self.room_width))
        values.extend(self.intersect_horizontal(x, y, dx, dy, self.room_height, 0.0, self.room_width))
        return values

    def intersect_box(
        self,
        x: float,
        y: float,
        dx: float,
        dy: float,
        box: BoxObstacle,
    ) -> List[float]:
        values: List[float] = []
        values.extend(self.intersect_vertical(x, y, dx, dy, box.x_min, box.y_min, box.y_max))
        values.extend(self.intersect_vertical(x, y, dx, dy, box.x_max, box.y_min, box.y_max))
        values.extend(self.intersect_horizontal(x, y, dx, dy, box.y_min, box.x_min, box.x_max))
        values.extend(self.intersect_horizontal(x, y, dx, dy, box.y_max, box.x_min, box.x_max))
        return values

    @staticmethod
    def intersect_vertical(
        x: float,
        y: float,
        dx: float,
        dy: float,
        x_line: float,
        y_min: float,
        y_max: float,
    ) -> List[float]:
        if abs(dx) < 1e-9:
            return []
        t = (x_line - x) / dx
        if t <= 0.0:
            return []
        yy = y + t * dy
        if y_min - 1e-9 <= yy <= y_max + 1e-9:
            return [t]
        return []

    @staticmethod
    def intersect_horizontal(
        x: float,
        y: float,
        dx: float,
        dy: float,
        y_line: float,
        x_min: float,
        x_max: float,
    ) -> List[float]:
        if abs(dy) < 1e-9:
            return []
        t = (y_line - y) / dy
        if t <= 0.0:
            return []
        xx = x + t * dx
        if x_min - 1e-9 <= xx <= x_max + 1e-9:
            return [t]
        return []

    @staticmethod
    def intersect_circle(
        x: float,
        y: float,
        dx: float,
        dy: float,
        circle: CylinderObstacle,
    ) -> float | None:
        ox = x - circle.cx
        oy = y - circle.cy

        b = 2.0 * (ox * dx + oy * dy)
        c = ox * ox + oy * oy - circle.radius * circle.radius
        disc = b * b - 4.0 * c
        if disc < 0.0:
            return None

        sqrt_disc = math.sqrt(disc)
        t1 = (-b - sqrt_disc) * 0.5
        t2 = (-b + sqrt_disc) * 0.5
        values = [t for t in (t1, t2) if t > 0.0]
        if not values:
            return None
        return min(values)

    def destroy_node(self) -> bool:
        if self._gpu_renderer is not None:
            try:
                self._gpu_renderer.close()
            except Exception:
                pass
            self._gpu_renderer = None
        return super().destroy_node()

    def reset_position(self, msg: Bool) -> None:
        if msg.data == True:
            self.x = float(self.get_parameter('initial_x').value)
            self.y = float(self.get_parameter('initial_y').value)
            self.z = float(self.get_parameter('initial_z').value)
            self.yaw = float(self.get_parameter('initial_yaw').value)
            self.floor_z = float(self.get_parameter('floor_z').value)
            self.ceiling_z = float(self.get_parameter('ceiling_z').value)
            self.cmd_linear_x = 0.0
            self.cmd_linear_y = 0.0
            self.cmd_linear_z = 0.0
            self.cmd_angular_z = 0.0


def main(args=None) -> None:
    # cleanup
    for filename in os.listdir('data'):
        os.remove(f"data/{filename}")
    rclpy.init(args=args)
    try:
        node = SimWorldNode()
    except RuntimeError:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._release_instance_lock()
        node.destroy_node()
        rclpy.shutdown()

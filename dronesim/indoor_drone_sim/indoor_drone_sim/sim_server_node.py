import json
import math
import socket
import struct
import subprocess
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def _resolve_local_ip() -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(('8.8.8.8', 80))
        return str(probe.getsockname()[0])
    except OSError:
        return '127.0.0.1'
    finally:
        probe.close()


def _is_ipv4_addr(text: str) -> bool:
    try:
        socket.inet_aton(text)
    except OSError:
        return False
    return True


def _resolve_default_gateway_ip() -> Optional[str]:
    try:
        output = subprocess.check_output(['ip', 'route', 'show', 'default'], text=True)
    except Exception:
        return None

    for line in output.splitlines():
        parts = line.strip().split()
        if not parts:
            continue
        if parts[0] != 'default':
            continue
        if 'via' not in parts:
            continue
        idx = parts.index('via')
        if idx + 1 >= len(parts):
            continue
        ip = parts[idx + 1].strip()
        if _is_ipv4_addr(ip):
            return ip
    return None


def _resolve_resolv_nameserver_ip() -> Optional[str]:
    # On WSL, /etc/resolv.conf often contains a Windows-side reachable DNS endpoint.
    try:
        with open('/etc/resolv.conf', 'r', encoding='utf-8') as f:
            for line in f:
                text = line.strip()
                if text.startswith('nameserver '):
                    parts = text.split()
                    if len(parts) >= 2:
                        ip = parts[1].strip()
                        if _is_ipv4_addr(ip):
                            return ip
    except OSError:
        return None
    return None


def _resolve_windows_host_targets() -> List[str]:
    candidates: List[str] = []
    gw = _resolve_default_gateway_ip()
    if gw:
        candidates.append(gw)
    ns = _resolve_resolv_nameserver_ip()
    if ns and ns not in candidates:
        candidates.append(ns)
    return candidates


class SimServerNode(Node):
    def __init__(self) -> None:
        super().__init__('sim_server_node')

        self.declare_parameter('ip_broadcast_port', 10000)
        self.declare_parameter('geiger_value_port', 10350)
        self.declare_parameter('imu_broadcast_port', 12000)
        self.declare_parameter('camera_imu_broadcast_port', 12001)
        self.declare_parameter('camera_tcp_port', 12002)
        self.declare_parameter('camera_stream_rate_hz', 20.0)
        self.declare_parameter('camera_zmq_enabled', True)
        self.declare_parameter('camera_zmq_port', 11000)
        self.declare_parameter('camera_zmq_rate_hz', 20.0)
        self.declare_parameter('camera_zmq_sndhwm', 1)
        self.declare_parameter('camera_zmq_width', 640)
        self.declare_parameter('camera_zmq_height', 480)
        self.declare_parameter('camera_zmq_trajectory_enabled', True)
        self.declare_parameter('camera_zmq_trajectory_port', 12080)
        self.declare_parameter('camera_zmq_trajectory_rate_hz', 20.0)
        self.declare_parameter('camera_zmq_trajectory_width', 640)
        self.declare_parameter('camera_zmq_trajectory_height', 480)
        self.declare_parameter('fc_zmq_enabled', True)
        self.declare_parameter('fc_sub_port', 15000)
        self.declare_parameter('fc_pub_port', 16000)
        self.declare_parameter('fc_rep_port', 15001)
        self.declare_parameter('fc_loop_rate_hz', 50.0)
        self.declare_parameter('fc_max_forward_mps', 1.0)
        self.declare_parameter('fc_max_lateral_mps', 1.0)
        self.declare_parameter('fc_max_vertical_mps', 0.4)
        self.declare_parameter('fc_max_yaw_rps', 1.5)
        self.declare_parameter('fc_yaw_channel_scale', 500.0)
        self.declare_parameter('geiger_value_rate_hz', 5.0)
        # self.declare_parameter('api_upscale_factor', 2)
        self.declare_parameter('ip_windows_unicast_enabled', True)
        self.declare_parameter('ip_windows_unicast_addr', '')
        self.declare_parameter('ip_windows_advertise_addr', '')

        self.ip_broadcast_port = int(self.get_parameter('ip_broadcast_port').value)
        self.geiger_value_port = int(self.get_parameter('geiger_value_port').value)
        self.imu_broadcast_port = int(self.get_parameter('imu_broadcast_port').value)
        self.camera_imu_broadcast_port = int(self.get_parameter('camera_imu_broadcast_port').value)
        self.camera_tcp_port = int(self.get_parameter('camera_tcp_port').value)
        self.camera_stream_rate_hz = float(self.get_parameter('camera_stream_rate_hz').value)
        self.camera_zmq_enabled = bool(self.get_parameter('camera_zmq_enabled').value)
        self.camera_zmq_port = int(self.get_parameter('camera_zmq_port').value)
        self.camera_zmq_rate_hz = float(self.get_parameter('camera_zmq_rate_hz').value)
        self.camera_zmq_sndhwm = int(self.get_parameter('camera_zmq_sndhwm').value)
        self.camera_zmq_width = int(self.get_parameter('camera_zmq_width').value)
        self.camera_zmq_height = int(self.get_parameter('camera_zmq_height').value)
        self.camera_zmq_trajectory_enabled = bool(self.get_parameter('camera_zmq_trajectory_enabled').value)
        self.camera_zmq_trajectory_port = int(self.get_parameter('camera_zmq_trajectory_port').value)
        self.camera_zmq_trajectory_rate_hz = float(self.get_parameter('camera_zmq_trajectory_rate_hz').value)
        self.camera_zmq_trajectory_width = int(self.get_parameter('camera_zmq_trajectory_width').value)
        self.camera_zmq_trajectory_height = int(self.get_parameter('camera_zmq_trajectory_height').value)
        self.fc_zmq_enabled = bool(self.get_parameter('fc_zmq_enabled').value)
        self.fc_sub_port = int(self.get_parameter('fc_sub_port').value)
        self.fc_pub_port = int(self.get_parameter('fc_pub_port').value)
        self.fc_rep_port = int(self.get_parameter('fc_rep_port').value)
        self.fc_loop_rate_hz = float(self.get_parameter('fc_loop_rate_hz').value)
        self.fc_max_forward_mps = float(self.get_parameter('fc_max_forward_mps').value)
        self.fc_max_lateral_mps = float(self.get_parameter('fc_max_lateral_mps').value)
        self.fc_max_vertical_mps = float(self.get_parameter('fc_max_vertical_mps').value)
        self.fc_max_yaw_rps = float(self.get_parameter('fc_max_yaw_rps').value)
        self.fc_yaw_channel_scale = float(self.get_parameter('fc_yaw_channel_scale').value)
        self.geiger_value_rate_hz = float(self.get_parameter('geiger_value_rate_hz').value)
        # self.api_upscale_factor = max(1, int(self.get_parameter('api_upscale_factor').value))
        self.ip_windows_unicast_enabled = bool(self.get_parameter('ip_windows_unicast_enabled').value)
        configured_unicast_addr = str(self.get_parameter('ip_windows_unicast_addr').value or '').strip()
        configured_advertise_addr = str(self.get_parameter('ip_windows_advertise_addr').value or '').strip()

        self.local_ip = _resolve_local_ip()
        if configured_unicast_addr and _is_ipv4_addr(configured_unicast_addr):
            self.windows_host_targets = [configured_unicast_addr]
        else:
            self.windows_host_targets = _resolve_windows_host_targets()
        self.windows_advertise_ip = configured_advertise_addr if _is_ipv4_addr(configured_advertise_addr) else self.local_ip
        self._data_lock = threading.Lock()
        self._running = True

        self.latest_depth: Optional[bytes] = None
        self.latest_depth_shape: Optional[Tuple[int, int]] = None
        self.latest_color_jpeg: Optional[bytes] = None
        self.latest_geiger_cps = 0.0
        self.geiger_value = 0.0
        self.latest_attitude = (0.0, 0.0, 0.0)

        self._zmq_ctx = None
        self._zmq_sock = None
        self._zmq_traj_sock = None
        self._fc_sub_sock = None
        self._fc_pub_sock = None
        self._fc_rep_sock = None
        self._rvl_mod = None
        self._geiger_server_sock: Optional[socket.socket] = None
        self._geiger_clients: List[socket.socket] = []
        self._geiger_clients_lock = threading.Lock()

        self._last_odom_time: Optional[float] = None
        self._last_linear_vel = np.zeros(3, dtype=np.float64)
        self._fc_armed = False
        self._fc_cur = [1500, 1500, 1000, 1500]
        self._fc_tgt = [1500, 1500, 1000, 1500]
        self._fc_inc = [1, 1, 4, 2]
        self._fc_last_cmd_time = time.time()

        time.sleep(2.0)  # Wait a moment for sim_world_node to be ready before starting to publish.

        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        depth_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, '/depth/image_raw', self._on_depth_image, depth_qos)
        self.create_subscription(Image, '/depth/image_viz', self._on_color_image, depth_qos)
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.create_subscription(Float32, '/geiger_count', self._on_geiger_cps, 20)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 20)
        self.reset_position_pub = self.create_publisher(Bool, '/reset_position', 10)

        # Keep compatibility with server/ip.py behavior: broadcast every second.
        ip_period = 1.0
        geiger_period = 1.0 / max(1.0, self.geiger_value_rate_hz)
        self.create_timer(ip_period, self._broadcast_ip)
        self._last_geiger_tick = time.time()
        self.create_timer(geiger_period, self._emit_geiger_values)

        self._tcp_thread = threading.Thread(target=self._tcp_camera_server_loop, daemon=True)
        self._tcp_thread.start()
        self._geiger_thread = threading.Thread(target=self._tcp_geiger_server_loop, daemon=True)
        self._geiger_thread.start()

        self._zmq_thread = None
        self._zmq_traj_thread = None
        self._fc_thread = None
        if self.camera_zmq_enabled or self.camera_zmq_trajectory_enabled:
            self._setup_camera_zmq()
        if self.fc_zmq_enabled:
            self._setup_fc_zmq()

        self.get_logger().info(
            f'sim_server_node started: UDP ip={self.ip_broadcast_port} '
            f'TCP geiger={self.geiger_value_port} TCP camera={self.camera_tcp_port}'
        )
        if self.ip_windows_unicast_enabled and self.windows_host_targets:
            self.get_logger().info(
                f'IP packets will also be unicast to Windows host targets: '
                f'{self.windows_host_targets} on {self.ip_broadcast_port}, '
                f'advertise_addr={self.windows_advertise_ip}'
            )

    def _setup_fc_zmq(self) -> None:
        try:
            import zmq  # type: ignore
        except Exception as exc:
            self.get_logger().warning(f'fc_zmq_enabled=true but zmq import failed ({exc}); FC bridge disabled')
            return

        if self._zmq_ctx is None:
            self._zmq_ctx = zmq.Context()

        self._fc_sub_sock = self._zmq_ctx.socket(zmq.SUB)
        self._fc_sub_sock.setsockopt_string(zmq.SUBSCRIBE, '')
        self._fc_sub_sock.bind(f'tcp://0.0.0.0:{self.fc_sub_port}')

        self._fc_pub_sock = self._zmq_ctx.socket(zmq.PUB)
        self._fc_pub_sock.bind(f'tcp://0.0.0.0:{self.fc_pub_port}')

        self._fc_rep_sock = self._zmq_ctx.socket(zmq.REP)
        self._fc_rep_sock.bind(f'tcp://0.0.0.0:{self.fc_rep_port}')

        self._fc_thread = threading.Thread(target=self._fc_zmq_loop, daemon=True)
        self._fc_thread.start()
        self.get_logger().info(
            f'FC ZMQ bridge enabled: SUB {self.fc_sub_port}, PUB {self.fc_pub_port}, REP {self.fc_rep_port}'
        )

    def _setup_camera_zmq(self) -> None:
        try:
            import zmq  # type: ignore
        except Exception as exc:
            self.get_logger().warning(
                f'camera_zmq_enabled=true but zmq import failed ({exc}); ZMQ camera APIs disabled'
            )
            return

        self._zmq_ctx = zmq.Context()

        if self.camera_zmq_enabled:
            try:
                import rvl  # type: ignore
                self._rvl_mod = rvl
                self._zmq_sock = self._zmq_ctx.socket(zmq.PUB)
                self._zmq_sock.setsockopt(zmq.SNDHWM, max(1, self.camera_zmq_sndhwm))
                self._zmq_sock.bind(f'tcp://0.0.0.0:{self.camera_zmq_port}')
                self._zmq_thread = threading.Thread(target=self._zmq_camera_pub_loop, daemon=True)
                self._zmq_thread.start()
                self.get_logger().info(
                    f'ZMQ camera API enabled at tcp://0.0.0.0:{self.camera_zmq_port} (multipart ts,color,depth_rvl)'
                )
            except Exception as exc:
                self.get_logger().warning(f'rvl import failed ({exc}); legacy RVL ZMQ stream disabled')
                self._rvl_mod = None
                self.camera_zmq_enabled = False

        if self.camera_zmq_trajectory_enabled:
            self._zmq_traj_sock = self._zmq_ctx.socket(zmq.PUB)
            self._zmq_traj_sock.setsockopt(zmq.SNDHWM, max(1, self.camera_zmq_sndhwm))
            self._zmq_traj_sock.bind(f'tcp://0.0.0.0:{self.camera_zmq_trajectory_port}')
            self._zmq_traj_thread = threading.Thread(target=self._zmq_trajectory_pub_loop, daemon=True)
            self._zmq_traj_thread.start()
            self.get_logger().info(
                f'Trajectory ZMQ API enabled at tcp://0.0.0.0:{self.camera_zmq_trajectory_port} '
                f'(multipart ts,color_jpeg,depth_raw16 {self.camera_zmq_trajectory_width}x{self.camera_zmq_trajectory_height})'
            )

    def _on_depth_image(self, msg: Image) -> None:
        if msg.encoding != '16UC1':
            return

        expected = int(msg.width) * int(msg.height) * 2
        if len(msg.data) != expected:
            return


        width = int(msg.width)
        height = int(msg.height)
        depth_bytes = bytes(msg.data)
        # Adaptive to target, if already same then as is.
        if width != self.camera_zmq_trajectory_width or height != self.camera_zmq_trajectory_height:
            depth_u16 = np.frombuffer(depth_bytes, dtype=np.uint16).reshape((height, width))
            depth_u16 = cv2.resize(depth_u16, (self.camera_zmq_trajectory_width, self.camera_zmq_trajectory_height), interpolation=cv2.INTER_NEAREST)
            depth_bytes = depth_u16.tobytes()
            width = self.camera_zmq_trajectory_width
            height = self.camera_zmq_trajectory_height

        with self._data_lock:
            self.latest_depth = depth_bytes
            self.latest_depth_shape = (width, height)

    def _on_color_image(self, msg: Image) -> None:
        try:
            image = self._to_bgr_image(msg)
            if image.shape[1] != self.camera_zmq_trajectory_width or image.shape[0] != self.camera_zmq_trajectory_height:
                image = cv2.resize(
                    image,
                    (
                        self.camera_zmq_trajectory_width,
                        self.camera_zmq_trajectory_height,
                    ),
                    interpolation=cv2.INTER_NEAREST,
                )
            success, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if not success:
                return
            with self._data_lock:
                self.latest_color_jpeg = encoded.tobytes()
        except Exception:
            return

    def _to_bgr_image(self, msg: Image) -> np.ndarray:
        width = int(msg.width)
        height = int(msg.height)

        if msg.encoding in ('rgb8', 'bgr8'):
            channels = 3
            row_stride = int(msg.step)
            expected_stride = width * channels
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            arr = buf.reshape((height, row_stride))[:, :expected_stride].reshape((height, width, channels))
            if msg.encoding == 'rgb8':
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return arr

        if msg.encoding == 'mono8':
            row_stride = int(msg.step)
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            arr = buf.reshape((height, row_stride))[:, :width]
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

        raise ValueError(f'Unsupported color encoding: {msg.encoding}')

    def _on_geiger_cps(self, msg: Float32) -> None:
        with self._data_lock:
            self.latest_geiger_cps = max(0.0, float(msg.data))
        self.geiger_value = self.latest_geiger_cps / 153.8  # Convert to uSv/h using a common conversion factor for Geiger tubes.

    def _on_odom(self, msg: Odometry) -> None:
        stamp = msg.header.stamp
        odom_time = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        now_ts = time.time()

        q = msg.pose.pose.orientation
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        linear_vel = np.array(
            [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.linear.z),
            ],
            dtype=np.float64,
        )
        angular_vel = np.array(
            [
                float(msg.twist.twist.angular.x),
                float(msg.twist.twist.angular.y),
                float(msg.twist.twist.angular.z),
            ],
            dtype=np.float64,
        )

        accel = np.zeros(3, dtype=np.float64)
        if self._last_odom_time is not None and odom_time > self._last_odom_time:
            dt = odom_time - self._last_odom_time
            accel = (linear_vel - self._last_linear_vel) / dt

        self._last_odom_time = odom_time
        self._last_linear_vel = linear_vel
        with self._data_lock:
            self.latest_attitude = (float(roll), float(pitch), float(yaw))

        imu_payload = {
            'timestamp': now_ts,
            'attitude': {
                'roll': float(roll),
                'pitch': float(pitch),
                'yaw': float(yaw),
            },
            'imu': {
                'ax': float(accel[0]),
                'ay': float(accel[1]),
                'az': float(accel[2]),
                'gx': float(angular_vel[0]),
                'gy': float(angular_vel[1]),
                'gz': float(angular_vel[2]),
            },
        }
        camera_imu_payload = {
            'timestamp': now_ts,
            'imu': imu_payload['imu'],
        }

        # try:
        #     # self._udp_sock.sendto(
        #     #     json.dumps(imu_payload).encode('utf-8'),
        #     #     ('<broadcast>', self.imu_broadcast_port),
        #     # )
        #     # self._udp_sock.sendto(
        #     #     json.dumps(camera_imu_payload).encode('utf-8'),
        #     #     ('<broadcast>', self.camera_imu_broadcast_port),
        #     # )
        # except OSError:
        #     return

    def _broadcast_ip(self) -> None:
        try:
            self.local_ip = _resolve_local_ip()
            payload = f'PI:{self.local_ip}'.encode('utf-8')
            self._udp_sock.sendto(
                payload,
                ('255.255.255.255', self.ip_broadcast_port),
            )
            if self.ip_windows_unicast_enabled and self.windows_host_targets:
                win_payload = f'PI:{self.windows_advertise_ip}'.encode('utf-8')
                for target_ip in self.windows_host_targets:
                    self._udp_sock.sendto(
                        win_payload,
                        (target_ip, self.ip_broadcast_port),
                    )
        except OSError:
            return

    def _emit_geiger_values(self) -> None:
        now = time.time()
        self._last_geiger_tick = now

        with self._data_lock:
            cps = self.latest_geiger_cps

        if cps <= 0.0:
            return

        payload = struct.pack('!dd', now, self.geiger_value)
        self._send_geiger_payload(payload)

    def _send_geiger_payload(self, payload: bytes) -> None:
        with self._geiger_clients_lock:
            clients = list(self._geiger_clients)

        if not clients:
            return

        dead_clients: List[socket.socket] = []
        for client in clients:
            try:
                client.sendall(payload)
            except (BrokenPipeError, ConnectionError, OSError, socket.timeout):
                dead_clients.append(client)
        self.get_logger().debug(f'Sent Geiger payload to {len(clients) - len(dead_clients)} clients, {len(dead_clients)} dead clients detected')
        if not dead_clients:
            return

        with self._geiger_clients_lock:
            for client in dead_clients:
                if client in self._geiger_clients:
                    self._geiger_clients.remove(client)
                try:
                    client.close()
                except OSError:
                    pass

    def _tcp_geiger_server_loop(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.geiger_value_port))
        server.listen(4)
        server.settimeout(1.0)
        self._geiger_server_sock = server

        try:
            while rclpy.ok() and self._running:
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    continue

                client.settimeout(1.0)
                with self._geiger_clients_lock:
                    self._geiger_clients.append(client)
                self.get_logger().info(f'Geiger TCP client connected: {addr[0]}:{int(addr[1])}')
        finally:
            try:
                server.close()
            except OSError:
                pass
            self._geiger_server_sock = None

    def _zmq_camera_pub_loop(self) -> None:
        if self._zmq_sock is None or self._rvl_mod is None:
            return

        period = 1.0 / max(1.0, self.camera_zmq_rate_hz)
        target_w = max(1, self.camera_zmq_width)
        target_h = max(1, self.camera_zmq_height)
        while rclpy.ok() and self._running:
            with self._data_lock:
                depth = self.latest_depth
                depth_shape = self.latest_depth_shape
                color = self.latest_color_jpeg

            if depth is None or depth_shape is None or color is None:
                time.sleep(0.01)
                continue

            try:
                width, height = depth_shape
                depth_u16 = np.frombuffer(depth, dtype=np.uint16).reshape((height, width))
                if width != target_w or height != target_h:
                    depth_u16 = cv2.resize(depth_u16, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
                depth_rvl = self._rvl_mod.compress(depth_u16.tobytes())

                color_img = cv2.imdecode(np.frombuffer(color, dtype=np.uint8), cv2.IMREAD_COLOR)
                if color_img is None:
                    time.sleep(0.01)
                    continue
                if color_img.shape[1] != target_w or color_img.shape[0] != target_h:
                    color_img = cv2.resize(color_img, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
                ok, color_jpeg = cv2.imencode('.jpg', color_img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if not ok:
                    time.sleep(0.01)
                    continue

                self._zmq_sock.send_multipart([
                    struct.pack('d', time.time()),
                    color_jpeg.tobytes(),
                    depth_rvl,
                ])
            except Exception:
                # Keep publisher loop alive on transient transport/compression failures.
                time.sleep(0.01)
                continue

            time.sleep(period)



    def _zmq_trajectory_pub_loop(self) -> None:
        if self._zmq_traj_sock is None:
            return

        period = 1.0 / max(1.0, self.camera_zmq_trajectory_rate_hz)
        target_w = max(1, self.camera_zmq_trajectory_width)
        target_h = max(1, self.camera_zmq_trajectory_height)

        while rclpy.ok() and self._running:
            with self._data_lock:
                depth = self.latest_depth
                depth_shape = self.latest_depth_shape
                color = self.latest_color_jpeg

            if depth is None or depth_shape is None or color is None:
                time.sleep(0.01)
                continue

            try:
                width, height = depth_shape
                depth_u16 = np.frombuffer(depth, dtype=np.uint16).reshape((height, width))
                if width != target_w or height != target_h:
                    depth_u16 = cv2.resize(depth_u16, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
                depth_raw = depth_u16.tobytes()

                color_img = cv2.imdecode(np.frombuffer(color, dtype=np.uint8), cv2.IMREAD_COLOR)
                if color_img is None:
                    time.sleep(0.01)
                    continue
                if color_img.shape[1] != target_w or color_img.shape[0] != target_h:
                    color_img = cv2.resize(color_img, (target_w, target_h), interpolation=cv2.INTER_NEAREST)
                ok, color_jpeg = cv2.imencode('.jpg', color_img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                if not ok:
                    time.sleep(0.01)
                    continue

                self._zmq_traj_sock.send_multipart([
                    struct.pack('d', time.time()),
                    color_jpeg.tobytes(),
                    depth_raw,
                ])
            except Exception:
                time.sleep(0.01)
                continue

            time.sleep(period)


    def _fc_zmq_loop(self) -> None:
        if self._fc_sub_sock is None or self._fc_pub_sock is None or self._fc_rep_sock is None:
            return

        try:
            import zmq  # type: ignore
        except Exception:
            return

        loop_period = 1.0 / max(1.0, self.fc_loop_rate_hz)
        while rclpy.ok() and self._running:
            loop_start = time.time()

            try:
                req = self._fc_rep_sock.recv(flags=zmq.NOBLOCK)
                if req == b'TOGGLE_ARM':
                    self._fc_armed = not self._fc_armed
                    self._fc_rep_sock.send(b'1' if self._fc_armed else b'0')
                elif req == b'CHECK_ARM':
                    self._fc_rep_sock.send(b'1' if self._fc_armed else b'0')
                elif req == b'RESET':
                    self._fc_rep_sock.send(b'1')
                    self.get_logger().info('Position reset requested via FC ZMQ API')
                    self.reset_position_pub.publish(Bool(data=True))
                else:
                    self._fc_rep_sock.send(b'-1')
            except zmq.Again:
                pass
            except Exception:
                pass

            while True:
                try:
                    pkt = self._fc_sub_sock.recv(flags=zmq.NOBLOCK)
                    if len(pkt) == 16:
                        r, p, y, t = struct.unpack('<ffff', pkt)
                        self._fc_last_cmd_time = time.time()
                        self._fc_tgt[0] = max(1000, min(2000, 1500 + int(r * 100)))
                        self._fc_tgt[1] = max(1000, min(2000, 1500 + int(p * 100)))
                        self._fc_tgt[2] = max(1000, min(2000, 1000 + int(t * 1000)))
                        self._fc_tgt[3] = max(1000, min(2000, 1500 + int(y * self.fc_yaw_channel_scale)))
                except zmq.Again:
                    break
                except Exception:
                    break

            now = time.time()
            if now - self._fc_last_cmd_time > 0.5:
                self._fc_tgt[0] = 1500
                self._fc_cur[0] = 1500
                self._fc_tgt[1] = 1500
                self._fc_cur[1] = 1500
            if now - self._fc_last_cmd_time > 1.0:
                self._fc_tgt[2] = 1000

            for i in range(4):
                if self._fc_cur[i] < self._fc_tgt[i]:
                    self._fc_cur[i] = min(self._fc_cur[i] + self._fc_inc[i], self._fc_tgt[i])
                elif self._fc_cur[i] > self._fc_tgt[i]:
                    self._fc_cur[i] = max(self._fc_cur[i] - self._fc_inc[i], self._fc_tgt[i])

            twist = Twist()
            if self._fc_armed:
                roll_norm = (self._fc_cur[0] - 1500.0) / 500.0
                pitch_norm = (self._fc_cur[1] - 1500.0) / 500.0
                yaw_norm = (self._fc_cur[3] - 1500.0) / 500.0
                throttle_norm = (self._fc_cur[2] - 1000.0) / 1000.0
                # Godot forward input produces negative pitch command; invert to make forward positive x.
                twist.linear.x = float(-pitch_norm * self.fc_max_forward_mps)
                # Map roll to lateral motion.
                twist.linear.y = float(-roll_norm * self.fc_max_lateral_mps)
                twist.linear.z = float((throttle_norm - 0.5) * 2.0 * self.fc_max_vertical_mps)
                twist.angular.z = float(-yaw_norm * self.fc_max_yaw_rps)
            self.cmd_vel_pub.publish(twist)

            with self._data_lock:
                roll, pitch, yaw = self.latest_attitude
            try:
                # Keep feedback yaw sign aligned with FC control convention.
                self._fc_pub_sock.send(struct.pack('<fff', float(roll), float(pitch), float(-yaw)))
            except Exception:
                pass

            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    def _tcp_camera_server_loop(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.camera_tcp_port))
        server.listen(1)
        server.settimeout(1.0)
        frame_period = 1.0 / max(1.0, self.camera_stream_rate_hz)

        try:
            while rclpy.ok() and self._running:
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    continue

                self.get_logger().info(f'Camera client connected: {addr[0]}:{int(addr[1])}')
                client.settimeout(3.0)
                try:
                    while rclpy.ok() and self._running:
                        with self._data_lock:
                            depth = self.latest_depth
                            depth_shape = self.latest_depth_shape
                            color = self.latest_color_jpeg

                        if depth is None or depth_shape is None or color is None:
                            time.sleep(0.01)
                            continue

                        width, height = depth_shape
                        header = struct.pack('<IIII', len(depth), len(color), width, height)
                        client.sendall(header + depth + color)
                        time.sleep(frame_period)
                except (BrokenPipeError, ConnectionError, OSError, socket.timeout):
                    pass
                finally:
                    try:
                        client.close()
                    except OSError:
                        pass
                    self.get_logger().info('Camera client disconnected')
        finally:
            try:
                server.close()
            except OSError:
                pass

    def destroy_node(self) -> None:
        self._running = False
        try:
            self._udp_sock.close()
        except OSError:
            pass
        if self._geiger_server_sock is not None:
            try:
                self._geiger_server_sock.close()
            except OSError:
                pass
            self._geiger_server_sock = None
        with self._geiger_clients_lock:
            geiger_clients = list(self._geiger_clients)
            self._geiger_clients.clear()
        for client in geiger_clients:
            try:
                client.close()
            except OSError:
                pass
        if self._zmq_sock is not None:
            try:
                self._zmq_sock.close(0)
            except Exception:
                pass
            self._zmq_sock = None
        if self._zmq_traj_sock is not None:
            try:
                self._zmq_traj_sock.close(0)
            except Exception:
                pass
            self._zmq_traj_sock = None
        if self._fc_sub_sock is not None:
            try:
                self._fc_sub_sock.close(0)
            except Exception:
                pass
            self._fc_sub_sock = None
        if self._fc_pub_sock is not None:
            try:
                self._fc_pub_sock.close(0)
            except Exception:
                pass
            self._fc_pub_sock = None
        if self._fc_rep_sock is not None:
            try:
                self._fc_rep_sock.close(0)
            except Exception:
                pass
            self._fc_rep_sock = None
        if self._zmq_ctx is not None:
            try:
                self._zmq_ctx.term()
            except Exception:
                pass
            self._zmq_ctx = None
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
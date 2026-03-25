import logging
import socket
import struct
import threading
import time
from signal import pause
from typing import List, Optional

from gpiozero import Button


class GeigerTcpEmitter:
    def __init__(self, geiger_value_port: int = 10350) -> None:
        self.geiger_value_port = geiger_value_port
        self._running = True

        self._last_geiger_tick = 0.0
        self.last_timestamp = -1.0
        self.last_usv = -1.0
        self.latest_geiger_cps = 0.0
        self.geiger_value = 0.0

        self._data_lock = threading.Lock()
        self._geiger_clients_lock = threading.Lock()
        self._geiger_clients: List[socket.socket] = []
        self._geiger_server_sock: Optional[socket.socket] = None

        self._server_thread = threading.Thread(
            target=self._tcp_geiger_server_loop,
            name="geiger-tcp-server",
            daemon=True,
        )

        self._logger = logging.getLogger("drone.geiger")

    def start(self) -> None:
        self._server_thread.start()

    def stop(self) -> None:
        self._running = False

        server = self._geiger_server_sock
        if server is not None:
            try:
                server.close()
            except OSError:
                pass

        with self._geiger_clients_lock:
            clients = list(self._geiger_clients)
            self._geiger_clients.clear()

        for client in clients:
            try:
                client.close()
            except OSError:
                pass

    def on_radiation_pulse(self) -> None:
        now = time.time()

        with self._data_lock:
            if self.last_timestamp != -1.0:
                delta = now - self.last_timestamp
                if delta > 0.0:
                    cpm = 60.0 / delta
                    usv = cpm / 153.8
                    self.last_usv = (
                        (0.2 * self.last_usv + 0.8 * usv)
                        if self.last_usv != -1.0
                        else usv
                    )
                    self.latest_geiger_cps = cpm / 60.0
                    self.geiger_value = self.last_usv
            self.last_timestamp = now

        self._emit_geiger_values()

    def _emit_geiger_values(self) -> None:
        now = time.time()
        self._last_geiger_tick = now

        with self._data_lock:
            cps = self.latest_geiger_cps

        if cps <= 0.0:
            return

        payload = struct.pack("!dd", now, self.geiger_value)
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

        self._logger.debug(
            "Sent Geiger payload to %d clients, %d dead clients detected",
            len(clients) - len(dead_clients),
            len(dead_clients),
        )

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
        server.bind(("0.0.0.0", self.geiger_value_port))
        server.listen(4)
        server.settimeout(1.0)
        self._geiger_server_sock = server

        try:
            while self._running:
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    continue

                client.settimeout(1.0)
                with self._geiger_clients_lock:
                    self._geiger_clients.append(client)

                self._logger.info(
                    "Geiger TCP client connected: %s:%d", addr[0], int(addr[1])
                )
        finally:
            try:
                server.close()
            except OSError:
                pass
            self._geiger_server_sock = None


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(name)s: %(message)s",
        force=True,
    )

    emitter = GeigerTcpEmitter(geiger_value_port=10350)
    emitter.start()

    geiger = Button(17, pull_up=True)
    geiger.when_pressed = emitter.on_radiation_pulse

    try:
        pause()
    finally:
        emitter.stop()


if __name__ == "__main__":
    main()
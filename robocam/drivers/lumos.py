"""XVisio FastUMI Pro Lumos tracker driver.

The Lumos hardware is consumed indirectly: ``xv_sdk`` runs inside a docker
container (ROS1 Noetic) and small companion senders ship pose + JPEG frames
over loopback TCP. This driver is the **TCP receiver** — it listens on two
ports, parses the framed binary protocol, and exposes the latest frame as
``CameraData`` like any other robocam driver.

Stack lifecycle (docker / xv_sdk / senders) is managed separately. Use
``robocam.drivers.lumos_stack`` to bring it up before instantiating, or
run ``python -m robocam.drivers.lumos_stack up`` from the shell.

Wire formats
------------
Image port (default 28998), length-prefixed binary, big-endian::

  [4 bytes : total_len]
  [2 bytes : header_len]
  [header_len bytes : utf-8 JSON header]
  [total_len - 2 - header_len bytes : payload]

  kind="image" (default if absent), payload = JPEG bytes:
    {"kind": "image",
     "topic": "left/fisheye/l"|"left/fisheye/r"|"left/color"|"right/...",
     "stamp_ns": int, "w": int, "h": int}

  kind="camera_info", payload empty:
    {"kind": "camera_info", "topic": same as above,
     "K": [9], "D": [...], "R": [9], "P": [12],
     "distortion_model": str, "width": int, "height": int}

Pose port (default 28999), newline-delimited JSON, one envelope per ~10 ms::

  {"left_pose":  {"position":[x,y,z], "orientation":[qx,qy,qz,qw]} | null,
   "right_pose": {...} | null,
   "left_clamp": float | null,
   "right_clamp": float | null,
   "left_imu":   {"stamp_ns": int,
                  "acceleration": [x,y,z], "gyroscope": [x,y,z]} | null,
   "right_imu":  {...} | null}
"""

from __future__ import annotations

import json
import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np
from loguru import logger

from robocam.camera import CameraData, IMUData
from robocam.drivers.lumos_stack import DEFAULT_IMAGE_TCP_PORT, DEFAULT_TCP_PORT

try:
    import cv2
except ImportError as _e:
    raise ImportError("opencv (cv2) is required: pip install opencv-contrib-python") from _e


# wire-protocol topic names produced by fastumi_image_tcp_sender.py
_TOPIC_FISHEYE_L = "fisheye/l"
_TOPIC_FISHEYE_R = "fisheye/r"
_TOPIC_COLOR = "color"


def _recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
    buf = bytearray()
    while len(buf) < n:
        try:
            chunk = sock.recv(n - len(buf))
        except (ConnectionResetError, OSError):
            return None
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


@dataclass
class LumosCamera:
    """FastUMI Pro Lumos tracker as a robocam camera.

    Parameters
    ----------
    side : str
        Which tracker to read — ``"left"`` or ``"right"``. Frames whose wire
        topic doesn't start with ``f"{side}/"`` are dropped.
    enable_color : bool
        Decode and expose the RGB stream under ``data.images["rgb"]``.
        Note: USB bandwidth contention can drop fisheye to ~12 Hz when on.
        Default off.
    listen_host : str
        Bind address for the TCP servers. Default ``"0.0.0.0"`` (all).
    image_tcp_port : int
        Port the in-container ``fastumi_image_tcp_sender.py`` connects to.
        Defaults to ``$FASTUMI_IMAGE_TCP_PORT`` if set, else 28998 — same
        precedence the stack uses, so they can't desync.
    pose_tcp_port : int
        Port for the pose+clamp sender. Set to 0 to disable pose ingest
        (then ``data.other_sensors`` will be empty). Defaults to
        ``$FASTUMI_TCP_PORT`` if set, else 28999.
    read_timeout_s : float
        ``read()`` raises ``TimeoutError`` if no new fisheye_left frame
        arrives within this many seconds. Default 5.
    name : str or None
        Human-readable label.

    Notes
    -----
    Only one ``LumosCamera`` instance can exist per host because both TCP
    sockets are exclusively bound. To consume from multiple processes,
    use ROS2 (a separate driver, not provided here).
    """

    side: str = "left"
    enable_color: bool = False
    listen_host: str = "0.0.0.0"
    image_tcp_port: int = field(default_factory=lambda: int(DEFAULT_IMAGE_TCP_PORT))
    pose_tcp_port: int = field(default_factory=lambda: int(DEFAULT_TCP_PORT))
    read_timeout_s: float = 5.0
    camera_type: str = "lumos_camera"
    name: Optional[str] = None

    _images: Dict[str, np.ndarray] = field(init=False, repr=False, default_factory=dict)
    _image_stamps: Dict[str, float] = field(init=False, repr=False, default_factory=dict)
    _intrinsics: Dict[str, Dict[str, Any]] = field(init=False, repr=False, default_factory=dict)
    _pose: Optional[Dict[str, Any]] = field(init=False, repr=False, default=None)
    _clamp: Optional[float] = field(init=False, repr=False, default=None)
    _imu: Optional[IMUData] = field(init=False, repr=False, default=None)
    _anchor_seq: int = field(init=False, repr=False, default=0)
    _cv: threading.Condition = field(init=False, repr=False)
    _stopped: bool = field(init=False, repr=False, default=False)
    _img_sock: Optional[socket.socket] = field(init=False, repr=False, default=None)
    _pose_sock: Optional[socket.socket] = field(init=False, repr=False, default=None)
    _img_thread: Optional[threading.Thread] = field(init=False, repr=False, default=None)
    _pose_thread: Optional[threading.Thread] = field(init=False, repr=False, default=None)
    _connected_at: Optional[float] = field(init=False, repr=False, default=None)

    def __repr__(self) -> str:
        return f"LumosCamera(side={self.side!r}, name={self.name!r}, enable_color={self.enable_color})"

    def __post_init__(self) -> None:
        if self.side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {self.side!r}")
        self._cv = threading.Condition()
        self._img_sock = self._bind_listen(self.image_tcp_port)
        self._img_thread = threading.Thread(target=self._image_server_loop, daemon=True, name="lumos-img-srv")
        self._img_thread.start()
        if self.pose_tcp_port:
            self._pose_sock = self._bind_listen(self.pose_tcp_port)
            self._pose_thread = threading.Thread(target=self._pose_server_loop, daemon=True, name="lumos-pose-srv")
            self._pose_thread.start()
        logger.info(
            "LumosCamera listening: image={}:{}, pose={}:{}, side={}",
            self.listen_host, self.image_tcp_port,
            self.listen_host, self.pose_tcp_port if self.pose_tcp_port else "disabled",
            self.side,
        )

    def _bind_listen(self, port: int) -> socket.socket:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((self.listen_host, port))
        except OSError as e:
            s.close()
            raise RuntimeError(
                f"LumosCamera could not bind {self.listen_host}:{port} ({e}). "
                "Another process is using it — usually launch_fastumi_ros2.py "
                "or another LumosCamera. Stop that first."
            ) from e
        s.listen(1)
        s.settimeout(0.5)
        return s

    # ---- image server (port 28998) ---------------------------------------
    def _image_server_loop(self) -> None:
        assert self._img_sock is not None
        while not self._stopped:
            try:
                conn, addr = self._img_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            logger.info("LumosCamera image sender connected: {}", addr)
            self._connected_at = time.time()
            conn.settimeout(30.0)
            try:
                self._image_recv_loop(conn)
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
                logger.info("LumosCamera image sender disconnected")

    def _image_recv_loop(self, conn: socket.socket) -> None:
        side_prefix = f"{self.side}/"
        while not self._stopped:
            hdr_raw = _recv_exact(conn, 4)
            if hdr_raw is None:
                return
            (total_len,) = struct.unpack(">I", hdr_raw)
            payload = _recv_exact(conn, total_len)
            if payload is None:
                return
            (hdr_len,) = struct.unpack(">H", payload[:2])
            try:
                header = json.loads(payload[2 : 2 + hdr_len].decode("utf-8"))
            except Exception as e:
                logger.warning("LumosCamera bad image header: {}", e)
                continue
            body = payload[2 + hdr_len :]
            wire_topic = header.get("topic", "")
            if not wire_topic.startswith(side_prefix):
                continue  # other tracker side
            local_topic = wire_topic[len(side_prefix) :]  # "fisheye/l" / "color"
            if local_topic == _TOPIC_COLOR and not self.enable_color:
                continue
            key = self._wire_to_key(local_topic)
            kind = header.get("kind", "image")
            if kind == "camera_info":
                self._store_camera_info(key, header)
                continue
            arr = cv2.imdecode(np.frombuffer(body, np.uint8), cv2.IMREAD_UNCHANGED)
            if arr is None:
                logger.warning("LumosCamera failed to decode JPEG ({} bytes)", len(body))
                continue
            stamp_ns = int(header.get("stamp_ns", 0))
            stamp_ms = stamp_ns / 1e6 if stamp_ns > 0 else time.time() * 1000.0
            with self._cv:
                self._images[key] = arr
                self._image_stamps[key] = stamp_ms
                if key == "fisheye_left":
                    self._anchor_seq += 1
                    self._cv.notify_all()

    def _store_camera_info(self, key: str, header: Dict[str, Any]) -> None:
        try:
            K = np.array(header["K"], dtype=np.float64).reshape(3, 3)
            D = np.array(header.get("D", []), dtype=np.float64)
        except (KeyError, ValueError) as e:
            logger.warning("LumosCamera bad camera_info for {}: {}", key, e)
            return
        width = int(header.get("width", 0))
        height = int(header.get("height", 0))
        # xv_sdk publishes empty CameraInfo on fisheye topics (width=height=0,
        # K all zeros) — it doesn't expose those intrinsics over ROS. Skip
        # rather than expose misleading zero-K entries to consumers.
        if width == 0 or height == 0 or not np.any(K):
            return
        info: Dict[str, Any] = {
            "K": K,
            "D": D,
            "width": width,
            "height": height,
            "distortion_model": str(header.get("distortion_model", "")),
        }
        R = header.get("R")
        P = header.get("P")
        if R and np.any(R):
            info["R"] = np.array(R, dtype=np.float64).reshape(3, 3)
        if P and np.any(P):
            info["P"] = np.array(P, dtype=np.float64).reshape(3, 4)
        with self._cv:
            self._intrinsics[key] = info

    @staticmethod
    def _wire_to_key(local_topic: str) -> str:
        return {
            _TOPIC_FISHEYE_L: "fisheye_left",
            _TOPIC_FISHEYE_R: "fisheye_right",
            _TOPIC_COLOR: "rgb",
        }.get(local_topic, local_topic)

    # ---- pose server (port 28999) ----------------------------------------
    def _pose_server_loop(self) -> None:
        assert self._pose_sock is not None
        while not self._stopped:
            try:
                conn, addr = self._pose_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            logger.info("LumosCamera pose sender connected: {}", addr)
            conn.settimeout(30.0)
            try:
                self._pose_recv_loop(conn)
            finally:
                try:
                    conn.close()
                except Exception:
                    pass
                logger.info("LumosCamera pose sender disconnected")

    def _pose_recv_loop(self, conn: socket.socket) -> None:
        buf = b""
        pose_key = f"{self.side}_pose"
        clamp_key = f"{self.side}_clamp"
        imu_key = f"{self.side}_imu"
        while not self._stopped:
            try:
                chunk = conn.recv(4096)
            except (socket.timeout, ConnectionResetError, OSError):
                return
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    state = json.loads(line.decode("utf-8"))
                except Exception:
                    continue
                imu_dict = state.get(imu_key)
                imu = self._imu_from_dict(imu_dict) if imu_dict else None
                with self._cv:
                    self._pose = state.get(pose_key)
                    clamp = state.get(clamp_key)
                    self._clamp = float(clamp) if clamp is not None else None
                    if imu is not None:
                        self._imu = imu

    @staticmethod
    def _imu_from_dict(d: Dict[str, Any]) -> Optional[IMUData]:
        try:
            stamp_ns = int(d.get("stamp_ns", 0))
            stamp_ms = stamp_ns / 1e6 if stamp_ns > 0 else time.time() * 1000.0
            acc = d.get("acceleration")
            gyr = d.get("gyroscope")
            return IMUData(
                timestamp=stamp_ms,
                acceleration=tuple(acc) if acc else None,
                gyroscope=tuple(gyr) if gyr else None,
            )
        except (TypeError, ValueError):
            return None

    # ---- public API ------------------------------------------------------
    def read(self) -> CameraData:
        deadline = time.time() + self.read_timeout_s
        with self._cv:
            last_seen_seq = getattr(self, "_last_returned_seq", 0)
            while self._anchor_seq <= last_seen_seq and not self._stopped:
                remaining = deadline - time.time()
                if remaining <= 0:
                    raise TimeoutError(
                        f"LumosCamera: no fisheye_left frame in {self.read_timeout_s}s. "
                        "Is the docker stack running? "
                        "Try `python -m robocam.drivers.lumos_stack up`."
                    )
                self._cv.wait(timeout=min(0.5, remaining))
            if self._stopped:
                raise RuntimeError("LumosCamera stopped")
            self._last_returned_seq = self._anchor_seq  # type: ignore[attr-defined]
            images = dict(self._images)
            timestamp = self._image_stamps.get("fisheye_left", time.time() * 1000.0)
            other: Dict[str, Any] = {}
            if self._pose is not None:
                other["pose"] = self._pose
            if self._clamp is not None:
                other["clamp"] = self._clamp
            imu = self._imu
        return CameraData(
            images=images,
            timestamp=timestamp,
            other_sensors=other or None,
            imu_data=imu,
        )

    def is_connected(self) -> bool:
        with self._cv:
            return self._anchor_seq > 0

    def get_camera_info(self) -> Dict[str, Any]:
        with self._cv:
            shape = self._images.get("fisheye_left")
            shape_t = (int(shape.shape[0]), int(shape.shape[1])) if shape is not None else (None, None)
        return {
            "camera_type": self.camera_type,
            "side": self.side,
            "name": self.name or "lumos_camera",
            "enable_color": self.enable_color,
            "image_tcp_port": self.image_tcp_port,
            "pose_tcp_port": self.pose_tcp_port,
            "fisheye_size": shape_t,
            "connected": self.is_connected(),
        }

    def read_calibration_data_intrinsics(self) -> Dict[str, Any]:
        with self._cv:
            # Shallow-copy each per-stream dict; numpy arrays are immutable
            # enough for read-only consumers and the inner dicts are small.
            return {key: dict(info) for key, info in self._intrinsics.items()}

    def stop(self) -> None:
        self._stopped = True
        with self._cv:
            self._cv.notify_all()
        for sock in (self._img_sock, self._pose_sock):
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        for t in (self._img_thread, self._pose_thread):
            if t is not None and t.is_alive():
                t.join(timeout=2.0)
        logger.info("LumosCamera stopped")

    @staticmethod
    def discover_devices() -> List[Dict[str, str]]:
        """List XVisio trackers visible on the host via sysfs.

        Returns ``[{"serial": uuid, "sysfs": path}, ...]``. Detects the
        device on the host, not inside the docker container.
        """
        import glob
        import os

        out = []
        for d in sorted(glob.glob("/sys/bus/usb/devices/*")):
            vid = os.path.join(d, "idVendor")
            ser = os.path.join(d, "serial")
            try:
                with open(vid) as f:
                    if f.read().strip().lower() != "040e":
                        continue
                with open(ser) as f:
                    serial = f.read().strip()
            except (FileNotFoundError, OSError):
                continue
            if serial:
                out.append({"serial": serial, "sysfs": d})
        return out

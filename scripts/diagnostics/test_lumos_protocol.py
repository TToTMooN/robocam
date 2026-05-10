#!/usr/bin/env python3
"""Loopback protocol test for LumosCamera.

No hardware, no docker — fakes the two in-container senders over loopback
TCP and asserts the receiver demuxes/parses the wire format correctly.

Usage:
    uv run scripts/diagnostics/test_lumos_protocol.py
"""

from __future__ import annotations

import json
import socket
import struct
import time

import cv2
import numpy as np
from loguru import logger

from robocam.drivers.lumos import LumosCamera


# Distinct port pairs per test so a previous test's TIME_WAIT socket
# can't block the next one — cheaper than coordinating SO_REUSEADDR.
_BASE = 38998


def _ports(offset: int) -> tuple[int, int]:
    return _BASE + offset, _BASE + offset + 1


def _pack(header_dict: dict, payload: bytes = b"") -> bytes:
    header = json.dumps(header_dict).encode("utf-8")
    total_len = 2 + len(header) + len(payload)
    return struct.pack(">IH", total_len, len(header)) + header + payload


def _jpeg(h: int = 60, w: int = 80, fill: int = 128) -> bytes:
    img = np.full((h, w, 3), fill, dtype=np.uint8)
    ok, buf = cv2.imencode(".jpg", img)
    assert ok
    return buf.tobytes()


def _connect(port: int, attempts: int = 40) -> socket.socket:
    """LumosCamera needs a moment to bind+listen; retry briefly."""
    for _ in range(attempts):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(("127.0.0.1", port))
            return s
        except ConnectionRefusedError:
            time.sleep(0.05)
    raise RuntimeError(f"Could not connect to 127.0.0.1:{port}")


def _make_cam(offset: int, side: str = "left", **kwargs) -> LumosCamera:
    image_port, pose_port = _ports(offset)
    return LumosCamera(
        side=side,
        listen_host="127.0.0.1",
        image_tcp_port=image_port,
        pose_tcp_port=pose_port,
        read_timeout_s=2.0,
        **kwargs,
    )


def test_image_round_trip() -> None:
    """A JPEG frame surfaces in cam.read().images with the right key and timestamp."""
    cam = _make_cam(0)
    try:
        s = _connect(_ports(0)[0])
        s.sendall(_pack(
            {"kind": "image", "topic": "left/fisheye/l", "stamp_ns": 123_000_000,
             "w": 80, "h": 60},
            _jpeg(60, 80),
        ))
        d = cam.read()
        assert "fisheye_left" in d.images, f"keys={list(d.images.keys())}"
        assert d.images["fisheye_left"].shape[:2] == (60, 80)
        assert abs(d.timestamp - 123.0) < 0.001, f"timestamp={d.timestamp}"
        logger.info("PASS image_round_trip: shape={}, ts={:.3f}ms",
                    d.images["fisheye_left"].shape, d.timestamp)
        s.close()
    finally:
        cam.stop()


def test_camera_info_round_trip() -> None:
    """K/D/width/height arrive intact and are returned as numpy from read_calibration."""
    cam = _make_cam(2)
    try:
        s = _connect(_ports(2)[0])
        s.sendall(_pack({
            "kind": "camera_info", "topic": "left/fisheye/l",
            "K": [400.0, 0, 320, 0, 400, 240, 0, 0, 1],
            "D": [0.1, -0.2, 0.0, 0.0, 0.05],
            "R": [], "P": [],
            "distortion_model": "plumb_bob",
            "width": 640, "height": 480,
        }))
        # Send a frame to unblock read() and ensure the camera_info packet
        # has been processed (both share the same recv loop, in order).
        s.sendall(_pack(
            {"kind": "image", "topic": "left/fisheye/l", "stamp_ns": 1,
             "w": 80, "h": 60},
            _jpeg(),
        ))
        cam.read()
        intr = cam.read_calibration_data_intrinsics()
        assert "fisheye_left" in intr, f"keys={list(intr.keys())}"
        info = intr["fisheye_left"]
        assert info["width"] == 640 and info["height"] == 480
        assert info["distortion_model"] == "plumb_bob"
        assert info["K"].shape == (3, 3)
        assert abs(info["K"][0, 0] - 400.0) < 1e-6
        assert len(info["D"]) == 5
        logger.info("PASS camera_info_round_trip: K[0,0]={}, D_len={}, model={}",
                    info["K"][0, 0], len(info["D"]), info["distortion_model"])
        s.close()
    finally:
        cam.stop()


def test_pose_and_imu() -> None:
    """left_pose, left_clamp, left_imu in the JSON envelope land on CameraData."""
    cam = _make_cam(4)
    image_port, pose_port = _ports(4)
    try:
        p = _connect(pose_port)
        envelope = {
            "left_pose": {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]},
            "right_pose": None,
            "left_clamp": 0.42,
            "right_clamp": None,
            "left_imu": {"stamp_ns": 500_000_000,
                         "acceleration": [0, 0, -9.81],
                         "gyroscope": [0.01, 0.02, 0.03]},
            "right_imu": None,
        }
        p.sendall(json.dumps(envelope).encode("utf-8") + b"\n")
        i = _connect(image_port)
        i.sendall(_pack(
            {"kind": "image", "topic": "left/fisheye/l", "stamp_ns": 1,
             "w": 80, "h": 60},
            _jpeg(),
        ))
        d = cam.read()
        assert d.imu_data is not None, "imu_data is None"
        assert d.imu_data.acceleration == (0, 0, -9.81)
        assert d.imu_data.gyroscope == (0.01, 0.02, 0.03)
        assert d.other_sensors is not None
        assert d.other_sensors["pose"] == envelope["left_pose"]
        assert d.other_sensors["clamp"] == 0.42
        logger.info("PASS pose_and_imu: acc={}, gyr={}, clamp={}",
                    d.imu_data.acceleration, d.imu_data.gyroscope,
                    d.other_sensors["clamp"])
        p.close(); i.close()
    finally:
        cam.stop()


def test_side_filtering() -> None:
    """LumosCamera(side='left') drops right/* frames before they overwrite anything."""
    cam = _make_cam(6, side="left")
    try:
        s = _connect(_ports(6)[0])
        # Right side first with a distinguishable fill — must be discarded.
        s.sendall(_pack(
            {"kind": "image", "topic": "right/fisheye/l", "stamp_ns": 1,
             "w": 80, "h": 60},
            _jpeg(fill=200),
        ))
        # Then a left frame to unblock read().
        s.sendall(_pack(
            {"kind": "image", "topic": "left/fisheye/l", "stamp_ns": 2,
             "w": 80, "h": 60},
            _jpeg(fill=50),
        ))
        d = cam.read()
        assert "fisheye_left" in d.images
        # If the side filter were broken, the right frame would have been
        # decoded under "fisheye_left" too (same local_topic). Detect via
        # mean pixel value: left=50, right=200.
        mean = float(d.images["fisheye_left"].mean())
        assert mean < 100, f"right frame leaked through (mean={mean:.1f})"
        logger.info("PASS side_filtering: mean={:.1f} (left=~50)", mean)
        s.close()
    finally:
        cam.stop()


def test_camera_info_persists_across_disconnect() -> None:
    """Intrinsics cached on the receiver survive sender disconnect."""
    cam = _make_cam(8)
    image_port = _ports(8)[0]
    try:
        a = _connect(image_port)
        a.sendall(_pack({
            "kind": "camera_info", "topic": "left/fisheye/l",
            "K": [400, 0, 320, 0, 400, 240, 0, 0, 1],
            "D": [0.1], "R": [], "P": [],
            "distortion_model": "plumb_bob",
            "width": 640, "height": 480,
        }))
        a.sendall(_pack(
            {"kind": "image", "topic": "left/fisheye/l", "stamp_ns": 1,
             "w": 80, "h": 60},
            _jpeg(),
        ))
        cam.read()
        a.close()
        time.sleep(0.2)  # let the recv loop notice the disconnect
        intr = cam.read_calibration_data_intrinsics()
        assert "fisheye_left" in intr, f"intrinsics lost on disconnect: {list(intr.keys())}"
        logger.info("PASS camera_info_persists_across_disconnect")
    finally:
        cam.stop()


def test_empty_camera_info_filtered() -> None:
    """CameraInfo with width=0 or K=zeros is dropped (xv_sdk fisheye case)."""
    cam = _make_cam(10)
    try:
        s = _connect(_ports(10)[0])
        s.sendall(_pack({
            "kind": "camera_info", "topic": "left/fisheye/l",
            "K": [0] * 9, "D": [], "R": [], "P": [],
            "distortion_model": "", "width": 0, "height": 0,
        }))
        s.sendall(_pack(
            {"kind": "image", "topic": "left/fisheye/l", "stamp_ns": 1,
             "w": 80, "h": 60},
            _jpeg(),
        ))
        cam.read()
        intr = cam.read_calibration_data_intrinsics()
        assert "fisheye_left" not in intr, f"empty camera_info leaked: {intr}"
        logger.info("PASS empty_camera_info_filtered")
        s.close()
    finally:
        cam.stop()


def test_read_timeout() -> None:
    """read() raises TimeoutError after read_timeout_s when no frames arrive."""
    cam = _make_cam(12)
    try:
        t0 = time.monotonic()
        try:
            cam.read()
        except TimeoutError:
            elapsed = time.monotonic() - t0
            assert 1.8 < elapsed < 3.0, f"unexpected elapsed={elapsed:.2f}s"
            logger.info("PASS read_timeout: raised after {:.2f}s", elapsed)
            return
        raise AssertionError("read() should have raised TimeoutError")
    finally:
        cam.stop()


def main() -> int:
    test_image_round_trip()
    test_camera_info_round_trip()
    test_pose_and_imu()
    test_side_filtering()
    test_camera_info_persists_across_disconnect()
    test_empty_camera_info_filtered()
    test_read_timeout()
    logger.info("All tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

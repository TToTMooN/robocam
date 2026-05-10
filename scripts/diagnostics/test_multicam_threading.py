#!/usr/bin/env python3
"""Hardware test: verify multi-camera capture with correct threading model.

Discovers all connected cameras (ZED + RealSense) and tests capture using the
appropriate strategy for each SDK:
- **ZED** — CaptureThread (background daemon thread). grab() is thread-safe.
- **RealSense** — main-thread sequential polling. wait_for_frames() requires
  the main thread; background threads stall after ~16 queued frames.

Usage:
    uv run scripts/diagnostics/test_multicam_threading.py
    uv run scripts/diagnostics/test_multicam_threading.py --duration 5.0
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict

import tyro
from loguru import logger

from robocam.camera import CameraDriver
from robocam.capture_thread import CaptureThread
from robocam.frame_buffer import FrameBuffer


@dataclass
class Args:
    """Test multi-camera capture with real hardware."""

    duration: float = 3.0
    """Seconds to run capture before reporting."""
    buffer_size: int = 32
    """FrameBuffer max_size per camera."""


def discover_cameras() -> tuple[Dict[str, CameraDriver], Dict[str, CameraDriver]]:
    """Discover and open all available cameras, split by type.

    Returns (rs_cameras, zed_cameras).
    """
    rs_cameras: Dict[str, CameraDriver] = {}
    zed_cameras: Dict[str, CameraDriver] = {}

    try:
        from robocam.drivers.realsense import RealsenseCamera, discover_devices

        for dev in discover_devices():
            label = f"realsense:{dev['serial']}"
            logger.info("Opening {}", label)
            try:
                rs_cameras[label] = RealsenseCamera(serial_number=dev["serial"], fps=30, enable_depth=True)
            except Exception as e:
                logger.warning("Failed to open {}: {}", label, e)
    except ImportError:
        logger.debug("pyrealsense2 not available")

    try:
        from robocam.drivers.zed import ZedCamera, discover_devices as zed_discover

        for dev in zed_discover():
            serial = dev["serial"]
            label = f"zed:{serial}"
            logger.info("Opening {}", label)
            try:
                zed_cameras[label] = ZedCamera(device_id=serial, fps=30, enable_depth=True)
            except Exception as e:
                logger.warning("Failed to open {}: {}", label, e)
    except ImportError:
        logger.debug("pyzed not available")

    return rs_cameras, zed_cameras


def main() -> None:
    args = tyro.cli(Args)

    rs_cameras, zed_cameras = discover_cameras()
    total = len(rs_cameras) + len(zed_cameras)
    if total == 0:
        logger.error("No cameras found. Connect hardware and retry.")
        return

    logger.info(
        "Found {} camera(s): {} RealSense (main-thread), {} ZED (CaptureThread)",
        total,
        len(rs_cameras),
        len(zed_cameras),
    )

    # ZED: start CaptureThreads
    zed_threads: Dict[str, CaptureThread] = {}
    zed_buffers: Dict[str, FrameBuffer] = {}
    for label, cam in zed_cameras.items():
        buf = FrameBuffer(max_size=args.buffer_size)
        ct = CaptureThread(camera_id=label, camera=cam, buffer=buf)
        zed_buffers[label] = buf
        zed_threads[label] = ct
        ct.start()

    # RealSense: poll on main thread
    rs_frame_counts: Dict[str, int] = {label: 0 for label in rs_cameras}
    rs_errors: Dict[str, int] = {label: 0 for label in rs_cameras}

    logger.info("Running for {:.1f}s...", args.duration)
    t0 = time.time()

    while time.time() - t0 < args.duration:
        for label, cam in rs_cameras.items():
            try:
                data = cam.read()
                rs_frame_counts[label] += 1
            except Exception as e:
                rs_errors[label] += 1
                if rs_errors[label] <= 3:
                    logger.warning("  {} read error: {}", label, e)

    elapsed = time.time() - t0

    # Report
    all_ok = True
    logger.info("--- Results ({:.1f}s) ---", elapsed)

    for label in rs_cameras:
        fps = rs_frame_counts[label] / elapsed
        status = "OK" if rs_frame_counts[label] > 0 and rs_errors[label] == 0 else "WARN"
        if rs_frame_counts[label] == 0:
            status = "FAILED"
            all_ok = False
        logger.info(
            "  {} [main-thread]: {} | {} frames | {:.1f} FPS | {} errors",
            label,
            status,
            rs_frame_counts[label],
            fps,
            rs_errors[label],
        )

    for label, ct in zed_threads.items():
        fps = ct.frame_count / elapsed
        buf_len = len(zed_buffers[label])
        status = "OK" if not ct.failed and ct.is_alive() else "FAILED"
        if ct.failed or not ct.is_alive():
            all_ok = False
        logger.info(
            "  {} [CaptureThread]: {} | {} frames | {:.1f} FPS | buffer: {}/{}",
            label,
            status,
            ct.frame_count,
            fps,
            buf_len,
            args.buffer_size,
        )

    # Verify we can read from ZED buffers
    for label, buf in zed_buffers.items():
        try:
            frame = buf.get_latest(timeout_sec=0.5)
            has_images = len(frame.images) > 0
            logger.info("  {} buffer read: images={}, ts={:.1f}ms", label, has_images, frame.timestamp)
        except TimeoutError:
            logger.error("  {} buffer read: TIMEOUT — buffer empty", label)
            all_ok = False

    # Shutdown
    for ct in zed_threads.values():
        ct.stop(timeout=2.0)
    for cam in {**rs_cameras, **zed_cameras}.values():
        cam.stop()

    if all_ok:
        logger.info("PASS — all cameras captured successfully")
    else:
        logger.error("FAIL — some cameras had issues")


if __name__ == "__main__":
    main()

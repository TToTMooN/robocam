#!/usr/bin/env python3
"""Live viewer for Insta360 X5 cameras over the in-process SDK driver.

One window per camera. Each camera is pulled by its own daemon thread, since
``Insta360Camera.read()`` blocks until the next frame; the display loop only
ever renders the latest frame, so a slow window never throttles capture.

The overlay reports stamp lag (``now - CameraData.timestamp``), which is the
number this driver exists to keep small: expect ~131 ms mean at 1920x960.

The camera must be in Android USB mode, and no other SDK session (including
the ROS2 driver) may hold it.

Controls
--------
s  - toggle camera-info overlay
r  - start / stop video recording (uses AsyncVideoWriter + NVENC)
q  - quit (or ESC)

Examples
--------
    uv run scripts/view_insta360.py
    uv run scripts/view_insta360.py --lens full --resolution 2560x1280
    uv run scripts/view_insta360.py --serials SERIAL_A,SERIAL_B
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np
import tyro
from loguru import logger

from robocam import AsyncVideoWriter
from robocam.camera import CameraData
from robocam.drivers.insta360 import Insta360Camera


def overlay_text(image: np.ndarray, text: str, position: tuple = (10, 30)) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale, thickness = 0.7, 2
    cv2.putText(image, text, (position[0] + 1, position[1] + 1), font, scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
    cv2.putText(image, text, position, font, scale, (0, 255, 0), thickness, cv2.LINE_AA)


@dataclass
class Args:
    """Insta360 camera live viewer & recorder."""

    serials: str = ""
    """Comma-separated SDK serials for multi-camera. Empty uses the first camera found."""
    lens: str = "front"
    """Which lens to show: full, front, or back."""
    resolution: str = "1920x960"
    """Stream resolution: 1920x960, 2560x1280 or 3840x1920 (X5 honors these only with --live-view-mode)."""
    live_view_mode: bool = True
    """Use the SDK live-view flow, which unlocks the X5 preview resolution."""
    fps: int = 30
    """Frame rate recorded into the video file."""
    show_info: bool = False
    """Show the camera-info overlay from the start."""
    base_service_port: int = 9999
    """With >1 camera, camera i gets service port base+i (one process)."""
    output_dir: Path = Path("recordings")
    """Base dir for recordings."""


def open_cameras(args: Args) -> Dict[str, Insta360Camera]:
    """Open every requested camera, releasing any that opened if one fails.

    A camera left claimed by a crashed process stays in ``LIBUSB_ERROR_BUSY``
    until it is physically replugged, so a partial open must not leak.
    """
    serials = [s.strip() for s in args.serials.split(",") if s.strip()] or [""]
    cams: Dict[str, Insta360Camera] = {}
    try:
        for i, serial in enumerate(serials):
            name = serial or "cam0"
            cams[name] = Insta360Camera(
                serial=serial,
                resolution=args.resolution,
                lens=args.lens,
                live_view_mode=args.live_view_mode,
                service_port=(args.base_service_port + i) if len(serials) > 1 else 0,
                name=name,
            )
    except Exception:
        for cam in cams.values():
            cam.stop()
        raise
    return cams


def main() -> None:
    args = tyro.cli(Args)

    cams = open_cameras(args)
    latest: Dict[str, CameraData] = {}
    lags: Dict[str, List[float]] = {n: [] for n in cams}
    stop_evt = threading.Event()

    def pull(name: str, cam: Insta360Camera) -> None:
        while not stop_evt.is_set():
            try:
                data = cam.read()
            except (TimeoutError, RuntimeError) as e:
                logger.warning("[{}] read stopped: {}", name, e)
                return
            latest[name] = data
            lags[name].append(time.time() * 1000 - data.timestamp)

    threads = [threading.Thread(target=pull, args=(n, c), daemon=True, name=f"pull-{n}") for n, c in cams.items()]
    for t in threads:
        t.start()

    for name in cams:
        cv2.namedWindow(f"Insta360 {name}", cv2.WINDOW_NORMAL)

    show_info = args.show_info
    recording = False
    writers: Dict[str, AsyncVideoWriter] = {}
    record_dir: Optional[Path] = None
    t0 = time.time()

    print("\nControls:  [s] toggle info overlay  |  [r] toggle recording  |  [q/ESC] quit\n")

    try:
        while True:
            for name, data in list(latest.items()):
                rgb = data.images["rgb"]
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

                elapsed = time.time() - t0
                if elapsed > 0:
                    overlay_text(bgr, f"{len(lags[name]) / elapsed:.1f} fps", (bgr.shape[1] - 150, 30))
                if lags[name]:
                    overlay_text(bgr, f"lag {lags[name][-1]:.0f} ms")
                if show_info:
                    overlay_text(bgr, f"{cams[name]}", (10, 60))
                if recording:
                    overlay_text(bgr, "REC", (bgr.shape[1] - 80, 60))

                cv2.imshow(f"Insta360 {name}", bgr)
                if recording and name in writers:
                    writers[name].write(rgb)

            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                show_info = not show_info
            if key == ord("r"):
                if not recording:
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    record_dir = args.output_dir / f"insta360_recording_{ts}"
                    record_dir.mkdir(parents=True, exist_ok=True)
                    for name, data in list(latest.items()):
                        h, w = data.images["rgb"].shape[:2]
                        path = str(record_dir / f"cam_{name}.mp4")
                        writers[name] = AsyncVideoWriter(path=path, width=w, height=h, fps=args.fps)
                        writers[name].start()
                    recording = True
                    logger.info("Recording -> {}", record_dir)
                else:
                    for writer in writers.values():
                        writer.stop()
                    writers.clear()
                    recording = False
                    logger.info("Recording stopped -> {}", record_dir)

    except KeyboardInterrupt:
        pass
    finally:
        stop_evt.set()
        for writer in writers.values():
            writer.stop()
        dt = time.time() - t0
        for name, cam in cams.items():
            lag = np.array(lags[name]) if lags[name] else np.array([0.0])
            logger.info(
                "[{}] {} frames in {:.1f}s = {:.1f} fps | stamp lag: mean {:.1f} ms, p95 {:.1f} ms",
                name,
                len(lags[name]),
                dt,
                len(lags[name]) / dt if dt > 0 else 0.0,
                lag.mean(),
                np.percentile(lag, 95),
            )
            logger.info("[{}] info: {}", name, cam.get_camera_info())
            cam.stop()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()

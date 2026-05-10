#!/usr/bin/env python3
"""Viser-based viewer for the Lumos tracker.

Web-based 3D alternative to ``view_lumos.py``. Connect a browser to
http://localhost:8080 (or whatever ``--port`` you pass) after launching.

Shows in one scene:
  * world grid + axes
  * tracker pose as an oriented frame + small camera frustum
  * SLAM trajectory as a Catmull-Rom spline
  * live camera streams as image billboards floating beside the world

Usage
-----
Pre-flight: stack already up (``python -m robocam.drivers.lumos_stack up``)::

    uv run scripts/view_lumos_viser.py

Or let it bring up the stack itself::

    uv run scripts/view_lumos_viser.py --bring-up
    uv run scripts/view_lumos_viser.py --bring-up --color
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np
import tyro
import viser
from loguru import logger

from robocam.drivers.lumos import LumosCamera


@dataclass
class Args:
    """Lumos tracker viser viewer."""

    bring_up: bool = False
    """Bring up the docker stack (xv_sdk + senders) before connecting."""
    serial: Optional[str] = None
    """Tracker UUID. Auto-detected via sysfs when omitted."""
    side: str = "left"
    """Which tracker arm to read: 'left' or 'right'."""
    color: bool = False
    """Enable RGB: decode + display, and (with --bring-up) start color_camera.
    Off by default — color contends with fisheye for USB bandwidth."""
    host: str = "0.0.0.0"
    """Viser bind host. 0.0.0.0 lets remote browsers connect."""
    port: int = 8080
    """Viser server port. Connect at http://<host>:<port>."""
    trail_max: int = 2000
    """Maximum points kept in the SLAM trail."""


# xv_sdk publishes orientation as XYZW; viser uses WXYZ.
def _xyzw_to_wxyz(q) -> Tuple[float, float, float, float]:
    return float(q[3]), float(q[0]), float(q[1]), float(q[2])


def _is_finite_pos(pos) -> bool:
    # Pre-converged SLAM emits e+221-magnitude garbage; gate on a generous
    # room-scale envelope so auto-scaled views don't blow up.
    return pos is not None and len(pos) >= 3 and all(abs(float(v)) < 1e4 for v in pos)


def _as_display_rgb(img: np.ndarray) -> np.ndarray:
    """Normalize whatever LumosCamera handed us to RGB uint8 for viser."""
    if img.ndim == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    # JPEG-decoded color stream comes out BGR (cv2 convention).
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


def _resize_for_billboard(img: np.ndarray, target_w: int = 320) -> np.ndarray:
    h, w = img.shape[:2]
    if w == target_w:
        return img
    return cv2.resize(img, (target_w, int(h * target_w / w)))


# Layout: a row of three image billboards floating at the side of the world,
# vertical (z up), facing toward +Y so the default viser camera sees them.
_BILLBOARD_LAYOUT: Dict[str, Tuple[float, float, float]] = {
    "fisheye_left":  (-1.5, -2.0, 1.5),
    "fisheye_right": ( 0.0, -2.0, 1.5),
    "rgb":           ( 1.5, -2.0, 1.5),
}


def main() -> None:
    args = tyro.cli(Args)

    stack = None
    if args.bring_up:
        from robocam.drivers import lumos_stack
        stack = lumos_stack.up(serial=args.serial, enable_color=args.color)
        serial = stack.serial
    else:
        serial = args.serial or "auto"

    cam = LumosCamera(side=args.side, enable_color=args.color)

    server = viser.ViserServer(host=args.host, port=args.port)
    logger.info("viser ready: http://localhost:{}", args.port)

    scene = server.scene
    scene.add_grid("/grid", width=10.0, height=10.0)
    scene.add_frame("/world", axes_length=0.3, axes_radius=0.01)

    # Tracker pose (updated each frame).
    tracker_frame = scene.add_frame("/tracker", axes_length=0.15, axes_radius=0.008)
    tracker_frustum = scene.add_camera_frustum(
        "/tracker/cam",
        fov=math.radians(120),  # fisheye-ish — roughly XVisio's FOV
        aspect=4 / 3,
        scale=0.12,
        color=(0, 200, 255),
    )

    # Trail (recreated each tick because the spline geometry grows).
    trail_history: Deque[Tuple[float, float, float]] = deque(maxlen=args.trail_max)
    trail_handle: Optional[viser.SplineCatmullRomHandle] = None

    image_handles: Dict[str, viser.ImageHandle] = {}

    # GUI panel.
    with server.gui.add_folder("Lumos"):
        gui_status = server.gui.add_text("status", "starting…", disabled=True)
        gui_show_trail = server.gui.add_checkbox("show trail", True)
        gui_show_frustum = server.gui.add_checkbox("show frustum", True)
        gui_clear = server.gui.add_button("clear trail")

    @gui_clear.on_click
    def _on_clear(_event) -> None:
        nonlocal trail_handle
        trail_history.clear()
        if trail_handle is not None:
            trail_handle.remove()
            trail_handle = None

    last_log_t = time.monotonic()
    frames_since_log = 0

    try:
        while True:
            try:
                data = cam.read()
            except TimeoutError as e:
                logger.warning("{}", e)
                continue
            frames_since_log += 1

            # ---- pose update -------------------------------------------------
            pose = (data.other_sensors or {}).get("pose")
            if pose is not None:
                pos = pose.get("position")
                ori = pose.get("orientation")
                if _is_finite_pos(pos) and ori is not None and len(ori) == 4:
                    tup = (float(pos[0]), float(pos[1]), float(pos[2]))
                    tracker_frame.position = tup
                    tracker_frame.wxyz = _xyzw_to_wxyz(ori)
                    tracker_frustum.position = tup
                    tracker_frustum.wxyz = _xyzw_to_wxyz(ori)
                    trail_history.append(tup)

            # ---- trail polyline ---------------------------------------------
            if gui_show_trail.value and len(trail_history) >= 2:
                pts = np.array(trail_history, dtype=np.float32)
                if trail_handle is not None:
                    trail_handle.remove()
                trail_handle = scene.add_spline_catmull_rom(
                    "/trail",
                    points=pts,
                    line_width=2.0,
                    color=(0, 200, 200),
                )
            elif not gui_show_trail.value and trail_handle is not None:
                trail_handle.remove()
                trail_handle = None

            tracker_frustum.visible = gui_show_frustum.value

            # ---- image billboards -------------------------------------------
            for key, anchor in _BILLBOARD_LAYOUT.items():
                img = data.images.get(key)
                if img is None:
                    continue
                rgb = _resize_for_billboard(_as_display_rgb(img))
                h, w = rgb.shape[:2]
                handle = image_handles.get(key)
                if handle is None:
                    image_handles[key] = scene.add_image(
                        f"/cam/{key}",
                        image=rgb,
                        render_width=1.2,
                        render_height=1.2 * h / w,
                        position=anchor,
                    )
                    # label above the billboard
                    scene.add_label(f"/cam/{key}/label", text=key,
                                    position=(anchor[0], anchor[1], anchor[2] + 0.7))
                else:
                    handle.image = rgb

            # ---- status -----------------------------------------------------
            now = time.monotonic()
            if now - last_log_t >= 1.0:
                fps = frames_since_log / (now - last_log_t)
                gui_status.value = (
                    f"sn={serial} | {fps:.1f} fps | "
                    f"trail n={len(trail_history)} | "
                    f"clients {len(server.get_clients())}"
                )
                frames_since_log = 0
                last_log_t = now

    except KeyboardInterrupt:
        logger.info("stopping…")
    finally:
        cam.stop()
        if stack is not None:
            stack.down()
        server.stop()


if __name__ == "__main__":
    main()

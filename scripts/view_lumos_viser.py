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


# xv_sdk's SLAM world frame is the optical-frame convention (X right,
# Y down, Z forward). Viser's scene is X right, Y forward, Z up. We
# rotate -90° about X to map between them: position (x, y, z) → (x, z, -y),
# and the orientation quaternion is pre-multiplied by the same rotation.
_SQRT_HALF = math.sqrt(0.5)
_Q_XV2VISER_WXYZ = (_SQRT_HALF, -_SQRT_HALF, 0.0, 0.0)  # -90° about X (WXYZ)


def _quat_mul_wxyz(
    q1: Tuple[float, float, float, float],
    q2: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """Hamilton product q1 ⊗ q2; both are WXYZ."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def _convert_position(pos) -> Tuple[float, float, float]:
    return float(pos[0]), float(pos[2]), -float(pos[1])


def _convert_orientation(ori_xyzw) -> Tuple[float, float, float, float]:
    # XYZW → WXYZ, then rotate by Q_XV2VISER from the left.
    q_xv_wxyz = (float(ori_xyzw[3]), float(ori_xyzw[0]),
                 float(ori_xyzw[1]), float(ori_xyzw[2]))
    return _quat_mul_wxyz(_Q_XV2VISER_WXYZ, q_xv_wxyz)


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

    # Trail (recreated each tick because the geometry grows). We render as
    # straight line segments + a point cloud of the raw samples — the spline
    # version interpolated between samples and amplified SLAM jitter.
    trail_history: Deque[Tuple[float, float, float]] = deque(maxlen=args.trail_max)
    trail_lines: Optional[viser.LineSegmentsHandle] = None
    trail_dots: Optional[viser.PointCloudHandle] = None

    image_handles: Dict[str, viser.ImageHandle] = {}

    # GUI panel.
    with server.gui.add_folder("Lumos"):
        gui_status = server.gui.add_text("status", "starting…", disabled=True)
        gui_confidence = server.gui.add_text("SLAM confidence", "—", disabled=True)
        gui_min_conf = server.gui.add_slider(
            "min confidence", min=0.0, max=1.0, step=0.05, initial_value=0.0,
        )
        gui_show_trail = server.gui.add_checkbox("show trail", True)
        gui_show_frustum = server.gui.add_checkbox("show frustum", True)
        gui_clear = server.gui.add_button("clear trail")

    @gui_clear.on_click
    def _on_clear(_event) -> None:
        nonlocal trail_lines, trail_dots
        trail_history.clear()
        if trail_lines is not None:
            trail_lines.remove()
            trail_lines = None
        if trail_dots is not None:
            trail_dots.remove()
            trail_dots = None

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
            confidence: Optional[float] = None
            if pose is not None:
                pos = pose.get("position")
                ori = pose.get("orientation")
                confidence = pose.get("confidence")
                if _is_finite_pos(pos) and ori is not None and len(ori) == 4:
                    pos_v = _convert_position(pos)
                    ori_v = _convert_orientation(ori)
                    # The frustum is a child of /tracker, so its pose is
                    # inherited from the parent — never touch its wxyz/position
                    # directly or it'd be applied twice.
                    tracker_frame.wxyz = ori_v
                    if confidence is None or confidence >= gui_min_conf.value:
                        tracker_frame.position = pos_v
                        trail_history.append(pos_v)
            gui_confidence.value = (
                f"{confidence:.2f}" if confidence is not None else "—"
            )

            # ---- trail polyline + dots --------------------------------------
            if gui_show_trail.value and len(trail_history) >= 1:
                pts = np.array(trail_history, dtype=np.float32)
                if trail_dots is not None:
                    trail_dots.remove()
                trail_dots = scene.add_point_cloud(
                    "/trail/dots", points=pts, colors=(0, 255, 255),
                    point_size=0.012, point_shape="circle",
                )
                if len(pts) >= 2:
                    # (N-1, 2, 3) — connect each consecutive pair as one segment.
                    segs = np.stack([pts[:-1], pts[1:]], axis=1)
                    if trail_lines is not None:
                        trail_lines.remove()
                    trail_lines = scene.add_line_segments(
                        "/trail/lines", points=segs, colors=(0, 200, 200),
                        line_width=2.0,
                    )
            elif not gui_show_trail.value:
                if trail_lines is not None:
                    trail_lines.remove(); trail_lines = None
                if trail_dots is not None:
                    trail_dots.remove(); trail_dots = None

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

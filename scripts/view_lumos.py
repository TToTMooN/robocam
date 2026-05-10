#!/usr/bin/env python3
"""Live viewer for the FastUMI Pro Lumos tracker.

Pre-flight: the docker stack (xv_sdk + in-container senders) must be up.
Either:

    python -m robocam.drivers.lumos_stack up
    uv run scripts/view_lumos.py

Or pass --bring-up so the script does it for you:

    uv run scripts/view_lumos.py --bring-up

Controls
--------
s  - toggle UUID/pose overlay
p  - toggle SLAM pose trail tile
c  - clear the SLAM pose trail (re-center)
r  - start / stop video recording (uses AsyncVideoWriter + NVENC)
q  - quit (or ESC)

Examples
--------
    uv run scripts/view_lumos.py
    uv run scripts/view_lumos.py --bring-up --serial 250801DR48FP25002333
    uv run scripts/view_lumos.py --color
    uv run scripts/view_lumos.py --bring-up --color
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Deque, List, Optional, Tuple

import cv2
import numpy as np
import tyro
from loguru import logger

from robocam import AsyncVideoWriter
from robocam.drivers.lumos import LumosCamera


class PoseTrail:
    """Top-down 2D minimap of the SLAM trajectory.

    Stores XY positions over time and renders a small canvas with the
    trail, current dot, and a heading arrow derived from the orientation
    quaternion's yaw.
    """

    def __init__(self, max_points: int = 2000, size: int = 320, margin: int = 16) -> None:
        self.history: Deque[Tuple[float, float, float]] = deque(maxlen=max_points)
        self.last_orientation: Optional[Tuple[float, float, float, float]] = None
        self.size = size
        self.margin = margin

    def add(self, pose: Optional[dict]) -> None:
        if not pose:
            return
        pos = pose.get("position")
        if pos is None or len(pos) < 3:
            return
        # Pre-converged SLAM produces e+221-magnitude garbage. Drop anything
        # outside any plausible room-scale envelope.
        if not all(abs(float(v)) < 1e4 for v in pos):
            return
        self.history.append((float(pos[0]), float(pos[1]), float(pos[2])))
        ori = pose.get("orientation")
        if ori is not None and len(ori) == 4:
            self.last_orientation = tuple(float(v) for v in ori)  # type: ignore[assignment]

    def clear(self) -> None:
        self.history.clear()
        self.last_orientation = None

    def render(self) -> np.ndarray:
        size, margin = self.size, self.margin
        canvas = np.full((size, size, 3), 30, dtype=np.uint8)

        # axis lines (just decorative anchors at the bottom-left)
        cv2.line(canvas, (margin, size - margin), (size - margin, size - margin), (80, 80, 80), 1)
        cv2.line(canvas, (margin, margin), (margin, size - margin), (80, 80, 80), 1)
        cv2.putText(canvas, "X", (size - margin + 2, size - margin + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1, cv2.LINE_AA)
        cv2.putText(canvas, "Y", (margin - 12, margin - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120, 120, 120), 1, cv2.LINE_AA)

        if not self.history:
            cv2.putText(canvas, "no pose yet", (margin, size // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1, cv2.LINE_AA)
            return canvas

        xs = [p[0] for p in self.history]
        ys = [p[1] for p in self.history]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        # Keep at least 1 m visible so a near-stationary trail doesn't pixel-jitter.
        span = max(x_max - x_min, y_max - y_min, 1.0)
        cx, cy = (x_min + x_max) / 2, (y_min + y_max) / 2
        plot = size - 2 * margin
        scale = plot / span

        def to_px(p: Tuple[float, float, float]) -> Tuple[int, int]:
            px = int(margin + plot / 2 + (p[0] - cx) * scale)
            py = int(margin + plot / 2 - (p[1] - cy) * scale)  # flip Y so +Y goes up
            return px, py

        pts = [to_px(p) for p in self.history]
        for i in range(1, len(pts)):
            cv2.line(canvas, pts[i - 1], pts[i], (0, 200, 200), 1, cv2.LINE_AA)

        # Current position
        cv2.circle(canvas, pts[-1], 4, (0, 255, 255), -1)

        # Heading from quaternion yaw (rotation around world Z, projected to XY plane)
        if self.last_orientation is not None:
            qx, qy, qz, qw = self.last_orientation
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            arrow_len = 18
            ax = pts[-1][0] + int(math.cos(yaw) * arrow_len)
            ay = pts[-1][1] - int(math.sin(yaw) * arrow_len)
            cv2.arrowedLine(canvas, pts[-1], (ax, ay), (0, 255, 255), 2, tipLength=0.4, line_type=cv2.LINE_AA)

        # Status line: span and sample count
        cv2.putText(canvas, f"span: {span:.2f}m  n={len(self.history)}",
                    (margin, size - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        return canvas


def overlay_text(image: np.ndarray, text: str, position: tuple = (10, 30)) -> None:
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale, thickness = 0.7, 2
    cv2.putText(image, text, (position[0] + 1, position[1] + 1), font, scale, (0, 0, 0), thickness + 1, cv2.LINE_AA)
    cv2.putText(image, text, position, font, scale, (0, 255, 0), thickness, cv2.LINE_AA)


def build_grid(tiles: List[np.ndarray], tile_w: int, tile_h: int) -> np.ndarray:
    n = len(tiles)
    if n == 0:
        return np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    grid = np.zeros((rows * tile_h, cols * tile_w, 3), dtype=np.uint8)
    for idx, tile in enumerate(tiles):
        r, c = divmod(idx, cols)
        if tile.ndim == 2:
            tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        elif tile.shape[2] == 1:
            tile = cv2.cvtColor(tile.squeeze(-1), cv2.COLOR_GRAY2BGR)
        resized = cv2.resize(tile, (tile_w, tile_h))
        grid[r * tile_h : (r + 1) * tile_h, c * tile_w : (c + 1) * tile_w] = resized
    return grid


@dataclass
class Args:
    """Lumos tracker live viewer & recorder."""

    bring_up: bool = False
    """Bring up the docker stack (xv_sdk + senders) before connecting."""
    serial: Optional[str] = None
    """Tracker UUID. Auto-detected via sysfs when omitted."""
    side: str = "left"
    """Which tracker arm to read: 'left' or 'right'."""
    color: bool = False
    """Enable RGB: decode + display, and (with --bring-up) start color_camera.
    Off by default — color contends with fisheye for USB bandwidth."""
    show_overlay: bool = True
    """Show pose / UUID overlay."""
    show_pose_trail: bool = True
    """Show top-down SLAM pose trail tile."""
    output_dir: Path = Path("recordings")
    """Base dir for recordings."""


def main() -> None:
    args = tyro.cli(Args)

    # Bring up the stack first if asked. Done before we bind TCP sockets.
    stack = None
    if args.bring_up:
        from robocam.drivers import lumos_stack

        stack = lumos_stack.up(serial=args.serial, enable_color=args.color)
        # use the stack's resolved serial
        serial = stack.serial
    else:
        serial = args.serial or "auto"

    cam = LumosCamera(side=args.side, enable_color=args.color)

    # warm up: wait until first frame arrives or timeout
    logger.info("waiting for first frame from {} ...", serial)
    waited = 0.0
    while not cam.is_connected() and waited < 30.0:
        time.sleep(0.5)
        waited += 0.5
    if not cam.is_connected():
        logger.error("no frames in 30s — is the stack up?")
        cam.stop()
        if stack is not None:
            stack.down()
        return

    show_overlay = args.show_overlay
    show_trail = args.show_pose_trail
    trail = PoseTrail()
    recording = False
    writers: dict[str, AsyncVideoWriter] = {}
    record_dir: Optional[Path] = None
    window_name = "Lumos"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("\nControls:  [s] overlay  |  [p] pose trail  |  [c] clear trail  "
          "|  [r] record  |  [q/ESC] quit\n")

    frame_count = 0
    t0 = time.time()

    try:
        while True:
            try:
                data = cam.read()
            except TimeoutError as e:
                logger.warning("{}", e)
                continue

            tiles: List[np.ndarray] = []
            tile_labels: List[str] = []

            for key in ("fisheye_left", "fisheye_right", "rgb"):
                img = data.images.get(key)
                if img is None:
                    continue
                # mono fisheye → BGR for display
                if img.ndim == 2:
                    disp = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                else:
                    disp = img.copy()
                tiles.append(disp)
                tile_labels.append(key)

            if not tiles:
                continue

            # Update the pose trail every frame, even when hidden — that way
            # toggling 'p' on shows the full history, not just from the toggle.
            trail.add((data.other_sensors or {}).get("pose"))

            frame_count += 1
            elapsed = time.time() - t0
            fps = frame_count / elapsed if elapsed > 0 else 0.0

            if show_overlay:
                pose = (data.other_sensors or {}).get("pose")
                clamp = (data.other_sensors or {}).get("clamp")
                for tile, label in zip(tiles, tile_labels):
                    overlay_text(tile, label, (10, 30))
                if pose is not None:
                    p = pose.get("position", [0, 0, 0])
                    overlay_text(
                        tiles[0],
                        f"xyz: {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}",
                        (10, 60),
                    )
                if clamp is not None:
                    overlay_text(tiles[0], f"clamp: {clamp:.2f}", (10, 90))
                overlay_text(tiles[0], f"{fps:.1f} fps", (10, 120))
                overlay_text(tiles[0], f"sn: {serial}", (10, 150))

            tile_h, tile_w = tiles[0].shape[:2]

            # Append the pose-trail tile last so it sits in the bottom-right
            # of the grid. Recording skips this tile (it's not a sensor).
            display_tiles = list(tiles)
            if show_trail:
                display_tiles.append(trail.render())

            grid = build_grid(display_tiles, tile_w, tile_h)

            if recording:
                overlay_text(grid, "REC", (grid.shape[1] - 80, 30))
                for tile, label in zip(tiles, tile_labels):
                    if label not in writers:
                        continue
                    rgb = cv2.cvtColor(tile, cv2.COLOR_BGR2RGB) if tile.shape[2] == 3 else tile
                    writers[label].write(rgb)

            cv2.imshow(window_name, grid)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                show_overlay = not show_overlay
            if key == ord("p"):
                show_trail = not show_trail
            if key == ord("c"):
                trail.clear()
            if key == ord("r"):
                if not recording:
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    record_dir = args.output_dir / f"lumos_{ts}"
                    record_dir.mkdir(parents=True, exist_ok=True)
                    for tile, label in zip(tiles, tile_labels):
                        h, w = tile.shape[:2]
                        path = str(record_dir / f"{label}.mp4")
                        wr = AsyncVideoWriter(path=path, width=w, height=h, fps=30)
                        wr.start()
                        writers[label] = wr
                        logger.info("Recording -> {}", path)
                    recording = True
                else:
                    for w in writers.values():
                        w.stop()
                    writers.clear()
                    recording = False
                    print(f"Recording stopped. Files saved to {record_dir}")

    except KeyboardInterrupt:
        pass
    finally:
        for w in writers.values():
            w.stop()
        if record_dir and not writers:
            print(f"Recording saved to {record_dir}")
        cam.stop()
        if stack is not None:
            stack.down()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()

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
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import tyro
from loguru import logger

from robocam import AsyncVideoWriter
from robocam.drivers.lumos import LumosCamera


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
    recording = False
    writers: dict[str, AsyncVideoWriter] = {}
    record_dir: Optional[Path] = None
    window_name = "Lumos"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("\nControls:  [s] toggle overlay  |  [r] toggle recording  |  [q/ESC] quit\n")

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
            grid = build_grid(tiles, tile_w, tile_h)

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

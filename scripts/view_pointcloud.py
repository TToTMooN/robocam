#!/usr/bin/env python3
"""Live 3-D point cloud viewer for Stereolabs ZED cameras using Viser.

Auto-discovers all connected ZED cameras, streams images as GUI panels,
and renders depth-enabled cameras as live point clouds in a web-based
3-D viewer.

Requires: ``pip install robocam[zed] viser``

Examples
--------
    uv run scripts/view_pointcloud.py
    uv run scripts/view_pointcloud.py --device-id 12345678
    uv run scripts/view_pointcloud.py --resolution HD720 --fps 15 --stride 4
    uv run scripts/view_pointcloud.py --port 8090
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import tyro
from loguru import logger

from robocam.drivers.zed import ZedCamera, decode_xyzrgba

try:
    import viser
except ImportError as _e:
    raise ImportError("viser is required for this script: pip install viser") from _e


@dataclass
class Args:
    """ZED camera 3-D point cloud viewer (Viser web GUI)."""

    device_id: Optional[str] = None
    """ZED serial number. None = all available cameras."""
    resolution: str = "HD720"
    """Resolution: HD2K, HD1200, HD1080, HD720, VGA, SVGA."""
    fps: int = 15
    """Target frame rate."""
    stride: int = 4
    """Point cloud spatial stride (higher = fewer points, faster)."""
    port: int = 8085
    """Viser web server port."""
    point_size: float = 0.005
    """Visual size of each point in the viewer."""


def main() -> None:
    args = tyro.cli(Args)

    from pyzed import sl

    devices = sl.Camera.get_device_list()
    if not devices:
        print("No ZED cameras found.")
        return

    if args.device_id:
        devices = [d for d in devices if str(d.serial_number) == args.device_id]
        if not devices:
            print(f"ZED camera {args.device_id} not found.")
            return

    cameras: Dict[int, ZedCamera] = {}
    for dev in devices:
        serial = str(dev.serial_number)
        logger.info("Opening ZED {} ({})", serial, dev.camera_model)
        try:
            cam = ZedCamera(
                device_id=serial,
                resolution=args.resolution,
                fps=args.fps,
                enable_depth=True,
            )
            cameras[dev.serial_number] = cam
        except Exception as e:
            logger.warning("Failed to open ZED {}: {}", serial, e)

    if not cameras:
        print("No cameras could be opened.")
        return

    server = viser.ViserServer(port=args.port)
    server.gui.set_panel_label("Point Cloud Viewer")
    server.gui.configure_theme(control_layout="fixed", control_width="large", dark_mode=True)
    server.scene.set_up_direction("+z")

    @server.on_client_connect
    def _(_client: viser.ClientHandle) -> None:
        _client.camera.position = (1.5, -1.5, 1.0)
        _client.camera.look_at = (0.0, 0.0, 0.3)
        _client.camera.up_direction = (0.0, 0.0, 1.0)

    image_handles: Dict[int, viser.GuiImageHandle] = {}
    pc_handles: Dict[int, viser.PointCloudHandle] = {}
    pc_checkbox = server.gui.add_checkbox("Stream point clouds", initial_value=True)

    for serial in cameras:
        placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
        with server.gui.add_folder(str(serial)):
            image_handles[serial] = server.gui.add_image(
                placeholder,
                label=str(serial),
                format="jpeg",
                jpeg_quality=80,
            )

    print(f"\nStreaming {len(cameras)} camera(s) — open http://localhost:{args.port}\n")

    try:
        while True:
            for serial, cam in cameras.items():
                xyzrgba = cam.read_xyzrgba()
                if xyzrgba.size == 0:
                    continue

                pc = decode_xyzrgba(xyzrgba, stride=args.stride)

                cam.zed.retrieve_image(cam.image_left, sl.VIEW.LEFT)
                frame = cam.image_left.get_data().copy()
                frame = np.ascontiguousarray(frame[..., :3][..., ::-1])
                image_handles[serial].image = frame

                if pc_checkbox.value and len(pc.points) > 0:
                    if serial in pc_handles:
                        pc_handles[serial].points = pc.points
                        pc_handles[serial].colors = pc.colors
                    else:
                        pc_handles[serial] = server.scene.add_point_cloud(
                            f"/pointclouds/{serial}",
                            points=pc.points,
                            colors=pc.colors,
                            point_size=args.point_size,
                        )
                elif len(pc.points) == 0:
                    logger.debug(
                        "[{}] 0 valid points — finite: {}/{}",
                        serial,
                        np.isfinite(xyzrgba[..., 0]).sum(),
                        xyzrgba[..., 0].size,
                    )

            time.sleep(1 / args.fps)
    except KeyboardInterrupt:
        pass
    finally:
        for cam in cameras.values():
            cam.stop()
        print("Done.")


if __name__ == "__main__":
    main()

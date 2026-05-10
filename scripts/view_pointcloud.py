#!/usr/bin/env python3
"""Live 3-D point cloud viewer for depth-capable cameras using Viser.

Auto-discovers ZED and RealSense cameras and renders live point clouds
in a web-based 3-D viewer.  ZED cameras use the fast ``decode_xyzrgba()``
path; RealSense cameras use ``depth_to_pointcloud()`` with intrinsics.

Requires: ``pip install viser`` (in addition to robocam deps)

Examples
--------
    uv run scripts/view_pointcloud.py
    uv run scripts/view_pointcloud.py --camera-type realsense
    uv run scripts/view_pointcloud.py --device-id 12345678
    uv run scripts/view_pointcloud.py --resolution HD720 --fps 15 --stride 4
    uv run scripts/view_pointcloud.py --port 8090
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np
import tyro
from loguru import logger

from robocam.camera import CameraDriver, PointCloudData
from robocam.utils import depth_to_pointcloud

try:
    import viser
except ImportError as _e:
    raise ImportError("viser is required for this script: pip install viser") from _e


@dataclass
class Args:
    """Live 3-D point cloud viewer for ZED and RealSense cameras (Viser web GUI)."""

    device_id: Optional[str] = None
    """Camera serial number. None = all available cameras."""
    resolution: str = "HD720"
    """ZED resolution: HD2K, HD1200, HD1080, HD720, VGA, SVGA."""
    fps: int = 15
    """Target frame rate."""
    stride: int = 4
    """Point cloud spatial stride (higher = fewer points, faster)."""
    port: int = 8085
    """Viser web server port."""
    point_size: float = 0.005
    """Visual size of each point in the viewer."""
    camera_type: str = "auto"
    """Camera type to discover: "auto", "zed", "realsense"."""
    realsense_resolution: str = "640x480"
    """RealSense resolution as WxH."""


@dataclass
class _CameraEntry:
    """Bookkeeping for a discovered depth camera."""

    label: str
    driver: CameraDriver
    kind: str  # "zed" or "realsense"
    intrinsics_K: Optional[np.ndarray] = None  # needed for RealSense pointcloud


def _discover_cameras(args: Args) -> List[_CameraEntry]:
    """Discover and open depth-capable cameras."""
    entries: List[_CameraEntry] = []

    if args.camera_type in ("auto", "zed"):
        try:
            from robocam.drivers.zed import ZedCamera, discover_devices as zed_discover

            for dev in zed_discover():
                serial = dev["serial"]
                if args.device_id and serial != args.device_id:
                    continue
                logger.info("Opening ZED {} ({})", serial, dev["name"])
                try:
                    cam = ZedCamera(
                        device_id=serial,
                        resolution=args.resolution,
                        fps=args.fps,
                        enable_depth=True,
                    )
                    entries.append(_CameraEntry(label=f"zed_{serial}", driver=cam, kind="zed"))
                except Exception as e:
                    logger.warning("Failed to open ZED {}: {}", serial, e)
        except ImportError:
            logger.debug("pyzed not available, skipping ZED discovery")

    if args.camera_type in ("auto", "realsense"):
        try:
            from robocam.drivers.realsense import RealsenseCamera, discover_devices

            rs_w, rs_h = (int(x) for x in args.realsense_resolution.split("x"))
            for dev in discover_devices():
                serial = dev["serial"]
                if args.device_id and serial != args.device_id:
                    continue
                logger.info("Opening RealSense {} ({})", serial, dev["name"])
                try:
                    cam = RealsenseCamera(
                        serial_number=serial,
                        resolution=(rs_w, rs_h),
                        fps=args.fps,
                        enable_depth=True,
                    )
                    intrinsics = cam.read_calibration_data_intrinsics()
                    K = intrinsics["K"]
                    entries.append(_CameraEntry(label=f"rs_{serial}", driver=cam, kind="realsense", intrinsics_K=K))
                except Exception as e:
                    logger.warning("Failed to open RealSense {}: {}", serial, e)
        except ImportError:
            logger.debug("pyrealsense2 not available, skipping RealSense discovery")

    return entries


def _pointcloud_from_zed(cam: CameraDriver, stride: int) -> tuple[Optional[PointCloudData], Optional[np.ndarray]]:
    """Get point cloud + preview image from a ZED camera using the fast XYZRGBA path.

    Returns ``(point_cloud, preview_rgb)``.  The preview is retrieved from
    the same grab that ``read_xyzrgba()`` performs internally.
    """
    from pyzed import sl

    from robocam.drivers.zed import ZedCamera, decode_xyzrgba

    assert isinstance(cam, ZedCamera)
    xyzrgba = cam.read_xyzrgba()
    if xyzrgba.size == 0:
        return None, None
    pc = decode_xyzrgba(xyzrgba, stride=stride)

    # Retrieve left image from the same grab (no extra grab needed)
    cam.zed.retrieve_image(cam.image_left, sl.VIEW.LEFT)
    frame = cam.image_left.get_data().copy()
    preview = np.ascontiguousarray(frame[..., :3][..., ::-1])
    return pc, preview


def _pointcloud_from_realsense(
    cam: CameraDriver,
    K: np.ndarray,
    stride: int,
) -> tuple[Optional[PointCloudData], Optional[np.ndarray]]:
    """Get point cloud + preview image from a RealSense camera via depth back-projection.

    Returns ``(point_cloud, preview_rgb)``.
    """
    data = cam.read()
    depth = data.depth_data
    rgb = data.images.get("rgb")
    if depth is None:
        return None, rgb

    # RealSense z16 depth is in millimetres -- convert to metres
    depth_m = depth.astype(np.float32) / 1000.0

    points = depth_to_pointcloud(depth_m, K, stride=stride)
    if points.shape[0] == 0:
        return None, rgb

    # Sample RGB colours at valid depth pixel locations
    if rgb is not None:
        h, w = depth.shape[:2]
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        valid = np.isfinite(depth_m[0:h:stride, 0:w:stride]) & (depth_m[0:h:stride, 0:w:stride] > 0)
        u_valid = u[valid]
        v_valid = v[valid]
        colors = rgb[v_valid, u_valid, :3].astype(np.uint8)
    else:
        colors = np.full((points.shape[0], 3), 128, dtype=np.uint8)

    # Rotate from camera convention (X-right, Y-down, Z-forward) to Z-up
    rotated = np.empty_like(points)
    rotated[:, 0] = points[:, 0]
    rotated[:, 1] = points[:, 2]
    rotated[:, 2] = -points[:, 1]

    return PointCloudData(points=rotated, colors=colors), rgb


def main() -> None:
    args = tyro.cli(Args)

    entries = _discover_cameras(args)
    if not entries:
        logger.error("No depth-capable cameras found.")
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

    image_handles: Dict[str, viser.GuiImageHandle] = {}
    pc_handles: Dict[str, viser.PointCloudHandle] = {}
    pc_checkbox = server.gui.add_checkbox("Stream point clouds", initial_value=True)

    for entry in entries:
        placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
        with server.gui.add_folder(entry.label):
            image_handles[entry.label] = server.gui.add_image(
                placeholder,
                label=entry.label,
                format="jpeg",
                jpeg_quality=80,
            )

    logger.info("Streaming {} camera(s) -- open http://localhost:{}", len(entries), args.port)

    try:
        while True:
            for entry in entries:
                if entry.kind == "zed":
                    pc, preview = _pointcloud_from_zed(entry.driver, args.stride)
                else:
                    pc, preview = _pointcloud_from_realsense(entry.driver, entry.intrinsics_K, args.stride)

                if preview is not None:
                    image_handles[entry.label].image = preview

                if pc is not None and pc_checkbox.value and len(pc.points) > 0:
                    if entry.label in pc_handles:
                        pc_handles[entry.label].points = pc.points
                        pc_handles[entry.label].colors = pc.colors
                    else:
                        pc_handles[entry.label] = server.scene.add_point_cloud(
                            f"/pointclouds/{entry.label}",
                            points=pc.points,
                            colors=pc.colors,
                            point_size=args.point_size,
                        )

            time.sleep(1 / args.fps)
    except KeyboardInterrupt:
        pass
    finally:
        for entry in entries:
            entry.driver.stop()
        logger.info("Done.")


if __name__ == "__main__":
    main()

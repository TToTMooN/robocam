#!/usr/bin/env python3
"""Diagnostics for point cloud utilities — unit tests + hardware tests.

Unit tests (no hardware):
    uv run scripts/diagnostics/test_pointcloud.py

Hardware tests (needs cameras):
    uv run scripts/diagnostics/test_pointcloud.py --hardware
    uv run scripts/diagnostics/test_pointcloud.py --hardware --camera-type zed
    uv run scripts/diagnostics/test_pointcloud.py --hardware --camera-type realsense
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
import tyro
from loguru import logger

from robocam.camera import PointCloudData
from robocam.utils import depth_to_pointcloud


# ---------------------------------------------------------------------------
# Unit tests (synthetic data, no hardware)
# ---------------------------------------------------------------------------


def test_depth_to_pointcloud_basic() -> None:
    """Back-projection of a known depth image should produce correct XYZ."""
    K = np.array([[100.0, 0.0, 2.0], [0.0, 100.0, 2.0], [0.0, 0.0, 1.0]])
    depth = np.ones((5, 5), dtype=np.float32) * 2.0  # all pixels at 2m

    pts = depth_to_pointcloud(depth, K, stride=1)
    assert pts.ndim == 2 and pts.shape[1] == 3, f"Expected (N, 3), got {pts.shape}"
    assert pts.shape[0] == 25, f"Expected 25 points, got {pts.shape[0]}"

    # Center pixel (2,2) should project to (0, 0, 2)
    center_mask = (pts[:, 0] == 0.0) & (pts[:, 1] == 0.0)
    assert center_mask.any(), "Center pixel should project to x=0, y=0"
    logger.info("PASS depth_to_pointcloud_basic: {} points, center at z={}", pts.shape[0], pts[center_mask][0, 2])


def test_depth_to_pointcloud_nan_filtering() -> None:
    """NaN and zero depth values should be filtered out."""
    K = np.eye(3) * 100
    K[2, 2] = 1.0
    depth = np.ones((4, 4), dtype=np.float32)
    depth[0, 0] = np.nan
    depth[1, 1] = np.inf
    depth[2, 2] = 0.0
    depth[3, 3] = -1.0  # negative depth

    pts = depth_to_pointcloud(depth, K, stride=1)
    assert pts.shape[0] == 12, f"Expected 12 valid points (16 - 4 invalid), got {pts.shape[0]}"
    assert np.all(np.isfinite(pts)), "All output points should be finite"
    logger.info("PASS nan_filtering: {} valid points from 16 pixels", pts.shape[0])


def test_depth_to_pointcloud_stride() -> None:
    """Stride should reduce point count by ~stride^2."""
    K = np.eye(3) * 100
    K[2, 2] = 1.0
    depth = np.ones((100, 100), dtype=np.float32)

    pts_s1 = depth_to_pointcloud(depth, K, stride=1)
    pts_s4 = depth_to_pointcloud(depth, K, stride=4)
    ratio = pts_s1.shape[0] / pts_s4.shape[0]
    assert 14 < ratio < 18, f"Expected ~16x reduction, got {ratio:.1f}x"
    logger.info("PASS stride: s1={} pts, s4={} pts, ratio={:.1f}x", pts_s1.shape[0], pts_s4.shape[0], ratio)


def test_decode_xyzrgba_synthetic() -> None:
    """decode_xyzrgba should unpack XYZRGBA and filter invalid points."""
    try:
        from robocam.drivers.zed import decode_xyzrgba
    except ImportError:
        logger.warning("SKIP decode_xyzrgba_synthetic: pyzed not available")
        return

    H, W = 4, 4
    # Start all-NaN, then set exactly 2 valid points
    xyzrgba = np.full((H, W, 4), np.nan, dtype=np.float32)

    # Pack a color: R=255, G=128, B=64 -> 0x00FF8040
    color_packed = np.array(0x00FF8040, dtype=np.uint32).view(np.float32)
    xyzrgba[0, 0] = [1.0, 2.0, 3.0, color_packed]
    xyzrgba[1, 1] = [4.0, 5.0, 6.0, color_packed]

    pc = decode_xyzrgba(xyzrgba, stride=1, rotate_to_z_up=False)
    assert isinstance(pc, PointCloudData)
    assert pc.points.shape[0] == 2, f"Expected 2 valid points, got {pc.points.shape[0]}"
    assert pc.colors.shape == (2, 3)
    assert pc.colors[0, 0] == 255, f"Expected R=255, got {pc.colors[0, 0]}"
    logger.info("PASS decode_xyzrgba_synthetic: {} points, first color={}", pc.points.shape[0], pc.colors[0])


# ---------------------------------------------------------------------------
# Hardware tests
# ---------------------------------------------------------------------------


def test_realsense_pointcloud() -> None:
    """Read depth from RealSense and back-project to point cloud."""
    from robocam.drivers.realsense import RealsenseCamera, discover_devices

    devices = discover_devices()
    if not devices:
        logger.warning("SKIP realsense_pointcloud: no RealSense cameras found")
        return

    dev = devices[0]
    cam = RealsenseCamera(serial_number=dev["serial"], fps=30, enable_depth=True)
    try:
        data = cam.read()
        assert data.depth_data is not None, "depth_data is None — enable_depth not working?"
        intrinsics = cam.read_calibration_data_intrinsics()
        K = intrinsics["K"]

        # RealSense depth is uint16 in mm by default — convert to meters
        depth_m = data.depth_data.astype(np.float32) * 0.001
        pts = depth_to_pointcloud(depth_m, K, stride=4)

        assert pts.shape[0] > 0, "No valid points"
        assert pts.shape[1] == 3
        logger.info(
            "PASS realsense_pointcloud: {} points, depth range [{:.2f}, {:.2f}]m",
            pts.shape[0],
            pts[:, 2].min(),
            pts[:, 2].max(),
        )
    finally:
        cam.stop()


def test_zed_pointcloud() -> None:
    """Read XYZRGBA from ZED and decode to point cloud."""
    try:
        from robocam.drivers.zed import ZedCamera, decode_xyzrgba, discover_devices as zed_discover
    except ImportError:
        logger.warning("SKIP zed_pointcloud: pyzed not available")
        return

    devices = zed_discover()
    if not devices:
        logger.warning("SKIP zed_pointcloud: no ZED cameras found")
        return

    serial = devices[0]["serial"]
    cam = ZedCamera(device_id=serial, fps=15, enable_depth=True)
    try:
        xyzrgba = cam.read_xyzrgba()
        assert xyzrgba.size > 0, "read_xyzrgba returned empty"

        pc = decode_xyzrgba(xyzrgba, stride=4)
        assert pc.points.shape[0] > 0, "No valid points after decode"
        assert pc.colors.shape[0] == pc.points.shape[0]
        logger.info(
            "PASS zed_pointcloud: {} points, depth range [{:.2f}, {:.2f}]m, colors shape {}",
            pc.points.shape[0],
            pc.points[:, 2].min(),
            pc.points[:, 2].max(),
            pc.colors.shape,
        )

        # Also test via read() + depth_data path
        data = cam.read()
        assert data.depth_data is not None, "depth_data not set in read()"
        intrinsics = cam.read_calibration_data_intrinsics()
        K = intrinsics["left"]["intrinsics_matrix"]
        pts = depth_to_pointcloud(data.depth_data, K, stride=4)
        assert pts.shape[0] > 0, "depth_to_pointcloud from read() failed"
        logger.info("PASS zed_depth_path: {} points via depth_to_pointcloud", pts.shape[0])
    finally:
        cam.stop()


@dataclass
class Args:
    """Point cloud diagnostics."""

    hardware: bool = False
    """Run hardware tests (needs connected cameras)."""
    camera_type: Optional[str] = None
    """Filter hardware tests: 'zed', 'realsense', or None for all."""


def main() -> None:
    args = tyro.cli(Args)

    logger.info("--- Point cloud unit tests ---")
    test_depth_to_pointcloud_basic()
    test_depth_to_pointcloud_nan_filtering()
    test_depth_to_pointcloud_stride()
    test_decode_xyzrgba_synthetic()

    if args.hardware:
        logger.info("--- Point cloud hardware tests ---")
        if args.camera_type in (None, "realsense"):
            test_realsense_pointcloud()
        if args.camera_type in (None, "zed"):
            test_zed_pointcloud()

    logger.info("--- All tests passed ---")


if __name__ == "__main__":
    main()

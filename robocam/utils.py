"""Generic image and point cloud utilities for robotics camera pipelines.

All functions work on plain numpy arrays — no framework dependency.
OpenCV and Pillow are lazy-imported so the base package stays lightweight.
"""

from __future__ import annotations

from typing import Any

import numpy as np


def resize_with_pad(
    images: np.ndarray,
    height: int,
    width: int,
) -> np.ndarray:
    """Resize images preserving aspect ratio, padding the remainder with black.

    Parameters
    ----------
    images : np.ndarray
        Shape ``(H, W, C)`` or ``(B, H, W, C)``.
    height, width : int
        Target dimensions.

    Returns
    -------
    np.ndarray
        Resized + padded image(s) with shape ``(height, width, C)`` or ``(B, height, width, C)``.
    """
    import cv2

    has_batch_dim = images.ndim == 4
    if not has_batch_dim:
        images = images[None]

    batch_size, cur_height, cur_width, channels = images.shape
    ratio = max(cur_width / width, cur_height / height)
    rh = int(cur_height / ratio)
    rw = int(cur_width / ratio)

    resized = np.zeros((batch_size, rh, rw, channels), dtype=images.dtype)
    for i in range(batch_size):
        resized[i] = cv2.resize(images[i], (rw, rh), interpolation=cv2.INTER_LINEAR)

    pad_h0, rem_h = divmod(height - rh, 2)
    pad_w0, rem_w = divmod(width - rw, 2)
    pad_value = -1.0 if images.dtype == np.float32 else 0

    padded = np.pad(
        resized,
        ((0, 0), (pad_h0, pad_h0 + rem_h), (pad_w0, pad_w0 + rem_w), (0, 0)),
        mode="constant",
        constant_values=pad_value,
    )

    return padded[0] if not has_batch_dim else padded


def resize_with_center_crop(
    images: np.ndarray,
    height: int,
    width: int,
) -> np.ndarray:
    """Resize images preserving aspect ratio, center-cropping the excess.

    Parameters
    ----------
    images : np.ndarray
        Shape ``(..., H, W, C)``  — arbitrary leading batch dimensions.
    height, width : int
        Target dimensions.

    Returns
    -------
    np.ndarray
        Resized + cropped image(s).
    """
    from PIL import Image

    if images.shape[-3:-1] == (height, width):
        return images

    orig_shape = images.shape
    flat = images.reshape(-1, *orig_shape[-3:])
    out = np.stack([np.array(_crop_single(Image.fromarray(im), height, width)) for im in flat])
    return out.reshape(*orig_shape[:-3], *out.shape[-3:])


def _crop_single(image: Any, height: int, width: int) -> Any:
    from PIL import Image as _PIL

    cur_w, cur_h = image.size
    if cur_w == width and cur_h == height:
        return image

    ratio = max(height / cur_h, width / cur_w)
    rw, rh = int(cur_w * ratio), int(cur_h * ratio)
    resized = image.resize((rw, rh), resample=_PIL.BILINEAR)

    x0 = max(0, (rw - width) // 2)
    y0 = max(0, (rh - height) // 2)
    cropped = resized.crop((x0, y0, min(rw, x0 + width), min(rh, y0 + height)))

    if cropped.size != (width, height):
        cropped = cropped.resize((width, height), resample=_PIL.BILINEAR)
    return cropped


def depth_to_pointcloud(
    depth: np.ndarray,
    K: np.ndarray,
    *,
    stride: int = 1,
) -> np.ndarray:
    """Back-project a depth image into a 3-D point cloud using camera intrinsics.

    Works with any depth source (RealSense, ZED depth-only, etc.).  For ZED
    cameras with packed XYZRGBA data, prefer :func:`robocam.drivers.zed.decode_xyzrgba`
    which is faster and preserves per-point colour.

    Parameters
    ----------
    depth : np.ndarray
        ``(H, W)`` depth image in arbitrary linear units (typically metres).
        Non-finite values (NaN, inf) are discarded.
    K : np.ndarray
        ``(3, 3)`` camera intrinsics matrix::

            [[fx,  0, cx],
             [ 0, fy, cy],
             [ 0,  0,  1]]
    stride : int
        Spatial down-sampling factor.  ``stride=2`` keeps every other pixel
        in both dimensions, reducing point count by ~4x.

    Returns
    -------
    np.ndarray
        ``(N, 3)`` float32 array of XYZ points in the camera frame.
    """
    h, w = depth.shape[:2]
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    v, u = np.mgrid[0:h:stride, 0:w:stride]
    z = depth[0:h:stride, 0:w:stride].astype(np.float32)

    valid = np.isfinite(z) & (z > 0)
    z = z[valid]
    u = u[valid].astype(np.float32)
    v = v[valid].astype(np.float32)

    x = (u - cx) * z / fx
    y = (v - cy) * z / fy

    return np.stack([x, y, z], axis=-1)

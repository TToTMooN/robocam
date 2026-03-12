"""Stereolabs ZED camera driver.

Requires: ``pip install robocam[zed]``  (or a local pyzed wheel)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict

import numpy as np
from loguru import logger

from robocam.camera import CameraData, PointCloudData

try:
    from pyzed import sl
except ImportError as _e:
    raise ImportError("pyzed is required for ZedCamera: pip install robocam[zed]") from _e

RESOLUTION_MAP = {
    "HD2K": sl.RESOLUTION.HD2K,
    "HD1200": sl.RESOLUTION.HD1200,
    "HD1080": sl.RESOLUTION.HD1080,
    "HD720": sl.RESOLUTION.HD720,
    "VGA": sl.RESOLUTION.VGA,
    "SVGA": sl.RESOLUTION.SVGA,
}
RESOLUTION_SIZE_MAP = {
    "HD2K": (2560, 1440),
    "HD1200": (1920, 1200),
    "HD1080": (1920, 1080),
    "HD720": (1280, 720),
    "VGA": (640, 480),
    "SVGA": (960, 600),
}
DEPTH_MODE_MAP = {
    "NEURAL_LIGHT": sl.DEPTH_MODE.NEURAL_LIGHT,
    "NEURAL": sl.DEPTH_MODE.NEURAL,
    "NEURAL_PLUS": sl.DEPTH_MODE.NEURAL_PLUS,
}


@dataclass
class ZedCamera:
    """ZED stereo camera driver.

    Parameters
    ----------
    resolution : str
        One of ``HD2K``, ``HD1200``, ``HD1080``, ``HD720``, ``VGA``, ``SVGA``.
    fps : int
        Target frame rate (must be valid for the chosen resolution).
    device_id : str or None
        Serial number. ``None`` picks the first available camera.
    image_transfer_time_offset_ms : float
        Milliseconds to subtract from device timestamp to approximate true capture time.
    concat_image : bool
        Concatenate left + right images into one wide frame.
    return_right_image : bool
        Include the right stereo image in output.
    enable_depth : bool
        Enable neural depth estimation.
    name : str or None
        Human-readable label.
    """

    resolution: str = "HD720"
    fps: int = 30
    device_id: str | None = None
    image_transfer_time_offset_ms: float = 70
    concat_image: bool = False
    return_right_image: bool = False
    name: str | None = None
    enable_depth: bool = False
    depth_mode: str = "NEURAL_PLUS"
    """Depth mode: NEURAL_LIGHT, NEURAL, NEURAL_PLUS."""

    def __repr__(self) -> str:
        return f"ZedCamera(device_id={self.device_id!r}, name={self.name!r}, resolution={self.resolution}, fps={self.fps})"

    @classmethod
    def check_available_cameras(cls) -> None:
        """Print all connected ZED cameras."""
        for c in sl.Camera.get_device_list():
            logger.info("ZED camera serial: {}", c.serial_number)

    def __post_init__(self) -> None:
        self.zed = sl.Camera()

        init_params = sl.InitParameters()
        if self.device_id:
            init_params.set_from_serial_number(int(self.device_id))
        init_params.camera_resolution = RESOLUTION_MAP[self.resolution]
        self.width, self.height = RESOLUTION_SIZE_MAP[self.resolution]
        init_params.camera_fps = self.fps
        if self.enable_depth:
            init_params.depth_mode = DEPTH_MODE_MAP[self.depth_mode]
            init_params.coordinate_units = sl.UNIT.METER
        else:
            init_params.depth_mode = sl.DEPTH_MODE.NONE

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {err!r}")

        logger.info("ZED camera opened (device_id={})", self.device_id)

        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE)
        self.zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN)

        self.image_left = sl.Mat()
        self.image_right = sl.Mat()
        if self.enable_depth:
            self.depth_map = sl.Mat()

        self.camera_info = self.zed.get_camera_information()
        self.runtime_parameters = sl.RuntimeParameters()
        self.runtime_parameters.confidence_threshold = 75
        self.camera_type = self.camera_info.camera_model.name

        self.serial_number: int = self.camera_info.serial_number if self.device_id is None else int(self.device_id)
        self._serial_prefix = str(self.serial_number)
        self.intrinsic_data = {
            f"{self._serial_prefix}_left": self._load_intrinsic_data("left"),
            f"{self._serial_prefix}_right": self._load_intrinsic_data("right"),
        }
        logger.info("ZED ready: {}", self)

    def _load_intrinsic_data(self, camera_side: str, raw: bool = False) -> dict:
        if raw:
            calib_params = self.camera_info.camera_configuration.calibration_parameters_raw
        else:
            calib_params = self.camera_info.camera_configuration.calibration_parameters
        cam = getattr(calib_params, f"{camera_side}_cam")
        K = np.array([[cam.fx, 0, cam.cx], [0, cam.fy, cam.cy], [0, 0, 1]])
        return {
            "intrinsics_matrix": K,
            "distortion_coefficients": list(cam.disto),
            "distortion_model": "zed_rectified",
        }

    def read(self) -> CameraData:
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.image_left, sl.VIEW.LEFT)
            if self.return_right_image:
                self.zed.retrieve_image(self.image_right, sl.VIEW.RIGHT)
            ts_image = int(self.zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_microseconds() / 1000)

            left_bgra = self.image_left.get_data()
            right_bgra = self.image_right.get_data() if self.return_right_image else None

            # Sanity check: detect all-black frames
            if np.all(left_bgra[::10, ::10, :3] < 8):
                raise RuntimeError(f"ZED camera {self.device_id} left image is all black")
            if self.return_right_image and np.all(right_bgra[::10, ::10, :3] < 8):
                raise RuntimeError(f"ZED camera {self.device_id} right image is all black")

            adjusted_ts = ts_image - self.image_transfer_time_offset_ms

            s = self._serial_prefix
            if self.concat_image:
                if not self.return_right_image:
                    raise RuntimeError("concat_image=True requires return_right_image=True")
                left_rgb = np.ascontiguousarray(left_bgra[:, :, :3][:, :, ::-1])
                right_rgb = np.ascontiguousarray(right_bgra[:, :, :3][:, :, ::-1])
                result = CameraData(
                    images={f"{s}_concatenated": np.concatenate([left_rgb, right_rgb], axis=1)},
                    timestamp=adjusted_ts,
                )
            else:
                left_rgb = np.ascontiguousarray(left_bgra[:, :, :3][:, :, ::-1])
                images: Dict[str, np.ndarray] = {f"{s}_left": left_rgb}
                if self.return_right_image:
                    images[f"{s}_right"] = np.ascontiguousarray(right_bgra[:, :, :3][:, :, ::-1])
                result = CameraData(images=images, timestamp=adjusted_ts)

            if self.enable_depth:
                self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
                result.depth_data = np.ascontiguousarray(self.depth_map.get_data())

            return result

        logger.warning("{}: Failed to grab image", self)
        s = self._serial_prefix
        if self.concat_image:
            return CameraData(images={f"{s}_concatenated": None}, timestamp=-1.0)  # type: ignore[arg-type]
        return CameraData(images={f"{s}_left": None, f"{s}_right": None}, timestamp=-1.0)  # type: ignore[arg-type]

    def read_depth(self) -> np.ndarray:
        """Read only depth map (requires ``enable_depth=True``)."""
        assert self.enable_depth, "Depth is not enabled"
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(self.depth_map, sl.MEASURE.DEPTH)
            return self.depth_map.get_data()
        logger.warning("{}: Failed to grab depth", self)
        return np.zeros((0, 0))

    def read_xyzrgba(self) -> np.ndarray:
        """Read XYZRGBA point cloud measure (requires ``enable_depth=True``).

        Returns
        -------
        np.ndarray
            ``(H, W, 4)`` float32 array where channels 0-2 are XYZ in metres
            and channel 3 is packed RGBA.  Pass to :func:`decode_xyzrgba` to
            get usable ``(N, 3)`` points and colours.
        """
        assert self.enable_depth, "Depth is not enabled"
        if not hasattr(self, "_xyzrgba_mat"):
            self._xyzrgba_mat = sl.Mat()
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_measure(self._xyzrgba_mat, sl.MEASURE.XYZRGBA)
            return self._xyzrgba_mat.get_data().copy()
        logger.warning("{}: Failed to grab XYZRGBA", self)
        return np.zeros((0, 0, 4), dtype=np.float32)

    def read_calibration_data_intrinsics(self) -> dict:
        return self.intrinsic_data

    def get_camera_info(self) -> dict:
        return {
            "camera_type": "zed",
            "device_id": str(self.device_id),
            "width": self.width,
            "height": self.height,
            "polling_fps": self.fps,
            "name": self.name if self.name is not None else "zed_camera",
            "image_transfer_time_offset_ms": self.image_transfer_time_offset_ms,
            "intrinsics": self.intrinsic_data,
            "concat_image": self.concat_image,
        }

    def stop(self) -> None:
        self.zed.close()
        logger.info("Stopped ZED camera: {}", self)


def decode_xyzrgba(
    xyzrgba: np.ndarray,
    *,
    stride: int = 4,
    rotate_to_z_up: bool = True,
) -> PointCloudData:
    """Decode a ZED ``XYZRGBA`` measure into a :class:`PointCloudData`.

    The ZED SDK packs RGBA into a single float32 channel.  This function
    unpacks it, filters invalid (NaN/inf) points, and optionally
    downsamples by *stride* for manageable point counts.

    Parameters
    ----------
    xyzrgba : np.ndarray
        ``(H, W, 4)`` float32 array from
        ``cam.retrieve_measure(..., sl.MEASURE.XYZRGBA)`` or
        :meth:`ZedCamera.read_xyzrgba`.
    stride : int
        Keep every *stride*-th valid point.  With HD720
        (921 600 pixels) a stride of 4 yields ~57 k points.
    rotate_to_z_up : bool
        If ``True`` (default), rotate points from ZED camera convention
        (X-right, Y-down, Z-forward) into Z-up convention (X-right,
        Y-forward, Z-up).  Set to ``False`` to keep points in the
        camera's native frame.

    Returns
    -------
    PointCloudData
        Decoded point cloud with ``points`` ``(N, 3)`` float32 and
        ``colors`` ``(N, 3)`` uint8 RGB.
    """
    flat = xyzrgba.reshape(-1, 4)
    valid = np.isfinite(flat[:, 0])
    flat = flat[valid]

    if stride > 1:
        flat = flat[::stride]

    cam_xyz = flat[:, :3]
    if rotate_to_z_up:
        points = np.empty_like(cam_xyz)
        points[:, 0] = cam_xyz[:, 0]
        points[:, 1] = cam_xyz[:, 2]
        points[:, 2] = -cam_xyz[:, 1]
    else:
        points = cam_xyz.copy()

    rgba_packed = flat[:, 3].view(np.uint32)
    colors = np.stack(
        [
            (rgba_packed >> 16) & 0xFF,
            (rgba_packed >> 8) & 0xFF,
            rgba_packed & 0xFF,
        ],
        axis=-1,
    ).astype(np.uint8)

    return PointCloudData(points=points, colors=colors)

"""Insta360 X5 live-stream camera driver (in-process, no ROS).

Insta360 CameraSDK callback -> native openh264 decode -> latest-frame slot,
pulled through a blocking :meth:`Insta360Camera.read`. Measured ~131 ms
photons-to-numpy at 1920x960, against ~156 ms for the same camera's ROS2
topic path.

The SDK delivers frames on its own callback thread and ``read()`` blocks on a
condition variable, so this driver is safe under :class:`CaptureThread` -
with **one reader per camera**. The native layer keeps a single latest-frame
slot and a single ``last_read_seq``, so two threads reading one handle steal
frames from each other rather than each seeing every frame.

Two cameras in one process need distinct ``service_port`` values (e.g. 9999
and 10000). Two *processes* cannot share the camera fleet at all: the SDK
binds its service port per process. The camera must be in **Android USB
mode**, and only one SDK session may hold a given camera - stop any ROS2
driver first.

``timestamp`` is capture-side, not arrival-side: the SDK's per-AU device
timestamp anchored to host ``CLOCK_REALTIME`` by the minimum observed delay
over the first ~90 access units, minus ``image_transfer_time_offset_ms``.
Unlike most robocam drivers, that offset is measured rather than guessed -
see "Insta360 SDK setup" in the README.

Requires the native core, built once per environment::

    INSTA360_SDK_ROOT=/path/to/insta360_sdk bash native/build.sh

That build artifact lives at the repo root rather than inside the package, so
this driver works from a source or editable install only. That is deliberate:
no prebuilt binary can be valid for an arbitrary environment.
"""

from __future__ import annotations

import ctypes
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
from loguru import logger

from robocam.camera import CameraData

# Repo root, NOT inside the package: flit packages everything under robocam/
# regardless of .gitignore, and an env-locked .so must never reach a wheel.
_NATIVE = Path(__file__).resolve().parents[2] / "_native" / "libinsta_source.so"

_LENS_CODE = {"full": 0, "front": 1, "back": 2}

# Largest frame the X5 can deliver; the buffer is regrown if a config
# somehow negotiates something bigger.
_MAX_FRAME_BYTES = 2656 * 1328 * 3


def _load_native() -> ctypes.CDLL:
    """Load ``libinsta_source.so`` and declare its C ABI.

    This is a build artifact rather than a pip dependency, so a missing
    library is a "you have not built it yet" error, not an ``ImportError``.
    """
    if not _NATIVE.exists():
        raise FileNotFoundError(
            f"{_NATIVE} not built. From a robocam source checkout, run:\n"
            f"    INSTA360_SDK_ROOT=/path/to/insta360_sdk bash native/build.sh\n"
            f"See the 'Insta360 SDK setup' section of the robocam README.\n"
            f"If that path looks wrong, robocam was installed non-editable: this "
            f"driver needs a source or editable install, since the native core is "
            f"built per environment and is never shipped in a wheel."
        )
    lib = ctypes.CDLL(str(_NATIVE))
    lib.ins_open.restype = ctypes.c_void_p
    lib.ins_open.argtypes = [
        ctypes.c_char_p,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
    ]
    lib.ins_read.restype = ctypes.c_int
    lib.ins_read.argtypes = [
        ctypes.c_void_p,
        ctypes.POINTER(ctypes.c_uint8),
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int64),
        ctypes.c_int,
        ctypes.c_double,
    ]
    lib.ins_frames.restype = ctypes.c_uint64
    lib.ins_frames.argtypes = [ctypes.c_void_p]
    lib.ins_decode_errors.restype = ctypes.c_uint64
    lib.ins_decode_errors.argtypes = [ctypes.c_void_p]
    lib.ins_close.restype = None
    lib.ins_close.argtypes = [ctypes.c_void_p]
    return lib


@dataclass
class Insta360Camera:
    """Insta360 live-stream camera driver.

    The default configuration is the low-latency one: 1920x960 through the
    live-view unlock, front lens only, eps = 86 ms. Re-measure
    ``image_transfer_time_offset_ms`` after changing resolution or lens - it
    is resolution-dependent (86 ms at 1920x960, 130 ms at 2656x1328).

    Parameters
    ----------
    serial : str
        SDK serial number. Empty picks the first discovered camera.
    resolution : str
        Requested stream resolution. On the X5 this is only honored with
        ``live_view_mode``.
    bitrate : int
        Encoder bitrate in bits/s.
    live_view_mode : bool
        SDK 2.1.1 live-view flow; unlocks the X5 preview resolution.
    lens : str
        Which lens to return: ``full``, ``front`` (right half), or ``back``.
    image_transfer_time_offset_ms : float
        Milliseconds subtracted from the device timestamp to approximate true
        capture time. QR-calibrated for this configuration.
    read_timeout_s : float
        ``read()`` raises :class:`TimeoutError` after this long with no frame.
    service_port : int
        SDK local service port; 0 uses the SDK default. Two cameras in one
        process need distinct ports.
    camera_type : str
        Driver tag echoed in :meth:`get_camera_info`.
    name : str or None
        Human-readable label.
    """

    serial: str = ""
    resolution: str = "1920x960"
    bitrate: int = 524288
    live_view_mode: bool = True
    lens: str = "front"
    image_transfer_time_offset_ms: float = 86.0
    read_timeout_s: float = 5.0
    service_port: int = 0
    camera_type: str = "insta360_camera"
    name: Optional[str] = None

    _lib: Optional[ctypes.CDLL] = field(init=False, repr=False, default=None)
    _handle: Optional[ctypes.c_void_p] = field(init=False, repr=False, default=None)
    _buf: Optional[np.ndarray] = field(init=False, repr=False, default=None)

    def __repr__(self) -> str:
        id_str = self.serial or "first-discovered"
        return f"Insta360Camera({id_str!r}, name={self.name!r}, resolution={self.resolution}, lens={self.lens})"

    def __post_init__(self) -> None:
        if self.lens not in _LENS_CODE:
            raise ValueError(f"lens must be one of {sorted(_LENS_CODE)}, got {self.lens!r}")
        self._lib = _load_native()
        err = ctypes.create_string_buffer(256)
        handle = self._lib.ins_open(
            self.serial.encode(),
            self.resolution.encode(),
            self.bitrate,
            int(self.live_view_mode),
            self.service_port,
            err,
            len(err),
        )
        if not handle:
            raise RuntimeError(f"insta_source open failed: {err.value.decode()}")
        self._handle = ctypes.c_void_p(handle)
        logger.info("Opened {} (service_port={})", self, self.service_port or "SDK default")

    def read(self) -> CameraData:
        """Block until a frame newer than the last returned one arrives."""
        if self._handle is None:
            raise RuntimeError(f"{self} is stopped")
        w = ctypes.c_int()
        h = ctypes.c_int()
        stamp_ns = ctypes.c_int64()
        lens_code = _LENS_CODE[self.lens]
        deadline = time.monotonic() + self.read_timeout_s
        while True:
            if self._buf is None:
                self._buf = np.empty(_MAX_FRAME_BYTES, dtype=np.uint8)
            ptr = self._buf.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))
            n = self._lib.ins_read(
                self._handle,
                ptr,
                self._buf.size,
                ctypes.byref(w),
                ctypes.byref(h),
                ctypes.byref(stamp_ns),
                lens_code,
                max(0.0, deadline - time.monotonic()),
            )
            if n > 0:
                # `.copy()` is load-bearing: `_buf` is reused by the next
                # read, so a view (or `np.ascontiguousarray`, which returns
                # the input untouched when it is already contiguous) would
                # alias into a buffer that is about to be overwritten.
                rgb = self._buf[:n].reshape(h.value, w.value, 3).copy()
                ts_ms = stamp_ns.value / 1e6 - self.image_transfer_time_offset_ms
                return CameraData(images={"rgb": rgb}, timestamp=ts_ms)
            if n == -1:  # buffer too small (unexpected resolution); regrow
                self._buf = np.empty(w.value * h.value * 3, dtype=np.uint8)
                continue
            if n == -2:
                raise RuntimeError(f"{self}: camera closed")
            raise TimeoutError(
                f"{self}: no frame within {self.read_timeout_s:.1f}s (driver stats: {self.get_camera_info()})"
            )

    def get_camera_info(self) -> Dict[str, Any]:
        info: Dict[str, Any] = {
            "camera_type": self.camera_type,
            "serial": self.serial or "(first discovered)",
            "resolution": self.resolution,
            "lens": self.lens,
            "live_view_mode": self.live_view_mode,
            "image_transfer_time_offset_ms": self.image_transfer_time_offset_ms,
        }
        if self.name:
            info["name"] = self.name
        if self._handle is not None:
            info["frames_decoded"] = int(self._lib.ins_frames(self._handle))
            info["decode_errors"] = int(self._lib.ins_decode_errors(self._handle))
        return info

    def read_calibration_data_intrinsics(self) -> Dict[str, Any]:
        # Insta360 intrinsics are per-unit and resolution-dependent; they live
        # in a downstream camera registry, not in the SDK.
        raise NotImplementedError(f"Calibration data reading is not implemented for {self}")

    def stop(self) -> None:
        """Release the camera.

        Always reach this. A killed process leaves the camera claimed on USB
        (``LIBUSB_ERROR_BUSY``) until it is physically replugged.
        """
        if self._handle is not None:
            self._lib.ins_close(self._handle)
            self._handle = None
            logger.info("Stopped {}", self)

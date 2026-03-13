"""Per-camera capture thread and multi-camera capture group.

:class:`CaptureThread` — one daemon thread per camera, calls ``read()`` in a
tight loop.  Works for cameras whose SDK supports concurrent reads from
separate threads (e.g. ZED ``grab()``).

:class:`CaptureGroup` — one daemon thread that polls **multiple** cameras
sequentially.  Useful when you want a single background thread to service
several thread-safe cameras.

Both write into per-camera :class:`FrameBuffer` instances so the consumer
API is identical.

Threading constraints by SDK
----------------------------
- **ZED** — ``sl.Camera.grab()`` is fully thread-safe.  Use CaptureThread
  (one thread per camera) or CaptureGroup freely.
- **RealSense** — ``pipeline.wait_for_frames()`` must be called from the
  **main thread**.  Background threads receive ~16 internally-queued frames
  then permanently stall (hard libusb / SDK limitation).  For multi-camera
  RealSense setups, either poll sequentially on the main thread or use
  separate **processes** (see ``limb``'s Portal RPC pattern or jc211/realsense
  SharedMemory approach).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Dict, Optional

from loguru import logger

from robocam.frame_buffer import FrameBuffer

if TYPE_CHECKING:
    from robocam.camera import CameraDriver


@dataclass
class CaptureThread:
    """Daemon thread that continuously reads from one camera into a FrameBuffer.

    Use this for cameras whose SDK supports concurrent reads (e.g. ZED).
    For RealSense multi-camera, use :class:`CaptureGroup` instead.

    Parameters
    ----------
    camera_id : str
        Label used for logging and the thread name.
    camera : CameraDriver
        The driver to poll. Must be already opened.
    buffer : FrameBuffer
        Destination buffer. Written by this thread; read by consumers.
    max_consecutive_errors : int
        How many successive ``camera.read()`` failures to tolerate before
        giving up and marking the thread as failed.

    Example
    -------
    >>> ct = CaptureThread(camera_id="zed:12345", camera=cam, buffer=FrameBuffer())
    >>> ct.start()
    >>> frame = ct.buffer.get_latest()
    >>> ct.stop()
    """

    camera_id: str
    camera: CameraDriver
    buffer: FrameBuffer
    max_consecutive_errors: int = 10

    _thread: Optional[threading.Thread] = field(init=False, repr=False, default=None)
    _stop_event: threading.Event = field(init=False, repr=False)
    _failed: bool = field(init=False, repr=False, default=False)
    _frame_count: int = field(init=False, repr=False, default=0)

    def __post_init__(self) -> None:
        self._stop_event = threading.Event()

    def start(self) -> None:
        """Spawn the capture thread. Call once after the camera is open."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError(f"CaptureThread {self.camera_id} already running")
        self._thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"cam-capture-{self.camera_id}",
        )
        self._thread.start()
        logger.debug("CaptureThread started: {}", self.camera_id)

    def stop(self, timeout: float = 2.0) -> None:
        """Signal the thread to stop and wait for it to exit.

        Must be called before ``camera.stop()`` to avoid a race between
        the capture loop and hardware teardown.
        """
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning("CaptureThread {}: did not exit within {}s", self.camera_id, timeout)
        logger.debug("CaptureThread stopped: {} ({} frames)", self.camera_id, self._frame_count)

    def is_alive(self) -> bool:
        """Return True if the capture thread is currently running."""
        return self._thread is not None and self._thread.is_alive()

    @property
    def frame_count(self) -> int:
        """Total frames captured since start (monotonically increasing)."""
        return self._frame_count

    @property
    def failed(self) -> bool:
        """True if the thread exited due to too many consecutive errors."""
        return self._failed

    def _capture_loop(self) -> None:
        error_count = 0
        while not self._stop_event.is_set():
            try:
                data = self.camera.read()
                self.buffer.put(data)
                self._frame_count += 1
                error_count = 0
            except Exception as exc:
                error_count += 1
                logger.warning(
                    "cam-capture-{}: read() error ({}/{}): {}",
                    self.camera_id,
                    error_count,
                    self.max_consecutive_errors,
                    exc,
                )
                if error_count >= self.max_consecutive_errors:
                    logger.error("cam-capture-{}: too many errors, halting", self.camera_id)
                    self._failed = True
                    break


@dataclass
class CaptureGroup:
    """One daemon thread that polls multiple cameras sequentially.

    Required for cameras like RealSense where concurrent ``wait_for_frames()``
    from separate threads causes libusb contention and frame delivery stalls.
    Sequential polling on a single thread matches the pattern that works
    reliably (identical to main-thread sequential reads, just off-loaded to a
    background thread).

    Parameters
    ----------
    cameras : dict
        ``{camera_id: CameraDriver}`` — all cameras to poll.
    buffers : dict
        ``{camera_id: FrameBuffer}`` — per-camera output buffers.
    max_consecutive_errors : int
        Per-camera error tolerance before that camera is skipped.

    Example
    -------
    >>> cameras = {"rs:123": cam1, "rs:456": cam2}
    >>> buffers = {label: FrameBuffer(max_size=16) for label in cameras}
    >>> group = CaptureGroup(cameras=cameras, buffers=buffers)
    >>> group.start()
    >>> frame = buffers["rs:123"].get_latest()
    >>> group.stop()
    """

    cameras: Dict[str, CameraDriver]
    buffers: Dict[str, FrameBuffer]
    max_consecutive_errors: int = 10

    _thread: Optional[threading.Thread] = field(init=False, repr=False, default=None)
    _stop_event: threading.Event = field(init=False, repr=False)
    _frame_counts: Dict[str, int] = field(init=False, repr=False)
    _failed_cameras: Dict[str, bool] = field(init=False, repr=False)

    def __post_init__(self) -> None:
        if set(self.cameras.keys()) != set(self.buffers.keys()):
            raise ValueError("cameras and buffers must have the same keys")
        self._stop_event = threading.Event()
        self._frame_counts = {label: 0 for label in self.cameras}
        self._failed_cameras = {label: False for label in self.cameras}

    def start(self) -> None:
        """Spawn the polling thread."""
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError("CaptureGroup already running")
        self._thread = threading.Thread(
            target=self._poll_loop,
            daemon=True,
            name="cam-capture-group",
        )
        self._thread.start()
        logger.debug("CaptureGroup started: {} cameras", len(self.cameras))

    def stop(self, timeout: float = 5.0) -> None:
        """Signal stop and wait for the thread to exit."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)
            if self._thread.is_alive():
                logger.warning("CaptureGroup did not exit within {}s", timeout)
        for label, count in self._frame_counts.items():
            logger.debug("CaptureGroup stopped: {} ({} frames)", label, count)

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def frame_count(self, camera_id: str) -> int:
        return self._frame_counts.get(camera_id, 0)

    def failed(self, camera_id: str) -> bool:
        return self._failed_cameras.get(camera_id, False)

    @property
    def total_frame_count(self) -> int:
        return sum(self._frame_counts.values())

    def _poll_loop(self) -> None:
        error_counts: Dict[str, int] = {label: 0 for label in self.cameras}
        active = set(self.cameras.keys())

        while not self._stop_event.is_set() and active:
            for label in list(active):
                if self._stop_event.is_set():
                    break
                try:
                    data = self.cameras[label].read()
                    self.buffers[label].put(data)
                    self._frame_counts[label] += 1
                    error_counts[label] = 0
                except Exception as exc:
                    error_counts[label] += 1
                    logger.warning(
                        "cam-group-{}: read() error ({}/{}): {}",
                        label,
                        error_counts[label],
                        self.max_consecutive_errors,
                        exc,
                    )
                    if error_counts[label] >= self.max_consecutive_errors:
                        logger.error("cam-group-{}: too many errors, removing from poll", label)
                        self._failed_cameras[label] = True
                        active.discard(label)

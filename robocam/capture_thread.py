"""Per-camera capture thread for parallel multi-camera acquisition.

Each :class:`CaptureThread` owns one camera driver and one :class:`FrameBuffer`.
It runs a daemon thread that calls ``camera.read()`` in a tight loop and pushes
frames into the buffer so consumers never block on camera I/O.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

from loguru import logger

from robocam.frame_buffer import FrameBuffer

if TYPE_CHECKING:
    from robocam.camera import CameraDriver


@dataclass
class CaptureThread:
    """Daemon thread that continuously reads from one camera into a FrameBuffer.

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

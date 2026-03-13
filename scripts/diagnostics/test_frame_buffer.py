#!/usr/bin/env python3
"""Smoke test for FrameBuffer Condition-based wakeup and CaptureThread.

No hardware needed — uses DummyCamera.

Usage:
    uv run scripts/diagnostics/test_frame_buffer.py
"""

from __future__ import annotations

import threading
import time

import numpy as np
from loguru import logger

from robocam.camera import CameraData, DummyCamera
from robocam.capture_thread import CaptureThread
from robocam.frame_buffer import FrameBuffer


def _make_frame(ts: float = 0.0) -> CameraData:
    return CameraData(images={"rgb": np.zeros((2, 2, 3), dtype=np.uint8)}, timestamp=ts)


def test_timeout_precision() -> None:
    """get_latest() should timeout close to the requested duration, not overshoot."""
    buf = FrameBuffer(max_size=4)
    timeout = 0.05
    t0 = time.monotonic()
    try:
        buf.get_latest(timeout_sec=timeout)
        raise AssertionError("Should have raised TimeoutError")
    except TimeoutError:
        elapsed = time.monotonic() - t0
        assert elapsed < timeout + 0.02, f"Timeout took {elapsed:.3f}s, expected ~{timeout}s"
        logger.info("PASS timeout_precision: {:.3f}s (expect ~{:.2f}s)", elapsed, timeout)


def test_instant_wakeup() -> None:
    """Consumer should wake immediately when a frame is put, not poll at 1ms intervals."""
    buf = FrameBuffer(max_size=4)
    delay = 0.02

    def delayed_put() -> None:
        time.sleep(delay)
        buf.put(_make_frame(1.0))

    threading.Thread(target=delayed_put).start()
    t0 = time.monotonic()
    frame = buf.get_latest(timeout_sec=1.0)
    elapsed = time.monotonic() - t0
    assert frame.timestamp == 1.0
    # Should wake within ~1ms of the put, so total ~delay. Old spin-poll would add up to 1ms jitter.
    assert elapsed < delay + 0.01, f"Wakeup took {elapsed:.3f}s, expected ~{delay}s"
    logger.info("PASS instant_wakeup: {:.3f}s (expect ~{:.3f}s)", elapsed, delay)


def test_get_last_k() -> None:
    """get_last_k returns correct number of frames in order."""
    buf = FrameBuffer(max_size=8)
    for i in range(5):
        buf.put(_make_frame(float(i)))
    history = buf.get_last_k(3)
    assert len(history) == 3
    assert [f.timestamp for f in history] == [2.0, 3.0, 4.0]
    logger.info("PASS get_last_k: got {} frames, timestamps={}", len(history), [f.timestamp for f in history])


def test_overflow() -> None:
    """Buffer should drop oldest frames when full."""
    buf = FrameBuffer(max_size=3)
    for i in range(5):
        buf.put(_make_frame(float(i)))
    assert len(buf) == 3
    latest = buf.get_latest(timeout_sec=0.1)
    assert latest.timestamp == 4.0
    logger.info("PASS overflow: len={}, latest.ts={}", len(buf), latest.timestamp)


def test_capture_thread_with_dummy() -> None:
    """CaptureThread should accumulate frames from DummyCamera."""
    cam = DummyCamera()
    buf = FrameBuffer(max_size=32)
    ct = CaptureThread(camera_id="dummy", camera=cam, buffer=buf, max_consecutive_errors=3)
    ct.start()

    # Let it run briefly
    time.sleep(0.1)
    count = ct.frame_count
    assert count > 0, f"Expected frames > 0, got {count}"
    assert ct.is_alive()
    assert not ct.failed
    logger.info("PASS capture_thread: {} frames in 0.1s", count)

    ct.stop(timeout=2.0)
    assert not ct.is_alive()
    cam.stop()
    logger.info("PASS capture_thread: clean shutdown, total {} frames", ct.frame_count)


def test_capture_thread_error_handling() -> None:
    """CaptureThread should mark failed after max_consecutive_errors."""

    class FailingCamera:
        def read(self) -> CameraData:
            raise RuntimeError("sensor fault")

        def read_calibration_data_intrinsics(self) -> dict:
            return {}

        def get_camera_info(self) -> dict:
            return {}

        def stop(self) -> None:
            pass

    cam = FailingCamera()
    buf = FrameBuffer(max_size=4)
    ct = CaptureThread(camera_id="failing", camera=cam, buffer=buf, max_consecutive_errors=3)
    ct.start()
    time.sleep(0.2)
    assert ct.failed, "Expected CaptureThread to be marked as failed"
    assert not ct.is_alive()
    logger.info("PASS error_handling: thread failed as expected after {} errors", ct.frame_count)
    ct.stop()


def main() -> None:
    logger.info("--- FrameBuffer + CaptureThread diagnostics ---")
    test_timeout_precision()
    test_instant_wakeup()
    test_get_last_k()
    test_overflow()
    test_capture_thread_with_dummy()
    test_capture_thread_error_handling()
    logger.info("--- All tests passed ---")


if __name__ == "__main__":
    main()

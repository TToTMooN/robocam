#!/usr/bin/env python3
"""Verify which stream resolutions an Insta360 camera actually honors, and the lens crops.

Requires a real camera in Android USB mode. The SDK silently substitutes a
different resolution rather than failing when it dislikes a request, so the only
trustworthy check is to open the stream and measure the decoded frame.

Each resolution runs in its own subprocess: SDK device discovery is one-shot per
process, so a fresh process per case avoids reopening from a cached descriptor.

Usage:
    uv run scripts/diagnostics/test_insta360_resolutions.py
    uv run scripts/diagnostics/test_insta360_resolutions.py --include-unsupported
    uv run scripts/diagnostics/test_insta360_resolutions.py --resolutions 2880x2880  # see WARNING below

WARNING: 2880x2880 is never included implicitly. It fails to stream AND drops the
camera out of Android USB mode, which needs the on-camera settings menu to undo -
a replug does not. Name it explicitly if you want to re-confirm that on new firmware.
"""

from __future__ import annotations

import json
import signal
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import tyro
from loguru import logger

from robocam.drivers import insta360
from robocam.drivers.insta360 import Insta360Camera

# Known-bad, kept here so this script can re-test them even though the driver
# now rejects them. See _SUPPORTED_RESOLUTIONS in the driver for the findings.
_SILENT_FALLBACK = ("1440x720", "2304x1152")
_CAMERA_RESETTING = ("2880x2880",)


@dataclass
class Args:
    """Insta360 resolution + lens verification."""

    resolutions: str = ""
    """Comma-separated resolutions to test. Empty uses the driver's supported set."""
    include_unsupported: bool = False
    """Also test the known silent-fallback resolutions. Excludes 2880x2880 - name it explicitly."""
    lens_check: bool = True
    """Also verify the front/back half-frame crops at each working resolution."""
    frames: int = 8
    """Frames to read before measuring, to let the stream settle."""
    child: str = ""
    """Internal: run a single resolution in this process and print one RESULT line."""


def _diff(a: np.ndarray, b: np.ndarray) -> float:
    return round(float(np.abs(a.astype(np.int16) - b.astype(np.int16)).mean()), 2)


def run_one(resolution: str, frames: int, lens_check: bool) -> dict:
    """Open at one resolution, measure what the camera actually delivers."""
    # A bare SIGTERM kills the interpreter without unwinding, so the finally
    # below never runs and the camera stays claimed on USB until replug.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit("SIGTERM"))

    out: dict = {"requested": resolution}
    cam = None
    try:
        # The driver refuses these by design; this script's whole job is to
        # re-check that verdict against hardware, so bypass it deliberately.
        if resolution not in insta360._SUPPORTED_RESOLUTIONS:
            insta360._SUPPORTED_RESOLUTIONS = (*insta360._SUPPORTED_RESOLUTIONS, resolution)

        cam = Insta360Camera(resolution=resolution, lens="full", read_timeout_s=20.0)
        for _ in range(frames):
            cam.read()

        full = cam.read().images["rgb"]
        h, w = full.shape[:2]
        out["actual"] = f"{w}x{h}"
        out["honored"] = out["actual"] == resolution

        if lens_check:
            left, right = full[:, : w // 2], full[:, w // 2 :]
            crops = {}
            for lens in ("front", "back"):
                cam.lens = lens  # lens is per-read in the shim, so no reopen
                crops[lens] = cam.read().images["rgb"]
            # Consecutive reads are different frames, so compare each crop
            # against both halves: on a static scene the right one is far closer.
            out["lens_shapes_ok"] = all(c.shape == (h, w // 2, 3) for c in crops.values())
            out["front_is_right_half"] = _diff(crops["front"], right) < _diff(crops["front"], left)
            out["back_is_left_half"] = _diff(crops["back"], left) < _diff(crops["back"], right)

        out["decode_errors"] = int(cam.get_camera_info()["decode_errors"])
        out["ok"] = True
    except Exception as exc:  # noqa: BLE001 - a probe reports failures, never raises
        out["ok"] = False
        out["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if cam is not None:
            cam.stop()  # must reach this or the camera stays USB-claimed
    return out


def main() -> None:
    args = tyro.cli(Args)

    if args.child:
        print("RESULT " + json.dumps(run_one(args.child, args.frames, args.lens_check)))
        return

    if args.resolutions:
        targets = [r.strip() for r in args.resolutions.split(",") if r.strip()]
    else:
        targets = list(insta360._SUPPORTED_RESOLUTIONS)
        if args.include_unsupported:
            targets += list(_SILENT_FALLBACK)

    for r in targets:
        if r in _CAMERA_RESETTING:
            logger.warning("{} knocks the X5 out of Android USB mode - recovery needs the camera's menu", r)

    results = []
    for r in targets:
        logger.info("Probing {} ...", r)
        proc = subprocess.run(
            [
                sys.executable,
                str(Path(__file__).resolve()),
                "--child",
                r,
                "--frames",
                str(args.frames),
                "--lens-check" if args.lens_check else "--no-lens-check",
            ],
            capture_output=True,
            text=True,
            # The SDK logs raw USB endpoint bytes, which are not valid UTF-8.
            encoding="utf-8",
            errors="replace",
            timeout=180,
            check=False,  # a failed probe is a result to report, not an exception
        )
        line = next((ln for ln in proc.stdout.splitlines() if ln.startswith("RESULT ")), None)
        results.append(
            json.loads(line[len("RESULT ") :]) if line else {"requested": r, "ok": False, "error": "no RESULT line"}
        )

    logger.info("")
    logger.info("{:<12} {:<12} {:<9} {}", "requested", "actual", "honored", "notes")
    all_ok = True
    for res in results:
        if not res.get("ok"):
            logger.error("{:<12} {:<12} {:<9} {}", res["requested"], "-", "-", res.get("error", ""))
            all_ok = False
            continue
        notes = []
        if not res["honored"]:
            notes.append("SILENT FALLBACK")
            all_ok = False
        if res.get("lens_shapes_ok") is False or res.get("front_is_right_half") is False:
            notes.append("LENS CROP WRONG")
            all_ok = False
        if res.get("decode_errors"):
            notes.append(f"{res['decode_errors']} decode errors")
        logger.info(
            "{:<12} {:<12} {:<9} {}",
            res["requested"],
            res["actual"],
            "yes" if res["honored"] else "NO",
            ", ".join(notes) or "ok",
        )

    logger.info("")
    if all_ok:
        logger.info("PASS - every resolution streamed at the size requested")
    else:
        logger.error("FAIL - see notes above")
        sys.exit(1)


if __name__ == "__main__":
    main()

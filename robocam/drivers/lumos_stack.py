"""Bring up / tear down the docker + xv_sdk + TCP-sender stack that feeds
``LumosCamera``.

The Lumos hardware is not directly accessible from Python — its driver
(``xv_sdk``) is a ROS1 Noetic node that lives in a docker container
(``fastumi``). Two small Python scripts run inside that container and
ship pose + JPEG frames over TCP to whichever process is listening (in
our case, ``LumosCamera`` on the host).

Usage from a Python script::

    from robocam.drivers import lumos_stack
    lumos_stack.up(serial="250801DR48FP25002333")
    # ... use LumosCamera ...
    lumos_stack.down()

Or as a CLI::

    python -m robocam.drivers.lumos_stack up   --serial 250801DR48FP25002333
    python -m robocam.drivers.lumos_stack down

Configuration
-------------
All env vars share the ``FASTUMI_*`` prefix used by the in-container
sender scripts. There is one source of truth for each setting; any field
on :class:`LumosStack` defaults to the matching env var.

==========================  =====================================================
``FASTUMI_CONTAINER``       Docker container name. Default ``fastumi``.
``FASTUMI_TCP_RECEIVER_IP`` Host IP the senders should connect to. Default
                            ``127.0.0.1`` (works because the container runs
                            ``--network host``). Same name the in-container
                            senders read.
``FASTUMI_TCP_PORT``        Pose+clamp TCP port. Default 28999.
``FASTUMI_IMAGE_TCP_PORT``  Image TCP port. Default 28998.
``FASTUMI_SLAM_WAIT_S``     Seconds to wait for SLAM convergence between
                            ``slam/start`` and ``color_camera/start``.
                            Default 12.
``FASTUMI_ENABLE_COLOR``    ``1`` to call ``color_camera/start`` (default),
                            ``0`` to skip RGB (saves USB bandwidth).
``FASTUMI_IMG_JPEG_QUALITY``  JPEG quality the image sender uses. Default 60.
``FASTUMI_IMG_MAX_FPS``     Per-topic FPS cap on the image sender. Default 30.
``FASTUMI_IMG_DOWNSCALE``   Resize factor in the image sender (1.0 = original).
==========================  =====================================================

Lifecycle notes
---------------
- ``up()`` always restarts ``xv_sdk`` cleanly. Calling ``slam/start`` on
  an already-running ``xv_sdk`` trips a "Time has to be finite" assertion
  in the SDK; clean restart avoids it.
- ``down()`` kills the senders but leaves ``xv_sdk`` running so SLAM
  state is preserved for the next consumer.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass, field
from importlib.resources import files
from pathlib import Path
from typing import Optional

from loguru import logger


# The two in-container sender scripts are vendored under
# robocam/vendor/lumos/. They get docker-cp'd into the container at
# runtime, so they don't need to be importable as modules — just present
# on disk where importlib.resources can find them.
_VENDOR_LUMOS_DIR = Path(str(files("robocam.vendor.lumos")))


# Single source of truth: read FASTUMI_* env vars once. Dataclass fields below
# default to these. The senders inside the container read the same names, so
# we just pass them straight through.
DEFAULT_CONTAINER = os.environ.get("FASTUMI_CONTAINER", "fastumi")
DEFAULT_RECEIVER_IP = os.environ.get("FASTUMI_TCP_RECEIVER_IP", "127.0.0.1")
DEFAULT_TCP_PORT = os.environ.get("FASTUMI_TCP_PORT", "28999")
DEFAULT_IMAGE_TCP_PORT = os.environ.get("FASTUMI_IMAGE_TCP_PORT", "28998")
DEFAULT_SLAM_WAIT_S = float(os.environ.get("FASTUMI_SLAM_WAIT_S", "12"))
DEFAULT_ENABLE_COLOR = os.environ.get("FASTUMI_ENABLE_COLOR", "1") == "1"
DEFAULT_JPEG_QUALITY = int(os.environ.get("FASTUMI_IMG_JPEG_QUALITY", "60"))
DEFAULT_MAX_FPS = float(os.environ.get("FASTUMI_IMG_MAX_FPS", "30"))
DEFAULT_DOWNSCALE = float(os.environ.get("FASTUMI_IMG_DOWNSCALE", "1.0"))


@dataclass
class LumosStack:
    """Programmatic interface to the docker stack.

    Every field defaults to the matching ``FASTUMI_*`` env var, so the
    CLI, Python API, and in-container senders all share one source of
    truth.
    """

    serial: str  # tracker UUID, e.g. 250801DR48FP25002333
    container: str = field(default_factory=lambda: DEFAULT_CONTAINER)
    receiver_ip: str = field(default_factory=lambda: DEFAULT_RECEIVER_IP)
    tcp_port: str = field(default_factory=lambda: DEFAULT_TCP_PORT)
    image_tcp_port: str = field(default_factory=lambda: DEFAULT_IMAGE_TCP_PORT)
    slam_wait_s: float = field(default_factory=lambda: DEFAULT_SLAM_WAIT_S)
    enable_color: bool = field(default_factory=lambda: DEFAULT_ENABLE_COLOR)
    # Image-sender tunables (propagated to the container as FASTUMI_IMG_*).
    jpeg_quality: int = field(default_factory=lambda: DEFAULT_JPEG_QUALITY)
    max_fps: float = field(default_factory=lambda: DEFAULT_MAX_FPS)
    downscale: float = field(default_factory=lambda: DEFAULT_DOWNSCALE)

    def __post_init__(self) -> None:
        if shutil.which("docker") is None:
            raise RuntimeError("docker not found on PATH")
        if not _VENDOR_LUMOS_DIR.is_dir():
            raise RuntimeError(
                f"vendored Lumos sender scripts missing: {_VENDOR_LUMOS_DIR}. "
                "robocam install is broken — reinstall the package."
            )

    # ---- docker helpers --------------------------------------------------
    def _container_running(self) -> bool:
        r = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True
        )
        return self.container in r.stdout.split()

    def _container_exists(self) -> bool:
        r = subprocess.run(
            ["docker", "ps", "-a", "--format", "{{.Names}}"], capture_output=True, text=True
        )
        return self.container in r.stdout.split()

    def _exec(self, cmd: str, detach: bool = False, check: bool = True, capture: bool = False):
        full = (
            "source /opt/ros/noetic/setup.bash && "
            "source /workspace/devel/setup.bash && " + cmd
        )
        args = ["docker", "exec"]
        if detach:
            args.append("-d")
        args += [self.container, "bash", "-lc", full]
        return subprocess.run(args, check=check, capture_output=capture, text=True)

    def _exec_raw(self, cmd: str, check: bool = False) -> subprocess.CompletedProcess:
        return subprocess.run(
            ["docker", "exec", self.container, "bash", "-c", cmd],
            check=check, capture_output=True, text=True,
        )

    def _wait_topic(self, topic_substring: str, timeout: float = 30.0) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            r = self._exec("rostopic list 2>/dev/null", check=False, capture=True)
            if topic_substring in (r.stdout or ""):
                return True
            time.sleep(0.5)
        return False

    # ---- public ----------------------------------------------------------
    def up(self) -> None:
        """Bring up the full stack. Idempotent — kills any prior run first."""
        if not self._container_running():
            if self._container_exists():
                logger.info("starting existing container '{}'", self.container)
                subprocess.check_call(["docker", "start", self.container])
            else:
                raise RuntimeError(
                    f"container '{self.container}' does not exist. Run "
                    f"`./run.sh` from the fastumi-docker directory to create it."
                )

        logger.info("killing any existing xv_sdk + senders")
        # The C++ binary is /workspace/devel/lib/xv_sdk/xv_sdk — match that path
        # so we hit it directly. (Older guidance said `xv_sdk_node`, which the
        # build doesn't actually use, so stale binaries survived every relaunch
        # and competed for the USB device.)
        self._exec_raw(
            "pkill -9 -f 'lib/xv_sdk/xv_sdk'; "
            "pkill -9 -f xv_sdk_node; "
            "pkill -9 -f roslaunch; pkill -9 -f roscore; "
            "pkill -f fastumi_tcp_sender; pkill -f fastumi_image_tcp_sender",
        )
        time.sleep(2)

        logger.info("starting xv_sdk (roslaunch xv_sdk xv_sdk.launch)")
        self._exec(
            "nohup roslaunch xv_sdk xv_sdk.launch </dev/null >/tmp/xv_sdk.log 2>&1",
            detach=True,
        )

        logger.info("waiting for /xv_sdk/{}/ namespace ...", self.serial)
        if not self._wait_topic(f"/xv_sdk/{self.serial}/", timeout=30):
            raise RuntimeError(
                "xv_sdk did not register the device in 30 s. "
                f"Check `docker exec {self.container} cat /tmp/xv_sdk.log` — "
                "common causes: USB perms (run `sudo chmod 666 /dev/bus/usb/...`), "
                "wrong serial, or tracker unplugged."
            )

        logger.info("calling slam/start, clamp/start")
        for svc in ("slam/start", "clamp/start"):
            self._exec(
                f"rosservice call /xv_sdk/{self.serial}/{svc} '{{}}' 2>/dev/null || true",
                check=False,
            )

        logger.info("waiting {:.0f}s for SLAM to converge — wiggle the tracker", self.slam_wait_s)
        time.sleep(self.slam_wait_s)

        if self.enable_color:
            logger.info("calling color_camera/start")
            self._exec(
                f"rosservice call /xv_sdk/{self.serial}/color_camera/start '{{}}' "
                "2>/dev/null || true",
                check=False,
            )

        logger.info("copying vendored TCP senders into container")
        for src in ("fastumi_tcp_sender.py", "fastumi_image_tcp_sender.py"):
            p = _VENDOR_LUMOS_DIR / src
            if not p.exists():
                raise RuntimeError(f"missing vendored sender script: {p}")
            subprocess.check_call(["docker", "cp", str(p), f"{self.container}:/tmp/{src}"])

        env_str = (
            f"export FASTUMI_TCP_RECEIVER_IP={self.receiver_ip} "
            f"FASTUMI_TCP_PORT={self.tcp_port} "
            f"FASTUMI_IMAGE_TCP_PORT={self.image_tcp_port} "
            f"FASTUMI_SERIAL_LEFT={self.serial} "
            f"FASTUMI_SERIAL_RIGHT= "
            f"FASTUMI_IMG_JPEG_QUALITY={self.jpeg_quality} "
            f"FASTUMI_IMG_MAX_FPS={self.max_fps} "
            f"FASTUMI_IMG_DOWNSCALE={self.downscale}"
        )
        logger.info("starting pose+clamp sender (port {})", self.tcp_port)
        self._exec(
            f"{env_str}; nohup python3 -u /tmp/fastumi_tcp_sender.py "
            "</dev/null >/tmp/tcp_sender.log 2>&1",
            detach=True,
        )
        logger.info(
            "starting image sender (port {}, jpeg_q={}, max_fps={}, downscale={})",
            self.image_tcp_port, self.jpeg_quality, self.max_fps, self.downscale,
        )
        self._exec(
            f"{env_str}; nohup python3 -u /tmp/fastumi_image_tcp_sender.py "
            "</dev/null >/tmp/img_sender.log 2>&1",
            detach=True,
        )
        logger.info("LumosStack up.")

    def down(self, also_stop_xv_sdk: bool = False) -> None:
        """Kill the in-container senders. Optionally also stop xv_sdk."""
        cmd = "pkill -f fastumi_tcp_sender; pkill -f fastumi_image_tcp_sender"
        if also_stop_xv_sdk:
            cmd += (
                "; pkill -9 -f 'lib/xv_sdk/xv_sdk'"
                "; pkill -9 -f xv_sdk_node"
                "; pkill -9 -f roslaunch; pkill -9 -f roscore"
            )
        self._exec_raw(cmd)
        logger.info("LumosStack down.")

    def __enter__(self) -> "LumosStack":
        self.up()
        return self

    def __exit__(self, *exc) -> None:
        self.down()


def up(serial: Optional[str] = None, **kwargs) -> LumosStack:
    """Convenience wrapper. Discovers serial via sysfs if not given."""
    if serial is None:
        from robocam.drivers.lumos import LumosCamera

        devs = LumosCamera.discover_devices()
        if not devs:
            raise RuntimeError("no XVisio tracker (vendor 040e) found in sysfs")
        if len(devs) > 1:
            raise RuntimeError(
                "multiple trackers found; pass --serial: " + ", ".join(d["serial"] for d in devs)
            )
        serial = devs[0]["serial"]
        logger.info("auto-detected serial: {}", serial)
    stack = LumosStack(serial=serial, **kwargs)
    stack.up()
    return stack


def down(container: str = DEFAULT_CONTAINER, also_stop_xv_sdk: bool = False) -> None:
    """Convenience teardown — doesn't need a serial."""
    # Bypass __post_init__'s checks since we only need to send a pkill into
    # the container.
    stack = LumosStack.__new__(LumosStack)
    stack.container = container
    stack.down(also_stop_xv_sdk=also_stop_xv_sdk)


def _cli() -> int:
    import argparse

    p = argparse.ArgumentParser(prog="python -m robocam.drivers.lumos_stack")
    sub = p.add_subparsers(dest="cmd", required=True)

    p_up = sub.add_parser("up", help="bring up docker + xv_sdk + senders")
    p_up.add_argument("--serial", default=None, help="tracker UUID (auto-detect if omitted)")
    p_up.add_argument("--no-color", action="store_true", help="don't start color_camera")
    p_up.add_argument("--receiver-ip", default=DEFAULT_RECEIVER_IP)
    p_up.add_argument("--container", default=DEFAULT_CONTAINER)
    p_up.add_argument("--tcp-port", default=DEFAULT_TCP_PORT)
    p_up.add_argument("--image-tcp-port", default=DEFAULT_IMAGE_TCP_PORT)
    p_up.add_argument("--slam-wait-s", type=float, default=DEFAULT_SLAM_WAIT_S)
    p_up.add_argument("--jpeg-quality", type=int, default=DEFAULT_JPEG_QUALITY,
                      help="JPEG quality 1-100; lower = smaller frames")
    p_up.add_argument("--max-fps", type=float, default=DEFAULT_MAX_FPS,
                      help="per-topic FPS cap on the image sender")
    p_up.add_argument("--downscale", type=float, default=DEFAULT_DOWNSCALE,
                      help="resize factor on the image sender (1.0 = original)")

    p_down = sub.add_parser("down", help="stop senders (xv_sdk left running)")
    p_down.add_argument("--also-stop-xv-sdk", action="store_true")
    p_down.add_argument("--container", default=DEFAULT_CONTAINER)

    args = p.parse_args()
    if args.cmd == "up":
        up(
            serial=args.serial,
            container=args.container,
            receiver_ip=args.receiver_ip,
            tcp_port=args.tcp_port,
            image_tcp_port=args.image_tcp_port,
            slam_wait_s=args.slam_wait_s,
            enable_color=not args.no_color,
            jpeg_quality=args.jpeg_quality,
            max_fps=args.max_fps,
            downscale=args.downscale,
        )
    elif args.cmd == "down":
        down(container=args.container, also_stop_xv_sdk=args.also_stop_xv_sdk)
    return 0


if __name__ == "__main__":
    sys.exit(_cli())

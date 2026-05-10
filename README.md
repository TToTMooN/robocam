# robocam

Minimal camera drivers, async video encoding, and frame buffering for robotics.

No framework lock-in — works with any control loop, any process model.

## Install

```bash
uv sync

# Or with pip
pip install robocam
```

### pyzed (not on PyPI)

`pyzed` is not published on PyPI (there's a fake unrelated `pyzed` on PyPI). The real wheel is hosted on the Stereolabs CDN and configured automatically in `[tool.uv.sources]`. If you consume robocam as a git dependency, add this to your project's `pyproject.toml`:

```toml
[tool.uv.sources]
pyzed = { url = "https://download.stereolabs.com/zedsdk/5.2/whl/linux_x86_64/pyzed-5.2-cp311-cp311-linux_x86_64.whl" }
```

### System dependencies by driver

| Driver | Python package | System SDK needed? | Notes |
|--------|---------------|-------------------|-------|
| **RealSense** | `pyrealsense2` | No | Wheel bundles `librealsense`. Just install and go. |
| **ZED** | `pyzed` | **Yes — ZED SDK required** | `pyzed` is only thin Python bindings. The runtime (`libsl_zed.so`, CUDA kernels, neural depth models) must be installed separately. |
| **OpenCV** | `opencv-contrib-python` | No | Self-contained wheel. |
| **Lumos** (FastUMI Pro) | none — TCP receiver | **Yes — docker stack + XVSDK + udev rules** | Frames arrive over TCP from `xv_sdk` running in a docker container. See [Lumos / FastUMI setup](#lumos--fastumi-setup). |

### ZED SDK setup

The ZED driver requires the Stereolabs ZED SDK installed at the system level. `pyzed` alone is not enough — it's just thin Python bindings. The actual runtime (`libsl_zed.so`, CUDA kernels, neural depth models) must be installed separately.

1. Go to the [Stereolabs developer downloads page](https://www.stereolabs.com/developers/release)
2. Download the ZED SDK installer matching your **Ubuntu version** and **CUDA version**
3. Run the installer (it will install to `/usr/local/zed/`)

Verify:

```bash
python -c "from pyzed import sl; print('ZED SDK OK')"
```

If you see `libsl_zed.so: cannot open shared object file`, the SDK is not installed or not on the library path:

```bash
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH
```

### Lumos / FastUMI setup

The Lumos tracker isn't talked to directly — `xv_sdk` runs inside a docker container (ROS1 Noetic) and TCP-ships frames + pose to the host, where `LumosCamera` receives them. Three things have to be in place before `LumosCamera` will work.

The upstream sources live under `~/fastumi_driver/` on this host (git-tracked). Adjust the paths below if you cloned elsewhere.

**1. Install the udev rule so the host opens the tracker at mode 666 on plug-in.** Without this, `xv_sdk` fails with `LIBUSB_ERROR_ACCESS` and `lumos_stack up` times out waiting for the device namespace.

The rule itself is one line — `SUBSYSTEM=="usb", ATTR{idVendor}=="040e", MODE="0666", GROUP="plugdev"` — matching XVisio's USB vendor ID.

```bash
sudo cp ~/fastumi_driver/FastUMI_Hardware_SDK/xv/scripts/99-xvisio.rules \
        /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
# unplug + replug the tracker, then verify (note: bus/device numbers
# change every replug — the lsusb-derived path resolves them):
ls -l $(lsusb | awk '/040e:/ {printf "/dev/bus/usb/%s/%s\n",$2,substr($4,1,3)}')
# should show: crw-rw-rw- 1 root plugdev   (mode 666, group plugdev)
```

This is one-time. Once installed, every replug fires the rule automatically — no per-plug `chmod` needed. If the verification still shows `crw-rw-r--`, the rule didn't match: confirm the file is in `/etc/udev/rules.d/` and the device VID matches `040e` (`lsusb | grep 040e`).

**2. Build the `fastumi` docker image / container** (one-time):

```bash
cd ~/fastumi_driver/fastumi-docker && sudo ./run.sh
```

The first run also builds the image. The container uses `--rm`, so it lives only as long as that shell — leave it open, or drop `--rm` from `run.sh` if you want it persistent across sessions.

**3. Install the XVSDK `.deb` inside the container** (must be redone any time the container is recreated, since `/usr/lib` doesn't persist):

```bash
docker exec -u root fastumi bash -c \
    "apt-get update && \
     apt-get install -y dh-exec libcereal-dev libv4l-dev && \
     dpkg -i /opt/fastumi_sdk/xv/sdk/20260312/XVSDK_focal_amd64.deb"
```

Without it, `xv_sdk` dies with `libxvsdk.so: cannot open shared object file`. To skip this every time, patch the docker image's entrypoint to call `setup_lumos` when `libxvsdk.so` is missing — see `~/fastumi_driver/fastumi-docker/scripts/entrypoint.sh`.

After all three, bring up the stack and run a viewer:

```bash
uv run scripts/view_lumos.py --bring-up
```

See `robocam/drivers/lumos.py` (host-side receiver) and `robocam/drivers/lumos_stack.py` (docker / xv_sdk lifecycle) for the runtime knobs (all `FASTUMI_*` env vars).

### Stream rates and USB bandwidth

Measured on this hardware (Lumos on USB-2, Bus 03):

| Stream | Source rate | Notes |
|---|---|---|
| `fisheye_left/right`, `left2/right2` | ~12 Hz | Firmware-capped — not tunable from xv_sdk. Designed for SLAM, paired with the 500 Hz IMU. |
| `color_camera` | ~16 Hz | Lower than the 30 Hz nominal because of USB-2 |
| `imu_sensor/data_raw` | ~500 Hz | Reaches `LumosCamera` at ~100 Hz (pose sender batches one envelope per 10 ms) |
| `slam/pose`, `clamp/Data` | ~30 Hz | Latched into the 100 Hz pose envelope |

**USB bandwidth caveat:** on a USB-2 link (`lsusb -t` shows `480M` next to the tracker), enabling color saturates the bus and starves fisheye to ~1.6 Hz. Two options:

- **Move the tracker to a USB-3 port.** Most laptops have both — check `lsusb -t` for a parent root hub at `5000M` or higher and replug there. Color and fisheye then coexist cleanly.
- **Stay on USB-2 but pick one or the other:** `lumos_stack up --no-color` for full ~12 Hz fisheye, or accept color and let fisheye degrade.

Measure rates yourself with `rostopic hz` inside the container, or with the receiver-side script in the PR description.

## Quick Start

### Read frames from a RealSense camera

```python
from robocam.drivers.realsense import RealsenseCamera

camera = RealsenseCamera(serial_number="346123070863", fps=30)
data = camera.read()  # CameraData with .images["rgb"], .timestamp
camera.stop()
```

### Buffer frames for observation history

```python
from robocam import FrameBuffer

buf = FrameBuffer(max_size=32)

# Producer (polling thread)
buf.put(camera.read())

# Consumer (control loop or policy)
latest = buf.get_latest()           # single newest frame
history = buf.get_last_k(4)         # last 4 frames, oldest-first
```

### Record video with hardware-accelerated encoding

```python
from robocam import AsyncVideoWriter

writer = AsyncVideoWriter("episode.mp4", width=640, height=480, fps=30)
writer.start()

for frame in frames:
    writer.write(frame)  # non-blocking, piped to ffmpeg in bg thread

writer.stop()  # flushes and waits for ffmpeg
```

## API Reference

### Data Types

| Class | Description |
|-------|-------------|
| `CameraData` | Single capture: `.images` dict, `.timestamp` (ms), optional `.calibration_data`, `.imu_data` |
| `CameraSpec` | Named shape/dtype descriptor: `.name`, `.shape`, `.dtype` |
| `IMUData` | Timestamp + 3D acceleration + gyroscope |
| `CameraDriver` | Protocol — any class with `read()`, `stop()`, `get_camera_info()`, `read_calibration_data_intrinsics()` |

### Drivers

All drivers are `@dataclass` classes that satisfy the `CameraDriver` protocol.

#### `robocam.drivers.realsense.RealsenseCamera`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_number` | `str \| None` | `None` | Device serial. `None` = first available |
| `resolution` | `(int, int)` | `(640, 480)` | `(width, height)` |
| `fps` | `int` | `30` | Target frame rate |
| `enable_depth` | `bool` | `False` | Enable aligned depth stream |
| `name` | `str \| None` | `None` | Human label |

Static method: `RealsenseCamera.discover_devices()` returns `[{serial, name}, ...]`

#### `robocam.drivers.zed.ZedCamera`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | `str` | `"HD720"` | `HD2K`, `HD1200`, `HD1080`, `HD720`, `VGA`, `SVGA` |
| `fps` | `int` | `30` | Target frame rate |
| `device_id` | `str \| None` | `None` | Serial number |
| `concat_image` | `bool` | `False` | Concatenate L+R into one wide frame |
| `return_right_image` | `bool` | `False` | Include right stereo image |
| `enable_depth` | `bool` | `False` | Enable neural depth (NEURAL_PLUS) |

Class method: `ZedCamera.check_available_cameras()`

#### `robocam.drivers.opencv.OpencvCamera`

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `device_path` | `str` | `""` | `/dev/videoX`. Ignored if `serial_number` set |
| `serial_number` | `str \| None` | `None` | Resolves via `/dev/v4l/by-id/` |
| `resolution` | `(int, int)` | `(640, 480)` | `(width, height)` |
| `fps` | `int` | `30` | Target frame rate |
| `image_transfer_time_offset` | `int` | `80` | ms subtracted from wall-clock time |

#### `robocam.camera.DummyCamera`

Generates random noise images for testing. Takes optional `camera_specs` list.

### FrameBuffer

Thread-safe ring buffer of `CameraData` frames.

| Method | Description |
|--------|-------------|
| `put(data)` | Append a frame (non-blocking) |
| `get_latest(timeout_sec=1.0)` | Most recent frame. Raises `TimeoutError` if empty. |
| `get_last_k(k)` | Last *k* frames, oldest-first. Returns fewer if not enough available. |
| `clear()` | Drop all buffered frames |
| `len(buf)` | Current buffer size |
| `buf.count` | Total frames ever inserted |

### CaptureThread

Daemon thread that continuously reads from one camera into a `FrameBuffer`. Decouples camera I/O from the control loop so consumers never block on `camera.read()`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_id` | `str` | — | Label for logging and thread name |
| `camera` | `CameraDriver` | — | Already-opened camera driver to poll |
| `buffer` | `FrameBuffer` | — | Destination buffer (written by thread, read by consumers) |
| `max_consecutive_errors` | `int` | `10` | Successive `read()` failures before the thread gives up |

| Method / Property | Description |
|-------------------|-------------|
| `start()` | Spawn the capture daemon thread |
| `stop(timeout=2.0)` | Signal stop and join. Call **before** `camera.stop()` |
| `is_alive()` | Whether the capture thread is currently running |
| `frame_count` | Total frames captured since `start()` |
| `failed` | `True` if the thread exited due to too many consecutive errors |

**Important: CaptureThread works with ZED but NOT RealSense.**
See [Threading Model](#threading-model) below for details.

```python
from robocam import CaptureThread, FrameBuffer
from robocam.drivers.zed import ZedCamera

cam = ZedCamera(device_id="38082408", fps=30)
buf = FrameBuffer(max_size=16)
ct = CaptureThread(camera_id="wrist", camera=cam, buffer=buf)
ct.start()

# Consumer (control loop)
frame = buf.get_latest()

# Shutdown
ct.stop()
cam.stop()
```

### AsyncVideoWriter

Non-blocking video writer using ffmpeg subprocess.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `path` | `str` | — | Output `.mp4` path |
| `width` | `int` | — | Frame width |
| `height` | `int` | — | Frame height |
| `fps` | `int` | `30` | Output frame rate |
| `codec` | `str` | `"auto"` | `"auto"` picks `hevc_nvenc` if available, else `libx264` |
| `crf` | `int` | `23` | Quality (lower = better) |
| `queue_size` | `int` | `300` | Max frames buffered in memory |

| Method | Description |
|--------|-------------|
| `start()` | Launch ffmpeg subprocess + writer thread |
| `write(frame)` | Enqueue RGB uint8 `(H, W, 3)` frame |
| `stop()` | Flush queue, wait for ffmpeg to finish |

### Image Utilities (`robocam.utils`)

| Function | Description |
|----------|-------------|
| `resize_with_pad(images, h, w)` | Aspect-preserving resize + black padding |
| `resize_with_center_crop(images, h, w)` | Aspect-preserving resize + center crop |

Both accept `(H, W, C)` or `(B, H, W, C)` numpy arrays.

## Threading Model

Different camera SDKs have fundamentally different threading constraints:

| SDK | Thread-safe? | Recommended pattern |
|-----|-------------|-------------------|
| **ZED** (`sl.Camera.grab()`) | Yes | `CaptureThread` — one daemon thread per camera |
| **RealSense** (`pipeline.wait_for_frames()`) | **No — main thread only** | Poll sequentially on the main thread |
| **OpenCV** (`cv2.VideoCapture.read()`) | Yes (per device) | `CaptureThread` or main-thread polling |

### RealSense main-thread constraint

`pyrealsense2`'s `pipeline.wait_for_frames()` must be called from the **main thread**. Background threads receive ~16 internally-queued frames then permanently stall. This is a hard SDK / libusb limitation — not a contention or configuration issue.

**Workarounds for multi-camera RealSense:**
1. **Sequential main-thread polling** — simple, no overhead, works for 2-4 cameras at moderate FPS. This is what `scripts/view_cameras.py` does.
2. **Separate processes** — one process per camera with SharedMemory IPC (see [jc211/realsense](https://github.com/jc211/realsense)) or Portal RPC (see limb). Required for high-frequency control loops where main-thread blocking is unacceptable.

### Mixed setups (RealSense + ZED)

`view_cameras.py` demonstrates the hybrid pattern:
- RealSense cameras: polled directly in the main-thread display loop (`cam.read()`)
- ZED cameras: each on its own `CaptureThread`, read via `FrameBuffer`

```python
# RealSense — main thread
for label, cam in rs_cameras.items():
    data = cam.read()  # blocking, but only ~33ms per camera at 30fps

# ZED — background thread
for label, buf in zed_buffers.items():
    data = buf.get_latest(timeout_sec=0.05)  # non-blocking read from CaptureThread
```

## Architecture Notes

### Why separate from your control framework?

robocam provides the **camera primitives** — drivers, frame buffering, video encoding. Your control framework (e.g., limb) provides the **glue** — process model (Portal RPC, multiprocessing), observation types, config system. This separation means:

- Camera code is testable without robot hardware
- Same drivers work in data collection, policy inference, and standalone scripts
- Video writer works for any recording pipeline, not just robotics

### AsyncVideoWriter vs cv2.VideoWriter

| | `cv2.VideoWriter` | `AsyncVideoWriter` |
|---|---|---|
| Encoding | Synchronous, blocks caller | Background thread, non-blocking |
| Codec | CPU-only (mp4v, xvid) | NVENC hardware or CPU (h264, hevc) |
| Quality | Fixed codec options | Configurable CRF, preset |
| Throughput | ~15-30ms per frame at 720p | <1ms per `write()` call (just enqueues) |

### FrameBuffer vs shared-memory ring buffer

For **in-process** use (camera polling thread -> control loop reader), `FrameBuffer` with `threading.Condition` is simpler and sufficient — `get_latest()` wakes up instantly when a new frame arrives (no spin-polling). Shared-memory ring buffers (like jc211/realsense) are needed when the reader and writer are in **different OS processes** — at the cost of ~500 lines of atomics/shared-memory infrastructure.

## License

MIT

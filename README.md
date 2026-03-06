# robocam

Minimal camera drivers, async video encoding, and frame buffering for robotics.

No framework lock-in — works with any control loop, any process model.

## Install

```bash
# Core (just protocol + buffer + video writer)
pip install robocam

# With camera drivers
pip install robocam[realsense]    # Intel RealSense — self-contained, no system deps
pip install robocam[zed]          # Stereolabs ZED — requires ZED SDK (see below)
pip install robocam[opencv]       # Generic V4L2 / webcam
pip install robocam[all]          # Everything

# Development (editable)
pip install -e .[all]
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
| `CameraData` | Single capture: `.images` dict, `.timestamp` (ms), optional `.calibration_data`, `.imu_data`, `.depth_data`, `.point_cloud` |
| `CameraSpec` | Named shape/dtype descriptor: `.name`, `.shape`, `.dtype` |
| `IMUData` | Timestamp + 3D acceleration + gyroscope |
| `PointCloudData` | Point cloud: `.points` `(N, 3)` float32 XYZ, `.colors` `(N, 3)` uint8 RGB |
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

Additional methods (require `enable_depth=True`):

| Method | Returns | Description |
|--------|---------|-------------|
| `read_depth()` | `np.ndarray` | `(H, W)` float32 depth map |
| `read_xyzrgba()` | `np.ndarray` | `(H, W, 4)` float32 XYZRGBA measure. Pass to `decode_xyzrgba()` for usable points + colours. |

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

### Point Cloud Utilities

| Function | Module | Description |
|----------|--------|-------------|
| `decode_xyzrgba(xyzrgba, *, stride=4, rotate_to_z_up=True)` | `robocam.drivers.zed` | Decode ZED XYZRGBA measure into `PointCloudData`. Filters invalid points, unpacks packed RGBA, optionally rotates to Z-up. |
| `depth_to_pointcloud(depth, K, *, stride=1)` | `robocam.utils` | Back-project any `(H, W)` depth image into `(N, 3)` XYZ points using intrinsics matrix K. Works with any camera. |

### Scripts

| Script | Requires | Description |
|--------|----------|-------------|
| `scripts/view_cameras.py` | `robocam[all]`, `opencv-contrib-python`, `tyro` | Live OpenCV viewer for all connected cameras |
| `scripts/view_realsense.py` | `robocam[realsense]`, `opencv-contrib-python`, `tyro` | Live OpenCV viewer for RealSense cameras |
| `scripts/view_zed.py` | `robocam[zed]`, `opencv-contrib-python`, `tyro` | Live OpenCV viewer for ZED cameras |
| `scripts/view_pointcloud.py` | `robocam[zed]`, `viser`, `tyro` | Live 3-D point cloud viewer for ZED cameras (web-based) |

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

For **in-process** use (camera polling thread -> control loop reader), `FrameBuffer` with a `threading.Lock` is simpler and sufficient. Shared-memory ring buffers (like jc211/realsense) are needed when the reader and writer are in **different OS processes** — at the cost of ~500 lines of atomics/shared-memory infrastructure.

## License

MIT

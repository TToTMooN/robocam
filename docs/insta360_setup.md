# Insta360 setup

Everything needed to get `robocam.drivers.insta360.Insta360Camera` streaming from an Insta360 X5 over USB.

This driver talks to the camera through Insta360's proprietary CameraSDK via a small native shim, so setup is more involved than the other robocam drivers: there is an SDK to obtain, a shim to compile, and several camera-side settings that fail in confusing ways when wrong.

The [Troubleshooting](#troubleshooting) and [Dead ends](#dead-ends) sections are the parts worth reading before you start, not after.

## Supported configurations

Verified against an Insta360 X5 on 2026-08-21 with `scripts/diagnostics/test_insta360_resolutions.py`.

| `resolution` | `lens="full"` | `lens="front"` | `lens="back"` | `image_transfer_time_offset_ms` |
|---|:---:|:---:|:---:|---|
| `1920x960` (default) | Y | Y | Y | **86** |
| `2560x1280` | Y | Y | Y | |
| `3840x1920` | Y | Y | Y | |
| `1440x720` | N | N | N | |
| `2304x1152` | N | N | N | |
| `2880x2880` | N | N | N | |

Reading this table:

- **N across a whole row means the resolution never streams**, not that the lens is unavailable.
  The driver raises `ValueError` on those three.
  Both failure modes are silent on the camera's side and worth knowing before you hit one: see [Stream resolutions and lenses](#6-stream-resolutions-and-lenses).
  `2880x2880` in particular resets the camera and needs physical access to recover.
- **A blank offset means never measured, not zero.**
  Only `1920x960` has a measured value, taken at `lens="front"`.
  The other two working resolutions inherit it and are wrong by the difference in the camera's encode buffer: see [Latency](#8-latency-and-the-image_transfer_time_offset_ms-constant).
- **`resolution` is always the full dual-fisheye frame size.**
  `lens` selects half of it at read time, so `front` at `3840x1920` returns 1920x1920.

## 1. Get the SDK

The CameraSDK is proprietary and not redistributable, so it is not vendored in this repo.
Apply for access at [insta360.com/sdk/home](https://www.insta360.com/sdk/home) and download **CameraSDK 2.1.1 for Linux**.

It must be a build posted after **2025-04-23**; earlier builds do not work with current firmware.

The download bundles several platform tarballs.
For an x86_64 desktop you want `CameraSDK-2.1.1-Linux.tar.gz`.
The `libMediaSDK-dev-*.tar.xz` in the same bundle is Insta360's offline stitching SDK and is unrelated to this driver.

Lay the extracted tree out like this, anywhere on disk:

```
<sdk-root>/
  include/camera/    # camera.h, device_discovery.h, photography_settings.h, ins_types.h
  include/stream/    # stream_delegate.h, stream_types.h
  lib/libCameraSDK.so
```

```bash
tar -xzf CameraSDK-2.1.1-Linux.tar.gz -C /tmp
SDK=$(echo /tmp/CameraSDK-*-Linux)
mkdir -p ~/insta360_sdk/include ~/insta360_sdk/lib
cp -r "$SDK/include/camera" "$SDK/include/stream" ~/insta360_sdk/include/
cp    "$SDK/lib/libCameraSDK.so"                   ~/insta360_sdk/lib/
```

You can also drop that tree into `vendor/insta360_sdk/` at this repo's root, which is gitignored and is the default location the build script checks.

> **Why it is not under `robocam/`.**
> flit packages the entire `robocam/` module directory and never consults `.gitignore`, so anything placed there would be baked into a wheel.
> For a proprietary SDK that is exactly the wrong outcome, so both the SDK drop-in and the build output live at the repo root instead.

## 2. Camera settings

Three settings on the camera itself.
All three cause failures that look like something else.

| Setting | Value | Why |
|---|---|---|
| Lens mode | **Dual-lens** | Not single-lens or panorama. |
| USB Mode | **Android** | Settings > General > USB Mode > Android. Webcam mode and U-Disk mode both fail. |
| Auto Power Off | **Never** | Settings > General > Auto Power Off. |

**USB Mode is the one people get wrong.**
U-Disk is the default and is mass storage, used for offloading recordings; it is mutually exclusive with Android mode, which is the only mode the live SDK can use.
In the wrong mode the camera enumerates as `070a:4027`; in Android mode it enumerates as `2e1a:0002` (Arashi Vision).
Check with `lsusb`.

**Auto Power Off must be Never.**
The camera's sleep timer is suppressed over Wi-Fi and Bluetooth but **not over USB**, so a preview-only SDK session hits the timer and the camera powers off mid-run.
There is no way to prevent this from code: the CameraSDK has no keep-alive or heartbeat API and no programmatic power-off setter.
Its only power call is `ShutdownCamera()`, which turns the camera off.

## 3. udev rules

Two rules, both one-time and host-wide.

**Permissions.** USB access to the camera otherwise requires root.

```bash
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", SYMLINK+="insta", MODE="0777"' \
  | sudo tee /etc/udev/rules.d/99-insta.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

If `/dev/insta` does not appear, first confirm the camera is in Android mode: the rule matches the Arashi Vision vendor ID, which is only what it enumerates as in that mode.
A manufacturer-string match works as a fallback:

```bash
echo 'SUBSYSTEM=="usb", ATTR{manufacturer}=="Arashi Vision", SYMLINK+="insta", MODE="0777"' \
  | sudo tee /etc/udev/rules.d/99-insta.rules
```

**Autosuspend.** On hosts with USB autosuspend armed (`usbcore.autosuspend`), the camera can be suspended out from under a live session, and `power/control` reverts to `auto` on every replug, reboot and resume.
Pin it:

```bash
echo 'ACTION=="add|change|bind", SUBSYSTEM=="usb", ATTR{idVendor}=="2e1a", TEST=="power/control", ATTR{power/control}="on"' \
  | sudo tee /etc/udev/rules.d/99-insta-nosuspend.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 4. Build the native shim

```bash
INSTA360_SDK_ROOT=~/insta360_sdk bash native/build.sh
```

`build.sh` links openh264, swscale and avutil out of `$CONDA_PREFIX` (override with `$CODEC_PREFIX`), so **run it inside the environment you will actually use the driver in**.

**Build it once per environment, and never copy the `.so` between environments.**
openh264 sonames differ across environments, so a binary built elsewhere fails to load with an `OSError` about a missing `libopenh264.so.N`.
The output lands in `_native/` at the repo root, which makes this driver source or editable-install only.
That is the honest constraint: no prebuilt binary can be valid for an arbitrary environment, so a wheel carrying one would only fail later and more confusingly.

## 5. First run

```bash
uv run scripts/view_insta360.py
```

You should see roughly **29.5 fps** and a stamp lag around **131 ms** in the overlay.

Then confirm colour is right: point the camera at something unambiguously **red** and check it renders red.
The native shim decodes directly to RGB24 so the driver can hand robocam its `images={"rgb": ...}` without a `cvtColor` on the critical path.
If that ever regresses, everything still looks perfectly sharp and correctly exposed, just with red and blue swapped, which is easy to miss.

Quit with `q`.
**Always let the process reach `stop()`.**
A killed process leaves the camera's USB interface in `LIBUSB_ERROR_BUSY`, where it drops out of discovery until physically replugged.

Only one SDK session may hold a camera at a time, so stop any ROS2 Insta360 driver first.

## 6. Stream resolutions and lenses

[Supported configurations](#supported-configurations) at the top of this page is the quick reference.
This section is why it looks like that.

`Insta360Camera(resolution=...)` accepts `1920x960`, `2560x1280` or `3840x1920`, and raises `ValueError` on anything else.
The SDK's `VideoResolution` enum offers more, and the shim used to map six of them, but only these three stream on an X5.

**The other three failed in ways the camera does not report.**
`1440x720` and `2304x1152` are rejected as a main stream, and instead of failing the camera silently keeps whatever it streamed last.
That previous resolution persists across processes and power cycles, so the size you get depends on which session ran before: with the camera last at 1920x960 a `1440x720` request delivers 1920x960, and last at 2560x1280 the same request delivers 2560x1280.

`2880x2880` is worse.
`StartLiveStreaming` rejects it, but only after `SetVideoCaptureParams` has already committed it to the camera.
That write drops the X5 out of Android USB mode and rewrites its normal-video recording resolution.
Recovery needs the on-camera settings menu; a replug does not do it, and the next SDK session fails with `no camera discovered` until you fix it by hand.

Re-check these on new firmware with:

```bash
uv run scripts/diagnostics/test_insta360_resolutions.py --include-unsupported
```

That deliberately excludes `2880x2880`.
Name it explicitly if you want to re-confirm it, and expect to walk to the camera afterwards.

**Resolutions are always the full dual-fisheye frame size.**
`lens` selects half of it at read time, so `lens="front"` at `3840x1920` returns 1920x1920.
`full` returns the whole frame, `front` the right half and `back` the left half.

The halves were checked against the frame they came from, not just for the right shape: a `front` crop matches the full frame's right half to within inter-frame noise (0.41 mean absolute difference) while differing from the left half by 60.24, and `back` mirrors that.
All three lenses work at all three streaming resolutions.

## 7. Multiple cameras

```bash
uv run scripts/view_insta360.py --serials SERIAL_A,SERIAL_B
```

Three rules, each of which is a real failure otherwise:

- **One process, distinct service ports.** The SDK binds a fixed local service port per process, so a second process aborts with `bind: Address already in use`. Give each camera its own `service_port` (the viewer does this automatically from `--base-service-port`).
- **Plug every camera in before the first open.** Device discovery is one-shot per process, and re-running it while another camera is streaming kills that stream with a libusb busy or I/O error.
- **Each camera on its own USB root port**, not behind a shared hub or dock. Behind one shared hub, decode errors rise roughly fivefold. A USB2 path is fine: the compressed stream is only ~8 Mbps per camera.

Occasional decode errors are normal.
A corrupted access unit self-heals at the next IDR, and a dropped frame simply means the latest-frame slot is re-read.
A handful per camera per minute is healthy; hundreds means cabling.

## 8. Latency and the `image_transfer_time_offset_ms` constant

`CameraData.timestamp` is capture-side, not arrival-side.
The shim stamps each frame from the SDK's per-access-unit device timestamp, maps it to host `CLOCK_REALTIME` using the minimum observed delay over the first ~90 access units, and the driver then subtracts `image_transfer_time_offset_ms`.

Unlike the other robocam drivers' transfer offsets, this one is measured rather than guessed, using the UMI Appendix A.1 QR-clock method:

| Stream resolution | `image_transfer_time_offset_ms` |
|---|---|
| 1920x960 (default, via live-view mode) | **86** |
| 2656x1328 (X5 plain-flow default) | **130** |

Those are the only two ever measured.
`2560x1280` and `3840x1920` stream correctly but have no measured offset, so they currently inherit the 1920x960 default and their timestamps are wrong by the difference in the camera's encode buffer.
Given the 44 ms spread between the two rows above, expect that error to be tens of milliseconds, not single digits.

The 44 ms difference is the camera's own encode buffer, which scales with frame size.
That is why 1920x960 is the default here despite being lower resolution.

**Re-measure after any change to resolution. Changing `lens` does not affect it.**
The stamp is assigned in `OnVideoData` from the SDK's per-AU device timestamp, before decode and before the mutex is taken; the lens crop happens later in `ins_read`, on the reader thread, and never touches it.
Measured across 120 frames per lens at both 1920x960 and 3840x1920, the median stamp lag spread between `full`, `front` and `back` was 0.9 ms and 1.4 ms against a per-lens standard deviation of 5 ms and 13 ms, with the ordering reversing between the two resolutions.
That is noise, not an effect.
The measurement procedure lives in the `portable_data_collection` repo, in `docs/camera_latency_check.md`, alongside `scripts/calibrate_camera_latency_ros.py`.

Long sessions drift: device-versus-host crystal drift runs on the order of 10-50 ppm, so tens of ms per hour.
A fresh anchor is taken at every open.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `lsusb` shows `070a:4027`, no `/dev/insta` | Camera is in the wrong USB mode. Set USB Mode to Android; it should then enumerate as `2e1a:0002`. |
| `FileNotFoundError` naming `_native/libinsta_source.so` | Shim not built in this environment. Run `native/build.sh`. |
| `OSError: libopenh264.so.N: cannot open shared object file` | The shim was built against a different environment. Rebuild it in this one. |
| Frames are not the size requested | Only `1920x960`, `2560x1280` and `3840x1920` stream on an X5. The driver raises on anything else; if you are on an older build that did not, check `get_camera_info()["resolution"]`, which reports the delivered size. |
| `no camera discovered` right after a failed open at a high resolution | A `2880x2880` attempt reset the camera's USB Mode. Set it back to Android on the camera; a replug alone does not fix it. |
| `insta_source open failed` | Another SDK session holds the camera (a ROS2 driver, or a previous run that did not exit cleanly), or the camera is not in Android mode. |
| `LIBUSB_ERROR_BUSY`, camera missing from discovery | A previous process was killed without reaching `stop()`. Physically replug the camera. |
| Stream stops partway through a session | Auto Power Off, which USB does not suppress. Set it to Never. Also check the camera screen: if it powered off with a temperature warning, see thermal below. |
| Stream dies after ~25 minutes at high resolution | Thermal shutdown, which is real on the X5 and worse at higher resolutions. Improve airflow or lower the stream resolution. |
| Random drops, `dmesg` shows `-71`/`EPROTO` or disconnects | USB transport or power. Insta360 SDK issue #92 attributes this to insufficient supply; try a different port or cable, and avoid unpowered hubs. |
| Second camera aborts with `bind: Address already in use` | Two processes cannot share the camera fleet. Use one process with distinct `service_port` values. |
| Hundreds of decode errors per minute | Both cameras behind one shared USB hub. Move each to its own root port. |

Note that the driver has no auto-reconnect: when SDK callbacks stop for any reason, `read()` raises `TimeoutError` and the stream does not recover on its own.
Any drop is therefore permanent until the process is restarted.

## Dead ends

Verified not to work, recorded so nobody spends a day on them again.

- **`using_lrv` / the low-res proxy stream** is a genuine no-op on X5 firmware 1.1.22. It still delivers 2656x1328.
- **`2880x2880` cannot be streamed, and trying resets the camera.** `StartLiveStreaming` rejects it only after `SetVideoCaptureParams` has committed it, which drops the X5 out of Android USB mode and rewrites its normal-video recording resolution. This is the same failure shape as `SetActiveSensor` below. Removed from the shim's resolution map; see [Stream resolutions and lenses](#6-stream-resolutions-and-lenses).
- **`1440x720` and `2304x1152` are rejected as a main stream without saying so.** The camera keeps its previous live-stream resolution instead, which persists across processes and power cycles, so the delivered size depends on which session ran last. Both removed from the map; the driver now raises on them.
- **`SetActiveSensor` is actively harmful on the X5.** It instantly drops the USB session, the SDK times out after ~13 s and then returns a bogus success, and the camera resets out of Android USB mode. Recovery needs re-selecting Android mode and a replug. Single-lens encode is not reachable camera-side; crop downstream instead, which is what the `lens` parameter does.
- **The requested bitrate is ignored.** The X5 delivers ~8.4 Mbps regardless of what `bitrate` is set to.
- **openh264's 1-frame output hold cannot be removed** from the stream side. Per-access-unit `FlushFrame` reaches zero lag but corrupts reference management, and SPS surgery does not move it. It is structural for High profile. Only patching openh264 itself would help.
- **`SetVideoSubMode` may persist after the session ends.** If later on-camera recordings look wrong, check the camera's mode or power-cycle it.

## Known limitations

- **No IMU.** The shim's `OnGyroData` handler is a no-op stub, so the camera's ~500 Hz live gyro is not surfaced and `CameraData.imu_data` is always `None`. Note also that the live stream's IMU runs at half the 1000 Hz written to the SD card during on-camera recording, so recorded `.insv` files are the better source for anything that needs full-rate IMU.
- **`read_calibration_data_intrinsics()` raises `NotImplementedError`.** Insta360 intrinsics are per-unit and resolution-dependent, so they belong in a downstream camera registry rather than in the SDK.
- **One reader per camera.** The native layer holds a single latest-frame slot and one read sequence number, so two threads reading one camera steal frames from each other. Use one `CaptureThread` per camera and fan out through a `FrameBuffer`.

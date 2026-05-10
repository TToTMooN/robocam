# Vendored Lumos in-container senders

These two scripts are **vendored from `/root/fastumi_driver/Fastumi/`**
(the git-tracked upstream on this host) — they run inside the `fastumi`
docker container (ROS1 Noetic), subscribe to `xv_sdk` topics, and ship
the data to the host via TCP.

`robocam.drivers.lumos_stack` `docker cp`s them into the container at
runtime, so they don't need to be importable Python modules — just
present on disk.

| File | Role | Connects to |
|------|------|-------------|
| `fastumi_tcp_sender.py` | pose+clamp ROS1 → newline-JSON TCP | `LumosCamera` pose port (28999) |
| `fastumi_image_tcp_sender.py` | fisheye+RGB ROS1 → length-prefixed JPEG TCP | `LumosCamera` image port (28998) |

## Updating

Vendored copies will drift from upstream — that's intentional (the
robocam wire protocol is whatever the bundled scripts say it is). When
you want to pull a newer version:

```bash
sudo cp /root/fastumi_driver/Fastumi/fastumi_tcp_sender.py \
        robocam/vendor/lumos/
sudo cp /root/fastumi_driver/Fastumi/fastumi_image_tcp_sender.py \
        robocam/vendor/lumos/
```

If the upstream protocol changes (header format, port semantics), update
`robocam/drivers/lumos.py` to match.

## Why these are vendored but the docker image isn't

The two scripts are tiny pure-Python and the wire protocol they speak
is robocam's API. Bundling them keeps `LumosCamera` self-contained at
the Python level — you never need to set `FASTUMI_DIR` or clone another
repo for the bridge to work.

The docker image (`fastumi-docker/`) and the XVisio SDK
(`FastUMI_Hardware_SDK/`, ~107 MB of multi-version `.deb` installers)
stay upstream — they're the system dependency, analogous to the ZED
SDK installed at `/usr/local/zed/`. See `README_ben.md` in
`/root/fastumi_driver/` for setup.

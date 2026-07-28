#!/usr/bin/env bash
# Build libinsta_source.so, the native core of robocam.drivers.insta360.
#
# Two external pieces are resolved at build time, neither of them vendored:
#
#   1. The Insta360 CameraSDK (proprietary; apply at insta360.com/sdk/home).
#      Looked up in $INSTA360_SDK_ROOT, then vendor/insta360_sdk/ at the repo
#      root. Either location must contain include/camera/, include/stream/ and
#      lib/libCameraSDK.so.
#
#      Both that drop-in and the output below live OUTSIDE robocam/ on purpose:
#      flit packages the whole module directory and never reads .gitignore, so
#      anything under robocam/ would be baked into a wheel - a proprietary SDK
#      or an env-locked binary is exactly what must not ship.
#
#   2. openh264 + swscale + avutil, taken from $CONDA_PREFIX (i.e. whatever
#      conda/pixi env you run this in), overridable with $CODEC_PREFIX.
#
# The codec prefix is why this must be built once PER ENVIRONMENT and never
# copied between them: openh264 sonames differ across envs (.so.7 vs .so.8),
# so a binary built elsewhere fails to dlopen at import time.
#
# Output lands in _native/ at the repo root (gitignored), which is where the
# driver looks for it. That makes this driver source/editable-install only,
# which is correct: no prebuilt binary can be valid for an arbitrary env.
#
# Usage:
#   INSTA360_SDK_ROOT=/path/to/insta360_sdk bash native/build.sh
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
OUT="$ROOT/_native"

SDK="${INSTA360_SDK_ROOT:-$ROOT/vendor/insta360_sdk}"
CODEC="${CODEC_PREFIX:-${CONDA_PREFIX:-}}"

if [ -z "$CODEC" ]; then
    echo "error: neither \$CODEC_PREFIX nor \$CONDA_PREFIX is set." >&2
    echo "       Run this inside your conda/pixi env, or set CODEC_PREFIX to a" >&2
    echo "       prefix providing openh264, swscale and avutil." >&2
    exit 1
fi

if [ ! -f "$SDK/lib/libCameraSDK.so" ] || [ ! -d "$SDK/include/camera" ]; then
    echo "error: no Insta360 CameraSDK at $SDK" >&2
    echo "       Expected \$SDK/lib/libCameraSDK.so and \$SDK/include/{camera,stream}/." >&2
    echo "       Set INSTA360_SDK_ROOT, or drop the SDK in vendor/insta360_sdk/." >&2
    echo "       See 'Insta360 SDK setup' in the robocam README." >&2
    exit 1
fi

if [ ! -f "$CODEC/lib/libopenh264.so" ]; then
    echo "error: no openh264 in $CODEC (looked for lib/libopenh264.so)" >&2
    echo "       Install it into this env, e.g. 'pixi add openh264 ffmpeg'." >&2
    exit 1
fi

mkdir -p "$OUT"
g++ -O2 -fPIC -shared -std=c++17 -Wall -Wextra \
    -I"$SDK/include" -I"$CODEC/include" \
    "$HERE/insta_source.cpp" \
    -L"$SDK/lib" -lCameraSDK \
    -L"$CODEC/lib" -lopenh264 -lswscale -lavutil \
    -Wl,-rpath,"$SDK/lib" -Wl,-rpath,"$CODEC/lib" \
    -o "$OUT/libinsta_source.so"
echo "built $OUT/libinsta_source.so"
echo "  SDK:   $SDK"
echo "  codec: $CODEC"

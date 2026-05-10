#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Image TCP sender for FastUMI Lumos cameras (runs inside the fastumi docker
container, ROS1 Noetic).

Subscribes (per LEFT/RIGHT tracker UUID):
  /xv_sdk/<UUID>/fisheye_cameras/left/image           + .../camera_info
  /xv_sdk/<UUID>/fisheye_cameras/right/image          + .../camera_info
  /xv_sdk/<UUID>/color_camera/image                   + .../camera_info
                                              (color only after color_camera/start)

JPEG-encodes each image frame and ships it over a TCP socket to
FASTUMI_TCP_RECEIVER_IP:FASTUMI_IMAGE_TCP_PORT. CameraInfo packets are
cached and replayed on every (re)connect so the receiver always sees
intrinsics regardless of subscribe order.

Wire format (length-prefixed binary, big-endian):
  [4 bytes : total_len]
  [2 bytes : header_len]
  [header_len bytes : header_json (utf-8)]
  [total_len - 2 - header_len bytes : payload (JPEG, or empty)]

header_json fields:
  kind == "image" (default if absent):
    {"kind": "image",
     "topic": "left/fisheye/l" | "left/fisheye/r" | "left/color"
              | "right/fisheye/l" | "right/fisheye/r" | "right/color",
     "stamp_ns": int, "w": int, "h": int}
    payload = JPEG bytes

  kind == "camera_info":
    {"kind": "camera_info", "topic": same as above,
     "K": [9 floats, row-major 3x3], "D": [floats], "R": [9 floats],
     "P": [12 floats], "distortion_model": str,
     "width": int, "height": int}
    payload = empty (0 bytes)

Env vars:
  FASTUMI_TCP_RECEIVER_IP        host IP (required)
  FASTUMI_IMAGE_TCP_PORT         default 28998
  FASTUMI_SERIAL_LEFT/RIGHT      tracker serials (override defaults)
  FASTUMI_IMG_JPEG_QUALITY       default 60
  FASTUMI_IMG_MAX_FPS            default 30 (per topic)
  FASTUMI_IMG_DOWNSCALE          default 1.0 (no downscale)
"""
from __future__ import print_function

import json
import os
import socket
import struct
import sys
import threading
import time
from collections import deque

import numpy as np

try:
    import rospy
    from sensor_msgs.msg import CameraInfo, Image
    _ROS_OK = True
except ImportError:
    _ROS_OK = False

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

DEFAULT_PORT = 28998

SERIAL_LEFT = os.environ.get("FASTUMI_SERIAL_LEFT", "250801DR48FP25002612")
SERIAL_RIGHT = os.environ.get("FASTUMI_SERIAL_RIGHT", "250801DR48FP25002624")
JPEG_QUALITY = int(os.environ.get("FASTUMI_IMG_JPEG_QUALITY", "60"))
MAX_FPS = float(os.environ.get("FASTUMI_IMG_MAX_FPS", "30"))
DOWNSCALE = float(os.environ.get("FASTUMI_IMG_DOWNSCALE", "1.0"))


def _build_topic_map():
    m = {}
    for side, uuid in (("left", SERIAL_LEFT), ("right", SERIAL_RIGHT)):
        if not uuid:
            continue
        m["/xv_sdk/{}/fisheye_cameras/left/image".format(uuid)] = "{}/fisheye/l".format(side)
        m["/xv_sdk/{}/fisheye_cameras/right/image".format(uuid)] = "{}/fisheye/r".format(side)
        m["/xv_sdk/{}/color_camera/image".format(uuid)] = "{}/color".format(side)
    return m


def _ros_image_to_cv(msg):
    enc = msg.encoding
    h, w = msg.height, msg.width
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    if enc == "mono8":
        return raw.reshape(h, w)
    if enc in ("bgr8", "rgb8"):
        img = raw.reshape(h, w, 3)
        if enc == "rgb8":
            img = img[..., ::-1]
        return img
    return raw.reshape(h, w, -1)


def _pack(header_bytes, payload_bytes):
    total_len = 2 + len(header_bytes) + len(payload_bytes)
    return struct.pack(">IH", total_len, len(header_bytes)) + header_bytes + payload_bytes


class ImageSender(object):
    def __init__(self, host, port):
        self._host, self._port = host, port
        self._q = deque(maxlen=20)
        self._cv = threading.Condition()
        self._stop = False
        # Latest camera_info header per out_topic; replayed on every reconnect
        # so the receiver always gets intrinsics even if it subscribed late.
        self._info_lock = threading.Lock()
        self._camera_info = {}  # out_topic -> header_bytes
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def push(self, header_bytes, payload_bytes):
        with self._cv:
            self._q.append((header_bytes, payload_bytes))
            self._cv.notify()

    def update_camera_info(self, out_topic, header_bytes):
        """Cache the latest camera_info header for replay on reconnect, and
        also push it through the live queue so currently-connected receivers
        see updates without waiting for a reconnect."""
        with self._info_lock:
            self._camera_info[out_topic] = header_bytes
        self.push(header_bytes, b"")

    def stop(self):
        with self._cv:
            self._stop = True
            self._cv.notify_all()

    def _replay_camera_info(self, sock):
        with self._info_lock:
            items = list(self._camera_info.values())
        for header_bytes in items:
            try:
                sock.sendall(_pack(header_bytes, b""))
            except (BrokenPipeError, ConnectionResetError, OSError):
                return False
        return True

    def _run(self):
        while not self._stop:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self._host, self._port))
                sock.settimeout(None)
                rospy.loginfo("[image-bridge] connected to %s:%d", self._host, self._port)
                if not self._replay_camera_info(sock):
                    raise OSError("camera_info replay failed")
                while not self._stop:
                    with self._cv:
                        while not self._q and not self._stop:
                            self._cv.wait(timeout=1.0)
                        if self._stop:
                            break
                        header_bytes, payload_bytes = self._q.popleft()
                    try:
                        sock.sendall(_pack(header_bytes, payload_bytes))
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break
                try:
                    sock.close()
                except Exception:
                    pass
            except (socket.error, socket.timeout) as e:
                rospy.logwarn_throttle(5, "[image-bridge] connect failed: %s", e)
            except Exception as e:
                rospy.logwarn_throttle(5, "[image-bridge] error: %s", e)
            if not self._stop:
                time.sleep(2.0)


def main():
    if not _ROS_OK:
        print("rospy not available; run inside ROS1 env (source /opt/ros/noetic/setup.bash and devel)", file=sys.stderr)
        return 1
    if not _CV2_OK:
        print("opencv-python not available; pip install opencv-python", file=sys.stderr)
        return 1
    host = os.environ.get("FASTUMI_TCP_RECEIVER_IP", "").strip()
    if not host:
        print("set FASTUMI_TCP_RECEIVER_IP=<host ip> (use 127.0.0.1 if container --network host)", file=sys.stderr)
        return 1
    port = int(os.environ.get("FASTUMI_IMAGE_TCP_PORT", str(DEFAULT_PORT)))

    rospy.init_node("fastumi_image_tcp_sender", anonymous=False)
    sender = ImageSender(host, port)
    topic_map = _build_topic_map()

    last_send = {v: 0.0 for v in topic_map.values()}
    min_dt = 1.0 / max(1.0, MAX_FPS)

    def make_cb(out_topic):
        def cb(msg):
            now = time.time()
            if now - last_send[out_topic] < min_dt:
                return
            try:
                cv = _ros_image_to_cv(msg)
                if DOWNSCALE != 1.0 and DOWNSCALE > 0:
                    new_w = int(cv.shape[1] * DOWNSCALE)
                    new_h = int(cv.shape[0] * DOWNSCALE)
                    cv = cv2.resize(cv, (new_w, new_h))
                ok, jpeg = cv2.imencode(".jpg", cv, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
                if not ok:
                    return
                stamp_ns = int(msg.header.stamp.to_nsec()) if msg.header.stamp else int(now * 1e9)
                header = json.dumps({
                    "kind": "image",
                    "topic": out_topic,
                    "stamp_ns": stamp_ns,
                    "w": int(cv.shape[1]),
                    "h": int(cv.shape[0]),
                }).encode("utf-8")
                sender.push(header, jpeg.tobytes())
                last_send[out_topic] = now
            except Exception as e:
                rospy.logwarn_throttle(5, "[image-bridge] cb error: %s", e)
        return cb

    def make_info_cb(out_topic):
        # CameraInfo is effectively static for a given session, but we update
        # the cache every time so a sender restart picks up any rebinding.
        # If DOWNSCALE is active, K (fx, fy, cx, cy) and image size are scaled
        # to match what the receiver actually sees in image packets.
        def cb(msg):
            try:
                K = list(msg.K)
                w, h = int(msg.width), int(msg.height)
                if DOWNSCALE != 1.0 and DOWNSCALE > 0:
                    s = DOWNSCALE
                    K = [
                        K[0] * s, K[1],     K[2] * s,
                        K[3],     K[4] * s, K[5] * s,
                        K[6],     K[7],     K[8],
                    ]
                    w = int(w * s)
                    h = int(h * s)
                header = json.dumps({
                    "kind": "camera_info",
                    "topic": out_topic,
                    "K": K,
                    "D": list(msg.D),
                    "R": list(msg.R),
                    "P": list(msg.P),
                    "distortion_model": msg.distortion_model or "",
                    "width": w,
                    "height": h,
                }).encode("utf-8")
                sender.update_camera_info(out_topic, header)
            except Exception as e:
                rospy.logwarn_throttle(5, "[image-bridge] info cb error: %s", e)
        return cb

    for ros_topic, out_topic in topic_map.items():
        rospy.Subscriber(ros_topic, Image, make_cb(out_topic), queue_size=1, buff_size=2 ** 24)
        rospy.loginfo("[image-bridge] subscribed: %s -> %s", ros_topic, out_topic)
        # /.../image  ->  /.../camera_info  (ROS convention used by xv_sdk)
        info_topic = ros_topic.rsplit("/", 1)[0] + "/camera_info"
        rospy.Subscriber(info_topic, CameraInfo, make_info_cb(out_topic), queue_size=1)
        rospy.loginfo("[image-bridge] subscribed: %s -> %s (camera_info)", info_topic, out_topic)

    rospy.loginfo("[image-bridge] streaming to %s:%d (jpeg q=%d, max_fps=%.0f, scale=%.2f)",
                  host, port, JPEG_QUALITY, MAX_FPS, DOWNSCALE)

    try:
        rospy.spin()
    finally:
        sender.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

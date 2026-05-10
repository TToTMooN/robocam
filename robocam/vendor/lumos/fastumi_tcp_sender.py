#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A 机（FastUMI 所在，仅 ROS1）：
  订阅 xv_sdk 的位姿、夹爪与 IMU，通过 TCP 发送到 B 机（R1Pro 所在）。
  B 机只需运行 ROS2，运行 fastumi_tcp_receiver_ros2.py 接收并发布到 ROS2 话题。

用法（在 A 机上，ROS1 环境）：
  export FASTUMI_TCP_RECEIVER_IP=192.168.1.xxx   # B 机 IP，必填
  export FASTUMI_TCP_PORT=28999                  # 可选，默认 28999
  source /opt/ros/noetic/setup.bash
  source ~/catkin_ws/devel/setup.bash
  python3 fastumi_tcp_sender.py

数据格式：每行一个 JSON，包含 left_pose, right_pose, left_clamp, right_clamp,
left_imu, right_imu。
IMU 字段：{"stamp_ns": int, "acceleration": [x,y,z], "gyroscope": [x,y,z]}
(linear_acceleration -> acceleration, angular_velocity -> gyroscope; null
until first sample arrives).

IMU topic suffix is FASTUMI_IMU_TOPIC_SUFFIX (default "imu_sensor/data_raw"),
giving /xv_sdk/<UUID>/<suffix>. If xv_sdk on your build uses a different
message type than sensor_msgs/Imu, change the import below.
"""
from __future__ import print_function

import copy
import json
import os
import socket
import sys
import threading
import time

try:
    import rospy
    from xv_sdk.msg import PoseStampedConfidence, Clamp
    from sensor_msgs.msg import Imu
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False

# Tracker serials can be overridden at runtime via env vars so we don't
# need to edit code when swapping FastUMI hardware:
#   export FASTUMI_SERIAL_LEFT=250801DR48FP25002612
#   export FASTUMI_SERIAL_RIGHT=250801DR48FP25002624
# Fallback defaults below match the pair currently on this host (read
# from /sys/bus/usb/devices/*/serial). Original project trackers were
# 250801DR48FP25002692 (LEFT) and 250801DR48FP25002313 (RIGHT).
SERIAL_LEFT = os.environ.get("FASTUMI_SERIAL_LEFT", "250801DR48FP25002612")
SERIAL_RIGHT = os.environ.get("FASTUMI_SERIAL_RIGHT", "250801DR48FP25002624")
IMU_TOPIC_SUFFIX = os.environ.get("FASTUMI_IMU_TOPIC_SUFFIX", "imu_sensor/data_raw")
DEFAULT_PORT = 28999


def _pose_to_dict(msg):
    # PoseStampedConfidence: geometry_msgs/PoseStamped 在 poseMsg 字段
    p = msg.poseMsg.pose.position
    q = msg.poseMsg.pose.orientation
    return {
        "position": [p.x, p.y, p.z],
        "orientation": [q.x, q.y, q.z, q.w],
        # 0..1 — drops to 0 when SLAM init failed or relocalization in progress.
        # Position values are unreliable when low.
        "confidence": float(getattr(msg, "confidence", 0.0)),
    }


def _clamp_value(msg):
    return float(getattr(msg, "clamp", getattr(msg, "data", 0.0)))


def _imu_to_dict(msg):
    a = msg.linear_acceleration
    w = msg.angular_velocity
    stamp_ns = int(msg.header.stamp.to_nsec()) if msg.header.stamp else 0
    return {
        "stamp_ns": stamp_ns,
        "acceleration": [a.x, a.y, a.z],
        "gyroscope": [w.x, w.y, w.z],
    }


def run_sender(state, lock, host, port):
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((host, port))
            sock.settimeout(None)
            if rospy.is_shutdown():
                break
            rospy.loginfo("已连接到 B 机 %s:%s，开始发送 FastUMI 数据", host, port)
            while not rospy.is_shutdown():
                with lock:
                    data = json.dumps(copy.deepcopy(state)) + "\n"
                try:
                    sock.sendall(data.encode("utf-8"))
                except (BrokenPipeError, ConnectionResetError, OSError):
                    break
                time.sleep(0.01)
            sock.close()
        except (socket.error, socket.timeout) as e:
            rospy.logwarn_throttle(5, "连接 B 机失败，5 秒后重试: %s", e)
        except Exception as e:
            rospy.logwarn_throttle(5, "发送异常: %s", e)
        if rospy.is_shutdown():
            break
        time.sleep(2.0)


def main():
    if not _ROS_AVAILABLE:
        print("请在 ROS1 环境中运行（rospy, xv_sdk）", file=sys.stderr)
        return 1
    host = os.environ.get("FASTUMI_TCP_RECEIVER_IP", "").strip()
    if not host:
        print("请设置 B 机 IP: export FASTUMI_TCP_RECEIVER_IP=192.168.1.xxx", file=sys.stderr)
        return 1
    port = int(os.environ.get("FASTUMI_TCP_PORT", str(DEFAULT_PORT)))

    rospy.init_node("fastumi_tcp_sender", anonymous=False)
    state = {
        "left_pose": None,
        "right_pose": None,
        "left_clamp": None,
        "right_clamp": None,
        "left_imu": None,
        "right_imu": None,
    }
    lock = threading.Lock()

    def cb_left_pose(msg):
        with lock:
            state["left_pose"] = _pose_to_dict(msg)

    def cb_right_pose(msg):
        with lock:
            state["right_pose"] = _pose_to_dict(msg)

    def cb_left_clamp(msg):
        with lock:
            state["left_clamp"] = _clamp_value(msg)

    def cb_right_clamp(msg):
        with lock:
            state["right_clamp"] = _clamp_value(msg)

    def cb_left_imu(msg):
        with lock:
            state["left_imu"] = _imu_to_dict(msg)

    def cb_right_imu(msg):
        with lock:
            state["right_imu"] = _imu_to_dict(msg)

    left_pose_topic = "/xv_sdk/{}/slam/pose".format(SERIAL_LEFT)
    right_pose_topic = "/xv_sdk/{}/slam/pose".format(SERIAL_RIGHT)
    left_clamp_topic = "/xv_sdk/{}/clamp/Data".format(SERIAL_LEFT)
    right_clamp_topic = "/xv_sdk/{}/clamp/Data".format(SERIAL_RIGHT)
    left_imu_topic = "/xv_sdk/{}/{}".format(SERIAL_LEFT, IMU_TOPIC_SUFFIX)
    right_imu_topic = "/xv_sdk/{}/{}".format(SERIAL_RIGHT, IMU_TOPIC_SUFFIX)
    rospy.Subscriber(left_pose_topic, PoseStampedConfidence, cb_left_pose, queue_size=1)
    rospy.Subscriber(right_pose_topic, PoseStampedConfidence, cb_right_pose, queue_size=1)
    rospy.Subscriber(left_clamp_topic, Clamp, cb_left_clamp, queue_size=1)
    rospy.Subscriber(right_clamp_topic, Clamp, cb_right_clamp, queue_size=1)
    rospy.Subscriber(left_imu_topic, Imu, cb_left_imu, queue_size=1)
    rospy.Subscriber(right_imu_topic, Imu, cb_right_imu, queue_size=1)

    t = threading.Thread(target=run_sender, args=(state, lock, host, port), daemon=True)
    t.start()
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())

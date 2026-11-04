r"""Grab one message from each sensor, save pictures of them and sanity-check the values.

    ros2_tools\run.bat ros2_tools\sensor_snapshot.py [--out DIR] [--timeout SEC]

Writes DIR\rgb.png (wrist camera), DIR\depth.png (depth, near = red, far = blue, no data = black) and
DIR\scan.png (lidar seen from above in the robot frame, robot in the centre, forward = up, 1 m rings),
then prints one PASS/FAIL line per sensor. Default DIR is .\snapshots. Works against the simulator and the real robot
(16UC1 depth in mm is converted too), so the two can be compared side by side. No RViz needed.
"""

import argparse
import math
import os
import struct
import sys
import time
import zlib

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState, LaserScan
from std_msgs.msg import UInt16
from tf2_msgs.msg import TFMessage

TOPICS = {
    "rgb": ("/depth_cam/rgb/image_raw", Image),
    "depth": ("/depth_cam/depth0/image_raw", Image),
    "scan": ("/scan", LaserScan),
    "imu": ("/imu", Imu),
    "joints": ("/joint_states", JointState),
    "battery": ("/ros_robot_controller/battery", UInt16),
}


def write_png(path, img):
    """Minimal PNG writer (uint8 HxW, HxWx3 or HxWx4), so no imaging library is needed."""
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape[:2]
    ch = 1 if img.ndim == 2 else img.shape[2]
    rows = np.hstack([np.zeros((h, 1), np.uint8), img.reshape(h, w * ch)])

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n")
        f.write(chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, {1: 0, 3: 2, 4: 6}[ch], 0, 0, 0)))
        f.write(chunk(b"IDAT", zlib.compress(rows.tobytes(), 6)))
        f.write(chunk(b"IEND", b""))


def image_array(m):
    """sensor_msgs/Image -> numpy array (respects the row step)."""
    dtype, ch = {"rgb8": (np.uint8, 3), "bgr8": (np.uint8, 3), "rgba8": (np.uint8, 4), "bgra8": (np.uint8, 4),
                 "mono8": (np.uint8, 1), "32FC1": (np.float32, 1), "16UC1": (np.uint16, 1)}[m.encoding]
    item = np.dtype(dtype).itemsize
    a = np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(m.height, m.step)[:, :m.width * ch * item]
    a = a.copy().view(np.dtype(dtype).newbyteorder(">" if m.is_bigendian else "<"))
    a = a.reshape(m.height, m.width, ch) if ch > 1 else a.reshape(m.height, m.width)
    if m.encoding.startswith("bgr"):
        a = a[:, :, [2, 1, 0]]
    return a[:, :, :3] if ch == 4 else a


def colorize(t):
    """0..1 -> jet-like RGB (0 = red/near, 1 = blue/far)."""
    t = 1.0 - np.clip(t, 0.0, 1.0)
    rgb = np.stack([np.clip(1.5 - np.abs(4 * t - k), 0, 1) for k in (3, 2, 1)], axis=-1)
    return (rgb * 255).astype(np.uint8)


def scan_picture(m, mount=(0.0, 0.0, 0.0), size=500, metres=5.0):
    """mount = (x, y, yaw) of the scan frame in the robot frame (from TF), so forward is always up."""
    img = np.full((size, size, 3), 255, np.uint8)
    c, res = size // 2, metres / (size // 2)
    yy, xx = np.mgrid[0:size, 0:size]
    rr = np.hypot(yy - c, xx - c) * res
    img[np.abs(rr - np.round(rr)) < res * 0.6] = (210, 210, 210)        # 1 m rings
    img[c - 1:c + 2, :] = img[:, c - 1:c + 2] = (230, 230, 230)
    r = np.asarray(m.ranges, dtype=np.float64)
    a = m.angle_min + np.arange(len(r)) * m.angle_increment
    ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
    a = a[ok] + mount[2]
    x, y = mount[0] + r[ok] * np.cos(a), mount[1] + r[ok] * np.sin(a)  # robot frame: x forward, y left
    row, col = np.round(c - x / res).astype(int), np.round(c - y / res).astype(int)
    inside = (row >= 1) & (row < size - 1) & (col >= 1) & (col < size - 1)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            img[row[inside] + dr, col[inside] + dc] = (220, 30, 30)
    img[c - 6:c + 7, c - 4:c + 5] = (40, 40, 40)                         # robot
    img[c - 12:c - 6, c - 1:c + 2] = (40, 40, 40)                        # forward arrow
    return img, int(ok.sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="snapshots")
    ap.add_argument("--timeout", type=float, default=30.0)
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("rosorin_sensor_snapshot")
    got, tf = {}, {}
    for key, (topic, mtype) in TOPICS.items():
        node.create_subscription(mtype, topic, lambda m, k=key: got.__setitem__(k, m), qos_profile_sensor_data)

    def on_tf(msg):
        for t in msg.transforms:
            tf[t.child_frame_id] = t

    node.create_subscription(TFMessage, "/tf", on_tf, qos_profile_sensor_data)
    node.create_subscription(TFMessage, "/tf_static", on_tf,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    print("waiting for %s ..." % ", ".join(t for t, _ in TOPICS.values()))
    end = time.time() + args.timeout
    while time.time() < end and (len(got) < len(TOPICS) or
                                 ("scan" in got and got["scan"].header.frame_id not in tf)):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()

    os.makedirs(args.out, exist_ok=True)
    results = []

    def check(name, ok, detail):
        results.append(ok)
        print("%-4s %-8s %s" % ("PASS" if ok else "FAIL", name, detail))

    if "rgb" in got:
        rgb = image_array(got["rgb"])
        write_png(os.path.join(args.out, "rgb.png"), rgb if rgb.ndim == 3 else np.dstack([rgb] * 3))
        check("camera", float(rgb.std()) > 5.0, "%dx%d %s, mean %.0f, contrast (std) %.1f -> rgb.png"
              % (rgb.shape[1], rgb.shape[0], got["rgb"].encoding, rgb.mean(), rgb.std()))
    else:
        check("camera", False, "no message on %s" % TOPICS["rgb"][0])

    if "depth" in got:
        d = image_array(got["depth"]).astype(np.float64)
        if got["depth"].encoding == "16UC1":
            d = d / 1000.0
        valid = np.isfinite(d) & (d > 0)
        lo, hi = np.percentile(d[valid], [1, 99]) if valid.any() else (0.15, 3.0)
        pic = colorize((d - lo) / max(hi - lo, 0.05))
        pic[~valid] = 0
        write_png(os.path.join(args.out, "depth.png"), pic)
        frac = valid.mean()
        rng = "%.2f-%.2f m" % (d[valid].min(), d[valid].max()) if valid.any() else "-"
        check("depth", frac > 0.10, "%s, %.0f %% pixels valid, range %s -> depth.png"
              % (got["depth"].encoding, 100 * frac, rng))
    else:
        check("depth", False, "no message on %s" % TOPICS["depth"][0])

    if "scan" in got:
        frame = got["scan"].header.frame_id
        mount, where = (0.0, 0.0, 0.0), "(drawn in %s, no TF found for it)" % frame
        if frame in tf:
            t = tf[frame].transform
            q = t.rotation
            mount = (t.translation.x, t.translation.y,
                     math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
            where = "(drawn in %s; %s yaw %.0f deg)" % (tf[frame].header.frame_id, frame, math.degrees(mount[2]))
        pic, n_ok = scan_picture(got["scan"], mount)
        write_png(os.path.join(args.out, "scan.png"), pic)
        rs = [x for x in got["scan"].ranges if math.isfinite(x) and got["scan"].range_min <= x <= got["scan"].range_max]
        check("lidar", n_ok >= 100, "%d beams, %d valid, nearest %.2f m -> scan.png %s"
              % (len(got["scan"].ranges), n_ok, min(rs) if rs else float("nan"), where))
    else:
        check("lidar", False, "no message on /scan")

    if "imu" in got:
        a = got["imu"].linear_acceleration
        g = math.sqrt(a.x ** 2 + a.y ** 2 + a.z ** 2)
        check("imu", 8.8 < g < 10.8, "|acceleration| = %.2f m/s^2 (gravity 9.81 when standing still)" % g)
    else:
        check("imu", False, "no message on /imu")

    if "joints" in got:
        js = dict(zip(got["joints"].name, got["joints"].position))
        arm = ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"]
        check("joints", all(j in js for j in arm), "%d joints, arm = [%s], gripper r_joint = %s" % (
            len(js), ", ".join("%.2f" % js[j] for j in arm[:5] if j in js),
            "%.2f" % js["r_joint"] if "r_joint" in js else "-"))
    else:
        check("joints", False, "no message on /joint_states")

    if "battery" in got:
        v = got["battery"].data / 1000.0
        check("battery", 9.0 < v < 12.8, "%.2f V" % v)
    else:
        check("battery", False, "no message on %s (published by the runtime, not by the USD graphs)"
              % TOPICS["battery"][0])

    print("\n%d / %d sensors OK, pictures in %s" % (sum(results), len(results), os.path.abspath(args.out)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

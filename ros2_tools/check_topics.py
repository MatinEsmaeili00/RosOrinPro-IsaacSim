r"""Check that the ROSOrin Pro topics are alive (simulation or real robot): rate, frame and a short summary.

    ros2_tools\run.bat ros2_tools\check_topics.py [seconds]
"""

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from std_msgs.msg import UInt16
from tf2_msgs.msg import TFMessage


def s_scan(m):
    r = [x for x in m.ranges if math.isfinite(x) and m.range_min <= x <= m.range_max]
    return "%d beams, %d valid, min %.2f m" % (len(m.ranges), len(r), min(r) if r else float("nan"))


def s_odom(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
    return "x=%.2f y=%.2f yaw=%.0f deg  v=(%.2f, %.2f, %.2f)" % (p.x, p.y, yaw, m.twist.twist.linear.x,
                                                              m.twist.twist.linear.y, m.twist.twist.angular.z)


TOPICS = [
    ("/clock", Clock, lambda m: "t=%.1f s" % (m.clock.sec + m.clock.nanosec * 1e-9)),
    ("/scan", LaserScan, s_scan),
    ("/odom", Odometry, s_odom),
    ("/imu", Imu, lambda m: "acc=(%.2f, %.2f, %.2f)" % (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)),
    ("/joint_states", JointState, lambda m: "%d joints" % len(m.name)),
    ("/tf", TFMessage, lambda m: "%d transforms" % len(m.transforms)),
    ("/depth_cam/rgb/image_raw", Image, lambda m: "%dx%d %s" % (m.width, m.height, m.encoding)),
    ("/depth_cam/rgb/camera_info", CameraInfo, lambda m: "fx=%.1f" % m.k[0]),
    ("/depth_cam/depth0/image_raw", Image, lambda m: "%dx%d %s" % (m.width, m.height, m.encoding)),
    ("/depth_cam/depth0/camera_info", CameraInfo, lambda m: "fx=%.1f" % m.k[0]),
    ("/depth_cam/depth0/points", PointCloud2, lambda m: "%d points" % (m.width * m.height)),
    ("/depth_cam/iro/image_raw", Image, lambda m: "%dx%d %s" % (m.width, m.height, m.encoding)),
    ("/ros_robot_controller/battery", UInt16, lambda m: "%.2f V" % (m.data / 1000.0)),
    ("/scan_raw", LaserScan, s_scan),
    ("/odom_raw", Odometry, s_odom),
    ("/imu_corrected", Imu, lambda m: ""),
    ("/ros_robot_controller/imu_raw", Imu, lambda m: ""),
    ("/controller_manager/joint_states", JointState, lambda m: ""),
]


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 5.0
    rclpy.init()
    node = rclpy.create_node("rosorin_check_topics")
    stats = {t: {"n": 0, "last": None} for t, _, _ in TOPICS}
    for t, mt, _ in TOPICS:
        def cb(m, t=t):
            stats[t]["n"] += 1
            stats[t]["last"] = m
        node.create_subscription(mt, t, cb, qos_profile_sensor_data)
    print("listening %.0f s ..." % secs)
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    ok = 0
    for t, _, summ in TOPICS:
        st = stats[t]
        m = st["last"]
        frame = getattr(getattr(m, "header", None), "frame_id", "") if m is not None else ""
        if st["n"]:
            ok += 1
        print("%-34s %6.1f Hz  %-16s %s" % (t, st["n"] / secs, frame, summ(m) if m is not None else "-- NO DATA --"))
    print("\n%d / %d topics alive (rates are wall-clock; the simulation may run slower than real time)" % (ok, len(TOPICS)))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok == len(TOPICS) else 1


if __name__ == "__main__":
    sys.exit(main())

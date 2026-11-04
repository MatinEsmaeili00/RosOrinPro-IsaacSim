r"""Mecanum demo: drive a 0.6 m square WITHOUT turning (forward, strafe left, backward, strafe right), then
spin 360 deg, closing the loop on /odom. Works on the simulation and on the real robot (use a clear floor!).

    ros2_tools\run.bat ros2_tools\demo_square.py [side_m] [speed_mps]
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data


def main():
    side = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
    rclpy.init()
    node = rclpy.create_node("rosorin_demo_square")
    pub = node.create_publisher(Twist, "cmd_vel", 10)
    odom = {}
    node.create_subscription(Odometry, "odom", lambda m: odom.__setitem__("m", m), qos_profile_sensor_data)

    def pose():
        m = odom["m"]
        q = m.pose.pose.orientation
        return (m.pose.pose.position.x, m.pose.pose.position.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def send(vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
        pub.publish(t)

    while "m" not in odom:
        rclpy.spin_once(node, timeout_sec=0.1)
    x0, y0, yaw0 = pose()
    corners = [(side, 0), (side, -side), (0, -side), (0, 0)]
    for cx, cy in corners:
        t_end = time.time() + 60.0 * side / speed       # generous per-leg timeout (sim may run < real time)
        tx = x0 + math.cos(yaw0) * cx - math.sin(yaw0) * cy
        ty = y0 + math.sin(yaw0) * cx + math.cos(yaw0) * cy
        while True:
            rclpy.spin_once(node, timeout_sec=0.02)
            x, y, yaw = pose()
            ex, ey = tx - x, ty - y
            dist = math.hypot(ex, ey)
            if dist < 0.01:
                break
            if time.time() > t_end:
                print("leg timed out (obstacle?) at x=%.3f y=%.3f" % (x - x0, y - y0), flush=True)
                break
            v = min(speed, 1.5 * dist)
            bx = (math.cos(yaw) * ex + math.sin(yaw) * ey) / dist * v     # body frame, no rotation
            by = (-math.sin(yaw) * ex + math.cos(yaw) * ey) / dist * v
            send(bx, by, 2.0 * math.atan2(math.sin(yaw0 - yaw), math.cos(yaw0 - yaw)))
        print("corner (%.2f, %.2f) reached: odom x=%.3f y=%.3f" % (cx, cy, x - x0, y - y0), flush=True)
    turned, last = 0.0, pose()[2]
    while turned < 2 * math.pi - 0.02:
        rclpy.spin_once(node, timeout_sec=0.02)
        yaw = pose()[2]
        turned += abs(math.atan2(math.sin(yaw - last), math.cos(yaw - last)))
        last = yaw
        send(wz=min(1.0, 2.0 * (2 * math.pi - turned) + 0.2))
    send()
    x, y, yaw = pose()
    print("done: final offset %.3f m, %.1f deg from the start" % (math.hypot(x - x0, y - y0), math.degrees(
        math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))), flush=True)
    for _ in range(5):
        send()
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

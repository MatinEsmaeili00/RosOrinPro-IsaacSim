r"""Print the QoS of every publisher/subscriber of the ROSOrin Pro topics (like `ros2 topic info -v`).

    ros2_tools\run.bat ros2_tools\qos_info.py
"""
import time

import rclpy

TOPICS = ["/clock", "/tf", "/odom", "/joint_states", "/imu", "/scan", "/depth_cam/rgb/image_raw",
          "/depth_cam/depth0/image_raw", "/depth_cam/depth0/points", "/ros_robot_controller/battery",
          "/robot_description", "/cmd_vel", "/joint_command"]

rclpy.init()
node = rclpy.create_node("rosorin_qos_info")
end = time.time() + 3.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
for t in TOPICS:
    for kind, infos in (("pub", node.get_publishers_info_by_topic(t)), ("sub", node.get_subscriptions_info_by_topic(t))):
        for i in infos:
            q = i.qos_profile
            print("%-32s %s %-28s %-12s %-10s %-9s depth=%s" % (t, kind, i.node_name, q.reliability.name,
                                                               q.durability.name, q.history.name, q.depth))
node.destroy_node()
rclpy.shutdown()

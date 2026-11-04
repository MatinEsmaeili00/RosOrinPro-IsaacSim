r"""List the ROS 2 topics currently visible on the network (like `ros2 topic list -t`)."""
import sys
import time

import rclpy

rclpy.init()
node = rclpy.create_node("rosorin_topic_list")
wait = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
end = time.time() + wait
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
for name, types in sorted(node.get_topic_names_and_types()):
    print("%-50s %s" % (name, ", ".join(types)))
node.destroy_node()
rclpy.shutdown()

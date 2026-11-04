r"""Command the ROSOrin Pro arm and gripper through /joint_command (sensor_msgs/JointState).

    ros2_tools\run.bat ros2_tools\arm.py pose init          named pose: init | pregrasp | pick | up
    ros2_tools\run.bat ros2_tools\arm.py joints 0 -0.5 1.6 0.9 0     joint1..joint5 [rad]
    ros2_tools\run.bat ros2_tools\arm.py gripper open       open | close | <r_joint rad>
    ros2_tools\run.bat ros2_tools\arm.py state              print the current /joint_states (arm + gripper)

Pick-up recipe (3 cm block centred 0.222 m in front of base_footprint): gripper open -> pose pregrasp ->
pose pick -> gripper close -> pose pregrasp (lift). tests/test_pick.py does exactly this.

Joint limits (HX-12H bus servos, 0-240 deg): joint1..5 +-2.09 rad. Gripper r_joint: +0.9 = open (58 mm),
-0.65 = closed (fingers touch); the other five gripper joints are mimics of r_joint (4-bar linkage).
"""

import sys
import time

import rclpy
from sensor_msgs.msg import JointState

POSES = {
    "init": [0.0, 0.0, 1.5708, 0.7854, 0.0],   # like the real robot's /init_pose: camera looks ahead and down
    "pregrasp": [0.0, 1.4399, 0.2967, 1.3614, 0.0],  # gripper pointing down, 7 cm above the pick point (FK)
    "pick": [0.0, 2.0246, 0.0, 0.9599, 0.0],     # fingertips 6 mm above the floor, 0.222 m in front of base_footprint
    "up": [0.0, 0.0, 0.0, 0.0, 0.0],            # URDF zero pose, arm straight up
}
ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]


def main(argv):
    if len(argv) < 2 or argv[1] in ("-h", "--help"):
        print(__doc__)
        return 0
    rclpy.init()
    node = rclpy.create_node("rosorin_arm_cli")
    if argv[1] == "state":
        got = {}
        node.create_subscription(JointState, "joint_states", lambda m: got.setdefault("m", m), 10)
        end = time.time() + 5.0
        while "m" not in got and time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "m" not in got:
            print("no /joint_states received (is the simulation playing?)")
            return 1
        d = dict(zip(got["m"].name, got["m"].position))
        for n in ARM + ["r_joint"]:
            print("%-8s %+.4f rad" % (n, d.get(n, float("nan"))))
        return 0
    pub = node.create_publisher(JointState, "joint_command", 10)
    js = JointState()
    if argv[1] == "pose":
        js.name, js.position = ARM, POSES[argv[2]]
    elif argv[1] == "joints":
        vals = [float(v) for v in argv[2:7]]
        js.name, js.position = ARM[:len(vals)], vals
    elif argv[1] == "gripper":
        v = {"open": 0.9, "close": -0.65}.get(argv[2])
        js.name, js.position = ["r_joint"], [float(v if v is not None else argv[2])]
    else:
        print(__doc__)
        return 1
    # wait for the simulator's subscriber, then publish a few times (JointState commands are idempotent)
    end = time.time() + 3.0
    while pub.get_subscription_count() == 0 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for _ in range(5):
        js.header.stamp = node.get_clock().now().to_msg()
        pub.publish(js)
        rclpy.spin_once(node, timeout_sec=0.05)
    print("sent %s -> %s (subscribers: %d)" % (js.name, [round(p, 4) for p in js.position], pub.get_subscription_count()))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))

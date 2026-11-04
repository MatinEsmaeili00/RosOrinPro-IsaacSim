r"""Keyboard teleop for the ROSOrin Pro (simulated or real): mecanum base, arm presets and gripper.

    ros2_tools\run.bat ros2_tools\teleop_keyboard.py          (Windows, Isaac Sim's bundled ROS 2)
    python3 ros2_tools/teleop_keyboard.py                     (Linux with ROS 2 sourced)

    w / s     forward / backward          a / d   strafe left / right (mecanum)
    q / e     turn left / right           x or space  stop
    + / -     faster / slower             o / c   gripper open / close
    1         arm init pose (camera looks ahead/down)       2  pre-grasp (gripper down, 7 cm up)
    4         grasp pose (fingertips at the floor, 0.22 m ahead)   3  arm straight up (URDF zero)
    Esc / Ctrl+C  quit

Publishes geometry_msgs/Twist on /cmd_vel (10 Hz while moving) and sensor_msgs/JointState on /joint_command.
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

POSES = {
    "1": {"joint1": 0.0, "joint2": 0.0, "joint3": 1.5708, "joint4": 0.7854, "joint5": 0.0},
    "2": {"joint1": 0.0, "joint2": 1.4399, "joint3": 0.2967, "joint4": 1.3614, "joint5": 0.0},   # pre-grasp
    "4": {"joint1": 0.0, "joint2": 2.0246, "joint3": 0.0, "joint4": 0.9599, "joint5": 0.0},      # grasp (floor)
    "3": {"joint1": 0.0, "joint2": 0.0, "joint3": 0.0, "joint4": 0.0, "joint5": 0.0},
}
GRIPPER = {"o": 0.9, "c": -0.65}  # r_joint [rad]: + opens (58 mm max), fingers touch at -0.65
MOVES = {"w": (1, 0, 0), "s": (-1, 0, 0), "a": (0, 1, 0), "d": (0, -1, 0), "q": (0, 0, 1), "e": (0, 0, -1)}


def getch_factory():
    if sys.platform == "win32":
        import msvcrt

        def getch():
            if msvcrt.kbhit():
                return msvcrt.getwch()
            return None
        return getch, lambda: None
    import select
    import termios
    import tty

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    def getch():
        r, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if r else None
    return getch, lambda: termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rclpy.init()
    node = rclpy.create_node("rosorin_teleop_keyboard")
    pub_cmd = node.create_publisher(Twist, "cmd_vel", 10)
    pub_joint = node.create_publisher(JointState, "joint_command", 10)
    getch, restore = getch_factory()
    lin, ang = 0.2, 1.0
    cmd = (0, 0, 0)
    print(__doc__.split("\n\n", 2)[2])
    last = 0.0
    try:
        while rclpy.ok():
            k = getch()
            if k:
                k = k.lower()
                if k in ("\x1b", "\x03"):
                    break
                if k in MOVES:
                    cmd = MOVES[k]
                elif k in ("x", " "):
                    cmd = (0, 0, 0)
                elif k in ("+", "="):
                    lin, ang = min(lin * 1.25, 0.5), min(ang * 1.25, 2.5)
                elif k == "-":
                    lin, ang = max(lin / 1.25, 0.02), max(ang / 1.25, 0.1)
                elif k in POSES or k in GRIPPER:
                    js = JointState()
                    js.header.stamp = node.get_clock().now().to_msg()
                    target = POSES[k] if k in POSES else {"r_joint": GRIPPER[k]}
                    js.name, js.position = list(target), [float(v) for v in target.values()]
                    pub_joint.publish(js)
                print("\r cmd vx=%+.2f vy=%+.2f wz=%+.2f   (speed %.2f m/s, %.2f rad/s)      " % (
                    cmd[0] * lin, cmd[1] * lin, cmd[2] * ang, lin, ang), end="", flush=True)
            now = time.time()
            if now - last >= 0.1:
                last = now
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = cmd[0] * lin, cmd[1] * lin, cmd[2] * ang
                pub_cmd.publish(t)
            rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        pub_cmd.publish(Twist())
        restore()
        print("\nstopped")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

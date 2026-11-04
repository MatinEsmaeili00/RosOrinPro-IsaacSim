"""Test of the Python runtime: battery topic + digital-twin mode.

A fake "real robot" publishes /joint_states and /odom (driving a circle) on ROS domain 42. The simulation runs in
twin mode following domain 42, while a checker on the simulation's own domain (ROS_DOMAIN_ID, default 0) verifies
that the simulated robot's /joint_states and /odom follow the fake robot and that /ros_robot_controller/battery
is published.
    launch\\run_python.bat tests\\test_twin_battery.py
"""

import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
app.update()
sys.path.insert(0, os.path.join(REPO, "launch"))
if "--no-fabric" not in sys.argv:
    import rosorin_runtime as _rt

    _rt.enable_fabric()  # same configuration as the launchers
import numpy as np
import omni.timeline
import omni.usd
import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16

sys.path.insert(0, os.path.join(REPO, "launch"))
import rosorin_runtime

TWIN_DOMAIN = 42
NO_ODOM = "--no-odom" in sys.argv
RESULTS = {"checks": []}


def check(name, ok, detail=""):
    RESULTS["checks"].append({"check": name, "pass": bool(ok), "detail": detail})
    print("[%s] %s  %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(os.path.dirname(REPO), "RosOrin_Claude.usd"))
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
stage = ctx.get_stage()
stage.GetPrimAtPath("/World/TestArena/Blocks").SetActive(False)
for g in ("DepthCamera", "Lidar"):  # not needed here
    stage.GetPrimAtPath("/World/rosorin_pro/Graphs/" + g).SetActive(False)
rosorin_runtime.deactivate_command_graphs(stage)
rt = rosorin_runtime.RosOrinRuntime(twin_domain=TWIN_DOMAIN)
rt.cfg["battery"]["time_scale"] = 600.0  # drain fast so the test sees the voltage drop
rt.start()

# fake real robot on domain 42
rctx = rclpy.Context()
rclpy.init(context=rctx, domain_id=TWIN_DOMAIN)
robot = rclpy.create_node("fake_rosorin", context=rctx)
pub_js = robot.create_publisher(JointState, "joint_states", 10)
pub_od = robot.create_publisher(Odometry, "odom", 10)

# checker on the simulation's domain
if not rclpy.ok():
    rclpy.init()
chk = rclpy.create_node("twin_checker")
LAST = {}
BAT = []
chk.create_subscription(JointState, "joint_states", lambda m: LAST.__setitem__("js", m), qos_profile_sensor_data)
chk.create_subscription(Odometry, "odom", lambda m: LAST.__setitem__("odom", m), qos_profile_sensor_data)
chk.create_subscription(UInt16, "ros_robot_controller/battery", lambda m: BAT.append(m.data), qos_profile_sensor_data)
ex = SingleThreadedExecutor()
ex.add_node(chk)

R, W = 0.5, 0.25  # fake robot drives a circle: radius 0.5 m, 0.25 rad/s


def fake_state(t):
    q = {"joint1": 0.5 * math.sin(0.8 * t), "joint2": -0.3, "joint3": 1.2, "joint4": 0.6, "joint5": 0.3,
         "r_joint": 0.4}
    yaw = W * t
    x, y = R * math.sin(yaw), R * (1 - math.cos(yaw))
    return q, (x, y, yaw), (R * W, 0.0, W)


tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(5):
    app.update()
t0 = tl.get_current_time()
while tl.get_current_time() - t0 < 8.0:
    t = tl.get_current_time() - t0
    q, (x, y, yaw), (vx, vy, wz) = fake_state(t)
    js = JointState()
    js.name, js.position = list(q), [float(v) for v in q.values()]
    pub_js.publish(js)
    od = Odometry()
    od.header.frame_id, od.child_frame_id = "odom", "base_footprint"
    od.pose.pose.position.x, od.pose.pose.position.y = x, y
    od.pose.pose.orientation.z, od.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    od.twist.twist.linear.x, od.twist.twist.angular.z = vx, wz
    if not NO_ODOM:
        pub_od.publish(od)
    app.update()
    if int(t * 60) % 30 == 0 and LAST.get("js"):
        d = dict(zip(LAST["js"].name, LAST["js"].position))
        print("TRACE t=%.2f joint2=%.3f joint3=%.3f joint4=%.3f" % (t, d["joint2"], d["joint3"], d["joint4"]), flush=True)
    for _ in range(30):
        ex.spin_once(timeout_sec=0.0)

t = tl.get_current_time() - t0
q, (x, y, yaw), _ = fake_state(t)
js = LAST.get("js")
sim_q = dict(zip(js.name, js.position)) if js else {}
err_q = max(abs(sim_q.get(k, 99) - v) for k, v in q.items())
check("twin: simulated arm/gripper follow the real /joint_states", err_q < 0.08, "max error %.3f rad %s" % (
    err_q, {k: (round(v, 3), round(sim_q.get(k, float("nan")), 3)) for k, v in q.items()}))
od = LAST.get("odom")
if od:
    p = od.pose.pose.position
    oq = od.pose.pose.orientation
    syaw = math.atan2(2 * (oq.w * oq.z + oq.x * oq.y), 1 - 2 * (oq.y ** 2 + oq.z ** 2))
    dpos = math.hypot(p.x - x, p.y - y)
    dyaw = abs(math.atan2(math.sin(syaw - yaw), math.cos(syaw - yaw)))
    check("twin: simulated base follows the real /odom (circle)", dpos < 0.05 and dyaw < 0.05,
          "pos err %.3f m, yaw err %.3f rad (real at %.2f, %.2f)" % (dpos, dyaw, x, y))
check("battery: /ros_robot_controller/battery (UInt16 mV) at ~1 Hz", 6 <= len(BAT) <= 9, "%d msgs: %s" % (len(BAT), BAT))
check("battery: plausible 3S voltage that drains", len(BAT) >= 2 and 9900 <= BAT[-1] <= 12600 and BAT[-1] < BAT[0],
      "%s -> %s mV" % (BAT[:1], BAT[-1:]))
tl.stop()
app.update()
rt.stop()
fails = [c for c in RESULTS["checks"] if not c["pass"]]
os.makedirs(os.path.join(REPO, "tests", "results"), exist_ok=True)
with open(os.path.join(REPO, "tests", "results", "report_twin_battery.json"), "w", encoding="utf-8") as f:
    json.dump(RESULTS, f, indent=2)
print("TEST DONE passed=%d failed=%d" % (len(RESULTS["checks"]) - len(fails), len(fails)), flush=True)
app.close()

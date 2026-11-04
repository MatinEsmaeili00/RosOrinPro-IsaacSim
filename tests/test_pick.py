"""Grasp test: the simulated ROSOrin Pro picks up a 3 cm block using only /joint_command (like a ROS user).

    launch\\run_python.bat tests\\test_pick.py
Sequence: open gripper -> pre-grasp -> descend (joint interpolation) -> close -> lift. Checks that the block is
lifted and that the fingers stop on the block (r_joint stays above the fully-closed value).
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
import rosorin_runtime

rosorin_runtime.enable_fabric()
import numpy as np
import omni.timeline
import omni.usd
import rclpy
from pxr import Gf, Usd, UsdGeom
from sensor_msgs.msg import JointState

ARM = ["joint1", "joint2", "joint3", "joint4", "joint5"]
INIT = [0.0, 0.0, 1.5708, 0.7854, 0.0]
PRE = [0.0, 1.4399, 0.2967, 1.3614, 0.0]
PICK = [0.0, 2.0246, 0.0, 0.9599, 0.0]
RESULTS = {"checks": [], "trace": []}


def check(name, ok, detail=""):
    RESULTS["checks"].append({"check": name, "pass": bool(ok), "detail": detail})
    print("[%s] %s  %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


ctx = omni.usd.get_context()
ctx.open_stage(os.path.join(os.path.dirname(REPO), "RosOrin_Claude.usd"))
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
stage = ctx.get_stage()
# put the red block 0.222 m in front of the robot (pick point), move the other blocks away
with Usd.EditContext(stage, stage.GetSessionLayer()):
    for name, pos in (("red_block", (0.222, 0.0, 0.016)), ("green_block", (0.5, 0.8, 0.016)), ("blue_block", (0.6, 0.8, 0.016))):
        xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/TestArena/Blocks/" + name))
        xf.GetOrderedXformOps()[0].Set(Gf.Vec3d(*pos))

if not rclpy.ok():
    rclpy.init()
node = rclpy.create_node("rosorin_pick_test")
pub = node.create_publisher(JointState, "joint_command", 10)


def send(names, values):
    js = JointState()
    js.name, js.position = list(names), [float(v) for v in values]
    pub.publish(js)


tl = omni.timeline.get_timeline_interface()


def run(seconds, names=None, values=None):
    t0 = tl.get_current_time()
    while tl.get_current_time() - t0 < seconds:
        if names:
            send(names, values)
        app.update()


from isaacsim.core.experimental.prims import Articulation, RigidPrim

tl.play()
run(0.3)
robot = Articulation("/World/rosorin_pro/Geometry/base_footprint")
block = RigidPrim("/World/TestArena/Blocks/red_block")  # physics pose (USD is not updated with Fabric)
app.update()


def block_z():
    return float(block.get_world_poses()[0].numpy()[0][2])


def joint(n):
    return float(robot.get_dof_positions().numpy()[0][robot.dof_names.index(n)])


run(1.0)
z0 = block_z()
run(1.5, ["r_joint"], [0.9])                       # open
run(2.5, ARM, PRE)                                 # pre-grasp above the block
for i in range(1, 21):                             # descend: joint-space interpolation
    a = i / 20.0
    run(0.08, ARM, [p + a * (q - p) for p, q in zip(PRE, PICK)])
run(1.5, ARM, PICK)
RESULTS["trace"].append({"phase": "at grasp", "block_z": block_z(), "r_joint": joint("r_joint")})
run(2.0, ["r_joint"], [-0.65])                     # close (commanded fully closed)
r_closed = joint("r_joint")
for i in range(1, 21):                             # lift back to pre-grasp
    a = i / 20.0
    run(0.08, ARM, [q + a * (p - q) for p, q in zip(PRE, PICK)])
run(2.0, ARM, PRE)
z1 = block_z()
RESULTS["trace"].append({"phase": "lifted", "block_z": z1, "r_joint": joint("r_joint")})
check("gripper stops on the 30 mm block (does not close fully)", r_closed > -0.4,
      "r_joint=%.3f rad (fully closed = -0.65)" % r_closed)
check("block lifted off the floor", z1 - z0 > 0.04, "block z %.3f -> %.3f m" % (z0, z1))
run(2.0, ARM, INIT)                                # carry it in the init pose
z2 = block_z()
check("block still held after moving the arm back to the init pose", z2 > z0 + 0.04, "block z %.3f m" % z2)
tl.stop()
app.update()
fails = [c for c in RESULTS["checks"] if not c["pass"]]
with open(os.path.join(REPO, "tests", "results", "report_pick.json"), "w", encoding="utf-8") as f:
    json.dump(RESULTS, f, indent=2)
print("TEST DONE passed=%d failed=%d" % (len(RESULTS["checks"]) - len(fails), len(fails)), flush=True)
app.close()

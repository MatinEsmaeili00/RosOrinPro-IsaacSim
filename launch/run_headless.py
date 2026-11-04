r"""Run the ROSOrin Pro simulation without a window (e.g. as a ROS 2 'robot' for Nav2/SLAM on this PC).

    launch\run_headless.bat [--scene PATH] [--twin-domain N] [--twin-ns NS] [--duration SEC] [--no-cameras]
"""

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=os.path.join(os.path.dirname(REPO), "RosOrin_Claude.usd"))
ap.add_argument("--twin-domain", type=int, default=None)
ap.add_argument("--twin-ns", default="")
ap.add_argument("--duration", type=float, default=0.0, help="seconds of sim time, 0 = forever")
ap.add_argument("--no-cameras", action="store_true", help="disable the depth camera graph (saves GPU)")
ap.add_argument("--no-fabric", action="store_true", help="keep physics -> USD updates (slower)")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
app.update()
import omni.timeline
import omni.usd
from pxr import Usd

sys.path.insert(0, HERE)
import rosorin_runtime

if not args.no_fabric:
    rosorin_runtime.enable_fabric()

ctx = omni.usd.get_context()
ctx.open_stage(args.scene)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
stage = ctx.get_stage()
if args.no_cameras:
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        stage.GetPrimAtPath("/World/rosorin_pro/Graphs/DepthCamera").SetActive(False)
if args.twin_domain is not None:
    rosorin_runtime.deactivate_command_graphs(stage)
rt = rosorin_runtime.RosOrinRuntime(twin_domain=args.twin_domain, twin_namespace=args.twin_ns)
rt.start()
tl = omni.timeline.get_timeline_interface()
tl.play()
print("[rosorin] headless simulation running (Ctrl+C to stop)", flush=True)
t_wall = time.time()
try:
    while app.is_running():
        app.update()
        if args.duration and tl.get_current_time() >= args.duration:
            break
except KeyboardInterrupt:
    pass
print("[rosorin] stopped after %.1f s sim / %.1f s wall" % (tl.get_current_time(), time.time() - t_wall), flush=True)
tl.stop()
rt.stop()
app.update()
app.close()

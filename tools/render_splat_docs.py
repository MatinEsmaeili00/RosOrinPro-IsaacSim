r"""Render the documentation images of the Gaussian-splat building (docs/images/splat/).

    launch\run_python.bat tools\render_splat_docs.py [--scene scenes\splat_world.usd] [--out docs\images\splat]

Views (all relative to the spawn pose in nav2/rosorin_nav2/config/worlds.yaml, so they follow the robot):
  hall_view.png    eye height, 1.5 m behind the spawn, looking along the hall
  robot_in_hall.png  third-person view of the robot
The robot's own camera images come from ros2_tools/sensor_snapshot.py against the running simulation.
"""

import argparse
import math
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=os.path.join(REPO, "scenes", "splat_world.usd"))
ap.add_argument("--out", default=os.path.join(REPO, "docs", "images", "splat"))
ap.add_argument("--worlds", default=os.path.join(REPO, "nav2", "rosorin_nav2", "config", "worlds.yaml"))
args, _ = ap.parse_known_args()
os.makedirs(args.out, exist_ok=True)

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 1280, "height": 720})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
app.update()
import numpy as np
import omni.replicator.core as rep
import omni.usd
import yaml
from PIL import Image
from pxr import Gf, UsdGeom

with open(args.worlds, encoding="utf-8") as f:
    sx, sy, syaw = (float(v) for v in yaml.safe_load(f)["splat"]["spawn"])
syaw = math.radians(syaw)

ctx = omni.usd.get_context()
ctx.open_stage(args.scene)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
stage = ctx.get_stage()
stage.SetEditTarget(stage.GetSessionLayer())  # nothing below is saved into the scene


def robot_frame(x, y, z):
    """Point given in the robot's spawn frame (x forward, y left) -> world."""
    c, s = math.cos(syaw), math.sin(syaw)
    return [sx + c * x - s * y, sy + s * x + c * y, z]


def look_at_camera(path, eye, target, focal):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 200.0))
    e, t = np.array(eye, float), np.array(target, float)
    f = (t - e) / np.linalg.norm(t - e)
    x = np.cross(f, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(*x, 0, *y, 0, *(-f), 0, *e, 1))
    return path


views = {
    "hall_view": look_at_camera("/RenderCams/hall", robot_frame(-1.5, 0.0, 1.3), robot_frame(4.0, 0.0, 0.7), 14.0),
    "robot_in_hall": look_at_camera("/RenderCams/robot", robot_frame(0.9, -0.75, 0.55), robot_frame(0.0, 0.0, 0.12), 16.0),
}
annots = {}
for name, cam in views.items():
    rp = rep.create.render_product(cam, (1280, 720))
    a = rep.AnnotatorRegistry.get_annotator("rgb")
    a.attach([rp])
    annots[name] = a
for _ in range(12):
    rep.orchestrator.step(rt_subframes=8)
for name, a in annots.items():
    img = np.asarray(a.get_data())[:, :, :3]
    Image.fromarray(img).save(os.path.join(args.out, name + ".png"))
    print("SAVED", name, img.shape, "mean %.1f" % float(img.mean()), flush=True)
app.close()

r"""Render documentation images: robot overview shots + what the robot's sensors see.

    launch\run_python.bat tools\render_docs.py [--out docs\images]
"""

import argparse
import math
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=os.path.join(os.path.dirname(REPO), "RosOrin_Claude.usd"))
ap.add_argument("--out", default=os.path.join(REPO, "docs", "images"))
ap.add_argument("--settle", type=float, default=2.0)
args, _ = ap.parse_known_args()
os.makedirs(args.out, exist_ok=True)

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 1280, "height": 720})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
app.update()
import numpy as np
import omni.replicator.core as rep
import omni.timeline
import omni.usd
from PIL import Image
from pxr import Gf, Sdf, UsdGeom

ctx = omni.usd.get_context()
ctx.open_stage(args.scene)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
stage = ctx.get_stage()
stage.SetEditTarget(stage.GetSessionLayer())  # nothing below is saved into the scene


def look_at_camera(path, eye, target, focal=18.0):
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(focal)
    cam.CreateHorizontalApertureAttr(20.955)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100.0))
    e, t = np.array(eye, float), np.array(target, float)
    f = (t - e) / np.linalg.norm(t - e)
    x = np.cross(f, [0, 0, 1.0])
    x /= np.linalg.norm(x)
    y = np.cross(x, f)
    m = Gf.Matrix4d(*x, 0, *y, 0, *(-f), 0, *e, 1)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(m)
    cp = cam.GetPrim()  # same exposure as the Kit viewport camera
    cp.ApplyAPI("OmniRtxCameraExposureAPI_1")
    cp.ApplyAPI("OmniRtxCameraAutoExposureAPI_1")
    for n, v in (("exposure:fStop", 5.0), ("exposure:time", 0.02), ("exposure:responsivity", 1.1026709)):
        a = cp.GetAttribute(n) or cp.CreateAttribute(n, Sdf.ValueTypeNames.Float)
        a.Set(v)
    a = cp.GetAttribute("omni:rtx:autoExposure:enabled") or cp.CreateAttribute("omni:rtx:autoExposure:enabled", Sdf.ValueTypeNames.Bool)
    a.Set(True)
    return path


views = {
    "overview_3q": look_at_camera("/RenderCams/q3", (0.75, -0.62, 0.48), (0.04, 0.0, 0.17), 20.0),
    "overview_side": look_at_camera("/RenderCams/side", (0.05, -0.95, 0.22), (0.05, 0.0, 0.2), 20.0),
    "overview_front": look_at_camera("/RenderCams/front", (0.85, 0.15, 0.30), (0.05, 0.0, 0.17), 22.0),
    "arena_top": look_at_camera("/RenderCams/top", (0.0, -0.01, 6.2), (0.0, 0.0, 0.0), 18.0),
}
R = "/World/rosorin_pro/Geometry/base_footprint/link1/link2/link3/link4/camera_link0/camera_link/depth_cam_frame"
robot_cams = {"robot_camera_rgb": R + "/rgb_camera"}

tl = omni.timeline.get_timeline_interface()
tl.play()
t0 = tl.get_current_time()
while tl.get_current_time() - t0 < args.settle:
    app.update()
tl.pause()

annots = {}
for name, cam in list(views.items()) + list(robot_cams.items()):
    res = (640, 400) if name.startswith("robot_") else (1280, 720)
    rp = rep.create.render_product(cam, res)
    a = rep.AnnotatorRegistry.get_annotator("rgb")
    a.attach([rp])
    annots[name] = a
for _ in range(12):  # let auto exposure adapt
    rep.orchestrator.step(rt_subframes=8)
for name, a in annots.items():
    d = a.get_data()
    img = np.asarray(d)[:, :, :3]
    Image.fromarray(img).save(os.path.join(args.out, name + ".png"))
    print("SAVED", name, img.shape, "mean", float(img.mean()), flush=True)
tl.stop()
app.update()
app.close()

r"""Performance benchmark: wall-clock time per simulated frame (1/60 s) for several configurations.

    launch\run_python.bat tests\bench_perf.py
Real-time factor = (1/60) / wall time per frame.
"""

import json
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import carb
import omni.kit.app

FABRIC = "--fabric" in sys.argv
em = omni.kit.app.get_app().get_extension_manager()
em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
if FABRIC:  # same as apps/isaacsim.exp.full.fabric.kit: physics results go to Fabric instead of USD
    em.set_extension_enabled_immediate("omni.physx.fabric", True)
    carb.settings.get_settings().set("/persistent/omnigraph/updateToUsd", False)
    carb.settings.get_settings().set("/persistent/physics/updateForceSensorsToUsd", False)
app.update()
import omni.timeline
import omni.usd
from pxr import PhysxSchema, Usd

SCENE = os.path.join(os.path.dirname(REPO), "RosOrin_Claude.usd")
CONFIGS = [
    ("full: 480 Hz + cameras + lidar (scene renderer)", dict(hz=480, cameras=True, lidar=True, mode=None)),
    ("480 Hz, no cameras", dict(hz=480, cameras=False, lidar=True, mode=None)),
    ("480 Hz, no cameras, no lidar", dict(hz=480, cameras=False, lidar=False, mode=None)),
    ("240 Hz + cameras + lidar", dict(hz=240, cameras=True, lidar=True, mode=None)),
    ("480 Hz + cameras + lidar, RTX real-time renderer", dict(hz=480, cameras=True, lidar=True, mode="RaytracedLighting")),
    ("480 Hz, all robot graphs off (physics only)", dict(hz=480, cameras=False, lidar=False, mode=None, graphs=False)),
    ("robot removed (scene only)", dict(hz=480, cameras=False, lidar=False, mode=None, graphs=False, robot=False)),
]
if "--only" in sys.argv:
    CONFIGS = [c for c in CONFIGS if c[1].get("graphs", True) is False or "full" in c[0]]
settings = carb.settings.get_settings()
default_mode = settings.get("/rtx/rendermode")
results = {}
tl = omni.timeline.get_timeline_interface()
for name, c in CONFIGS:
    ctx = omni.usd.get_context()
    ctx.open_stage(SCENE)
    while ctx.get_stage_loading_status()[2] > 0:
        app.update()
    stage = ctx.get_stage()
    settings.set("/rtx/rendermode", c["mode"] or default_mode)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath("/World/PhysicsScene")).GetTimeStepsPerSecondAttr().Set(c["hz"])
        stage.GetPrimAtPath("/World/rosorin_pro/Graphs/DepthCamera").SetActive(c["cameras"])
        stage.GetPrimAtPath("/World/rosorin_pro/Graphs/Lidar").SetActive(c["lidar"])
        if not c.get("graphs", True):
            for g in ("MecanumDrive", "ArmGripper", "RobotState"):
                stage.GetPrimAtPath("/World/rosorin_pro/Graphs/" + g).SetActive(False)
        if not c.get("robot", True):
            stage.GetPrimAtPath("/World/rosorin_pro").SetActive(False)
    tl.play()
    for _ in range(60):     # warm-up (shader compile, render products)
        app.update()
    t0 = time.time()
    n = 240
    for _ in range(n):
        app.update()
    dt = (time.time() - t0) / n
    tl.stop()
    app.update()
    results[name] = {"ms_per_frame": dt * 1000, "real_time_factor": (1 / 60) / dt, **c}
    print("BENCH%s %-50s %6.1f ms/frame  RTF %.2f" % (" [fabric]" if FABRIC else "", name, dt * 1000, (1 / 60) / dt), flush=True)
settings.set("/rtx/rendermode", default_mode)
print("renderer from scene/app:", default_mode)
with open(os.path.join(REPO, "tests", "results", "bench_perf%s.json" % ("_fabric" if FABRIC else "")), "w", encoding="utf-8") as f:
    json.dump(results, f, indent=2)
app.close()

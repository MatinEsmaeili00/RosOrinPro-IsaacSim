r"""Startup script for the Isaac Sim GUI (passed with --exec by launch\run_isaacsim.bat).

Opens the ROSOrin Pro scene and starts the Python runtime (battery, optional digital-twin mode).
Environment variables (set by run_isaacsim.bat):
    ROSORIN_SCENE        scene to open (default: ..\RosOrin_Claude.usd)
    ROSORIN_TWIN_DOMAIN  if set: follow the real robot on this ROS_DOMAIN_ID
    ROSORIN_TWIN_NS      optional namespace of the real robot's topics
    ROSORIN_AUTOPLAY     1 = press Play automatically once the scene is loaded
"""

import asyncio
import os
import sys

import carb
import omni.kit.app
import omni.timeline
import omni.usd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rosorin_runtime  # noqa: E402

SCENE = os.environ.get("ROSORIN_SCENE") or os.path.join(os.path.dirname(os.path.dirname(HERE)), "RosOrin_Claude.usd")
TWIN = os.environ.get("ROSORIN_TWIN_DOMAIN") or None
NS = os.environ.get("ROSORIN_TWIN_NS", "")


async def _main():
    app = omni.kit.app.get_app()
    for _ in range(10):  # let the UI come up
        await app.next_update_async()
    # The robot's graphs use ROS 2 node types: the bridge must be loaded BEFORE the scene is opened, otherwise
    # OmniGraph cannot instantiate them ("Could not find node type interface for isaacsim.ros2.bridge...").
    app.get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    for _ in range(10):
        await app.next_update_async()
    ctx = omni.usd.get_context()
    carb.log_info("[rosorin] opening %s" % SCENE)
    print("[rosorin] opening", SCENE, flush=True)
    await ctx.open_stage_async(SCENE)
    while ctx.get_stage_loading_status()[2] > 0:
        await app.next_update_async()
    stage = ctx.get_stage()
    try:  # frame the robot in the viewport (the scene's saved camera may look elsewhere)
        import math

        from isaacsim.core.rendering_manager import ViewportManager
        from pxr import Usd, UsdGeom

        # same view as before, expressed in the robot frame, so it also works where the robot does not
        # spawn at the origin (e.g. scenes/splat_world.usd)
        px, py, pz, yaw = 0.0, 0.0, 0.0, 0.0
        robot = stage.GetPrimAtPath("/World/rosorin_pro")
        if robot:
            m = UsdGeom.Xformable(robot).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            t, xa = m.ExtractTranslation(), m.TransformDir((1.0, 0.0, 0.0))
            px, py, pz, yaw = t[0], t[1], t[2], math.atan2(xa[1], xa[0])

        def to_world(x, y, z):
            c, s = math.cos(yaw), math.sin(yaw)
            return [px + c * x - s * y, py + s * x + c * y, pz + z]

        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=to_world(1.15, -0.95, 0.75),
                                        target=to_world(0.05, 0.0, 0.15))
    except Exception as e:
        carb.log_warn("[rosorin] could not set the viewport camera: %s" % e)
    if TWIN is not None:
        rosorin_runtime.deactivate_command_graphs(stage)
    rt = rosorin_runtime.RosOrinRuntime(twin_domain=TWIN, twin_namespace=NS)
    rt.start()
    # keep a reference for the lifetime of the app
    globals()["ROSORIN_RUNTIME"] = rt
    if os.environ.get("ROSORIN_AUTOPLAY", "0") == "1":
        omni.timeline.get_timeline_interface().play()
    print("[rosorin] ready - press Play (space) to start the robot", flush=True)


asyncio.ensure_future(_main())

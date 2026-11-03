"""Step 4 - put the ROSOrin Pro into a scene (by default the user's RosOrin_Claude.usd).

    C:\\isaac-sim-standalone-6.2.0\\python.bat scripts\\build_scene.py                       # RosOrin_Claude.usd
    C:\\isaac-sim-standalone-6.2.0\\python.bat scripts\\build_scene.py --scene scenes\\rosorin_world.usd --new

Adds (all idempotent, re-running replaces them):
  /World/PhysicsScene      240 Hz PhysX scene (TGS, CPU) - required for the robot to move
  /World/GroundPlane       20 x 20 m floor with a physics material
  /World/rosorin_pro       reference to usd/rosorin_pro.usd (robot + sensors + ROS 2 graphs)
  /World/TestArena         reference to scenes/test_arena.usd (walls, line track, coloured blocks); --no-arena skips
  /World/Graphs/ROS_Clock  /clock publisher (use_sim_time)
  /Environment/FillDome    soft dome light so the robot camera images are not dark
The original scene is backed up once to scenes/RosOrin_Claude.original.usda.
"""

import argparse
import math
import os
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ISAAC = os.path.dirname(REPO)
sys.path.insert(0, os.path.join(REPO, "scripts"))

ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=os.path.join(ISAAC, "RosOrin_Claude.usd"))
ap.add_argument("--robot", default=os.path.join(REPO, "usd", "rosorin_pro.usd"))
ap.add_argument("--arena", default=os.path.join(REPO, "scenes", "test_arena.usd"))
ap.add_argument("--no-arena", action="store_true")
ap.add_argument("--new", action="store_true", help="create the scene file from scratch")
ap.add_argument("--config", default=os.path.join(REPO, "config", "rosorin_pro_sim.yaml"))
ap.add_argument("--spawn", type=float, nargs=3, default=[0.0, 0.0, 0.005], metavar=("X", "Y", "Z"))
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.app

em = omni.kit.app.get_app().get_extension_manager()
for ext in ("isaacsim.ros2.bridge", "isaacsim.core.nodes"):
    em.set_extension_enabled_immediate(ext, True)
app.update()

import omni.usd
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

import ros_graphs

scene = os.path.abspath(args.scene)
os.makedirs(os.path.dirname(scene), exist_ok=True)


def rel(path):
    r = os.path.relpath(os.path.abspath(path), os.path.dirname(scene)).replace("\\", "/")
    return r if r.startswith(".") else "./" + r


def preview_material(stage, path, rgb, roughness=0.6, metallic=0.0):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, path.AppendChild("PreviewSurface"))
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def physics_material(stage, path, static, dynamic):
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(static)
    api.CreateDynamicFrictionAttr(dynamic)
    api.CreateRestitutionAttr(0.0)
    return mat


def box(stage, path, size, pos, rgb, collide=True, mass=None, mat=None, semantic=None):
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*pos))
    cube = UsdGeom.Cube.Define(stage, path.AppendChild("geom"))
    cube.CreateSizeAttr(1.0)
    cube.AddScaleOp().Set(Gf.Vec3f(*size))
    cube.CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
        preview_material(stage, path.AppendChild("look"), rgb))
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        if mat is not None:
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.MassAPI.Apply(xf.GetPrim()).CreateMassAttr(mass)
    if semantic:  # class labels -> ground-truth segmentation / bounding boxes
        from pxr import UsdSemantics

        UsdSemantics.LabelsAPI.Apply(xf.GetPrim(), "class").CreateLabelsAttr([semantic])
    return xf


def build_arena(path):
    """Test arena: 4 x 4 m walls (lidar / SLAM), a black line loop (line following), coloured blocks."""
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/TestArena")
    stage.SetDefaultPrim(root.GetPrim())
    wall_mat = physics_material(stage, Sdf.Path("/TestArena/PhysicsMaterials/wall"), 0.6, 0.5)
    block_mat = physics_material(stage, Sdf.Path("/TestArena/PhysicsMaterials/block"), 0.9, 0.8)
    half, h, t = 2.0, 0.30, 0.05
    for name, size, pos in (("north", (2 * half + t, t, h), (0, half, h / 2)),
                            ("south", (2 * half + t, t, h), (0, -half, h / 2)),
                            ("east", (t, 2 * half + t, h), (half, 0, h / 2)),
                            ("west", (t, 2 * half + t, h), (-half, 0, h / 2)),
                            ("pillar", (0.25, 0.25, h), (1.2, -1.2, h / 2)),
                            ("crate", (0.40, 0.30, 0.25), (-1.3, 1.1, 0.125))):
        box(stage, Sdf.Path("/TestArena/Walls/" + name), size, pos, (0.75, 0.72, 0.68), mat=wall_mat, semantic="wall")
    # black line loop, 2.5 cm wide, radius 1.2 m, centred 0.6 m in front of the robot (line following)
    import numpy as np
    n, r, w = 96, 1.2, 0.025
    cx, cy = 0.6, 0.0
    pts, idx, cnt = [], [], []
    for i in range(n):
        a = 2 * math.pi * i / n
        for rr in (r - w / 2, r + w / 2):
            pts.append(Gf.Vec3f(cx + rr * math.cos(a), cy + rr * math.sin(a), 0.0008))
    for i in range(n):
        j = (i + 1) % n
        idx += [2 * i, 2 * i + 1, 2 * j + 1, 2 * j]
        cnt.append(4)
    line = UsdGeom.Mesh.Define(stage, "/TestArena/LineTrack")
    line.CreatePointsAttr(pts)
    line.CreateFaceVertexIndicesAttr(idx)
    line.CreateFaceVertexCountsAttr(cnt)
    line.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * len(pts))
    line.CreateSubdivisionSchemeAttr("none")
    UsdShade.MaterialBindingAPI.Apply(line.GetPrim()).Bind(
        preview_material(stage, Sdf.Path("/TestArena/Looks/line_black"), (0.02, 0.02, 0.02), roughness=0.9))
    # graspable 3 cm blocks (the gripper lifts <= 400 g) for colour tracking / sorting / pick-and-place
    # front-left of the start pose: inside the wrist camera's view (init pose) but off the straight-ahead path
    for name, rgb, pos in (("red_block", (0.85, 0.05, 0.05), (0.45, 0.22, 0.015)),
                           ("green_block", (0.05, 0.7, 0.1), (0.50, 0.30, 0.015)),
                           ("blue_block", (0.05, 0.2, 0.85), (0.45, 0.38, 0.015))):
        box(stage, Sdf.Path("/TestArena/Blocks/" + name), (0.03, 0.03, 0.03), pos, rgb, mass=0.03,
            mat=block_mat, semantic=name.replace("_block", ""))
    stage.GetRootLayer().Save()


OURS = ["/World/PhysicsScene", "/World/GroundPlane", "/World/rosorin_pro", "/World/TestArena", "/World/Graphs",
        "/Environment/FillDome"]


def remove_specs(path, prim_paths, children_of=()):
    """Remove prim specs (and/or all child specs of some prims) directly on a layer file. Returns the count."""
    if not os.path.exists(path):
        return 0
    layer = Sdf.Layer.FindOrOpen(path)
    edit = Sdf.BatchNamespaceEdit()
    n = 0
    for p in prim_paths:
        if layer.GetPrimAtPath(p):
            edit.Add(Sdf.NamespaceEdit.Remove(Sdf.Path(p)))
            n += 1
    for p in children_of:
        spec = layer.GetPrimAtPath(p)
        if spec:
            for c in list(spec.nameChildren):
                edit.Add(Sdf.NamespaceEdit.Remove(c.path))
                n += 1
    if n:
        layer.Apply(edit)
        layer.Save()
    return n


# ------------------------------------------------------------------------------------------------
def main():
    if not args.no_arena:
        if os.path.exists(args.arena):
            os.remove(args.arena)
        build_arena(args.arena)

    backup = os.path.join(REPO, "scenes", "RosOrin_Claude.original.usda")
    if os.path.basename(scene) == "RosOrin_Claude.usd" and os.path.exists(scene) and not os.path.exists(backup):
        Sdf.Layer.FindOrOpen(scene).Export(backup)
        print("backed up original scene ->", backup)

    ctx = omni.usd.get_context()
    if args.new or not os.path.exists(scene):
        ctx.new_stage()
        app.update()
        st = ctx.get_stage()
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        st.SetDefaultPrim(UsdGeom.Xform.Define(st, "/World").GetPrim())
        light = UsdLux.DistantLight.Define(st, "/Environment/defaultLight")
        light.CreateIntensityAttr(3000.0)
        light.CreateAngleAttr(1.0)
        UsdGeom.Xformable(light).AddRotateXYZOp().Set(Gf.Vec3f(45, 0, 45))
        ctx.save_as_stage(scene)
        app.update()
    # Remove what a previous run added on the *layer* (Sdf), while no stage is open. Removing the robot from a
    # live stage makes OmniGraph "disconnect" the referenced graphs and author connection-delete opinions into
    # this layer, which silently kills every robot graph after the robot is added again.
    remove_specs(scene, OURS)
    ctx.open_stage(scene)
    for _ in range(5):
        app.update()
    stage = ctx.get_stage()
    world = stage.GetPrimAtPath("/World") or UsdGeom.Xform.Define(stage, "/World").GetPrim()
    stage.SetDefaultPrim(world)

    # physics scene: 480 Hz (config), TGS, CPU (one articulation -> CPU is faster than GPU dynamics)
    import yaml

    with open(args.config, "r", encoding="utf-8") as f:
        CFG = yaml.safe_load(f)
    ps = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    ps.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(ps.GetPrim())
    px.CreateTimeStepsPerSecondAttr(int(CFG["physics"]["steps_per_second"]))
    px.CreateSolverTypeAttr("TGS")
    px.CreateEnableGPUDynamicsAttr(False)
    px.CreateBroadphaseTypeAttr("MBP")
    px.CreateEnableStabilizationAttr(True)
    px.CreateEnableCCDAttr(False)

    # ground: visual quad + box collider with a floor material
    ground = UsdGeom.Xform.Define(stage, "/World/GroundPlane")
    quad = UsdGeom.Mesh.Define(stage, "/World/GroundPlane/visual")
    s = 10.0
    quad.CreatePointsAttr([Gf.Vec3f(-s, -s, 0), Gf.Vec3f(s, -s, 0), Gf.Vec3f(s, s, 0), Gf.Vec3f(-s, s, 0)])
    quad.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    quad.CreateFaceVertexCountsAttr([4])
    quad.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    quad.CreateSubdivisionSchemeAttr("none")
    UsdShade.MaterialBindingAPI.Apply(quad.GetPrim()).Bind(
        preview_material(stage, Sdf.Path("/World/GroundPlane/Looks/floor"), (0.55, 0.55, 0.52), roughness=0.8))
    col = UsdGeom.Cube.Define(stage, "/World/GroundPlane/collider")
    col.CreateSizeAttr(1.0)
    col.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.05))
    col.AddScaleOp().Set(Gf.Vec3f(2 * s, 2 * s, 0.1))
    col.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    floor_mat = physics_material(stage, Sdf.Path("/World/GroundPlane/PhysicsMaterials/floor"), 0.9, 0.8)
    UsdShade.MaterialBindingAPI.Apply(col.GetPrim()).Bind(floor_mat, UsdShade.Tokens.weakerThanDescendants, "physics")

    # robot
    robot = UsdGeom.Xform.Define(stage, "/World/rosorin_pro")
    robot.GetPrim().GetReferences().AddReference(rel(args.robot))
    robot.AddTranslateOp().Set(Gf.Vec3d(*args.spawn))
    robot.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))

    # test arena
    if not args.no_arena:
        arena = UsdGeom.Xform.Define(stage, "/World/TestArena")
        arena.GetPrim().GetReferences().AddReference(rel(args.arena))

    # lights: keep the user's lights, add a soft fill dome
    dome = UsdLux.DomeLight.Define(stage, "/Environment/FillDome")
    dome.CreateIntensityAttr(400.0)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.9, 1.0))

    # /clock
    UsdGeom.Scope.Define(stage, "/World/Graphs")
    ros_graphs.build_clock_graph("/World/Graphs/ROS_Clock")

    # nice default viewpoint for the GUI
    stage.GetRootLayer().customLayerData = dict(stage.GetRootLayer().customLayerData)
    stage.SetTimeCodesPerSecond(60)
    ctx.save_stage()
    ctx.close_stage()
    app.update()
    # the scene must only *reference* the robot / arena: drop any override OmniGraph may have authored below them
    stray = remove_specs(scene, [], children_of=["/World/rosorin_pro", "/World/TestArena"])
    print("SCENE DONE", scene, "robot ->", rel(args.robot), "stray overrides removed:", stray, flush=True)


try:
    main()
except Exception:  # make failures visible to build_all.bat (Kit would otherwise exit with code 0)
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    os._exit(1)
app.close()

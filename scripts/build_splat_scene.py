r"""Build the Gaussian-splat navigation scene for the ROSOrin Pro (run by launch\build_splat_world.bat).

    C:\isaac-sim-standalone-6.2.0\python.bat scripts\build_splat_scene.py --stage assets [--skip-convert]
    C:\isaac-sim-standalone-6.2.0\python.bat scripts\build_splat_scene.py --stage compose

--stage assets (inputs: config/splat_world.yaml, the PLY, nav2/rosorin_nav2/config/worlds.yaml "splat")
  1. reads the 3DGS PLY, crops it to the building and converts it to USD (ParticleField3DGaussianSplat,
     Isaac Sim's omni.kit.converter.gsplat) -> usd/splat/building.usd (generated, not in git)
  2. occupancy grid from the Gaussian centres (scripts/splat_occupancy.py) -> Nav2 map
     nav2/rosorin_nav2/maps/splat_building.{pgm,yaml} and the preview docs/images/splat_map.png
  3. collision + lidar proxy from the occupied cells: invisible full-height collision boxes and a thin
     camera-hidden strip at the lidar height -> scenes/splat_proxy.usd
  4. checks that every named place is free and reachable from the spawn pose
     -> tests/results/splat_build_report.json
--stage compose (after scripts/build_scene.py --scene scenes/splat_world.usd --new --no-arena)
  adds the splat and the proxy to scenes/splat_world.usd, moves the robot to the spawn pose, hides the grey
  ground quad (the splat has its own floor) and enlarges the floor collider to the building.
"""

import argparse
import json
import math
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

ap = argparse.ArgumentParser()
ap.add_argument("--stage", choices=["assets", "compose"], required=True)
ap.add_argument("--config", default=os.path.join(REPO, "config", "splat_world.yaml"))
ap.add_argument("--worlds", default=os.path.join(REPO, "nav2", "rosorin_nav2", "config", "worlds.yaml"))
ap.add_argument("--skip-convert", action="store_true", help="reuse an existing splat USD")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.kit.app

em = omni.kit.app.get_app().get_extension_manager()
for ext in ("omni.pip.compute", "omni.kit.converter.gsplat"):  # scipy / the 3DGS -> USD converter
    em.set_extension_enabled_immediate(ext, True)
app.update()

import numpy as np
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

import splat_occupancy as so


def repo_path(p):
    return os.path.normpath(p if os.path.isabs(p) else os.path.join(REPO, p))


def rel(path, from_file):
    r = os.path.relpath(path, os.path.dirname(from_file)).replace("\\", "/")
    return r if r.startswith(".") else "./" + r


with open(args.config, encoding="utf-8") as f:
    CFG = yaml.safe_load(f)
with open(args.worlds, encoding="utf-8") as f:
    WORLD = yaml.safe_load(f)["splat"]
SPAWN = [float(v) for v in WORLD["spawn"]]


def _box_mesh(stage, path, rects, grid, z0, z1):
    """One mesh with a box (z0..z1) per merged rectangle of occupied cells."""
    res, o = grid["resolution"], grid["origin"]
    pts, counts, idx = [], [], []
    faces = ((0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7))
    for r0, c0, r1, c1 in rects:
        x0, x1 = o[0] + c0 * res, o[0] + c1 * res
        y0, y1 = o[1] + r0 * res, o[1] + r1 * res
        b = len(pts)
        for zz in (z0, z1):
            pts += [Gf.Vec3f(x0, y0, zz), Gf.Vec3f(x1, y0, zz), Gf.Vec3f(x1, y1, zz), Gf.Vec3f(x0, y1, zz)]
        for fc in faces:
            counts.append(4)
            idx += [b + i for i in fc]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(idx)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.35, 0.2)])
    return mesh, len(pts)


def build_proxy(grid, path, height, lidar_band, render):
    """Stand-in geometry for what the Gaussians look like (they have no collision and no lidar return).

    /SplatProxy/collision  boxes 0..height, PhysX triangle-mesh collider, invisible (physics ignores visibility)
    /SplatProxy/lidar      thin boxes around the lidar plane, no collider, hidden for cameras only.
    Why two meshes (all tested, docs/SPLAT_ENVIRONMENT.md): the RTX lidar ignores invisible and guide prims, and
    every prim the lidar does see also cuts the Gaussians behind it out of the camera image (dark patches). A thin
    strip at the beam height keeps that artefact to a few centimetres. doNotCastShadows would remove it but also
    hides the prim from the lidar.
    """
    rects = so.rectangles(grid["occupied"])
    if os.path.exists(path):
        os.remove(path)
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/SplatProxy")
    stage.SetDefaultPrim(root.GetPrim())
    col, n_col = _box_mesh(stage, "/SplatProxy/collision", rects, grid, 0.0, height)
    UsdPhysics.CollisionAPI.Apply(col.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(col.GetPrim()).CreateApproximationAttr(UsdPhysics.Tokens.none)
    col.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
    lid, n_lid = _box_mesh(stage, "/SplatProxy/lidar", rects, grid, lidar_band[0], lidar_band[1])
    if render == "hide_for_camera":
        UsdGeom.PrimvarsAPI(lid.GetPrim()).CreatePrimvar("hideForCamera", Sdf.ValueTypeNames.Bool).Set(True)
    elif render != "visible":  # "visible": debug view of what the lidar sees
        raise ValueError("proxy.render must be hide_for_camera or visible, not %r" % render)
    root.GetPrim().SetCustomDataByKey("rosorin:source", "scripts/build_splat_scene.py (occupied cells of the splat map)")
    stage.GetRootLayer().Save()
    return len(rects), n_col + n_lid


def stage_assets():
    t0 = time.time()
    sc, oc, pc, mc = CFG["splat"], CFG["occupancy"], CFG["proxy"], CFG["map"]
    ply = repo_path(sc["ply"])
    if not os.path.isfile(ply):
        raise FileNotFoundError("splat PLY not found: %s (config/splat_world.yaml splat.ply)" % ply)
    from usd_convert_gsplat.ply_reader import GaussianSplatData, read_ply

    print("[splat] reading", ply, flush=True)
    data = read_ply(ply)
    keep = so.crop_mask(data.positions, sc["crop_min"], sc["crop_max"])
    print("[splat] %d Gaussians, %d inside the crop box (%.1f s)" % (data.count, int(keep.sum()), time.time() - t0),
          flush=True)
    report = {"ply": os.path.basename(ply), "gaussians": int(data.count), "gaussians_cropped": int(keep.sum())}

    splat_usd = repo_path(sc["usd"])
    if args.skip_convert and os.path.exists(splat_usd):
        print("[splat] reusing", splat_usd, flush=True)
    else:
        from usd_convert_gsplat.usd_writer import write_gaussian_splat_usd

        cropped = GaussianSplatData(
            positions=data.positions[keep], scales=data.scales[keep], rotations=data.rotations[keep],
            opacities=data.opacities[keep], f_dc=data.f_dc[keep],
            f_rest=None if data.f_rest is None else data.f_rest[keep], sh_layout=data.sh_layout)
        os.makedirs(os.path.dirname(splat_usd), exist_ok=True)
        write_gaussian_splat_usd(cropped, splat_usd, source_file=ply, prim_name="building",
                                 up_axis=sc.get("up_axis", "Z"), half_precision=bool(sc.get("half_precision", True)))
        del cropped
    report["splat_usd_mb"] = round(os.path.getsize(splat_usd) / 1e6, 1)

    pos, op, smax = so.gaussian_arrays(data.positions[keep], data.opacities[keep], data.scales[keep])
    del data
    grid = so.occupancy(pos, op, smax, oc, sc["crop_min"][:2], sc["crop_max"][:2])
    report["occupancy"] = grid["stats"]
    print("[splat] occupancy:", grid["stats"], flush=True)

    out_dir = repo_path(mc["out_dir"])
    os.makedirs(out_dir, exist_ok=True)
    img = so.write_nav2_map(grid, out_dir, mc["name"])
    report["map"] = os.path.relpath(os.path.join(out_dir, mc["name"] + ".yaml"), REPO).replace("\\", "/")

    places = {k: [float(v) for v in p] for k, p in WORLD["places"].items()}
    checks = so.check_places(grid, dict(places, spawn=SPAWN), SPAWN, min_clearance=0.20)
    report["places"] = checks
    bad = [k for k, c in checks.items() if c["clearance_m"] < 0.25 or not c["reachable"]]
    for k, c in checks.items():
        print("[splat] place %-16s clearance %.2f m reachable %s" % (k, c["clearance_m"], c["reachable"]), flush=True)
    so.preview(img, grid, places, repo_path(mc["preview"]), spawn=SPAWN)

    proxy = repo_path(pc["usd"])
    n_rect, n_pts = build_proxy(grid, proxy, float(pc["height"]), [float(v) for v in pc["lidar_band"]],
                                pc.get("render", "hide_for_camera"))
    report["proxy"] = {"rectangles": n_rect, "points": n_pts, "height_m": pc["height"], "render": pc.get("render"),
                       "lidar_band_m": pc["lidar_band"],
                       "usd_kb": round(os.path.getsize(proxy) / 1e3, 1)}
    report["ok"] = not bad
    report["seconds"] = round(time.time() - t0, 1)
    os.makedirs(os.path.join(REPO, "tests", "results"), exist_ok=True)
    with open(os.path.join(REPO, "tests", "results", "splat_build_report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    print("[splat] proxy: %d boxes -> %s" % (n_rect, proxy), flush=True)
    if bad:
        raise RuntimeError("places not free/reachable: %s (edit worlds.yaml 'splat')" % bad)
    print("SPLAT ASSETS DONE (%.0f s)" % (time.time() - t0), flush=True)


def stage_compose():
    scene = repo_path(CFG["scene"]["usd"])
    if not os.path.exists(scene):
        raise FileNotFoundError("%s missing: run scripts/build_scene.py --scene ... --new --no-arena first" % scene)
    stage = Usd.Stage.Open(scene)
    # Typeless prim: the type (ParticleField3DGaussianSplat) must come from the reference. A local "Xform"
    # type (e.g. UsdGeom.Xform.Define) is a stronger opinion and turns the splat into an empty Xform.
    splat = stage.DefinePrim("/World/Splat")
    splat.ClearTypeName()
    splat.GetReferences().ClearReferences()
    splat.GetReferences().AddReference(rel(repo_path(CFG["splat"]["usd"]), scene), "/building")
    proxy = UsdGeom.Xform.Define(stage, "/World/SplatProxy")
    proxy.GetPrim().GetReferences().ClearReferences()
    proxy.GetPrim().GetReferences().AddReference(rel(repo_path(CFG["proxy"]["usd"]), scene))
    # robot at the spawn pose (build_scene.py places it at the origin with an identity orientation)
    yaw = math.radians(SPAWN[2])
    robot = stage.GetPrimAtPath("/World/rosorin_pro")
    z0 = robot.GetAttribute("xformOp:translate").Get()[2]
    robot.GetAttribute("xformOp:translate").Set(Gf.Vec3d(SPAWN[0], SPAWN[1], z0))
    robot.GetAttribute("xformOp:orient").Set(Gf.Quatf(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)))
    # the splat brings its own floor: hide the grey quad, keep (and enlarge) the floor collider
    UsdGeom.Imageable(stage.GetPrimAtPath("/World/GroundPlane/visual")).MakeInvisible()
    col = stage.GetPrimAtPath("/World/GroundPlane/collider")
    gc, gs = CFG["scene"]["ground_center"], CFG["scene"]["ground_size"]
    col.GetAttribute("xformOp:translate").Set(Gf.Vec3d(gc[0], gc[1], -0.05))
    col.GetAttribute("xformOp:scale").Set(Gf.Vec3f(gs[0], gs[1], 0.1))
    stage.GetRootLayer().customLayerData = dict(stage.GetRootLayer().customLayerData,
                                                rosorin_spawn=Gf.Vec3d(*SPAWN))
    stage.GetRootLayer().Save()
    print("SPLAT SCENE DONE", scene, "spawn", SPAWN, flush=True)


try:
    stage_assets() if args.stage == "assets" else stage_compose()
except Exception:  # Kit swallows exceptions (exit code 0): make failures visible to the .bat
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    os._exit(1)
app.close()

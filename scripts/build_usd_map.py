r"""Nav2 map from the colliders of a USD scene (exact geometry, for scenes built from boxes and meshes).

    C:\isaac-sim-standalone-6.2.0\python.bat scripts\build_usd_map.py            (the test arena, defaults below)
    ... scripts\build_usd_map.py --usd scenes\test_arena.usd --name test_arena --free -2 -2 2 2 --bounds -2.5 -2.5 2.5 2.5

Every prim with a collision API whose world bounding box overlaps the robot's height band becomes occupied.
Cells inside the --free rectangle that are not occupied are free; the rest of --bounds is unknown.
Dynamic bodies (rigid bodies, e.g. the 3 cm blocks) are skipped: they move.
"""

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))
ap = argparse.ArgumentParser()
ap.add_argument("--usd", default=os.path.join(REPO, "scenes", "test_arena.usd"))
ap.add_argument("--name", default="test_arena")
ap.add_argument("--out-dir", default=os.path.join(REPO, "nav2", "rosorin_nav2", "maps"))
ap.add_argument("--bounds", type=float, nargs=4, default=[-2.5, -2.5, 2.5, 2.5], metavar=("X0", "Y0", "X1", "Y1"))
ap.add_argument("--free", type=float, nargs=4, default=[-2.0, -2.0, 2.0, 2.0], metavar=("X0", "Y0", "X1", "Y1"))
ap.add_argument("--band", type=float, nargs=2, default=[0.05, 0.50], help="robot height band (m)")
ap.add_argument("--resolution", type=float, default=0.05)
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("omni.pip.compute", True)
app.update()

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

import splat_occupancy as so


def main():
    stage = Usd.Stage.Open(args.usd)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.guide])
    res = args.resolution
    x0, y0, x1, y1 = args.bounds
    W, H = int(round((x1 - x0) / res)), int(round((y1 - y0) / res))
    occ = np.zeros((H, W), bool)
    used = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        p, dynamic = prim, False
        while p and not p.IsPseudoRoot():
            dynamic |= p.HasAPI(UsdPhysics.RigidBodyAPI)
            p = p.GetParent()
        if dynamic:
            continue
        box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        lo, hi = box.GetMin(), box.GetMax()
        if hi[2] < args.band[0] or lo[2] > args.band[1]:
            continue
        c0, c1 = int(np.floor((lo[0] - x0) / res)), int(np.ceil((hi[0] - x0) / res))
        r0, r1 = int(np.floor((lo[1] - y0) / res)), int(np.ceil((hi[1] - y0) / res))
        occ[max(r0, 0):max(min(r1, H), 0), max(c0, 0):max(min(c1, W), 0)] = True
        used.append(str(prim.GetPath()))
    yy, xx = np.mgrid[0:H, 0:W]
    cx, cy = x0 + (xx + 0.5) * res, y0 + (yy + 0.5) * res
    fx0, fy0, fx1, fy1 = args.free
    free = (cx > fx0) & (cx < fx1) & (cy > fy0) & (cy < fy1) & ~occ
    grid = {"free": free, "occupied": occ, "origin": np.array([x0, y0]), "resolution": res}
    os.makedirs(args.out_dir, exist_ok=True)
    so.write_nav2_map(grid, args.out_dir, args.name,
                      note="scripts/build_usd_map.py from the colliders of %s" % os.path.basename(args.usd))
    print("MAP DONE %s: %d obstacles (%s), %.1f m2 free" % (os.path.join(args.out_dir, args.name + ".yaml"),
          len(used), ", ".join(u.split("/")[-2] for u in used), free.sum() * res * res), flush=True)


try:
    main()
except Exception:
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    os._exit(1)
app.close()

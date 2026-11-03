"""Mecanum physics tuning sweep (documented in docs/DECISIONS.md, "Mecanum physics").

Runs the robot with exact mecanum inverse kinematics on the wheel joints (no ROS) for several physics
parameter sets and reports how well forward / strafe / rotate commands are tracked (mean and noise of
the body velocity). Usage:
    launch\\run_python.bat tests\\tune_mecanum.py [--variants A,B,...]
"""

import argparse
import json
import math
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument("--scene", default=os.path.join(os.path.dirname(REPO), "RosOrin_Claude.usd"))
ap.add_argument("--variants", default="")
ap.add_argument("--out", default=os.path.join(REPO, "tests", "results", "tune_mecanum.json"))
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.kit.app

em = omni.kit.app.get_app().get_extension_manager()
em.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
app.update()
import numpy as np
import omni.timeline
import omni.usd
from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade

ctx = omni.usd.get_context()
ctx.open_stage(args.scene)
while ctx.get_stage_loading_status()[2] > 0:
    app.update()
stage = ctx.get_stage()
# the ROS drive graph would overwrite our wheel targets, and the cameras cost time: switch graphs off
blocks = stage.GetPrimAtPath("/World/TestArena/Blocks")  # keep the driving area clear
if blocks:
    blocks.SetActive(False)
for g in ("MecanumDrive", "DepthCamera", "Lidar"):
    p = stage.GetPrimAtPath("/World/rosorin_pro/Graphs/" + g)
    if p:
        p.SetActive(False)

R = "/World/rosorin_pro"
roller_joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint) and "_roller_" in p.GetName()]
roller_cols = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI) and "_roller_" in str(p.GetPath())]
roller_bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and "_roller_" in p.GetName()]
wheel_joints = [stage.GetPrimAtPath(R + "/Physics/" + n) for n in
                ("left_front_wheel", "left_back_wheel", "right_front_wheel", "right_back_wheel")]
roller_mat = stage.GetPrimAtPath(R + "/PhysicsMaterials/roller_rubber")
scene = stage.GetPrimAtPath("/World/PhysicsScene")
art_root = stage.GetPrimAtPath(R + "/Geometry/base_footprint")

BASE = dict(hz=240, pos_iter=32, vel_iter=4, compliant_k=0.0, compliant_c=0.0, armature=0.0, hull_verts=64,
            wheel_damping=2.0, roller_mass=0.005, mu=1.0, depen=None, roller_damping=0.0)
VARIANTS = {
    "A_baseline": {},
    "B_compliant": dict(compliant_k=2.0e4, compliant_c=150.0),
    "C_compliant_soft": dict(compliant_k=8.0e3, compliant_c=100.0),
    "D_B_armature": dict(compliant_k=2.0e4, compliant_c=150.0, armature=2e-6),
    "E_B_480hz": dict(compliant_k=2.0e4, compliant_c=150.0, hz=480),
    "F_B_hull128_vel8": dict(compliant_k=2.0e4, compliant_c=150.0, hull_verts=128, vel_iter=8),
    "G_B_wheeldamp1": dict(compliant_k=2.0e4, compliant_c=150.0, wheel_damping=1.0),
    "H_B_armature_480_vel8": dict(compliant_k=2.0e4, compliant_c=150.0, armature=2e-6, hz=480, vel_iter=8),
    "I_softer": dict(compliant_k=4.0e3, compliant_c=80.0),
    "J_C_480hz": dict(compliant_k=8.0e3, compliant_c=100.0, hz=480),
    "K_C_damped": dict(compliant_k=8.0e3, compliant_c=200.0),
    "L_C_mu12": dict(compliant_k=8.0e3, compliant_c=100.0, mu=1.2),
}
if args.variants:
    VARIANTS = {k: v for k, v in VARIANTS.items() if k.split("_")[0] in args.variants.split(",")}

D2R = math.pi / 180


def apply(p):
    PhysxSchema.PhysxSceneAPI(scene).GetTimeStepsPerSecondAttr().Set(int(p["hz"]))
    a = PhysxSchema.PhysxArticulationAPI(art_root)
    a.GetSolverPositionIterationCountAttr().Set(int(p["pos_iter"]))
    a.GetSolverVelocityIterationCountAttr().Set(int(p["vel_iter"]))
    pm = PhysxSchema.PhysxMaterialAPI.Apply(roller_mat)
    pm.CreateCompliantContactStiffnessAttr(float(p["compliant_k"]))
    pm.CreateCompliantContactDampingAttr(float(p["compliant_c"]))
    UsdPhysics.MaterialAPI(roller_mat).GetStaticFrictionAttr().Set(float(p["mu"]))
    UsdPhysics.MaterialAPI(roller_mat).GetDynamicFrictionAttr().Set(float(p["mu"]) * 0.9)
    for j in roller_joints:
        pj = PhysxSchema.PhysxJointAPI.Apply(j)
        pj.CreateArmatureAttr(float(p["armature"]))
    for c in roller_cols:
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(c).CreateHullVertexLimitAttr(int(p["hull_verts"]))
    for b in roller_bodies:
        UsdPhysics.MassAPI(b).GetMassAttr().Set(float(p["roller_mass"]))
        if p["depen"] is not None:
            PhysxSchema.PhysxRigidBodyAPI.Apply(b).CreateMaxDepenetrationVelocityAttr(float(p["depen"]))
    for w in wheel_joints:
        UsdPhysics.DriveAPI(w, "angular").GetDampingAttr().Set(float(p["wheel_damping"]) * D2R)


build = json.load(open(os.path.join(REPO, "description", "build_report.json")))
wheels = {w["joint"]: w for w in build["wheels"]}
order = ["left_front_wheel", "left_back_wheel", "right_front_wheel", "right_back_wheel"]
geo = []
for n in order:
    w = wheels[n]
    ring = np.mean([r["center"] for r in w["rollers"]], axis=0)
    u = next(r["axis"] for r in sorted(w["rollers"], key=lambda r: r["center"][2]))
    ux, uy = u[0], u[1]
    if ux < 0:
        ux, uy = -ux, -uy
    geo.append((w["xyz"][0] + ring[0], w["xyz"][1] + ring[1], ux, uy, w["outer_radius"]))


def ik(vx, vy, wz):
    """Exact mecanum inverse kinematics from the measured bottom-roller axes."""
    out = []
    for x, y, ux, uy, r in geo:
        out.append(((vx - wz * y) * ux + (vy + wz * x) * uy) / (r * ux))
    return np.array(out, dtype=np.float32)


tl = omni.timeline.get_timeline_interface()
from isaacsim.core.experimental.prims import Articulation

RESULTS = {}
for name, over in VARIANTS.items():
    p = dict(BASE, **over)
    tl.stop()
    app.update()
    apply(p)
    app.update()
    tl.play()
    for _ in range(3):
        app.update()
    art = Articulation(R + "/Geometry/base_footprint")
    app.update()
    wi = art.get_dof_indices(order)
    for _ in range(60):
        app.update()
    res = {}
    for test, cmd in (("forward", (0.2, 0, 0)), ("strafe", (0, 0.2, 0)), ("rotate", (0, 0, 1.0)),
                      ("diagonal", (0.14, 0.14, 0))):
        tgt = ik(*cmd)
        samples, wheel_dev, tilts, zs = [], [], [], []
        for i in range(150):
            art.set_dof_velocity_targets(tgt[None, :], dof_indices=wi)
            app.update()
            if i >= 30:
                lin, ang = art.get_velocities()
                lin, ang = lin.numpy()[0], ang.numpy()[0]
                _, q = art.get_world_poses()
                q = q.numpy()[0]
                yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2))
                c, s = math.cos(yaw), math.sin(yaw)
                samples.append([c * lin[0] + s * lin[1], -s * lin[0] + c * lin[1], ang[2]])
                wv = art.get_dof_velocities(dof_indices=wi).numpy()[0]
                wheel_dev.append(np.abs(wv - tgt).max())
                tilts.append(math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q[1] ** 2 + q[2] ** 2))))))
                zs.append(float(art.get_world_poses()[0].numpy()[0][2]))
        smp = np.array(samples)
        res[test] = {"cmd": cmd, "mean": smp.mean(0).round(4).tolist(), "std": smp.std(0).round(4).tolist(),
                     "wheel_max_dev": float(np.max(wheel_dev)), "wheel_p95_dev": float(np.percentile(wheel_dev, 95)),
                     "max_tilt_deg": float(np.max(tilts)), "z_range": [float(np.min(zs)), float(np.max(zs))]}
        for _ in range(40):
            art.set_dof_velocity_targets(np.zeros((1, 4), np.float32), dof_indices=wi)
            app.update()
    # score: tracking error + noise, normalised by the command
    err = 0.0
    for t, r in res.items():
        cmd = np.array(r["cmd"])
        err += float(np.abs(np.array(r["mean"]) - cmd).sum() + np.array(r["std"]).sum())
    RESULTS[name] = {"params": p, "tests": res, "score": err}
    print("VARIANT %-24s score=%.3f " % (name, err) + " | ".join(
        "%s mean=%s std=%s wdev95=%.2f tilt=%.1f z=%s" % (t, r["mean"], r["std"], r["wheel_p95_dev"], r["max_tilt_deg"], np.round(r["z_range"], 3).tolist()) for t, r in res.items()), flush=True)

os.makedirs(os.path.dirname(args.out), exist_ok=True)
with open(args.out, "w", encoding="utf-8") as f:
    json.dump(RESULTS, f, indent=2)
best = min(RESULTS, key=lambda k: RESULTS[k]["score"])
print("BEST", best, RESULTS[best]["score"], flush=True)
tl.stop()
app.update()
app.close()

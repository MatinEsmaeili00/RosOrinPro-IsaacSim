"""Step 3 - turn the URDF-importer output into the final simulation robot asset  usd/rosorin_pro.usd

    C:\\isaac-sim-standalone-6.2.0\\python.bat scripts\\configure_robot.py

What it does (see docs/ARCHITECTURE.md):
  1. selects the PhysX physics variant of the importer package and flattens it into ONE self-contained
     file (no instancing, no payloads) so it can be referenced from any scene
  2. renames the importer's mesh holders to <mesh>_visual / <mesh>_collision and re-creates the URDF
     frames that merge_fixed_joints removed (base_link, lidar_frame, imu_link, camera_link0, camera_link,
     depth_cam_frame, gripper_link, end_effector_link) -> verified against URDF forward kinematics
  3. physics: articulation solver settings, servo/motor drives from the real specs, PhysX mimic joints
     for the 4-bar gripper, free-spinning mecanum rollers, collider approximations, rubber materials
  4. sensors: COIN-D6 RTX lidar, Aurora 930 Pro RGB + depth cameras, MPU6050 IMU
  5. OmniGraph: mecanum drive from /cmd_vel, arm/gripper from /joint_command, and the ROS 2 topics of
     the real robot (/scan, /odom, /imu, /joint_states, /tf, /depth_cam/...)
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "scripts"))

ap = argparse.ArgumentParser()
ap.add_argument("--import-usd", default=os.path.join(REPO, "usd", "rosorin_pro", "rosorin_pro_sim", "rosorin_pro_sim.usda"))
ap.add_argument("--urdf", default=os.path.join(REPO, "description", "urdf", "rosorin_pro_sim.urdf"))
ap.add_argument("--config", default=os.path.join(REPO, "config", "rosorin_pro_sim.yaml"))
ap.add_argument("--out", default=os.path.join(REPO, "usd", "rosorin_pro.usd"))
ap.add_argument("--no-graphs", action="store_true")
args, _ = ap.parse_known_args()

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.kit.app

em = omni.kit.app.get_app().get_extension_manager()
for ext in ("isaacsim.ros2.bridge", "isaacsim.sensors.experimental.rtx", "isaacsim.sensors.experimental.physics",
            "isaacsim.sensors.physics.nodes", "isaacsim.robot.wheeled_robots.nodes", "isaacsim.core.nodes"):
    em.set_extension_enabled_immediate(ext, True)
app.update()

import numpy as np
import omni.usd
import yaml
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

import ros_graphs

with open(args.config, "r", encoding="utf-8") as f:
    CFG = yaml.safe_load(f)
with open(os.path.join(REPO, "description", "build_report.json"), "r", encoding="utf-8") as f:
    BUILD = json.load(f)

REPORT = {"checks": []}


def check(name, ok, detail=""):
    REPORT["checks"].append({"check": name, "pass": bool(ok), "detail": detail})
    print("[%s] %s %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


# ------------------------------------------------------------------------------------------------
# URDF helpers (forward kinematics for verification + fixed-frame origins)
# ------------------------------------------------------------------------------------------------

def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return Gf.Quatd(cr * cp * cy + sr * sp * sy,
                    Gf.Vec3d(sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy))


def origin_matrix(xyz, rpy):
    m = Gf.Matrix4d(1.0)
    m.SetRotateOnly(rpy_to_quat(*rpy))
    m.SetTranslateOnly(Gf.Vec3d(*xyz))
    return m


URDF = ET.parse(args.urdf).getroot()
JOINTS = {}
for j in URDF.findall("joint"):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()] if o is not None else [0, 0, 0]
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0, 0, 0]
    JOINTS[j.get("name")] = {"type": j.get("type"), "parent": j.find("parent").get("link"),
                             "child": j.find("child").get("link"), "xyz": xyz, "rpy": rpy}
CHILD_JOINT = {v["child"]: k for k, v in JOINTS.items()}


def fk(link):
    """World (= base_footprint at origin) transform of a URDF link with every joint at 0."""
    m = Gf.Matrix4d(1.0)
    while link in CHILD_JOINT:
        jd = JOINTS[CHILD_JOINT[link]]
        # Gf uses row vectors: child-in-world = child-in-parent * parent-in-world
        m = m * origin_matrix(jd["xyz"], jd["rpy"])
        link = jd["parent"]
    return m


# ------------------------------------------------------------------------------------------------
# 1. flatten the importer package (PhysX variant) into one plain layer
# ------------------------------------------------------------------------------------------------

def main():
    src = Usd.Stage.Open(args.import_usd)
    src.GetDefaultPrim().GetVariantSet("Physics").SetVariantSelection("physx")
    s1 = Usd.Stage.Open(src.Flatten())
    while True:
        inst = [p for p in s1.Traverse() if p.IsInstanceable()]
        if not inst:
            break
        for p in inst:
            p.SetInstanceable(False)
    layer = s1.Flatten()
    edit = Sdf.BatchNamespaceEdit()
    for spec in list(layer.rootPrims):
        if spec.name.startswith("Flattened_Prototype"):
            edit.Add(Sdf.NamespaceEdit.Remove(spec.path))
    layer.Apply(edit)

    # rename mesh holders:  X -> X_visual,  X_1 -> X_collision  (frees names such as camera_link / gripper_link)
    s2 = Usd.Stage.Open(layer)
    edit = Sdf.BatchNamespaceEdit()
    renamed = []
    for body in [p for p in s2.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]:
        for child in body.GetChildren():
            if child.HasAPI(UsdPhysics.RigidBodyAPI) or child.GetTypeName() != "Xform":
                continue
            is_col = any(d.HasAPI(UsdPhysics.CollisionAPI) for d in Usd.PrimRange(child))
            base = child.GetName()
            if base.endswith("_1") and body.GetChild(base[:-2]):
                base = base[:-2]
            new = base + ("_collision" if is_col else "_visual")
            edit.Add(Sdf.NamespaceEdit.Rename(child.GetPath(), new))
            renamed.append((str(child.GetPath()), new))
    ok = layer.Apply(edit)
    check("mesh holders renamed to *_visual / *_collision", ok, "%d prims" % len(renamed))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    layer.Export(args.out)

    ctx = omni.usd.get_context()
    ctx.open_stage(args.out)
    for _ in range(5):
        app.update()
    stage = ctx.get_stage()
    ROOT = stage.GetDefaultPrim()
    RP = ROOT.GetPath()

    BODIES = {p.GetName(): p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)}
    JOINT_PRIMS = {p.GetName(): p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)}
    ART_ROOT = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    check("exactly one articulation root", len(ART_ROOT) == 1, str([str(p.GetPath()) for p in ART_ROOT]))
    ART_ROOT = ART_ROOT[0]
    check("articulation root is base_footprint", ART_ROOT.GetName() == "base_footprint")
    check("rigid bodies", len(BODIES) == 52, "%d (1 base + 4 wheels + 36 rollers + 5 arm + 6 gripper)" % len(BODIES))

    # ------------------------------------------------------------------------------------------------
    # 2. URDF frames removed by merge_fixed_joints -> plain Xforms
    # ------------------------------------------------------------------------------------------------

    FRAME_SPECS = [  # (frame, parent prim name (body or frame), urdf joint)
        ("base_link", "base_footprint", "base_joint"),
        ("lidar_frame", "base_link", "lidar_joint"),
        ("imu_link", "base_link", "imu_joint"),
        ("camera_link0", "link4", "camera_connect_joint"),
        ("camera_link", "camera_link0", "camera_joint"),
        ("depth_cam_frame", "camera_link", "depth_cam_joint_sim"),
        ("gripper_link", "link5", "gripper_joint"),
        ("end_effector_link", "link5", "end_effector_joint"),
    ]
    FRAMES = {}


    def set_local_pose(prim, xyz, quat):
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*xyz))
        xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(quat)


    for name, parent, joint in FRAME_SPECS:
        parent_prim = BODIES.get(parent) or FRAMES.get(parent)
        prim = UsdGeom.Xform.Define(stage, parent_prim.GetPath().AppendChild(name)).GetPrim()
        jd = JOINTS[joint]
        set_local_pose(prim, jd["xyz"], rpy_to_quat(*jd["rpy"]))
        FRAMES[name] = prim

    # verify every body and frame against URDF FK
    cache = UsdGeom.XformCache()
    root_inv = cache.GetLocalToWorldTransform(ROOT).GetInverse()
    pos_err, rot_err, worst = 0.0, 0.0, ""
    for name, prim in list(BODIES.items()) + list(FRAMES.items()):
        if name not in CHILD_JOINT and name != "base_footprint":
            continue
        m_usd = cache.GetLocalToWorldTransform(prim) * root_inv
        m_urdf = fk(name)
        dp = (m_usd.ExtractTranslation() - m_urdf.ExtractTranslation()).GetLength()
        q1 = m_usd.ExtractRotationQuat().GetNormalized()
        q2 = m_urdf.ExtractRotationQuat().GetNormalized()
        d = abs(q1.GetReal() * q2.GetReal() + Gf.Dot(q1.GetImaginary(), q2.GetImaginary()))
        dr = 2 * math.acos(min(1.0, d))
        if dp > pos_err:
            pos_err, worst = dp, name
        rot_err = max(rot_err, dr)
    check("bodies + frames match URDF forward kinematics", pos_err < 1e-5 and rot_err < 1e-4,
          "max pos err %.2e m (%s), max rot err %.2e rad" % (pos_err, worst, rot_err))

    # ------------------------------------------------------------------------------------------------
    # 3. physics
    # ------------------------------------------------------------------------------------------------

    D2R = math.pi / 180.0
    pcfg, dcfg = CFG["physics"], CFG["drives"]

    art = PhysxSchema.PhysxArticulationAPI.Apply(ART_ROOT)
    art.CreateSolverPositionIterationCountAttr(int(pcfg["solver_position_iterations"]))
    art.CreateSolverVelocityIterationCountAttr(int(pcfg["solver_velocity_iterations"]))
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSleepThresholdAttr(0.0)
    art.CreateStabilizationThresholdAttr(0.0)


    def set_drive(prim, kind, stiffness, damping, max_force, max_vel_rad):
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(float(stiffness) * D2R)   # USD angular drives are per degree
        drive.CreateDampingAttr(float(damping) * D2R)
        drive.CreateMaxForceAttr(float(max_force))
        if kind == "position":
            drive.CreateTargetPositionAttr(0.0)
        else:
            drive.CreateTargetVelocityAttr(0.0)
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(float(max_vel_rad) / D2R)


    def remove_drive(prim):
        if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            prim.RemoveAPI(UsdPhysics.DriveAPI, "angular")
        for a in list(prim.GetAttributes()):
            if a.GetName().startswith("drive:angular:physics:"):
                prim.RemoveProperty(a.GetName())


    arm, grip, wheel, roller = dcfg["arm"], dcfg["gripper"], dcfg["wheel"], dcfg["roller"]
    wheel_names = CFG["wheels"]["names"]
    mimic_report = []
    counts = {"arm": 0, "gripper": 0, "mimic": 0, "wheel": 0, "roller": 0}
    for name, jp in JOINT_PRIMS.items():
        if name in arm["joints"]:
            set_drive(jp, "position", arm["stiffness"], arm["damping"], arm["max_effort"], arm["max_velocity"])
            counts["arm"] += 1
        elif name == grip["joint"]:
            set_drive(jp, "position", grip["stiffness"], grip["damping"], grip["max_effort"], grip["max_velocity"])
            counts["gripper"] += 1
        elif jp.HasAPI("NewtonMimicAPI"):
            leader = jp.GetRelationship("newton:mimicJoint").GetTargets()[0]
            c1 = jp.GetAttribute("newton:mimicCoef1").Get() if jp.GetAttribute("newton:mimicCoef1") else None
            c0 = jp.GetAttribute("newton:mimicCoef0").Get() if jp.GetAttribute("newton:mimicCoef0") else None
            c1 = 1.0 if c1 is None else float(c1)
            c0 = 0.0 if c0 is None else float(c0)
            axis = "rot" + jp.GetAttribute("physics:axis").Get()
            remove_drive(jp)
            mimic = PhysxSchema.PhysxMimicJointAPI.Apply(jp, axis)
            mimic.CreateReferenceJointRel().SetTargets([leader])
            mimic.CreateReferenceJointAxisAttr("rotX")
            # PhysX constraint: q + gearing * q_ref + offset = 0   <=>   q = c1 * q_ref + c0
            mimic.CreateGearingAttr(-c1)
            mimic.CreateOffsetAttr(-c0)
            PhysxSchema.PhysxJointAPI.Apply(jp).CreateMaxJointVelocityAttr(float(grip["max_velocity"]) * 2 / D2R)
            mimic_report.append({"joint": name, "leader": str(leader.name), "multiplier": c1, "offset": c0, "axis": axis})
            counts["mimic"] += 1
        elif name in wheel_names:
            set_drive(jp, "velocity", 0.0, wheel["damping"], wheel["max_effort"], wheel["max_velocity"])
            counts["wheel"] += 1
        elif "_roller_" in name:
            remove_drive(jp)
            pj = PhysxSchema.PhysxJointAPI.Apply(jp)
            pj.CreateJointFrictionAttr(0.0)
            pj.CreateMaxJointVelocityAttr(20000.0)
            counts["roller"] += 1
    # start in the real robot's init pose: drive targets + initial joint state (USD joint state is in degrees)
    for jn, q in CFG["init_pose"].items():
        jp = JOINT_PRIMS[jn]
        UsdPhysics.DriveAPI(jp, "angular").GetTargetPositionAttr().Set(float(q) / D2R)
        js = PhysxSchema.JointStateAPI.Apply(jp, "angular")
        js.CreatePositionAttr(float(q) / D2R)
        js.CreateVelocityAttr(0.0)
    check("joint drives configured", counts == {"arm": 5, "gripper": 1, "mimic": 5, "wheel": 4, "roller": 36}, str(counts))
    REPORT["mimic"] = mimic_report

    # physics materials
    mat_scope = UsdGeom.Scope.Define(stage, RP.AppendChild("PhysicsMaterials"))


    def physics_material(name, static, dynamic, combine="average"):
        mat = UsdShade.Material.Define(stage, mat_scope.GetPath().AppendChild(name))
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(float(static))
        api.CreateDynamicFrictionAttr(float(dynamic))
        api.CreateRestitutionAttr(0.0)
        px = PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
        px.CreateFrictionCombineModeAttr(combine)
        return mat


    roller_mat = physics_material("roller_rubber", pcfg["roller_static_friction"], pcfg["roller_dynamic_friction"])
    _px = PhysxSchema.PhysxMaterialAPI(roller_mat.GetPrim())
    _px.CreateCompliantContactStiffnessAttr(float(pcfg["roller_compliant_stiffness"]))
    _px.CreateCompliantContactDampingAttr(float(pcfg["roller_compliant_damping"]))
    pad_mat = physics_material("gripper_pad", 1.2, 1.0, "max")
    body_mat = physics_material("robot_body", 0.5, 0.4)

    # colliders
    n_col = {"convexHull": 0, "convexDecomposition": 0}
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        path = str(prim.GetPath())
        if prim.IsA(UsdGeom.Mesh):
            approx = "convexDecomposition" if "base_link_chassis" in path else "convexHull"
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
            if approx == "convexDecomposition":
                cd = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
                cd.CreateMaxConvexHullsAttr(48)
                cd.CreateHullVertexLimitAttr(64)
                cd.CreateVoxelResolutionAttr(500000)
                cd.CreateShrinkWrapAttr(True)
            n_col[approx] += 1
        pc = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        pc.CreateContactOffsetAttr(0.002)
        pc.CreateRestOffsetAttr(0.0)
        if "_roller_" in path:
            mat = roller_mat
        elif any(("/%s/" % g) in path for g in ("l_link", "r_link", "l_in_link", "r_in_link", "l_out_link", "r_out_link")):
            mat = pad_mat
        else:
            mat = body_mat
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")
    check("colliders configured", n_col["convexDecomposition"] >= 1 and n_col["convexHull"] >= 36, str(n_col))

    # ------------------------------------------------------------------------------------------------
    # 4. sensors
    # ------------------------------------------------------------------------------------------------

    scfg = CFG["sensors"]


    def set_attr(prim, name, value, vtype):
        attr = prim.GetAttribute(name)
        if not attr:
            attr = prim.CreateAttribute(name, vtype)
        attr.Set(value)


    # 4a. COIN-D6 lidar (single-beam 2D ToF, 4000 samples/s, 10 Hz -> 400 points = 0.9 deg per scan)
    lc = scfg["lidar"]
    lidar = stage.DefinePrim(FRAMES["lidar_frame"].GetPath().AppendChild("coin_d6"), "OmniLidar")
    lidar.ApplyAPI("OmniSensorGenericLidarCoreAPI")
    UsdGeom.Xformable(lidar).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    F, U, T, B = Sdf.ValueTypeNames.Float, Sdf.ValueTypeNames.UInt, Sdf.ValueTypeNames.Token, Sdf.ValueTypeNames.Bool
    core = "omni:sensor:Core:"
    firing = int(lc["samples_per_rev"] * lc["rate_hz"])
    for n, v, t in [
        ("scanType", "ROTARY", T), ("rotationDirection", "CW", T), ("rayType", "IDEALIZED", T),
        ("scanRateBaseHz", int(lc["rate_hz"]), U), ("patternFiringRateHz", firing, U),
        ("numberOfEmitters", 1, U), ("numberOfChannels", 1, U), ("maxReturns", 1, U),
        ("nearRangeM", float(lc["range_min"]), F), ("farRangeM", float(lc["range_max"]), F),
        ("rangeAccuracyM", 0.01, F), ("rangeResolutionM", 0.001, F),
        ("azimuthErrorStd", 0.05, F), ("elevationErrorStd", 0.0, F),
        ("minReflectance", 0.05, F), ("minReflectionRangeM", float(lc["range_max"]), F),
        ("accumulateOutputs", True, B), ("skipDroppingInvalidPoints", False, B),
        ("validStartAzimuthDeg", 0.0, F), ("validEndAzimuthDeg", 360.0, F), ("startAzimuthOffsetDeg", 0.0, F),
    ]:
        set_attr(lidar, core + n, v, t)
    es = core + "emitterState:s001:"
    set_attr(lidar, es + "azimuthDeg", Vt.FloatArray([0.0]), Sdf.ValueTypeNames.FloatArray)
    set_attr(lidar, es + "elevationDeg", Vt.FloatArray([0.0]), Sdf.ValueTypeNames.FloatArray)
    set_attr(lidar, es + "channelId", Vt.UIntArray([1]), Sdf.ValueTypeNames.UIntArray)
    set_attr(lidar, es + "fireTimeNs", Vt.UIntArray([0]), Sdf.ValueTypeNames.UIntArray)
    for n in ("distanceCorrectionM", "emitterPeakPowerW", "focalDistM", "focalSlope", "horOffsetM", "vertOffsetM"):
        set_attr(lidar, es + n, Vt.FloatArray([]), Sdf.ValueTypeNames.FloatArray)
    set_attr(lidar, "omni:sensor:tickRate", float(lc["rate_hz"]), F)
    set_attr(lidar, "omni:sensor:modelName", "COIN-D6", Sdf.ValueTypeNames.String)
    set_attr(lidar, "omni:sensor:marketName", "ROSOrin Pro lidar (COIN-D6)", Sdf.ValueTypeNames.String)
    LIDAR_PATH = str(lidar.GetPath())

    # 4b. Aurora 930 Pro: RGB + depth cameras on the optical frame depth_cam_frame (z fwd, x right, y down).
    #     A USD camera looks down -Z with +Y up -> rotate 180 deg about X.
    cc = scfg["depth_camera"]
    h_ap = 3.6
    focal = h_ap / (2.0 * math.tan(math.radians(cc["hfov_deg"]) / 2.0))
    v_ap = h_ap * cc["height"] / cc["width"]
    CAMS = {}
    for cname, clip in (("rgb_camera", (0.01, 1000.0)), ("depth_camera", (cc["depth_min"], cc["depth_max"]))):
        cam = UsdGeom.Camera.Define(stage, FRAMES["depth_cam_frame"].GetPath().AppendChild(cname))
        cam.CreateFocalLengthAttr(focal)
        cam.CreateHorizontalApertureAttr(h_ap)
        cam.CreateVerticalApertureAttr(v_ap)
        cam.CreateClippingRangeAttr(Gf.Vec2f(*clip))
        cam.CreateProjectionAttr("perspective")
        set_local_pose(cam.GetPrim(), (0, 0, 0), Gf.Quatd(0.0, Gf.Vec3d(1.0, 0.0, 0.0)))
        # exposure like the Kit viewport camera (f/5, 1/50 s, ISO 100) + RTX auto exposure, as the real
        # camera auto-exposes. The schema default (1 s) gives completely over-exposed images.
        cp = cam.GetPrim()
        cp.ApplyAPI("OmniRtxCameraExposureAPI_1")
        cp.ApplyAPI("OmniRtxCameraAutoExposureAPI_1")
        for n, v in (("exposure:fStop", 5.0), ("exposure:time", 0.02), ("exposure:iso", 100.0),
                     ("exposure:responsivity", 1.1026709)):
            set_attr(cp, n, v, Sdf.ValueTypeNames.Float)
        set_attr(cp, "omni:rtx:autoExposure:enabled", True, Sdf.ValueTypeNames.Bool)
        set_attr(cp, "omni:sensor:tickRate", float(cc["fps"]), Sdf.ValueTypeNames.Float)  # 15 fps like the Aurora
        CAMS[cname] = str(cam.GetPath())
    REPORT["camera"] = {"focal_length": focal, "h_aperture": h_ap, "v_aperture": v_ap,
                        "hfov_deg": cc["hfov_deg"], "vfov_deg": math.degrees(2 * math.atan(v_ap / (2 * focal)))}

    # 4c. MPU6050 IMU on imu_link
    from isaacsim.sensors.experimental.physics import IMU

    imu = IMU(str(FRAMES["imu_link"].GetPath().AppendChild("imu_sensor")), translations=[[0, 0, 0]],
              orientations=[[1, 0, 0, 0]])
    IMU_PATH = str(FRAMES["imu_link"].GetPath().AppendChild("imu_sensor"))
    check("sensors created", stage.GetPrimAtPath(LIDAR_PATH).IsValid() and stage.GetPrimAtPath(IMU_PATH).IsValid()
          and all(stage.GetPrimAtPath(p).IsValid() for p in CAMS.values()), "%s, %s, %s" % (LIDAR_PATH, IMU_PATH, list(CAMS.values())))

    # ------------------------------------------------------------------------------------------------
    # 5. OmniGraph: drive + ROS 2 interface
    # ------------------------------------------------------------------------------------------------

    wheels = {w["joint"]: w for w in BUILD["wheels"]}
    geo = []
    for wn in wheel_names:
        w = wheels[wn]
        ring = np.mean([r["center"] for r in w["rollers"]], axis=0)       # mid-width of the roller ring
        bottom = min(w["rollers"], key=lambda r: r["center"][2])           # roller touching the ground
        ux, uy = bottom["axis"][0], bottom["axis"][1]
        if ux < 0:
            ux, uy = -ux, -uy
        geo.append((w["xyz"][0] + ring[0], w["xyz"][1] + ring[1], ux, uy, w["outer_radius"]))
    IK = ros_graphs.mecanum_ik_matrix(geo)
    REPORT["mecanum_ik"] = {"wheels": wheel_names, "contact_x_y_ux_uy_r": geo, "matrix_rows_vx_vy_wz": IK[:3]}

    if not args.no_graphs:
        gscope = UsdGeom.Scope.Define(stage, RP.AppendChild("Graphs")).GetPath()
        art_root = str(ART_ROOT.GetPath())
        ros_graphs.build_drive_graph(
            str(gscope.AppendChild("MecanumDrive")), art_root, wheel_names, IK,
            cmd_topics=["cmd_vel", "controller/cmd_vel", "app/cmd_vel"])
        ros_graphs.build_arm_graph(str(gscope.AppendChild("ArmGripper")), art_root, "joint_command")
        # /tf with the URDF's parent/child structure (what robot_state_publisher publishes on the real robot)
        P = {**{k: str(v.GetPath()) for k, v in BODIES.items()}, **{k: str(v.GetPath()) for k, v in FRAMES.items()}}
        tf_tree = [
            ("base_footprint", ["base_link"]),
            ("base_link", ["lidar_frame", "imu_link", "link1", "left_front_wheel_link", "left_back_wheel_link",
                           "right_front_wheel_link", "right_back_wheel_link"]),
            ("link1", ["link2"]), ("link2", ["link3"]), ("link3", ["link4"]),
            ("link4", ["camera_link0", "link5"]), ("camera_link0", ["camera_link"]), ("camera_link", ["depth_cam_frame"]),
            ("link5", ["gripper_link", "end_effector_link"]),
            ("gripper_link", ["l_link", "r_link", "l_in_link", "r_in_link"]),
            ("l_in_link", ["l_out_link"]), ("r_in_link", ["r_out_link"]),
        ]
        for parent, kids in tf_tree:  # every pair must be a URDF joint
            for k in kids:
                assert JOINTS[CHILD_JOINT[k]]["parent"] == parent, (parent, k)
        tf_groups = [(P[parent], [P[k] for k in kids]) for parent, kids in tf_tree]
        ros_graphs.build_state_graph(
            str(gscope.AppendChild("RobotState")), art_root, art_root, "base_footprint", tf_groups,
            IMU_PATH, scfg["imu"]["frame_id"],
            joint_state_topics=["joint_states", "controller_manager/joint_states"],
            odom_topics=["odom", "odom_raw"],
            imu_topics=[("imu", True), ("imu_corrected", False), ("ros_robot_controller/imu_raw", False)])
        ros_graphs.build_lidar_graph(str(gscope.AppendChild("Lidar")), LIDAR_PATH, lc["frame_id"], ["scan", "scan_raw"])
        ros_graphs.build_camera_graph(
            str(gscope.AppendChild("DepthCamera")), CAMS["rgb_camera"], CAMS["depth_camera"], cc["width"], cc["height"],
            cc["frame_id"],
            {"rgb": "depth_cam/rgb/image_raw", "rgb_info": "depth_cam/rgb/camera_info",
             "depth": "depth_cam/depth0/image_raw", "depth_info": "depth_cam/depth0/camera_info",
             "points": "depth_cam/depth0/points", "ir": "depth_cam/iro/image_raw"})
        check("graphs created", True, str([p.GetName() for p in stage.GetPrimAtPath(gscope).GetChildren()]))

    # robot metadata
    ROOT.SetCustomDataByKey("rosorin:source", "RosOrinPro-IsaacSim scripts/configure_robot.py")
    ROOT.SetCustomDataByKey("rosorin:lidar_variant", BUILD["lidar_variant"])
    stage.GetRootLayer().customLayerData = dict(stage.GetRootLayer().customLayerData,
                                                creator="RosOrinPro-IsaacSim configure_robot.py")
    stage.Save()
    REPORT["output"] = args.out
    REPORT["renamed"] = len(renamed)
    with open(os.path.join(REPO, "usd", "configure_report.json"), "w", encoding="utf-8") as f:
        json.dump(REPORT, f, indent=2)
    fails = [c for c in REPORT["checks"] if not c["pass"]]
    print("CONFIGURE DONE", args.out, "fails:", len(fails), flush=True)


try:
    main()
except Exception:  # make failures visible to build_all.bat (Kit would otherwise exit with code 0)
    import traceback

    traceback.print_exc()
    sys.stdout.flush()
    os._exit(1)
app.close()

"""
build_rosorinpro_blender.py
===========================

Builds the Hiwonder ROSOrin Pro (mecanum chassis + 5-DOF arm + gripper + depth camera + lidar)
in Blender from the `rosorin_description` URDF/Xacro package, verifies the result against the
URDF, saves a .blend, exports an Unreal-ready FBX and writes a text report + JSON joint map.

The URDF/Xacro files are the single source of truth. Nothing is placed by eye:
  * every link  -> one Blender object named exactly like the URDF link
  * every joint -> parent/child relation + the joint <origin> as the child's local transform
  * visual meshes are imported from the STL files referenced by the URDF (meters, no rescale)

Run inside Blender (tested with Blender 5.2 LTS), either
  * from the Blender MCP / Python console:
        ROSORIN_CFG = {...}; exec(open(r"<this file>").read())
  * or headless:
        blender -b --factory-startup --python build_rosorinpro_blender.py -- --pkg <rosorin_description> --out <dir>

Coordinate frames
-----------------
Blender scene == ROS: right-handed, +X forward, +Y left, +Z up, 1 Blender unit = 1 m.
Unreal:            left-handed,  +X forward, +Y right, +Z up, 1 uu = 1 cm.
The FBX is written in centimeters with Z-up / -Y-front axis metadata, which is exactly the
axis system Unreal's FBX importer converts to, so Unreal maps (x, y, z)_ROS [m] to
(100x, -100y, 100z)_UE [cm] without adding any root rotation or scale.
"""

import bpy
import os
import re
import sys
import math
import json
import struct
import hashlib
import datetime
import xml.etree.ElementTree as ET

import numpy as np
from mathutils import Matrix, Vector, Euler, Quaternion

# --------------------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------------------

_HERE = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()

DEFAULT_CFG = {
    # rosorin_description package root (the folder that contains urdf/ and meshes/)
    "pkg_dir": os.path.normpath(os.path.join(_HERE, "..", "..", "rosorin_description")),
    # where ROSOrinPro_Unreal.blend / .fbx / report are written
    "out_dir": os.path.normpath(os.path.join(_HERE, "..", "..")),
    # Environment variables the xacro files read with $(env ...). MACHINE_TYPE selects the Pro
    # model in urdf/rosorin.xacro. LIDAR_TYPE is NOT defined anywhere in the package; MS200,
    # LD19 and SCLIDAR only differ in the lidar_joint yaw (0, 3.1416, pi) - see report.
    "env": {"MACHINE_TYPE": "ROSOrin_Mecanum_Pro", "LIDAR_TYPE": "MS200"},
    "entry_xacro": "urdf/rosorin.xacro",
    # The Pro URDF has no wheel links (the wheels are baked into meshes/pro/mecanum/base_link.stl).
    # When True, the four wheel assemblies are split out of base_link.stl losslessly and attached
    # with the wheel frames defined in urdf/rosorin_old.xacro (same chassis, see report).
    "split_wheels": True,
    "wheel_frames_xacro": "urdf/rosorin_old.xacro",
    "wheel_axis": (0.0, 1.0, 0.0),        # rotation axis in the wheel frame (lateral axis)
    "wheel_radial_max": 0.045,            # m, parts within this radius of the wheel axis ...
    "wheel_inner_tolerance": 0.0005,      # ... and outboard of the wheel frame plane
    "collection": "ROSOrinPro",
    "blend_name": "ROSOrinPro_Unreal.blend",
    "fbx_name": "ROSOrinPro_Unreal.fbx",
    "report_name": "ROSOrinPro_Export_Report.txt",
    "joint_map_name": "ROSOrinPro_joint_map.json",
    "urdf_out_name": "rosorinpro_resolved.urdf",
    "overwrite": False,                   # refuse to overwrite existing output files
    "smooth_angle_deg": 30.0,
    "empty_size": 0.02,
    "delete_default_objects": ("Cube", "Camera", "Light"),
}

TOL_POS = 1e-6       # m
TOL_ROT = 1e-6       # rad
TOL_AXIS = 1e-5      # rad (angle between expected and measured axis)

XNS = "{http://ros.org/wiki/xacro}"


def get_cfg():
    cfg = dict(DEFAULT_CFG)
    cfg["env"] = dict(DEFAULT_CFG["env"])
    user = globals().get("ROSORIN_CFG") or {}
    for k, v in user.items():
        if k == "env":
            cfg["env"].update(v)
        else:
            cfg[k] = v
    if "--" in sys.argv:
        argv = sys.argv[sys.argv.index("--") + 1:]
        it = iter(argv)
        for a in it:
            if a == "--pkg":
                cfg["pkg_dir"] = next(it)
            elif a == "--out":
                cfg["out_dir"] = next(it)
            elif a == "--lidar":
                cfg["env"]["LIDAR_TYPE"] = next(it)
            elif a == "--overwrite":
                cfg["overwrite"] = True
            elif a == "--no-wheel-split":
                cfg["split_wheels"] = False
    cfg["pkg_dir"] = os.path.normpath(cfg["pkg_dir"])
    cfg["out_dir"] = os.path.normpath(cfg["out_dir"])
    return cfg


# --------------------------------------------------------------------------------------
# Minimal xacro processor (covers everything used by rosorin_description)
# --------------------------------------------------------------------------------------

def _fmt(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return repr(v)
    return str(v)


def _parse_val(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return s


class Xacro:
    def __init__(self, pkg_map, env, args=None):
        self.pkg_map = pkg_map
        self.env = env
        self.args = dict(args or {})
        self.props = {}
        self.macros = {}
        self.files = []
        self.log = []

    def _dollar(self, m):
        parts = m.group(1).split()
        cmd = parts[0]
        if cmd == "arg":
            return str(self.args[parts[1]])
        if cmd == "env":
            if parts[1] not in self.env:
                raise KeyError("$(env %s) is not set" % parts[1])
            return self.env[parts[1]]
        if cmd == "optenv":
            return self.env.get(parts[1], " ".join(parts[2:]))
        if cmd == "find":
            return self.pkg_map[parts[1]]
        raise ValueError("unsupported substitution $(%s)" % m.group(1))

    def eval(self, expr):
        ns = {"pi": math.pi, "math": math, "radians": math.radians, "degrees": math.degrees,
              "sin": math.sin, "cos": math.cos, "true": True, "false": False,
              "True": True, "False": False}
        ns.update(self.props)
        return eval(expr, {"__builtins__": {}}, ns)

    def subst(self, s):
        if s is None:
            return s
        s = re.sub(r"\$\(([^)]*)\)", self._dollar, s)
        return re.sub(r"\$\{([^}]*)\}", lambda m: _fmt(self.eval(m.group(1))), s)

    def cond(self, s):
        s = s.strip()
        m = re.fullmatch(r"\$\{(.*)\}", s)
        v = self.eval(m.group(1)) if m else self.subst(s)
        if isinstance(v, str):
            v = v.strip().lower()
            if v in ("true", "1"):
                return True
            if v in ("false", "0"):
                return False
            raise ValueError("invalid xacro condition %r" % s)
        return bool(v)

    def process_file(self, path, out):
        path = os.path.normpath(path)
        self.files.append(path)
        root = ET.parse(path).getroot()
        self.process_children(root, out)
        return root

    def process_children(self, elem, out):
        for ch in list(elem):
            tag = ch.tag
            if not isinstance(tag, str):
                continue
            if tag == XNS + "arg":
                name = ch.get("name")
                if name not in self.args:
                    self.args[name] = self.subst(ch.get("default", ""))
            elif tag == XNS + "property":
                if ch.get("value") is None:
                    raise NotImplementedError("block xacro:property is not supported")
                self.props[ch.get("name")] = _parse_val(self.subst(ch.get("value")))
            elif tag == XNS + "include":
                self.process_file(self.subst(ch.get("filename")), out)
            elif tag in (XNS + "if", XNS + "unless"):
                c = self.cond(ch.get("value"))
                if tag == XNS + "unless":
                    c = not c
                self.log.append("%s value=%r -> %s" % (tag[len(XNS):], ch.get("value"), c))
                if c:
                    self.process_children(ch, out)
            elif tag == XNS + "macro":
                self.macros[ch.get("name")] = ch
            elif tag.startswith(XNS):
                raise NotImplementedError("xacro element %s is not supported" % tag)
            else:
                n = ET.Element(tag, {k: self.subst(v) for k, v in ch.attrib.items()})
                n.text = self.subst(ch.text) if ch.text and ch.text.strip() else None
                self.process_children(ch, n)
                out.append(n)


def resolve_xacro(cfg):
    pkg_map = {"rosorin_description": cfg["pkg_dir"]}
    x = Xacro(pkg_map, cfg["env"])
    entry = os.path.join(cfg["pkg_dir"], cfg["entry_xacro"])
    robot = ET.Element("robot")
    root = x.process_file(entry, robot)
    robot.set("name", root.get("name", "robot"))
    return robot, x


# --------------------------------------------------------------------------------------
# URDF model
# --------------------------------------------------------------------------------------

def _vec(s, default):
    if s is None:
        return tuple(default)
    return tuple(float(t) for t in s.split())


def parse_urdf(robot):
    materials = {}
    for m in robot.findall("material"):
        c = m.find("color")
        if c is not None:
            materials[m.get("name")] = _vec(c.get("rgba"), (1, 1, 1, 1))
    links, joints = {}, {}
    for l in robot.findall("link"):
        name = l.get("name")
        if name in links:
            raise ValueError("duplicate link %s" % name)
        visuals = []
        for v in l.findall("visual"):
            o = v.find("origin")
            g = v.find("geometry")
            mesh = g.find("mesh") if g is not None else None
            if mesh is None:
                raise NotImplementedError("link %s: only <mesh> visuals are supported" % name)
            mat = v.find("material")
            rgba, mat_name = None, ""
            if mat is not None:
                mat_name = mat.get("name", "")
                c = mat.find("color")
                if c is not None:
                    rgba = _vec(c.get("rgba"), (1, 1, 1, 1))
                elif mat_name in materials:
                    rgba = materials[mat_name]
            visuals.append({
                "filename": mesh.get("filename"),
                "scale": _vec(mesh.get("scale"), (1, 1, 1)),
                "xyz": _vec(o.get("xyz") if o is not None else None, (0, 0, 0)),
                "rpy": _vec(o.get("rpy") if o is not None else None, (0, 0, 0)),
                "rgba": rgba or (0.8, 0.8, 0.8, 1.0),
                "material_name": mat_name,
            })
        collisions = []
        for c in l.findall("collision"):
            mesh = c.find("geometry/mesh")
            if mesh is not None:
                collisions.append(mesh.get("filename"))
        links[name] = {"name": name, "visuals": visuals, "collisions": collisions,
                       "has_inertial": l.find("inertial") is not None}
    for j in robot.findall("joint"):
        name = j.get("name")
        if name in joints:
            raise ValueError("duplicate joint %s" % name)
        o = j.find("origin")
        a = j.find("axis")
        lim = j.find("limit")
        mim = j.find("mimic")
        jt = j.get("type")
        joints[name] = {
            "name": name, "type": jt,
            "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "xyz": _vec(o.get("xyz") if o is not None else None, (0, 0, 0)),
            "rpy": _vec(o.get("rpy") if o is not None else None, (0, 0, 0)),
            "xyz_text": (o.get("xyz") if o is not None else "0 0 0"),
            "rpy_text": (o.get("rpy") if o is not None else "0 0 0"),
            # URDF default axis is (1,0,0); fixed joints ignore it
            "axis": _vec(a.get("xyz") if a is not None else None, (1, 0, 0)),
            "axis_text": (a.get("xyz") if a is not None else "(default 1 0 0)"),
            "limit": None if lim is None else {k: float(lim.get(k)) for k in
                                               ("lower", "upper", "effort", "velocity") if lim.get(k) is not None},
            "mimic": None if mim is None else {"joint": mim.get("joint"),
                                               "multiplier": float(mim.get("multiplier", 1.0)),
                                               "offset": float(mim.get("offset", 0.0))},
            "source": "URDF",
        }
    return materials, links, joints


def tree_order(links, joints):
    child_of = {j["child"]: j for j in joints.values()}
    roots = [l for l in links if l not in child_of]
    if len(roots) != 1:
        raise ValueError("URDF must have exactly one root link, found %s" % roots)
    children = {}
    for j in joints.values():
        children.setdefault(j["parent"], []).append(j)
    order, stack = [], [roots[0]]
    while stack:
        l = stack.pop()
        order.append(l)
        for j in sorted(children.get(l, []), key=lambda j: j["name"], reverse=True):
            stack.append(j["child"])
    return roots[0], order, child_of, children


# --------------------------------------------------------------------------------------
# Math helpers (numpy, independent of Blender for verification)
# --------------------------------------------------------------------------------------

def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    # URDF: fixed-axis roll(X), pitch(Y), yaw(Z)  ->  R = Rz(y) Ry(p) Rx(r)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def T_from(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def axis_angle_R(axis, ang):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def R_to_axis_angle(R):
    # atan2 form stays accurate for tiny angles (acos of the trace does not with float32 matrices)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = 0.5 * float(np.linalg.norm(v))
    ang = math.atan2(s, (np.trace(R) - 1) / 2)
    if s < 1e-15:
        return np.array([1.0, 0, 0]), ang
    return v / (2 * s), ang


def R_to_quat(R):
    q = Matrix([list(r) for r in R]).to_quaternion()
    return (q.w, q.x, q.y, q.z)


def mat_np(m):
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], float)


def angle_between(a, b):
    a = np.asarray(a, float) / np.linalg.norm(a)
    b = np.asarray(b, float) / np.linalg.norm(b)
    return math.acos(max(-1.0, min(1.0, float(a @ b))))


def ros_to_ue(xyz, rpy):
    """ROS pose (m, rad, rpy) -> Unreal relative location (cm), FRotator (deg) and FQuat."""
    x, y, z = xyz
    r, p, yw = rpy
    w, qx, qy, qz = R_to_quat(rpy_to_R(r, p, yw))
    return {"location_cm": [100.0 * x, -100.0 * y, 100.0 * z],
            "rotator_deg": {"pitch": -math.degrees(p), "yaw": -math.degrees(yw), "roll": math.degrees(r)},
            "quat_xyzw": [-qx, qy, -qz, w]}


def ros_axis_to_ue(axis):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    return [float(-a[0]), float(a[1]), float(-a[2])]


# --------------------------------------------------------------------------------------
# STL helpers
# --------------------------------------------------------------------------------------

def stl_info(path):
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack("<I", data[80:84])[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * n:
        arr = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
        v = arr["v"].reshape(-1, 3).astype(np.float64)
        kind = "binary"
    else:
        txt = data.decode("ascii", "ignore")
        v = np.array([[float(t) for t in l.split()[1:4]] for l in txt.splitlines() if l.strip().startswith("vertex")])
        n = len(v) // 3
        kind = "ascii"
    return {"kind": kind, "triangles": int(n), "bbox_min": v.min(0).tolist(), "bbox_max": v.max(0).tolist(),
            "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)}


def resolve_mesh_path(fn, pkg_map):
    if fn.startswith("package://"):
        pkg, rel = fn[len("package://"):].split("/", 1)
        return os.path.normpath(os.path.join(pkg_map[pkg], rel))
    if fn.startswith("file://"):
        return os.path.normpath(fn[len("file://"):])
    return os.path.normpath(fn)


def mesh_arrays(me):
    co = np.empty(len(me.vertices) * 3)
    me.vertices.foreach_get("co", co)
    lt = np.empty(len(me.polygons), dtype=np.int32)
    me.polygons.foreach_get("loop_total", lt)
    if len(lt) and not (lt == 3).all():
        raise ValueError("mesh %s is not triangulated" % me.name)
    vi = np.empty(len(me.loops), dtype=np.int32)
    me.loops.foreach_get("vertex_index", vi)
    return co.reshape(-1, 3), vi.reshape(-1, 3)


def mesh_from_arrays(name, co, tris):
    used, inv = np.unique(tris.reshape(-1), return_inverse=True)
    me = bpy.data.meshes.new(name)
    me.from_pydata(co[used].tolist(), [], inv.reshape(-1, 3).tolist())
    me.validate(clean_customdata=False)
    me.update()
    return me


def triangle_components(co, tris, weld=1e-6):
    """Connected components of a triangle soup (vertices welded on a 1 micron grid)."""
    q = np.round(co / weld).astype(np.int64)
    _, w = np.unique(q, axis=0, return_inverse=True)
    t = w.reshape(-1)[tris]
    lab = np.arange(t.max() + 1)
    e = np.concatenate([t[:, [0, 1]], t[:, [1, 2]]])
    while True:
        m = np.minimum(lab[e[:, 0]], lab[e[:, 1]])
        new = lab.copy()
        np.minimum.at(new, e[:, 0], m)
        np.minimum.at(new, e[:, 1], m)
        new = new[new]
        if np.array_equal(new, lab):
            break
        lab = new
    return lab[t[:, 0]]


# --------------------------------------------------------------------------------------
# Blender helpers
# --------------------------------------------------------------------------------------

def srgb_to_linear(c):
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def get_material(rgba, name_hint, cache):
    key = tuple(round(c, 6) for c in rgba)
    if key in cache:
        return cache[key]
    if name_hint:
        name = "M_" + name_hint
    else:
        name = "M_rgba_%02x%02x%02x" % tuple(int(round(c * 255)) for c in rgba[:3])
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    if mat.node_tree is None:           # Blender < 5 creates node-less materials
        mat.use_nodes = True
    nt = mat.node_tree
    bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        bsdf = nt.nodes.new("ShaderNodeBsdfPrincipled")
        outn = next((n for n in nt.nodes if n.type == "OUTPUT_MATERIAL"), None) or nt.nodes.new("ShaderNodeOutputMaterial")
        nt.links.new(bsdf.outputs[0], outn.inputs[0])
    lin = [srgb_to_linear(c) for c in rgba[:3]]
    sock = {s.identifier: s for s in bsdf.inputs}
    sock["Base Color"].default_value = (lin[0], lin[1], lin[2], 1.0)
    if "Roughness" in sock:
        sock["Roughness"].default_value = 0.5
    if "Metallic" in sock:
        sock["Metallic"].default_value = 0.0
    mat.diffuse_color = (lin[0], lin[1], lin[2], rgba[3])
    mat["urdf_rgba"] = " ".join("%g" % c for c in rgba)
    cache[key] = mat
    return mat


def shade(me, angle_deg):
    me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
    me.set_sharp_from_angle(angle=math.radians(angle_deg))
    me.update()


def import_stl(path, coll):
    before = set(bpy.data.objects)
    bpy.ops.wm.stl_import(filepath=path, global_scale=1.0, use_scene_unit=False,
                          forward_axis="Y", up_axis="Z", use_facet_normal=False, use_mesh_validate=True)
    new = [o for o in bpy.data.objects if o not in before]
    if len(new) != 1:
        raise RuntimeError("STL import of %s produced %d objects" % (path, len(new)))
    ob = new[0]
    for c in list(ob.users_collection):
        c.objects.unlink(ob)
    coll.objects.link(ob)
    return ob


def set_local(ob, xyz, rpy):
    ob.matrix_parent_inverse = Matrix.Identity(4)
    ob.rotation_mode = "XYZ"          # Blender XYZ Euler == URDF rpy (R = Rz*Ry*Rx)
    ob.location = Vector(xyz)
    ob.rotation_euler = Euler(rpy, "XYZ")
    ob.scale = (1.0, 1.0, 1.0)


def clean_scene(cfg, notes):
    removed = []
    for name in cfg["delete_default_objects"]:
        ob = bpy.data.objects.get(name)
        if ob is not None:
            data = ob.data
            bpy.data.objects.remove(ob, do_unlink=True)
            removed.append(name)
            if data is not None and data.users == 0:
                for coll in (bpy.data.meshes, bpy.data.cameras, bpy.data.lights):
                    if data.name in coll and coll[data.name] == data:
                        coll.remove(data)
                        break
    for m in list(bpy.data.materials):
        if m.users == 0 and not m.use_fake_user:
            bpy.data.materials.remove(m)
    # the startup file's "Collection" is empty once Cube/Camera/Light are gone
    dc = bpy.data.collections.get("Collection")
    if dc is not None and not dc.all_objects and not dc.children and removed:
        bpy.data.collections.remove(dc)
        removed.append("Collection (empty default collection)")
    # remove a previous build of this script (idempotent re-runs)
    old = bpy.data.collections.get(cfg["collection"])
    if old is not None:
        for ob in list(old.all_objects):
            data = ob.data
            bpy.data.objects.remove(ob, do_unlink=True)
            if isinstance(data, bpy.types.Mesh) and data.users == 0:
                bpy.data.meshes.remove(data)
        bpy.data.collections.remove(old)
        notes.append("removed previous '%s' collection" % cfg["collection"])
    for me in list(bpy.data.meshes):
        if me.users == 0:
            bpy.data.meshes.remove(me)
    for m in list(bpy.data.materials):
        if m.users == 0 and not m.use_fake_user:
            bpy.data.materials.remove(m)
    return removed


# --------------------------------------------------------------------------------------
# Wheels (derived from rosorin_old.xacro + geometry split of base_link.stl)
# --------------------------------------------------------------------------------------

def wheel_frames_from_source(cfg):
    path = os.path.join(cfg["pkg_dir"], cfg["wheel_frames_xacro"])
    root = ET.parse(path).getroot()          # plain XML: only <joint> elements are read
    frames = []
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        if not child.endswith("_wheel_link"):
            continue
        o = j.find("origin")
        frames.append({"joint": j.get("name"), "child": child, "parent": j.find("parent").get("link"),
                       "xyz": _vec(o.get("xyz"), (0, 0, 0)), "rpy": _vec(o.get("rpy"), (0, 0, 0)),
                       "xyz_text": o.get("xyz"), "rpy_text": o.get("rpy"),
                       "orig_type": j.get("type"), "orig_axis": (j.find("axis").get("xyz") if j.find("axis") is not None else None)})
    return frames, path


def split_wheels(base_ob, frames, cfg, coll):
    me = base_ob.data
    co, tris = mesh_arrays(me)
    comp = triangle_components(co, tris)
    ids = np.unique(comp)
    assign = np.full(len(tris), -1, dtype=np.int32)
    per_wheel = {i: {"components": 0, "triangles": 0} for i in range(len(frames))}
    for cid in ids:
        sel = np.nonzero(comp == cid)[0]
        v = co[tris[sel].reshape(-1)]
        hits = []
        for i, f in enumerate(frames):
            T = T_from(f["xyz"], f["rpy"])
            vl = (v - T[:3, 3]) @ T[:3, :3]                  # into wheel frame
            side = math.copysign(1.0, f["xyz"][1])
            outboard = (vl[:, 1] * side >= -cfg["wheel_inner_tolerance"]).all()
            radial = np.sqrt(vl[:, 0] ** 2 + vl[:, 2] ** 2).max()
            if outboard and radial <= cfg["wheel_radial_max"]:
                hits.append(i)
        if len(hits) > 1:
            raise RuntimeError("base_link component %d matches several wheels" % cid)
        if hits:
            assign[sel] = hits[0]
            per_wheel[hits[0]]["components"] += 1
            per_wheel[hits[0]]["triangles"] += len(sel)
    wheel_obs, geo = [], []
    for i, f in enumerate(frames):
        sel = np.nonzero(assign == i)[0]
        if len(sel) == 0:
            raise RuntimeError("no geometry found for %s" % f["child"])
        T = T_from(f["xyz"], f["rpy"])
        co_local = (co - T[:3, 3]) @ T[:3, :3]
        wme = mesh_from_arrays("SM_" + f["child"], co_local, tris[sel])
        ob = bpy.data.objects.new(f["child"], wme)
        coll.objects.link(ob)
        wheel_obs.append(ob)
        # geometry cross-check of the source frame: area-weighted centroid and symmetry axis
        tv = co_local[tris[sel]]
        cr = np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0])
        area = 0.5 * np.linalg.norm(cr, axis=1)
        cen = tv.mean(axis=1)
        c = (cen * area[:, None]).sum(0) / area.sum()
        d = cen - c
        cov = (d * area[:, None]).T @ d / area.sum()
        evals, evecs = np.linalg.eigh(cov)
        sym = evecs[:, 0]                                  # thinnest direction = wheel axis
        radius = np.sqrt(co_local[tris[sel].reshape(-1)][:, 0] ** 2 + co_local[tris[sel].reshape(-1)][:, 2] ** 2).max()
        vy = co_local[tris[sel].reshape(-1)][:, 1]
        # mecanum rollers = the loose parts that reach the outer radius; the one lowest in the wheel frame
        # touches the ground, and its axis (PCA) gives the roller direction used for wheel kinematics
        rollers = []
        for cid in np.unique(comp[sel]):
            rv = co_local[tris[comp == cid].reshape(-1)]
            if np.sqrt(rv[:, 0] ** 2 + rv[:, 2] ** 2).max() >= radius - 5e-4:
                rollers.append(rv)
        bottom = min(rollers, key=lambda rv: rv[:, 2].mean())
        dr = bottom - bottom.mean(0)
        u = np.linalg.eigh(dr.T @ dr)[1][:, 2]
        u_xy = np.array([u[0], u[1]]) / math.hypot(u[0], u[1])
        if u_xy[0] < 0:
            u_xy = -u_xy
        geo.append({"link": f["child"], "components": per_wheel[i]["components"], "triangles": int(len(sel)),
                    "rollers": len(rollers),
                    "bottom_roller_axis_xy": u_xy.tolist(),
                    "bottom_roller_angle_deg": math.degrees(math.atan2(u_xy[1], u_xy[0])),
                    # mean ground contact: straight below the wheel axis, mid-width of the wheel
                    "contact_xy_m": [float(f["xyz"][0]), float(f["xyz"][1] + 0.5 * (vy.max() + vy.min()))],
                    "centroid_offset_xz_m": [float(c[0]), float(c[2])],
                    "centroid_offset_norm_m": float(math.hypot(c[0], c[2])),
                    "symmetry_axis": sym.tolist(),
                    "symmetry_axis_angle_to_Y_deg": math.degrees(angle_between(np.abs(sym), [0, 1, 0])),
                    "outer_radius_m": float(radius),
                    "width_m": float(vy.max() - vy.min()),
                    "hub_center_y_m": float(f["xyz"][1] + 0.5 * (vy.max() + vy.min()))})
    rest = np.nonzero(assign < 0)[0]
    new_base = mesh_from_arrays(me.name + "_tmp", co, tris[rest])
    old = base_ob.data
    old_name = old.name
    base_ob.data = new_base
    bpy.data.meshes.remove(old)
    new_base.name = old_name
    return wheel_obs, geo, {"components_total": int(len(ids)), "triangles_total": int(len(tris)),
                            "triangles_base_after": int(len(rest))}


# --------------------------------------------------------------------------------------
# Build
# --------------------------------------------------------------------------------------

def build(cfg):
    notes, warnings = [], []
    t0 = datetime.datetime.now()
    pkg_map = {"rosorin_description": cfg["pkg_dir"]}

    out_paths = {k: os.path.join(cfg["out_dir"], cfg[k]) for k in
                 ("blend_name", "fbx_name", "report_name", "joint_map_name", "urdf_out_name")}
    existing = [p for p in out_paths.values() if os.path.exists(p)]
    if existing and not cfg["overwrite"]:
        raise FileExistsError("refusing to overwrite existing output files: %s" % existing)

    # ---- 1. resolve + parse URDF -------------------------------------------------------
    robot, xa = resolve_xacro(cfg)
    materials, links, joints = parse_urdf(robot)
    urdf_links = set(links)
    root_link, order, child_of, children = tree_order(links, joints)

    # ---- 2. clean scene -----------------------------------------------------------------
    scene = bpy.context.scene
    removed = clean_scene(cfg, notes)
    scene.unit_settings.system = "METRIC"
    scene.unit_settings.scale_length = 1.0
    scene.unit_settings.length_unit = "METERS"
    coll = bpy.data.collections.new(cfg["collection"])
    scene.collection.children.link(coll)
    layer_coll = next(lc for lc in bpy.context.view_layer.layer_collection.children if lc.collection == coll)
    bpy.context.view_layer.active_layer_collection = layer_coll

    # ---- 3. create link objects ---------------------------------------------------------
    objs, mesh_map, mat_cache, stl_meta = {}, {}, {}, {}
    for lname in order:
        L = links[lname]
        if not L["visuals"]:
            ob = bpy.data.objects.new(lname, None)
            ob.empty_display_type = "ARROWS"
            ob.empty_display_size = cfg["empty_size"]
            coll.objects.link(ob)
        else:
            parts = []
            for v in L["visuals"]:
                path = resolve_mesh_path(v["filename"], pkg_map)
                if not os.path.isfile(path):
                    raise FileNotFoundError("link %s references missing mesh %s (%s)" % (lname, v["filename"], path))
                stl_meta[path] = stl_info(path)
                p = import_stl(path, coll)
                Tv = Matrix(T_from(v["xyz"], v["rpy"]).tolist()) @ Matrix.Diagonal((*v["scale"], 1.0))
                if Tv != Matrix.Identity(4):
                    p.data.transform(Tv)          # bake visual <origin>/scale so the object origin stays = link frame
                p.data.materials.clear()
                p.data.materials.append(get_material(v["rgba"], v["material_name"], mat_cache))
                parts.append(p)
                mesh_map.setdefault(lname, []).append({
                    "urdf_filename": v["filename"], "path": path,
                    "visual_xyz": v["xyz"], "visual_rpy": v["rpy"], "scale": v["scale"],
                    "rgba": v["rgba"], "material": p.data.materials[0].name})
            ob = parts[0]
            if len(parts) > 1:
                with bpy.context.temp_override(active_object=ob, selected_editable_objects=parts, object=ob):
                    bpy.ops.object.join()
            ob.name = lname
            # mesh datablock -> Unreal static mesh asset name (SM_ prefix avoids Interchange '_001' suffixes
            # caused by the scene node and the mesh sharing one name)
            ob.data.name = "SM_" + lname
        if ob.name != lname:
            raise RuntimeError("object name clash: wanted %s got %s" % (lname, ob.name))
        objs[lname] = ob

    # ---- 4. wheels ----------------------------------------------------------------------
    wheel_info = None
    extra_joints = {}
    if cfg["split_wheels"]:
        frames, src = wheel_frames_from_source(cfg)
        if len(frames) != 4:
            raise RuntimeError("expected 4 wheel frames in %s, found %d" % (src, len(frames)))
        for f in frames:
            if f["parent"] not in objs:
                raise RuntimeError("wheel parent %s not in the Pro URDF" % f["parent"])
        base_mat = objs["base_link"].data.materials[0]
        wheel_obs, geo, split_stats = split_wheels(objs["base_link"], frames, cfg, coll)
        for f, ob in zip(frames, wheel_obs):
            objs[f["child"]] = ob
            extra_joints[f["joint"]] = {
                "name": f["joint"], "type": "continuous", "parent": f["parent"], "child": f["child"],
                "xyz": f["xyz"], "rpy": f["rpy"], "xyz_text": f["xyz_text"], "rpy_text": f["rpy_text"],
                "axis": tuple(cfg["wheel_axis"]), "axis_text": "%g %g %g (derived)" % tuple(cfg["wheel_axis"]),
                "limit": None, "mimic": None,
                "source": "DERIVED: frame from %s (type=%s axis=%s there); geometry split from base_link.stl"
                          % (cfg["wheel_frames_xacro"], f["orig_type"], f["orig_axis"])}
        # base_link mesh was rebuilt from arrays: re-assign its URDF material, wheels share it
        for ob in wheel_obs + [objs["base_link"]]:
            ob.data.materials.clear()
            ob.data.materials.append(base_mat)
        wheel_info = {"source": src, "frames": frames, "geometry_check": geo, "split": split_stats}

    all_joints = dict(joints)
    all_joints.update(extra_joints)

    # ---- 5. hierarchy + joint origins ---------------------------------------------------
    for jn, J in all_joints.items():
        ch, par = objs[J["child"]], objs[J["parent"]]
        ch.parent = par
        set_local(ch, J["xyz"], J["rpy"])
    root = objs[root_link]
    root.parent = None
    set_local(root, (0, 0, 0), (0, 0, 0))

    for ob in objs.values():
        if ob.type == "MESH":
            shade(ob.data, cfg["smooth_angle_deg"])

    # ---- 6. metadata --------------------------------------------------------------------
    for lname, ob in objs.items():
        ob["ros_link"] = lname
        ob["ros_in_urdf"] = lname in urdf_links
        J = next((j for j in all_joints.values() if j["child"] == lname), None)
        if J is None:
            ob["ros_role"] = "root (drive from /odom -> %s)" % lname
            continue
        ob["ros_parent_link"] = J["parent"]
        ob["ros_joint"] = J["name"]
        ob["ros_joint_type"] = J["type"]
        ob["ros_joint_origin_xyz"] = J["xyz_text"]
        ob["ros_joint_origin_rpy"] = J["rpy_text"]
        if J["type"] != "fixed":
            ob["ros_joint_axis"] = "%s" % " ".join("%.9g" % a for a in J["axis"])
            ob["ue_joint_axis"] = " ".join("%.9g" % a for a in ros_axis_to_ue(J["axis"]))
        if J["limit"]:
            for k, v in J["limit"].items():
                ob["ros_limit_" + k] = v
        if J["mimic"]:
            ob["ros_mimic_joint"] = J["mimic"]["joint"]
            ob["ros_mimic_multiplier"] = J["mimic"]["multiplier"]
            ob["ros_mimic_offset"] = J["mimic"]["offset"]
        if lname in mesh_map:
            ob["ros_mesh"] = "; ".join(m["urdf_filename"] for m in mesh_map[lname])
        ob["ros_source"] = J["source"]
    if wheel_info:
        for g in wheel_info["geometry_check"]:
            objs[g["link"]]["ros_mesh"] = "split from package://rosorin_description/meshes/pro/mecanum/base_link.stl"
            objs[g["link"]]["wheel_radius_m"] = g["outer_radius_m"]
    root["ros_robot_name"] = robot.get("name")
    root["ros_env"] = json.dumps(cfg["env"])

    bpy.context.view_layer.update()
    return {"robot": robot, "xacro": xa, "materials": materials, "links": links, "joints": joints,
            "all_joints": all_joints, "extra_joints": extra_joints, "root": root_link, "order": order,
            "objs": objs, "mesh_map": mesh_map, "stl_meta": stl_meta, "wheel_info": wheel_info,
            "removed_defaults": removed, "notes": notes, "warnings": warnings, "out_paths": out_paths,
            "t0": t0}


# --------------------------------------------------------------------------------------
# Verification
# --------------------------------------------------------------------------------------

def fk_world(all_joints, root_link):
    Tw = {root_link: np.eye(4)}
    by_parent = {}
    for j in all_joints.values():
        by_parent.setdefault(j["parent"], []).append(j)
    stack = [root_link]
    while stack:
        p = stack.pop()
        for j in by_parent.get(p, []):
            Tw[j["child"]] = Tw[p] @ T_from(j["xyz"], j["rpy"])
            stack.append(j["child"])
    return Tw


def verify(B, cfg):
    objs, links, joints, all_joints = B["objs"], B["links"], B["joints"], B["all_joints"]
    res, fails = [], []

    def check(name, ok, detail):
        res.append(("PASS" if ok else "FAIL", name, detail))
        if not ok:
            fails.append(name)

    coll = bpy.data.collections[cfg["collection"]]
    robot_obs = list(coll.all_objects)

    # 1. meshes -----------------------------------------------------------------------
    referenced = {}
    for l in links.values():
        for v in l["visuals"]:
            referenced.setdefault(resolve_mesh_path(v["filename"], {"rosorin_description": cfg["pkg_dir"]}), []).append(l["name"])
    missing = [p for p in referenced if not os.path.isfile(p)]
    check("all referenced visual mesh files exist", not missing, "%d referenced, missing=%s" % (len(referenced), missing))
    not_imported = [ln for ln, l in links.items() if l["visuals"] and (ln not in objs or objs[ln].type != "MESH")]
    check("every URDF visual mesh imported onto its link object", not not_imported,
          "%d mesh links, not imported=%s" % (sum(1 for l in links.values() if l["visuals"]), not_imported))
    tri_rows = []
    split_links = {g["link"] for g in (B["wheel_info"]["geometry_check"] if B["wheel_info"] else [])}
    for path, lnames in referenced.items():
        ln = lnames[0]
        n_stl = B["stl_meta"][path]["triangles"]
        n_obj = len(objs[ln].data.polygons)
        if ln == "base_link" and split_links:
            n_obj += sum(len(objs[w].data.polygons) for w in split_links)
        tri_rows.append((ln, os.path.relpath(path, cfg["pkg_dir"]), n_stl, n_obj))
    bad_tris = [r for r in tri_rows if r[2] != r[3]]
    check("triangle count of every imported mesh == STL file (wheels included for base_link)", not bad_tris,
          "mismatches=%s" % bad_tris)

    # bbox of imported geometry in link frame vs raw STL (proves unit/scale untouched)
    bb_err = 0.0
    for path, lnames in referenced.items():
        ln = lnames[0]
        me = objs[ln].data
        co = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", co)
        co = co.reshape(-1, 3)
        if ln == "base_link" and split_links:
            parts = [co]
            for w in split_links:
                wm = objs[w].data
                wc = np.empty(len(wm.vertices) * 3)
                wm.vertices.foreach_get("co", wc)
                wc = wc.reshape(-1, 3)
                parts.append(wc + np.array(objs[w].location))
            co = np.concatenate(parts)
        smin, smax = np.array(B["stl_meta"][path]["bbox_min"]), np.array(B["stl_meta"][path]["bbox_max"])
        bb_err = max(bb_err, float(np.abs(co.min(0) - smin).max()), float(np.abs(co.max(0) - smax).max()))
    check("imported vertex bounds == STL bounds (1:1 scale, meters)", bb_err < 1e-6, "max deviation %.3g m" % bb_err)

    # 2. hierarchy ---------------------------------------------------------------------
    expected = set(links) | set(B["extra_joints"][j]["child"] for j in B["extra_joints"])
    names = {o.name for o in robot_obs}
    check("object set == URDF links (+ documented derived wheel links)", names == expected,
          "objects=%d expected=%d extra=%s missing=%s" % (len(names), len(expected), sorted(names - expected), sorted(expected - names)))
    wrong_parent = [(j["child"], objs[j["child"]].parent.name if objs[j["child"]].parent else None, j["parent"])
                    for j in all_joints.values() if objs[j["child"]].parent is None or objs[j["child"]].parent.name != j["parent"]]
    check("every child object parented to its URDF parent link", not wrong_parent, "wrong=%s" % wrong_parent)
    check("single root = %s with no parent" % B["root"], objs[B["root"]].parent is None and
          sum(1 for o in robot_obs if o.parent is None) == 1, "roots=%s" % [o.name for o in robot_obs if o.parent is None])
    bad_pinv = [o.name for o in robot_obs if (mat_np(o.matrix_parent_inverse) - np.eye(4)).__abs__().max() > 1e-9]
    check("no hidden parent-inverse offsets", not bad_pinv, "objects with non-identity parent inverse: %s" % bad_pinv)

    # 3. pivots / transforms vs independent FK ----------------------------------------
    bpy.context.view_layer.update()
    Tw = fk_world(all_joints, B["root"])
    pos_err, rot_err, worst = 0.0, 0.0, ("", 0.0)
    for ln, T in Tw.items():
        M = mat_np(objs[ln].matrix_world)
        e = float(np.linalg.norm(M[:3, 3] - T[:3, 3]))
        r = R_to_axis_angle(M[:3, :3].T @ T[:3, :3])[1]
        if e > worst[1]:
            worst = (ln, e)
        pos_err, rot_err = max(pos_err, e), max(rot_err, r)
    check("joint pivots (object origins) == URDF joint origins (FK)", pos_err < TOL_POS,
          "max position error %.3g m (worst %s)" % (pos_err, worst[0]))
    check("link orientations == URDF FK", rot_err < TOL_ROT, "max rotation error %.3g rad" % rot_err)

    # 4. rotation axes: rotate every movable joint and measure the world motion --------
    axis_rows, ax_err, piv_err = [], 0.0, 0.0
    test = 0.35
    for jn, J in all_joints.items():
        if J["type"] == "fixed":
            continue
        ob = objs[J["child"]]
        rest = ob.matrix_basis.copy()
        W0 = mat_np(ob.matrix_world)
        Rj = np.eye(4)
        Rj[:3, :3] = axis_angle_R(J["axis"], test)
        ob.matrix_basis = Matrix((mat_np(rest) @ Rj).tolist())
        bpy.context.view_layer.update()
        W1 = mat_np(ob.matrix_world)
        D = W1 @ np.linalg.inv(W0)
        a_meas, ang = R_to_axis_angle(D[:3, :3])
        a_exp = Tw[J["child"]][:3, :3] @ (np.asarray(J["axis"]) / np.linalg.norm(J["axis"]))
        e_ax = angle_between(a_meas, a_exp)
        pivot = Tw[J["child"]][:3, 3]
        e_piv = float(np.linalg.norm(D[:3, :3] @ pivot + D[:3, 3] - pivot))
        set_local(ob, J["xyz"], J["rpy"])      # restore the literal URDF rpy (not a re-decomposed Euler)
        bpy.context.view_layer.update()
        ax_err, piv_err = max(ax_err, e_ax), max(piv_err, e_piv)
        axis_rows.append((jn, J["child"], a_exp.tolist(), e_ax, abs(ang - test), e_piv))
    check("rotation axes == URDF <axis> (measured by rotating each joint %.2f rad)" % test, ax_err < TOL_AXIS,
          "max axis error %.3g rad over %d movable joints" % (ax_err, len(axis_rows)))
    check("joints rotate about their own origin (pivot fixed while rotating)", piv_err < TOL_POS,
          "max pivot drift %.3g m" % piv_err)
    rest_err = max(float(np.abs(mat_np(objs[ln].matrix_world) - Tw[ln]).max()) for ln in Tw)
    check("rest pose restored after axis test", rest_err < 1e-6, "max deviation %.3g" % rest_err)

    # 5. scale ----------------------------------------------------------------------------
    us = bpy.context.scene.unit_settings
    check("scene units metric, 1 BU = 1 m", us.system == "METRIC" and abs(us.scale_length - 1.0) < 1e-12,
          "system=%s scale_length=%s length_unit=%s" % (us.system, us.scale_length, us.length_unit))
    bad_scale = [o.name for o in robot_obs if (np.abs(np.array(o.scale) - 1) > 1e-9).any()]
    bad_ws = [o.name for o in robot_obs if abs(abs(np.linalg.det(mat_np(o.matrix_world)[:3, :3])) - 1) > 1e-6]
    check("every object scale == 1 (local and world)", not bad_scale and not bad_ws, "bad=%s %s" % (bad_scale, bad_ws))
    allv = []
    for o in robot_obs:
        if o.type == "MESH":
            M = mat_np(o.matrix_world)
            co = np.empty(len(o.data.vertices) * 3)
            o.data.vertices.foreach_get("co", co)
            co = co.reshape(-1, 3)
            allv.append(co @ M[:3, :3].T + M[:3, 3])
    allv = np.concatenate(allv)
    dims = allv.max(0) - allv.min(0)
    plausible = 0.25 < dims[0] < 0.35 and 0.18 < dims[1] < 0.25 and 0.30 < dims[2] < 0.60
    check("overall robot size physically plausible (m)", plausible,
          "L x W x H = %.4f x %.4f x %.4f m, lowest point z=%.4f m above base_footprint" % (dims[0], dims[1], dims[2], allv.min(0)[2]))

    # 6. duplicates -----------------------------------------------------------------------
    dup_suffix = [o.name for o in bpy.data.objects if re.search(r"\.\d{3}$", o.name)]
    check("no '.001'-style duplicate object names", not dup_suffix, "found=%s" % dup_suffix)
    stray = [o.name for o in bpy.data.objects if o.name not in names]
    check("no objects outside the robot (default cube/camera/light removed)", not stray, "stray=%s" % stray)
    shared = [m.name for m in bpy.data.meshes if m.users > 1]
    check("no shared/duplicated mesh datablocks", not shared and len(bpy.data.meshes) == sum(1 for o in robot_obs if o.type == "MESH"),
          "meshes=%d mesh objects=%d shared=%s" % (len(bpy.data.meshes), sum(1 for o in robot_obs if o.type == "MESH"), shared))
    hashes = {}
    for o in robot_obs:
        if o.type == "MESH":
            co = np.empty(len(o.data.vertices) * 3)
            o.data.vertices.foreach_get("co", co)
            h = hashlib.md5(np.round(co, 7).tobytes() + np.round(mat_np(o.matrix_world), 7).tobytes()).hexdigest()
            hashes.setdefault(h, []).append(o.name)
    dup_geo = [v for v in hashes.values() if len(v) > 1]
    check("no two objects with identical geometry at the same place", not dup_geo, "duplicates=%s" % dup_geo)

    bme = objs["base_link"].data
    bco = np.empty(len(bme.vertices) * 3)
    bme.vertices.foreach_get("co", bco)
    bco = bco.reshape(-1, 3)
    return {"results": res, "fails": fails, "tri_rows": tri_rows, "axis_rows": axis_rows, "dims": dims.tolist(),
            "lowest_z": float(allv.min(0)[2]), "fk": Tw, "base_link_dims_m": (bco.max(0) - bco.min(0)).tolist()}


# --------------------------------------------------------------------------------------
# Export + FBX verification
# --------------------------------------------------------------------------------------

FBX_SETTINGS = dict(
    use_selection=True,
    use_visible=False,
    use_active_collection=False,
    object_types={"EMPTY", "MESH"},
    global_scale=1.0,
    apply_unit_scale=True,
    apply_scale_options="FBX_SCALE_NONE",
    use_space_transform=True,
    # 'Apply Transform' stays OFF: it corrupts nested hierarchies (alternating 0.01/1 scales, huge
    # translations, seen with Blender 5.2). Centimeters are written natively instead, see export_fbx().
    bake_space_transform=False,
    axis_forward="Y",
    axis_up="Z",
    use_mesh_modifiers=True,
    mesh_smooth_type="FACE",
    use_subsurf=False,
    use_mesh_edges=False,
    use_tspace=False,
    use_triangles=True,
    use_custom_props=True,
    colors_type="NONE",
    add_leaf_bones=False,
    bake_anim=False,
    path_mode="AUTO",
    embed_textures=False,
    batch_mode="OFF",
    use_metadata=True,
)


def export_fbx(B, cfg, path):
    """Write the FBX in centimeters without any node scale.

    For the duration of the export the scene is switched to 1 BU = 1 cm (unit scale 0.01) and every
    vertex and local translation is multiplied by 100. With apply_unit_scale the exporter's unit factor
    is then 100 * 0.01 = 1, so the global matrix is identity: the file stores plain centimeter values,
    UnitScaleFactor = 1 and every node scale = 1. Afterwards the original float32 vertex and location
    values are restored bit-exactly (the .blend is saved before this runs).
    """
    scene = bpy.context.scene
    us = scene.unit_settings
    objs = list(B["objs"].values())
    settings = dict(FBX_SETTINGS)
    # validate enum identifiers against the installed exporter before calling it
    rna = bpy.ops.export_scene.fbx.get_rna_type().properties
    for k, v in settings.items():
        p = rna[k]
        if p.type == "ENUM":
            ids = {i.identifier for i in p.enum_items}
            vals = v if isinstance(v, set) else {v}
            if not vals <= ids:
                raise ValueError("FBX option %s=%s not in %s" % (k, v, ids))
    saved_co, saved_loc, saved_scale_length = {}, {}, us.scale_length
    try:
        for ob in objs:
            saved_loc[ob.name] = ob.location.copy()
            ob.location = ob.location * 100.0
            if ob.type == "MESH" and ob.data.name not in saved_co:
                co = np.empty(len(ob.data.vertices) * 3, dtype=np.float32)
                ob.data.vertices.foreach_get("co", co)
                saved_co[ob.data.name] = co
                ob.data.vertices.foreach_set("co", co * np.float32(100.0))
                ob.data.update()
        us.scale_length = 0.01
        bpy.context.view_layer.update()
        for o in bpy.context.view_layer.objects:
            o.select_set(False)
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = B["objs"][B["root"]]
        r = bpy.ops.export_scene.fbx(filepath=path, check_existing=False, **settings)
        if "FINISHED" not in r:
            raise RuntimeError("FBX export failed: %s" % r)
    finally:
        us.scale_length = saved_scale_length
        for ob in objs:
            if ob.name in saved_loc:
                ob.location = saved_loc[ob.name]
            if ob.type == "MESH" and ob.data.name in saved_co:
                ob.data.vertices.foreach_set("co", saved_co[ob.data.name])
                ob.data.update()
        bpy.context.view_layer.update()
    settings["(temporary during export) scene.unit_settings.scale_length"] = 0.01
    settings["(temporary during export) vertices & local translations"] = "x100 (m -> cm), restored afterwards"
    return settings


def verify_fbx(path, B):
    from io_scene_fbx import parse_fbx
    root, version = parse_fbx.parse(path)

    def child(e, name):
        return next((c for c in e.elems if c.id == name), None)

    def props70(e):
        p = child(e, b"Properties70")
        out = {}
        if p is not None:
            for c in p.elems:
                out[c.props[0].decode()] = c.props[4:] if len(c.props) > 4 else ()
        return out

    gs = props70(child(root, b"GlobalSettings"))
    objects = child(root, b"Objects")
    models, geoms = {}, {}
    for e in objects.elems:
        if e.id == b"Model":
            name = e.props[1].split(b"\x00\x01")[0].decode()
            p = props70(e)
            models[e.props[0]] = {"name": name, "kind": e.props[2].decode(),
                                  "t": list(p.get("Lcl Translation", (0, 0, 0))),
                                  "r": list(p.get("Lcl Rotation", (0, 0, 0))),
                                  "s": list(p.get("Lcl Scaling", (1, 1, 1))),
                                  "rot_order": p.get("RotationOrder", (0,))[0] if p.get("RotationOrder") else 0,
                                  "pre": list(p.get("PreRotation", (0, 0, 0))), "post": list(p.get("PostRotation", (0, 0, 0)))}
        elif e.id == b"Geometry":
            v = child(e, b"Vertices")
            geoms[e.props[0]] = np.array(v.props[0], float).reshape(-1, 3) if v is not None else None
    parent, geom_of = {}, {}
    for c in child(root, b"Connections").elems:
        if c.props[0] == b"OO":
            a, b_ = c.props[1], c.props[2]
            if a in models and (b_ in models or b_ == 0):
                parent[a] = b_
            elif a in geoms and b_ in models:
                geom_of[b_] = a
    by_name = {m["name"]: uid for uid, m in models.items()}

    def local(m):
        R = np.array(Euler([math.radians(a) for a in m["r"]], "XYZ").to_matrix())
        T = np.eye(4)
        T[:3, :3] = R @ np.diag(m["s"])
        T[:3, 3] = m["t"]
        return T

    world = {}

    def W(uid):
        if uid in world:
            return world[uid]
        p = parent.get(uid, 0)
        world[uid] = (W(p) if p else np.eye(4)) @ local(models[uid])
        return world[uid]

    pos_err, rot_err, scale_dev, nonzero_prepost = 0.0, 0.0, 0.0, []
    hier_bad = []
    for ln, T in B["fk"].items():
        uid = by_name.get(ln)
        if uid is None:
            hier_bad.append((ln, "missing"))
            continue
        m = models[uid]
        exp_parent = B["objs"][ln].parent.name if B["objs"][ln].parent else None
        got_parent = models[parent[uid]]["name"] if parent.get(uid) else None
        if exp_parent != got_parent:
            hier_bad.append((ln, got_parent, exp_parent))
        Wm = W(uid)
        pos_err = max(pos_err, float(np.linalg.norm(Wm[:3, 3] / 100.0 - T[:3, 3])))
        rot_err = max(rot_err, R_to_axis_angle(Wm[:3, :3].T @ T[:3, :3])[1])
        scale_dev = max(scale_dev, float(np.abs(np.array(m["s"]) - 1).max()))
        if any(abs(a) > 1e-9 for a in m["pre"] + m["post"]):
            nonzero_prepost.append(ln)
    # geometry of base_link in cm
    bl = by_name.get("base_link")
    g = geoms.get(geom_of.get(bl))
    bl_dims = (g.max(0) - g.min(0)).tolist() if g is not None else None
    return {
        "fbx_version": version,
        "UnitScaleFactor": gs.get("UnitScaleFactor", ("?",))[0],
        "OriginalUnitScaleFactor": gs.get("OriginalUnitScaleFactor", ("?",))[0],
        "UpAxis": (gs.get("UpAxis", ("?",))[0], gs.get("UpAxisSign", ("?",))[0]),
        "FrontAxis": (gs.get("FrontAxis", ("?",))[0], gs.get("FrontAxisSign", ("?",))[0]),
        "CoordAxis": (gs.get("CoordAxis", ("?",))[0], gs.get("CoordAxisSign", ("?",))[0]),
        "models": len(models), "mesh_models": sum(1 for m in models.values() if m["kind"] == "Mesh"),
        "null_models": sum(1 for m in models.values() if m["kind"] == "Null"),
        "names": sorted(m["name"] for m in models.values()),
        "hierarchy_mismatch": hier_bad, "world_pos_err_m": pos_err, "world_rot_err_rad": rot_err,
        "max_scale_deviation": scale_dev, "nonzero_pre_post_rotation": nonzero_prepost,
        "base_link_vertex_bbox_cm": bl_dims,
        "size_bytes": os.path.getsize(path),
    }


# --------------------------------------------------------------------------------------
# Joint map + report
# --------------------------------------------------------------------------------------

def wheel_kinematics(B):
    """Mecanum inverse kinematics derived from the wheel geometry (bottom roller direction)."""
    if not B["wheel_info"]:
        return None
    rows = []
    for g, f in zip(B["wheel_info"]["geometry_check"], B["wheel_info"]["frames"]):
        ux, uy = g["bottom_roller_axis_xy"]
        rows.append({"joint": f["joint"], "link": g["link"], "contact_x_m": g["contact_xy_m"][0],
                     "contact_y_m": g["contact_xy_m"][1], "radius_m": g["outer_radius_m"],
                     "bottom_roller_axis_xy": [ux, uy], "bottom_roller_angle_deg": g["bottom_roller_angle_deg"],
                     "sign_s": 1 if ux * uy > 0 else -1})
    return {
        "equation": "omega_i = ((vx - wz*y_i)*ux_i + (vy + wz*x_i)*uy_i) / (r_i * ux_i)  "
                    "~= (vx - wz*y_i + s_i*(vy + wz*x_i)) / r_i for 45 deg rollers",
        "inputs": "vx, vy [m/s], wz [rad/s] = /odom twist (child frame base_footprint); omega_i [rad/s] about the "
                  "wheel joint axis +Y; integrate omega_i*dt for the wheel joint position",
        "derivation": "the bottom roller spins freely about its own axis u_i, so the contact point velocity along u_i "
                      "must equal the wheel-driven velocity (omega_i * r_i along +X) projected on u_i",
        "wheels": rows}


def joint_map(B):
    out = {"robot": B["robot"].get("name"), "root_link": B["root"], "units": {"ros": "m, rad", "unreal": "cm, deg"},
           "conversion": {"location": "UE = (100*x, -100*y, 100*z)",
                          "rotator": "UE FRotator(Pitch=-pitch, Yaw=-yaw, Roll=roll) in degrees",
                          "quaternion": "UE (X,Y,Z,W) = (-qx, qy, -qz, qw)",
                          "joint_axis": "UE axis = (-ax, ay, -az); UE joint rotation = FQuat(UEAxis, position_rad) (same sign as /joint_states)",
                          "drive": "Component.SetRelativeRotation(RestRelativeQuat * FQuat(UEAxis, q))"},
           "joints": []}
    for jn, J in B["all_joints"].items():
        ue = ros_to_ue(J["xyz"], J["rpy"])
        a = np.asarray(J["axis"], float)
        out["joints"].append({
            "name": jn, "type": J["type"], "parent": J["parent"], "child": J["child"],
            "origin_xyz_m": list(J["xyz"]), "origin_rpy_rad": list(J["rpy"]),
            "axis": None if J["type"] == "fixed" else list(J["axis"]),
            "axis_normalized": None if J["type"] == "fixed" else (a / np.linalg.norm(a)).tolist(),
            "ue_relative_location_cm": ue["location_cm"], "ue_relative_rotator_deg": ue["rotator_deg"],
            "ue_relative_quat_xyzw": ue["quat_xyzw"],
            "ue_axis": None if J["type"] == "fixed" else ros_axis_to_ue(J["axis"]),
            "limit": J["limit"], "mimic": J["mimic"], "source": J["source"]})
    out["wheel_kinematics"] = wheel_kinematics(B)
    out["unreal_names"] = {"actor_or_component": "URDF link name", "static_mesh_asset": "SM_<link name>",
                           "materials": sorted({m.name for m in bpy.data.materials})}
    return out


def tree_text(B):
    objs = B["objs"]
    jn_of = {J["child"]: J for J in B["all_joints"].values()}
    lines = []

    def label(ob):
        J = jn_of.get(ob.name)
        tag = ""
        if J:
            tag = "   <- %s [%s]" % (J["name"], J["type"])
            if J["mimic"]:
                tag += " mimic %s x%g" % (J["mimic"]["joint"], J["mimic"]["multiplier"])
            if not J["source"].startswith("URDF"):
                tag += "  (DERIVED)"
        return "%s (%s)%s" % (ob.name, "mesh" if ob.type == "MESH" else "empty", tag)

    def rec(ob, prefix):
        kids = sorted(ob.children, key=lambda o: o.name)
        for i, c in enumerate(kids):
            last = i == len(kids) - 1
            lines.append("   " + prefix + ("`-- " if last else "|-- ") + label(c))
            rec(c, prefix + ("    " if last else "|   "))

    lines.append("   " + label(objs[B["root"]]))
    rec(objs[B["root"]], "")
    return "\n".join(lines)


def write_report(B, V, F, fbx_settings, cfg, path):
    L = []
    w = L.append
    now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    w("ROSOrin Pro -> Blender -> Unreal export report")
    w("=" * 78)
    w("Generated : %s   (Blender %s)" % (now, bpy.app.version_string))
    w("Package   : %s" % cfg["pkg_dir"])
    w("Entry     : %s   with env %s" % (cfg["entry_xacro"], json.dumps(cfg["env"])))
    w("Robot name: %s   root link: %s" % (B["robot"].get("name"), B["root"]))
    w("Outputs   : %s" % ", ".join(os.path.basename(p) for p in B["out_paths"].values()))
    absent = [n for n in cfg["delete_default_objects"] if n not in B["removed_defaults"]]
    w("Removed   : %s%s" % (", ".join(B["removed_defaults"]) or "nothing",
                            "; already absent (removed earlier): %s" % ", ".join(absent) if absent else ""))
    w("")
    w("Xacro files processed (in include order):")
    for f in B["xacro"].files:
        w("  - %s" % os.path.relpath(f, cfg["pkg_dir"]))
    w("Xacro conditionals:")
    for s in B["xacro"].log:
        w("  - %s" % s)
    w("")
    w("VERIFICATION SUMMARY: %s" % ("ALL CHECKS PASSED" if not V["fails"] and not F.get("fails") else
                                     "FAILURES: %s" % (V["fails"] + F.get("fails", []))))
    for st, name, detail in V["results"] + F.get("results", []):
        w("  [%s] %s -- %s" % (st, name, detail))
    w("")
    w("1. FINAL HIERARCHY (Blender object == URDF link == Unreal component)")
    w("-" * 78)
    w(tree_text(B))
    w("")
    w("   %d objects: %d URDF links (%d meshes, %d empties) + %d derived wheel links" % (
        len(B["objs"]), len(B["links"]), sum(1 for l in B["links"] if B["objs"][l].type == "MESH"),
        sum(1 for l in B["links"] if B["objs"][l].type == "EMPTY"), len(B["extra_joints"])))
    w("")
    w("2. LINK -> MESH MAPPING")
    w("-" * 78)
    w("   %-20s %-52s %8s %8s  %s" % ("link", "mesh (URDF filename)", "STL tris", "Blender", "material"))
    tri = {r[0]: r for r in V["tri_rows"]}
    for ln in B["order"]:
        if ln in B["mesh_map"]:
            for m in B["mesh_map"][ln]:
                n_obj = len(B["objs"][ln].data.polygons)
                w("   %-20s %-52s %8d %8d  %s" % (ln, m["urdf_filename"].replace("file://" + cfg["pkg_dir"], "$(find rosorin_description)"),
                                                 tri[ln][2], n_obj, m["material"]))
        else:
            w("   %-20s %-52s" % (ln, "(no visual - Empty)"))
    if B["wheel_info"]:
        for g in B["wheel_info"]["geometry_check"]:
            w("   %-20s %-52s %8s %8d  %s" % (g["link"], "(split from meshes/pro/mecanum/base_link.stl)", "-",
                                             len(B["objs"][g["link"]].data.polygons), B["objs"][g["link"]].data.materials[0].name))
        s = B["wheel_info"]["split"]
        w("   base_link.stl: %d triangles in %d loose parts -> base_link keeps %d, wheels get %d (lossless)" % (
            s["triangles_total"], s["components_total"], s["triangles_base_after"], s["triangles_total"] - s["triangles_base_after"]))
    w("   Unreferenced package meshes (not imported on purpose): " + ", ".join(sorted(
        os.path.relpath(p, cfg["pkg_dir"]) for p in B["unreferenced_meshes"])))
    w("")
    w("3. JOINTS: PARENT/CHILD, TYPE, AXIS, LIMITS, MIMIC")
    w("-" * 78)
    w("   %-22s %-10s %-14s %-18s %-32s %s" % ("joint", "type", "parent", "child", "axis (joint frame)", "limits / mimic"))
    for jn, J in B["all_joints"].items():
        ax = "-" if J["type"] == "fixed" else J["axis_text"]
        extra = ""
        if J["limit"] and J["type"] != "fixed":
            extra = "[%g, %g] rad, effort %g, vel %g" % (J["limit"].get("lower", 0), J["limit"].get("upper", 0),
                                                           J["limit"].get("effort", 0), J["limit"].get("velocity", 0))
        if J["mimic"]:
            extra += "  mimic %s*%g%+g" % (J["mimic"]["joint"], J["mimic"]["multiplier"], J["mimic"]["offset"])
        if not J["source"].startswith("URDF"):
            extra += "  DERIVED"
        w("   %-22s %-10s %-14s %-18s %-32s %s" % (jn, J["type"], J["parent"], J["child"], ax, extra))
    w("")
    w("   Measured axis check (each movable joint rotated %.2f rad in Blender):" % 0.35)
    w("   %-22s %-18s %-40s %10s %10s" % ("joint", "child", "expected world axis at rest", "axis err", "pivot drift"))
    for jn, ch, a, e, ea, ep in V["axis_rows"]:
        w("   %-22s %-18s (%+.6f, %+.6f, %+.6f)       %.2e   %.2e m" % (jn, ch, a[0], a[1], a[2], e, ep))
    w("")
    w("4. JOINT ORIGIN TRANSFORMS (child frame in parent frame, at joint position 0)")
    w("-" * 78)
    w("   ROS/Blender: meters, rpy radians (URDF literal).  Unreal: cm, FRotator degrees (Pitch, Yaw, Roll).")
    for jn, J in B["all_joints"].items():
        ue = ros_to_ue(J["xyz"], J["rpy"])
        w("   %s  (%s -> %s)" % (jn, J["parent"], J["child"]))
        w("       ROS xyz = %s   rpy = %s" % (J["xyz_text"], J["rpy_text"]))
        w("       UE  loc = (%.5f, %.5f, %.5f) cm   rot = (P=%.5f, Y=%.5f, R=%.5f) deg   quat(xyzw) = (%.7f, %.7f, %.7f, %.7f)" % (
            *ue["location_cm"], ue["rotator_deg"]["pitch"], ue["rotator_deg"]["yaw"], ue["rotator_deg"]["roll"], *ue["quat_xyzw"]))
        if J["type"] != "fixed":
            w("       UE joint axis = (%.7f, %.7f, %.7f)" % tuple(ros_axis_to_ue(J["axis"])))
    w("")
    w("   World (base_footprint) pose of every link at zero joint positions [m]:")
    for ln in B["order"] + [g["link"] for g in (B["wheel_info"]["geometry_check"] if B["wheel_info"] else [])]:
        T = V["fk"][ln]
        w("     %-20s (% .6f, % .6f, % .6f)" % (ln, T[0, 3], T[1, 3], T[2, 3]))
    w("")
    w("5. BLENDER -> UNREAL COORDINATE CONVERSION")
    w("-" * 78)
    w("""   Blender scene = ROS REP-103 frame: right-handed, +X forward, +Y left, +Z up, 1 BU = 1 m.
   Unreal:                           left-handed,  +X forward, +Y right, +Z up, 1 uu = 1 cm.
   FBX file: UnitScaleFactor = 1 (cm), Up = +Z, Front = -Y, Coord = +X, right-handed.
   That is exactly Unreal's FBX import axis system (Z-up, -Y front), so the importer applies no
   axis rotation and only its right->left handed flip (Y -> -Y). Net mapping:
       position     : UE(x, y, z) cm = (100*x, -100*y, 100*z)  with ROS (x, y, z) in m
       orientation  : UE FRotator(Pitch, Yaw, Roll) = (-pitch, -yaw, +roll) of the URDF rpy, in degrees
       quaternion   : UE (X, Y, Z, W) = (-qx, qy, -qz, qw)
       joint axis   : UE axis = (-ax, ay, -az);  rotation = FQuat(UEAxis, q) with q straight from /joint_states
       /odom        : actor location = (100*x, -100*y, 100*z) of odom->base_footprint,
                      actor rotation = (-qx, qy, -qz, qw) of its quaternion.
   Driving a joint in Unreal:  Comp->SetRelativeRotation(RestRelativeQuat * FQuat(UEAxis, q))
   (RestRelativeQuat = the component's imported relative rotation; FQuat A*B applies B first).
   Mimic joints (gripper): q_child = multiplier * q_r_joint + offset (see table above).
   Import in Unreal with: Force Front X Axis = OFF, Convert Scene = ON (default), Convert Scene Unit = ON
   (no-op, file is already cm), Import Uniform Scale = 1.0, Transform Vertex To Absolute = OFF,
   Bake Pivot In Vertex = OFF, Combine Meshes = OFF. Use File > Import Into Level to get the hierarchy:
   one actor per URDF link (named like the link, attached like the URDF tree) under an identity
   'RootNode' actor that Interchange creates for the FBX scene root; static mesh assets are named
   SM_<link> (UE 5.8 Interchange appends '_001', e.g. SM_link1_001 - cosmetic).
   Drive base_footprint (or RootNode) from /odom. If you convert the actors to one Blueprint
   (Convert Selection to Blueprint Class), re-check the component attachment afterwards.""")
    w("")
    wk = wheel_kinematics(B)
    if wk:
        w("   WHEEL KINEMATICS (derived wheels, not in /joint_states):")
        w("     %s" % wk["equation"])
        w("     inputs: %s" % wk["inputs"])
        w("     %-22s %10s %10s %9s %22s %6s" % ("wheel joint", "x_i [m]", "y_i [m]", "r_i [m]", "bottom roller dir (XY)", "s_i"))
        for r in wk["wheels"]:
            w("     %-22s %10.5f %10.5f %9.5f   (%+.3f, %+.3f) %+5.1fdeg %+3d" % (
                r["joint"], r["contact_x_m"], r["contact_y_m"], r["radius_m"], r["bottom_roller_axis_xy"][0],
                r["bottom_roller_axis_xy"][1], r["bottom_roller_angle_deg"], r["sign_s"]))
        w("     (x_i, y_i = mean ground contact below each wheel axis in base_footprint; s_i from the measured")
        w("      direction of the roller currently at the bottom of each wheel - all rollers of a wheel share it.)")
    w("")
    w("6. FBX EXPORT SETTINGS USED (bpy.ops.export_scene.fbx)")
    w("-" * 78)
    for k, v in fbx_settings.items():
        w("   %-24s = %s" % (k, sorted(v) if isinstance(v, set) else v))
    w("   Meaning: during export the scene temporarily uses 1 BU = 1 cm (all vertices and local translations")
    w("   x100, unit scale 0.01). apply_unit_scale then gives a unit factor of 100*0.01 = 1 and axis Y-forward/")
    w("   Z-up is identity, so the exporter's global matrix is identity: plain centimeter values, UnitScaleFactor 1,")
    w("   every node scale 1, no root correction rotation. The original meter values are restored bit-exactly")
    w("   afterwards and re-verified (see 'post-export' checks). 'Apply Transform' (bake_space_transform) is")
    w("   deliberately OFF: in Blender 5.2 it corrupts nested hierarchies (verified during development).")
    w("")
    w("   FBX read-back verification (parsed from the written file):")
    for k in ("fbx_version", "UnitScaleFactor", "OriginalUnitScaleFactor", "UpAxis", "FrontAxis", "CoordAxis",
              "models", "mesh_models", "null_models", "world_pos_err_m", "world_rot_err_rad", "max_scale_deviation",
              "nonzero_pre_post_rotation", "hierarchy_mismatch", "base_link_vertex_bbox_cm", "size_bytes"):
        w("     %-26s %s" % (k, F.get(k)))
    w("")
    w("7. WARNINGS / UNRESOLVED ISSUES / DECISIONS")
    w("-" * 78)
    for i, s in enumerate(B["warnings"], 1):
        w("   W%d. %s" % (i, s))
    for s in B["notes"]:
        w("   note: %s" % s)
    w("")
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write("\n".join(L))


def collect_warnings(B, cfg):
    Wn = B["warnings"]
    env = cfg["env"]
    Wn.append("LIDAR_TYPE is read with $(env LIDAR_TYPE) and is not defined anywhere in the package. The three lidar "
              "xacros use the same mesh and position and differ only in lidar_joint yaw: MS200 = 0, LD19 = 3.1416, "
              "SCLIDAR = 3.1415926535897931. This build uses LIDAR_TYPE=%s (yaw %s). If your robot has another lidar, "
              "re-run the script with --lidar LD19 / SCLIDAR (or rotate lidar_frame 180 deg about its Z)."
              % (env["LIDAR_TYPE"], B["joints"]["lidar_joint"]["rpy_text"]))
    if B["wheel_info"]:
        g = B["wheel_info"]["geometry_check"]
        Wn.append("The Pro URDF (car_mecanum.urdf.xacro) has NO wheel links/joints: the four mecanum wheels are baked into "
                  "meshes/pro/mecanum/base_link.stl. To allow independent wheel rotation they were split out losslessly "
                  "(%d of %d triangles; whole loose parts only, %s loose parts per wheel = hub, hub cap, %s rollers and "
                  "their roller pins; motors stay in base_link). Wheel link and joint names and frames come from "
                  "urdf/rosorin_old.xacro (left_front_wheel etc., declared 'fixed', axis 0 0 0 there; same chassis); here "
                  "they are typed 'continuous' about +Y of the wheel frame. Geometry cross-check: wheel centroid is %.2f mm "
                  "max from the source frame axis, symmetry axis within %.3f deg of Y, outer radius %.4f m. These joints are "
                  "NOT in the robot's /joint_states - compute wheel spin in Unreal from the /odom twist with the mecanum "
                  "equations in the WHEEL KINEMATICS section. Positive rotation about +Y rolls the robot forward on both sides."
                  % (B["wheel_info"]["split"]["triangles_total"] - B["wheel_info"]["split"]["triangles_base_after"],
                     B["wheel_info"]["split"]["triangles_total"],
                     "/".join(sorted({str(x["components"]) for x in g})), "/".join(sorted({str(x["rollers"]) for x in g})),
                     1000 * max(x["centroid_offset_norm_m"] for x in g),
                     max(x["symmetry_axis_angle_to_Y_deg"] for x in g), float(np.mean([x["outer_radius_m"] for x in g]))))
    Wn.append("Gripper: closed-chain linkage expressed with mimic joints. Only r_joint is independent; l_joint (x1), "
              "l_in_joint (x-1), l_out_joint (x1), r_in_joint (x1), r_out_joint (x-1) follow it. The FBX has no "
              "constraints - apply q = multiplier * q_r_joint + offset in Unreal (joint_state_publisher also publishes "
              "the mimic joints, so they may already be present in /joint_states).")
    Wn.append("URDF values are used literally, including rounded pi values (e.g. gripper_joint rpy = -3.1416 -1.5681 "
              "3.1416, l_joint rpy = 3.1416 1.5681 3.1416). This matches what robot_state_publisher computes on the robot.")
    non_unit = []
    for jn, J in B["joints"].items():
        if J["type"] != "fixed":
            n = float(np.linalg.norm(J["axis"]))
            if abs(n - 1) > 1e-7:
                non_unit.append("%s |axis|=%.7f" % (jn, n))
    if non_unit:
        Wn.append("Some URDF joint axes are not exactly unit length (%s); they are normalized (as ROS/KDL does) for "
                  "rotation. Literal values are kept in the ros_joint_axis custom property." % ", ".join(non_unit))
    Wn.append("Commented-out / unused files: urdf/pro/common.urdf.xacro (back_shell_black/green links) references "
              "meshes/pro/common/back_shell_*.stl that do not exist in the package, and urdf/pro/connect.urdf.xacro "
              "defines a conflicting joint1 origin. Both includes are commented out in rosorinpro.xacro, so they are not "
              "part of the Pro model and nothing is missing.")
    Wn.append("Non-Pro chassis meshes (meshes/base_link_ack.stl, base_link_diff.stl, base_link_mec.stl) belong to the "
              "ROSOrin_Acker / _Differential / _Mecanum machine types and were not imported.")
    Wn.append("Collision geometry in the URDF is identical to the visual STLs; no separate collision meshes were "
              "exported (in Unreal use 'Use Complex as Simple' or generate convex hulls).")
    Wn.append("base_joint z = 0.025 m while the lowest base_link vertex is at z = -0.0233 m, so the model rests %.1f mm "
              "above the base_footprint plane. Kept as-is (URDF exact)." % (1000 * B["lowest_z"]))
    Wn.append("imu_link has no visual; its URDF inertial block duplicates base_link's mass (0.8087 kg). Irrelevant for "
              "visuals, kept as an Empty frame.")
    Wn.append("URDF colours are applied as sRGB (converted to linear Base Color) on %d shared materials; wheels use "
              "base_link's material because they were part of its visual." % len({m.name for m in bpy.data.materials}))
    Wn.append("On import Unreal 5.8 logs 'degenerate tangent bases' / 'nearly zero bi-normals' Display messages for the "
              "small gripper, camera and link meshes (thin CAD sliver triangles in the source STLs). They are "
              "informational only; geometry and transforms are unaffected.")


def run():
    cfg = get_cfg()
    B = build(cfg)
    V = verify(B, cfg)
    B["lowest_z"] = V["lowest_z"]
    B["fk"] = V["fk"]
    # meshes present in the package but not referenced by this URDF
    ref = {m["path"] for ms in B["mesh_map"].values() for m in ms}
    allm = set()
    for dp, dn, fn in os.walk(os.path.join(cfg["pkg_dir"], "meshes")):
        for f in fn:
            if f.lower().endswith(".stl"):
                allm.add(os.path.normpath(os.path.join(dp, f)))
    B["unreferenced_meshes"] = sorted(allm - ref)
    collect_warnings(B, cfg)
    if V["fails"]:
        print("VERIFICATION FAILED:", V["fails"])
    op = B["out_paths"]
    # resolved URDF for reference
    ET.indent(B["robot"])
    with open(op["urdf_out_name"], "wb") as f:
        f.write(b'<?xml version="1.0" encoding="utf-8"?>\n<!-- generated by build_rosorinpro_blender.py from '
                + cfg["entry_xacro"].encode() + b" with env " + json.dumps(cfg["env"]).encode() + b" -->\n")
        f.write(ET.tostring(B["robot"], encoding="utf-8"))
    bpy.ops.wm.save_as_mainfile(filepath=op["blend_name"], check_existing=False, compress=True)
    fbx_settings = export_fbx(B, cfg, op["fbx_name"])
    F = verify_fbx(op["fbx_name"], B)
    fres, ffails = [], []

    def fcheck(name, ok, detail):
        fres.append(("PASS" if ok else "FAIL", name, detail))
        if not ok:
            ffails.append(name)
    fcheck("FBX units = centimeters (UnitScaleFactor 1)", abs(float(F["UnitScaleFactor"]) - 1.0) < 1e-9, "UnitScaleFactor=%s" % F["UnitScaleFactor"])
    fcheck("FBX axis system = Z up, -Y front, +X coord (Unreal import axes)",
           tuple(F["UpAxis"]) == (2, 1) and tuple(F["FrontAxis"]) == (1, -1) and tuple(F["CoordAxis"]) == (0, 1),
           "Up=%s Front=%s Coord=%s" % (F["UpAxis"], F["FrontAxis"], F["CoordAxis"]))
    fcheck("FBX contains every robot object once", F["names"] == sorted(B["objs"]), "models=%d (mesh %d, null %d)" % (F["models"], F["mesh_models"], F["null_models"]))
    fcheck("FBX hierarchy == Blender/URDF hierarchy", not F["hierarchy_mismatch"], "mismatch=%s" % F["hierarchy_mismatch"])
    fcheck("FBX world transforms == URDF FK (cm -> m)", F["world_pos_err_m"] < 1e-5 and F["world_rot_err_rad"] < 1e-5,
           "pos err %.3g m, rot err %.3g rad" % (F["world_pos_err_m"], F["world_rot_err_rad"]))
    fcheck("FBX node scales all 1, no pre/post rotations", F["max_scale_deviation"] < 1e-6 and not F["nonzero_pre_post_rotation"],
           "max |s-1|=%.3g, pre/post=%s" % (F["max_scale_deviation"], F["nonzero_pre_post_rotation"]))
    bl_exp = np.array(V["base_link_dims_m"]) * 100.0
    fcheck("FBX geometry in centimeters (base_link vertex bbox = Blender bbox x 100)",
           F["base_link_vertex_bbox_cm"] is not None and float(np.abs(np.array(F["base_link_vertex_bbox_cm"]) - bl_exp).max()) < 1e-3,
           "FBX %s cm vs expected %s cm" % (np.round(F["base_link_vertex_bbox_cm"], 4).tolist(), np.round(bl_exp, 4).tolist()))
    # the export temporarily rescales the scene: prove the Blender data is exactly as before
    V2 = verify(B, cfg)
    same = not V2["fails"] and V2["dims"] == V["dims"]
    fcheck("post-export: Blender scene restored exactly (all Blender checks re-run)", same,
           "re-run fails=%s, dims identical=%s" % (V2["fails"], V2["dims"] == V["dims"]))
    bpy.ops.wm.save_mainfile(check_existing=False, compress=True)   # keep the saved .blend identical to the verified state
    F["results"], F["fails"] = fres, ffails
    jm = joint_map(B)
    with open(op["joint_map_name"], "w", encoding="utf-8") as f:
        json.dump(jm, f, indent=2)
    write_report(B, V, F, fbx_settings, cfg, op["report_name"])
    summary = {"verification_fails": V["fails"] + ffails,
               "checks": [(s, n) for s, n, d in V["results"] + fres],
               "outputs": {k: (p, os.path.getsize(p)) for k, p in op.items()},
               "fbx": {k: F[k] for k in ("UnitScaleFactor", "UpAxis", "FrontAxis", "CoordAxis", "models",
                                         "world_pos_err_m", "world_rot_err_rad", "base_link_vertex_bbox_cm")},
               "dims_m": V["dims"]}
    print(json.dumps(summary, indent=1, default=str))
    return summary


if __name__ == "__main__" or "ROSORIN_CFG" in globals():
    ROSORIN_SUMMARY = run()

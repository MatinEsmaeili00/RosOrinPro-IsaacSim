"""Step 1 - build the simulation URDF for the ROSOrin Pro.

Input : the Hiwonder rosorin_description package + the flattened Pro URDF (see config/rosorin_pro_sim.yaml)
Output: description/urdf/rosorin_pro_sim.urdf, description/meshes/**, description/build_report.json

What it changes compared with the Hiwonder URDF (all documented in docs/DECISIONS.md):
  * lidar_joint yaw from the configured lidar variant (real robot: SCLIDAR / COIN-D6 -> yaw = pi)
  * the 4 mecanum wheels are split out of meshes/pro/mecanum/base_link.stl (same loose-part rule as the
    RosOrinProMesh Blender build), and every wheel is split again into a hub and 9 free-spinning rollers
    -> wheel joints are continuous about +Y, roller joints are continuous about the roller axis (PCA)
  * realistic link masses (inertia tensors scaled with the mass, CAD centre of mass kept)
  * imu_link's duplicated chassis inertial removed
  * joint effort/velocity limits from the real servo/motor specs

Runs with any Python that has numpy + pyyaml, e.g. the Isaac Sim interpreter:
    C:\\isaac-sim-standalone-6.2.0\\python.bat scripts\\build_sim_urdf.py
"""

import argparse
import json
import math
import os
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

LIDAR_YAW = {"MS200": "0", "LD19": "3.1416", "SCLIDAR": "3.1415926535897931"}


# ----------------------------------------------------------------------------------------------
# math helpers (URDF conventions: rpy = fixed-axis roll, pitch, yaw -> R = Rz * Ry * Rx)
# ----------------------------------------------------------------------------------------------

def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def vec(text, default=(0.0, 0.0, 0.0)):
    if text is None:
        return np.array(default, float)
    return np.array([float(t) for t in text.split()], float)


def fmt(values):
    return " ".join("%.9g" % float(v) for v in values)


# ----------------------------------------------------------------------------------------------
# STL I/O
# ----------------------------------------------------------------------------------------------

def read_stl(path):
    """Return (vertices[N*3,3], triangles[N,3]) as an unwelded triangle soup in file units (meters)."""
    with open(path, "rb") as f:
        data = f.read()
    n = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if len(data) == 84 + 50 * n:
        rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]),
                            count=n, offset=84)
        co = rec["v"].reshape(-1, 3).astype(np.float64)
    else:  # ASCII STL
        txt = data.decode("utf-8", "ignore")
        co = np.array([[float(t) for t in line.split()[1:4]] for line in txt.splitlines()
                       if line.strip().startswith("vertex")])
    tris = np.arange(len(co)).reshape(-1, 3)
    return co, tris


def write_stl(path, co, tris, name="rosorin"):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    v = co[tris].astype(np.float32)
    nrm = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    ln = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.where(ln > 0, nrm / np.maximum(ln, 1e-30), 0).astype(np.float32)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = nrm, v
    with open(path, "wb") as f:
        f.write(name.encode()[:80].ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def triangle_components(co, tris, weld=1e-6):
    """Connected components of a triangle soup (vertices welded on a 1 micron grid).

    Identical to triangle_components() in RosOrinProMesh/scripts/build_rosorinpro_blender.py so the
    wheel split matches the Blender/Unreal model triangle for triangle.
    """
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


def area_centroid(co, tris):
    tv = co[tris]
    area = 0.5 * np.linalg.norm(np.cross(tv[:, 1] - tv[:, 0], tv[:, 2] - tv[:, 0]), axis=1)
    return (tv.mean(axis=1) * area[:, None]).sum(0) / area.sum()


# ----------------------------------------------------------------------------------------------
# URDF helpers
# ----------------------------------------------------------------------------------------------

def sub(parent, tag, **attrs):
    e = ET.SubElement(parent, tag)
    for k, v in attrs.items():
        e.set(k, v)
    return e


def add_inertial(link, mass, com, inertia):
    """inertia = 3x3 tensor about the COM, expressed in the link frame."""
    inn = sub(link, "inertial")
    sub(inn, "origin", xyz=fmt(com), rpy="0 0 0")
    sub(inn, "mass", value="%.9g" % mass)
    I = inertia
    sub(inn, "inertia", ixx="%.9g" % I[0, 0], ixy="%.9g" % I[0, 1], ixz="%.9g" % I[0, 2],
        iyy="%.9g" % I[1, 1], iyz="%.9g" % I[1, 2], izz="%.9g" % I[2, 2])


def add_mesh_geom(link, tag, uri, rgba=None, material_name=None):
    g = sub(link, tag)
    sub(g, "origin", xyz="0 0 0", rpy="0 0 0")
    geo = sub(g, "geometry")
    sub(geo, "mesh", filename=uri)
    if tag == "visual" and rgba is not None:
        m = sub(g, "material", name=material_name or "")
        sub(m, "color", rgba=rgba)
    return g


def cylinder_inertia(m, r, h, axis):
    """Solid cylinder inertia (about its centre) with symmetry axis `axis` (unit 3-vector)."""
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    i_axis = 0.5 * m * r * r
    i_perp = m * (3 * r * r + h * h) / 12.0
    return i_perp * np.eye(3) + (i_axis - i_perp) * np.outer(a, a)


def indent(elem, level=0):
    pad = "\n" + "  " * level
    if len(elem):
        if not (elem.text or "").strip():
            elem.text = pad + "  "
        for child in elem:
            indent(child, level + 1)
        if not (child.tail or "").strip():
            child.tail = pad
    if level and not (elem.tail or "").strip():
        elem.tail = pad


# ----------------------------------------------------------------------------------------------
# main build
# ----------------------------------------------------------------------------------------------

def build(cfg_path, lidar_override=None):
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    dcfg, wcfg, masses = cfg["description"], cfg["wheels"], cfg["masses"]
    pkg_dir = os.path.normpath(os.path.join(REPO, dcfg["source_package"]))
    resolved = os.path.normpath(os.path.join(REPO, dcfg["resolved_urdf"]))
    out_urdf = os.path.join(REPO, dcfg["output_urdf"])
    out_mesh_dir = os.path.join(REPO, dcfg["output_meshes"])
    pkg_name = dcfg["package_name"]
    lidar_variant = lidar_override or dcfg["lidar_variant"]
    report = {"source_package": os.path.relpath(pkg_dir, REPO).replace("\\", "/"),
              "resolved_urdf": os.path.relpath(resolved, REPO).replace("\\", "/"), "lidar_variant": lidar_variant}

    def uri(rel):
        return "package://%s/meshes/%s" % (pkg_name, rel.replace("\\", "/"))

    tree = ET.parse(resolved)
    robot = tree.getroot()
    robot.set("name", "rosorin_pro")
    links = {l.get("name"): l for l in robot.findall("link")}
    joints = {j.get("name"): j for j in robot.findall("joint")}

    # 1. lidar variant ---------------------------------------------------------------------------
    lj = joints["lidar_joint"].find("origin")
    lj.set("rpy", "0 0 %s" % LIDAR_YAW[lidar_variant])

    # 2. copy referenced meshes and rewrite their URIs -----------------------------------------------
    if os.path.isdir(out_mesh_dir):
        shutil.rmtree(out_mesh_dir)
    copied = []
    for mesh in robot.iter("mesh"):
        fn = mesh.get("filename").replace("\\", "/")
        rel = fn[fn.index("meshes/") + len("meshes/"):]
        src = os.path.join(pkg_dir, "meshes", rel)
        dst = os.path.join(out_mesh_dir, rel)
        if not os.path.isfile(src):
            raise FileNotFoundError(src)
        if rel not in copied:
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copyfile(src, dst)
            copied.append(rel)
        mesh.set("filename", uri(rel))
    report["copied_meshes"] = copied

    # 3. masses ----------------------------------------------------------------------------------------
    mass_report = {}
    for name, link in links.items():
        if name not in masses:
            continue
        inn = link.find("inertial")
        if masses[name] is None:
            if inn is not None:
                link.remove(inn)
            mass_report[name] = {"urdf": float(inn.find("mass").get("value")) if inn is not None else None,
                                 "sim": None, "note": "inertial removed (massless frame)"}
            continue
        m_old = float(inn.find("mass").get("value"))
        m_new = float(masses[name])
        s = m_new / m_old
        inn.find("mass").set("value", "%.9g" % m_new)
        it = inn.find("inertia")
        for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            it.set(k, "%.9g" % (float(it.get(k)) * s))
        mass_report[name] = {"urdf": m_old, "sim": m_new, "inertia_scale": s}

    # 4. realistic joint limits ------------------------------------------------------------------------
    arm, grip = cfg["drives"]["arm"], cfg["drives"]["gripper"]
    for j in joints.values():
        lim = j.find("limit")
        if lim is None:
            continue
        lim.set("effort", "%g" % arm["max_effort"])
        lim.set("velocity", "%g" % arm["max_velocity"])
    # gripper: keep the fingers from passing through each other (finger gap 0 at r_joint ~ -0.65 rad)
    glim = joints[grip["joint"]].find("limit")
    glim.set("lower", "%g" % grip["limits"][0])
    glim.set("upper", "%g" % grip["limits"][1])

    # 5. wheels: split base_link.stl ----------------------------------------------------------------------
    base_stl = os.path.join(pkg_dir, "meshes", "pro", "mecanum", "base_link.stl")
    co, tris = read_stl(base_stl)
    comp = triangle_components(co, tris)
    ids = np.unique(comp)

    old = ET.parse(os.path.join(pkg_dir, wcfg_path(dcfg))).getroot()
    frames = []
    for j in old.findall("joint"):
        child = j.find("child").get("link")
        if child.endswith("_wheel_link"):
            o = j.find("origin")
            frames.append({"joint": j.get("name"), "link": child, "xyz": vec(o.get("xyz")), "rpy": vec(o.get("rpy"))})
    frames.sort(key=lambda f: wcfg["names"].index(f["joint"]))
    assert [f["joint"] for f in frames] == wcfg["names"], frames

    assign = np.full(len(tris), -1, dtype=np.int32)
    for cid in ids:
        sel = np.nonzero(comp == cid)[0]
        v = co[tris[sel].reshape(-1)]
        hits = []
        for i, f in enumerate(frames):
            R = rpy_to_R(*f["rpy"])
            vl = (v - f["xyz"]) @ R
            side = math.copysign(1.0, f["xyz"][1])
            outboard = (vl[:, 1] * side >= -wcfg["inner_tolerance"]).all()
            radial = np.sqrt(vl[:, 0] ** 2 + vl[:, 2] ** 2).max()
            if outboard and radial <= wcfg["radial_max"]:
                hits.append(i)
        if len(hits) > 1:
            raise RuntimeError("base_link component %d matches several wheels" % cid)
        if hits:
            assign[sel] = hits[0]

    rest = np.nonzero(assign < 0)[0]
    write_stl(os.path.join(out_mesh_dir, "sim", "base_link_chassis.stl"), co, tris[rest], "base_link_chassis")
    report["base_split"] = {"triangles_total": int(len(tris)), "components_total": int(len(ids)),
                            "triangles_chassis": int(len(rest))}
    base = links["base_link"]
    for tag in ("visual", "collision"):
        base.find(tag).find("geometry").find("mesh").set("filename", uri("sim/base_link_chassis.stl"))

    wheel_rgba = "0.15 0.15 0.15 1"
    roller_rgba = "0.08 0.08 0.08 1"
    insert_at = list(robot).index(joints["lidar_joint"]) + 1
    new_elems = []
    wheel_report = []
    for i, f in enumerate(frames):
        sel = np.nonzero(assign == i)[0]
        R = rpy_to_R(*f["rpy"])
        col = (co - f["xyz"]) @ R  # all vertices in this wheel frame
        wcomp = comp[sel]
        radius = np.sqrt(col[tris[sel].reshape(-1)][:, 0] ** 2 + col[tris[sel].reshape(-1)][:, 2] ** 2).max()
        rollers, hub_tris = [], []
        for cid in np.unique(wcomp):
            ct = tris[comp == cid]
            rv = col[ct.reshape(-1)]
            if np.sqrt(rv[:, 0] ** 2 + rv[:, 2] ** 2).max() >= radius - wcfg["roller_outer_tolerance"]:
                rollers.append(ct)
            else:
                hub_tris.append(ct)
        if len(rollers) != wcfg["expected_rollers"]:
            raise RuntimeError("%s: found %d rollers, expected %d" % (f["link"], len(rollers), wcfg["expected_rollers"]))
        hub_tris = np.concatenate(hub_tris)
        # sort rollers by angle around the wheel axis so names are stable (roller_0 starts at +X)
        def roller_angle(ct):
            c = col[ct.reshape(-1)].mean(0)
            return math.atan2(-c[2], c[0]) % (2 * math.pi)
        rollers.sort(key=roller_angle)

        wl = f["link"]
        hub_rel = "sim/%s_hub.stl" % wl
        write_stl(os.path.join(out_mesh_dir, hub_rel), col, hub_tris, wl + "_hub")
        link = ET.Element("link", name=wl)
        hub_c = area_centroid(col, hub_tris)
        add_inertial(link, masses["wheel_hub"], hub_c, cylinder_inertia(masses["wheel_hub"], 0.030, 0.030, [0, 1, 0]))
        add_mesh_geom(link, "visual", uri(hub_rel), wheel_rgba, "wheel_hub")
        joint = ET.Element("joint", name=f["joint"], type="continuous")
        sub(joint, "origin", xyz=fmt(f["xyz"]), rpy=fmt(f["rpy"]))
        sub(joint, "parent", link="base_link")
        sub(joint, "child", link=wl)
        sub(joint, "axis", xyz=fmt(wcfg["axis"]))
        sub(joint, "limit", effort="%g" % cfg["drives"]["wheel"]["max_effort"],
            velocity="%g" % cfg["drives"]["wheel"]["max_velocity"])
        sub(joint, "dynamics", damping="0", friction="0")
        new_elems += [link, joint]

        r_rep = []
        for k, ct in enumerate(rollers):
            rv = col[ct.reshape(-1)]
            c0 = rv.mean(0)
            d = rv - c0
            evals, evecs = np.linalg.eigh(d.T @ d)
            u = evecs[:, 2]                       # longest direction = roller spin axis
            if u[0] < 0:
                u = -u
            t = d @ u
            center = c0 + u * 0.5 * (t.max() + t.min())
            perp = (rv - center) - np.outer((rv - center) @ u, u)
            r_roll = float(np.linalg.norm(perp, axis=1).max())
            length = float(t.max() - t.min())
            rname = "%s_roller_%d" % (f["joint"], k)
            rrel = "sim/%s.stl" % rname
            write_stl(os.path.join(out_mesh_dir, rrel), col - center, ct, rname)
            rl = ET.Element("link", name=rname)
            add_inertial(rl, masses["roller"], [0, 0, 0], cylinder_inertia(masses["roller"], r_roll, length, u))
            add_mesh_geom(rl, "visual", uri(rrel), roller_rgba, "roller")
            add_mesh_geom(rl, "collision", uri(rrel))
            rj = ET.Element("joint", name=rname + "_joint", type="continuous")
            sub(rj, "origin", xyz=fmt(center), rpy="0 0 0")
            sub(rj, "parent", link=wl)
            sub(rj, "child", link=rname)
            sub(rj, "axis", xyz=fmt(u))
            sub(rj, "dynamics", damping="%g" % cfg["drives"]["roller"]["damping"], friction="0")
            new_elems += [rl, rj]
            r_rep.append({"name": rname, "center": center.tolist(), "axis": u.tolist(), "radius": r_roll,
                          "length": length, "triangles": int(len(ct))})
        # roller closest to the ground (min z in wheel frame) -> mecanum angle of this wheel
        bottom = min(r_rep, key=lambda r: r["center"][2])
        ux, uy = bottom["axis"][0], bottom["axis"][1]
        wheel_report.append({
            "joint": f["joint"], "link": wl, "xyz": f["xyz"].tolist(), "rpy": f["rpy"].tolist(),
            "triangles": int(len(sel)), "hub_triangles": int(len(hub_tris)), "outer_radius": float(radius),
            "hub_centroid": hub_c.tolist(), "rollers": r_rep,
            "bottom_roller_angle_deg": math.degrees(math.atan2(uy, ux)),
        })
    for k, e in enumerate(new_elems):
        robot.insert(insert_at + k, e)
    report["wheels"] = wheel_report
    report["masses"] = mass_report

    # 6. total mass check ----------------------------------------------------------------------------------
    total = sum(float(l.find("inertial/mass").get("value")) for l in robot.findall("link") if l.find("inertial") is not None)
    report["total_mass_kg"] = total

    # 7. write ---------------------------------------------------------------------------------------------
    robot.insert(0, ET.Comment(
        " GENERATED by RosOrinPro-IsaacSim/scripts/build_sim_urdf.py - do not edit by hand. "
        "Source: Hiwonder rosorin_description (MACHINE_TYPE=ROSOrin_Mecanum_Pro, LIDAR_TYPE=%s). "
        "Changes: wheels + rollers split out of base_link.stl, realistic masses, servo/motor limits. " % lidar_variant))
    indent(robot)
    os.makedirs(os.path.dirname(out_urdf), exist_ok=True)
    tree.write(out_urdf, encoding="utf-8", xml_declaration=True)
    with open(os.path.join(REPO, "description", "build_report.json"), "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2)
    return report, out_urdf


def wcfg_path(dcfg):
    return dcfg["wheel_frames_xacro"]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=os.path.join(REPO, "config", "rosorin_pro_sim.yaml"))
    ap.add_argument("--lidar", choices=sorted(LIDAR_YAW), default=None, help="override the lidar variant")
    a = ap.parse_args()
    report, out = build(a.config, a.lidar)
    print("wrote", out)
    print("base_link split:", report["base_split"])
    for w in report["wheels"]:
        print("  %-18s tris=%6d hub=%6d rollers=%d r=%.4f bottom roller angle=%7.2f deg" % (
            w["joint"], w["triangles"], w["hub_triangles"], len(w["rollers"]), w["outer_radius"],
            w["bottom_roller_angle_deg"]))
    print("total mass: %.3f kg" % report["total_mass_kg"])


if __name__ == "__main__":
    main()
